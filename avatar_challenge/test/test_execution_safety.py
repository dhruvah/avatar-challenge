"""Execution-safety tests: what happens when the robot stops answering.

These use fakes rather than a live robot, so they run anywhere. They cover the
cases that are hard to reproduce on hardware but dangerous in the field: an
action goal that is accepted and then never completes, and a failure part-way
through a multi-shape run.

Run:  python3 -m pytest avatar_challenge/test/test_execution_safety.py -q
"""

import os
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# --- stub out rclpy so this imports without a ROS install --------------------
if "rclpy" not in sys.modules:                                   # pragma: no cover
    fake = types.ModuleType("rclpy")
    fake.ok = lambda: True

    def spin_until_future_complete(node, future, timeout_sec=None):
        return None

    fake.spin_until_future_complete = spin_until_future_complete
    sys.modules["rclpy"] = fake
    action_mod = types.ModuleType("rclpy.action")
    action_mod.ActionClient = object
    sys.modules["rclpy.action"] = action_mod
    class _CartesianRequest:
        """Mirrors the fields BlendedPathExecutor.plan() populates."""

        def __init__(self):
            self.header = types.SimpleNamespace(frame_id="")
            self.group_name = ""
            self.link_name = ""
            self.waypoints = []
            self.max_step = 0.0
            self.jump_threshold = 0.0
            self.avoid_collisions = False
            self.start_state = types.SimpleNamespace(
                is_diff=False, joint_state=types.SimpleNamespace(name=[], position=[]))

    class _Goal:
        def __init__(self):
            self.trajectory = None

    action_msgs = types.ModuleType("moveit_msgs.action")
    action_msgs.ExecuteTrajectory = type("ExecuteTrajectory", (), {"Goal": _Goal})
    sys.modules["moveit_msgs.action"] = action_msgs
    srv_msgs = types.ModuleType("moveit_msgs.srv")
    srv_msgs.GetCartesianPath = type("GetCartesianPath", (), {"Request": _CartesianRequest})
    sys.modules["moveit_msgs.srv"] = srv_msgs
    sys.modules.setdefault("moveit_msgs", types.ModuleType("moveit_msgs"))

from avatar_challenge.blended_path import BlendedPathExecutor  # noqa: E402


class FakeLogger:
    def __init__(self):
        self.errors, self.warns, self.infos = [], [], []

    def error(self, m):
        self.errors.append(m)

    def warn(self, m):
        self.warns.append(m)

    def info(self, m):
        self.infos.append(m)


class FakeNode:
    def __init__(self):
        self._log = FakeLogger()

    def get_logger(self):
        return self._log

    def create_client(self, *a, **k):
        return None


class FakeFuture:
    def __init__(self, value):
        self._value = value

    def result(self):
        return self._value


class HangingGoalHandle:
    """Accepted, but its result future never resolves."""

    def __init__(self, cancel_ack=True):
        self.accepted = True
        self.cancel_requested = False
        self._cancel_ack = cancel_ack

    def get_result_async(self):
        return FakeFuture(None)          # never completes

    def cancel_goal_async(self):
        self.cancel_requested = True
        return FakeFuture(object() if self._cancel_ack else None)


def _executor(handle):
    node = FakeNode()
    ex = BlendedPathExecutor.__new__(BlendedPathExecutor)
    ex._node = node
    ex._timeout = 0.01
    ex._exec_cli = types.SimpleNamespace(send_goal_async=lambda goal: FakeFuture(handle))
    return ex, node


def test_hanging_goal_is_cancelled_not_abandoned():
    """The node exiting does not stop the controller, so it must cancel."""
    handle = HangingGoalHandle()
    ex, node = _executor(handle)

    with pytest.raises(RuntimeError, match="timed out"):
        ex.execute(trajectory=object(), description="shape:blended")

    assert handle.cancel_requested, "a hung execution must request cancellation"
    assert any("cancel" in m.lower() for m in node.get_logger().errors)


def test_unacknowledged_cancel_says_the_arm_may_still_be_moving():
    """If cancellation is not acknowledged, the message must not imply safety."""
    handle = HangingGoalHandle(cancel_ack=False)
    ex, _ = _executor(handle)

    with pytest.raises(RuntimeError, match="may still be moving"):
        ex.execute(trajectory=object(), description="shape:blended")


def test_rejected_goal_raises_without_cancelling():
    handle = HangingGoalHandle()
    handle.accepted = False
    ex, _ = _executor(handle)

    with pytest.raises(RuntimeError, match="rejected"):
        ex.execute(trajectory=object(), description="shape:blended")
    assert not handle.cancel_requested


# --- retiming ----------------------------------------------------------------

class FakePoint:
    def __init__(self, t, vel, acc):
        self.time_from_start = types.SimpleNamespace(
            sec=int(t), nanosec=int(round((t % 1) * 1e9)))
        self.velocities = list(vel)
        self.accelerations = list(acc)

    @property
    def t(self):
        return self.time_from_start.sec + self.time_from_start.nanosec * 1e-9


def _traj(times):
    return types.SimpleNamespace(joint_trajectory=types.SimpleNamespace(
        points=[FakePoint(t, [2.0], [4.0]) for t in times]))


@pytest.mark.parametrize("speed", [0.25, 0.5, 0.75])
def test_retime_stretches_time_and_scales_derivatives(speed):
    traj = _traj([0.0, 1.0, 2.5])
    BlendedPathExecutor.retime(traj, speed)
    pts = traj.joint_trajectory.points
    assert pts[-1].t == pytest.approx(2.5 / speed, rel=1e-6)
    # v scales with speed, a with speed^2
    assert pts[0].velocities[0] == pytest.approx(2.0 * speed)
    assert pts[0].accelerations[0] == pytest.approx(4.0 * speed * speed)


def test_retime_is_monotonic_and_preserves_ordering():
    traj = _traj([0.0, 0.4, 0.9, 1.7])
    BlendedPathExecutor.retime(traj, 0.3)
    ts = [p.t for p in traj.joint_trajectory.points]
    assert all(b > a for a, b in zip(ts, ts[1:]))


@pytest.mark.parametrize("total,speed", [
    (0.2999999999, 0.3),      # rounds to exactly 1.0 s
    (0.1, 0.1), (0.5, 0.5), (0.0999999999, 0.1),
])
def test_retime_never_emits_nanosec_at_or_above_one_second(total, speed):
    """builtin_interfaces requires nanosec < 1e9; rounding can land on 1e9."""
    traj = _traj([0.0, total])
    expected_total = traj.joint_trajectory.points[-1].t
    BlendedPathExecutor.retime(traj, speed)
    for p in traj.joint_trajectory.points:
        assert 0 <= p.time_from_start.nanosec < 1_000_000_000, p.time_from_start
        assert p.time_from_start.sec >= 0
    last = traj.joint_trajectory.points[-1]
    # compare against the timestamp the fixture actually stored, since building
    # it already quantised to whole nanoseconds
    assert last.t == pytest.approx(expected_total / speed, abs=2e-9)


def test_retime_carry_normalises_to_whole_seconds():
    traj = _traj([0.0, 0.2999999999])
    BlendedPathExecutor.retime(traj, 0.3)
    last = traj.joint_trajectory.points[-1].time_from_start
    assert (last.sec, last.nanosec) == (1, 0)


# --- planning guards ---------------------------------------------------------

class FakePlanResult:
    def __init__(self, fraction, err=1):
        self.fraction = fraction
        self.solution = _traj([0.0, 1.0])
        self.error_code = types.SimpleNamespace(val=err)


def _planner(result, min_fraction=0.99):
    node = FakeNode()
    ex = BlendedPathExecutor.__new__(BlendedPathExecutor)
    ex._node = node
    ex._timeout = 0.01
    ex._max_step = 0.005
    ex._min_fraction = min_fraction
    sent = []
    ex._plan_cli = types.SimpleNamespace(call_async=lambda req: FakeFuture(result))
    ex._exec_cli = types.SimpleNamespace(
        send_goal_async=lambda goal: sent.append(goal) or FakeFuture(HangingGoalHandle()))
    return ex, sent


@pytest.mark.parametrize("fraction", [0.0, 0.5, 0.9, 0.98, 0.995])
def test_partial_cartesian_path_is_rejected(fraction):
    """A path that is 99.5% complete still misses part of the shape."""
    ex, sent = _planner(FakePlanResult(fraction), min_fraction=0.999)
    with pytest.raises(RuntimeError, match="only"):
        ex.plan(poses=[object()], description="s:blended")
    assert sent == [], "a truncated plan must never reach the controller"


def test_missing_plan_response_is_rejected():
    ex, sent = _planner(None)
    with pytest.raises(RuntimeError, match="timed out"):
        ex.plan(poses=[object()], description="s:blended")
    assert sent == []


def test_complete_path_is_accepted():
    ex, _ = _planner(FakePlanResult(1.0))
    traj, fraction = ex.plan(poses=[object()], description="s:blended")
    assert fraction == 1.0 and traj is not None


def test_full_speed_retime_is_a_no_op():
    traj = _traj([0.0, 1.0, 2.0])
    BlendedPathExecutor.retime(traj, 1.0)
    assert [p.t for p in traj.joint_trajectory.points] == [0.0, 1.0, 2.0]
