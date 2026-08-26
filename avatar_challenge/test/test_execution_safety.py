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
    for name, attrs in [("moveit_msgs.action", ["ExecuteTrajectory"]),
                        ("moveit_msgs.srv", ["GetCartesianPath"])]:
        mod = types.ModuleType(name)
        for a in attrs:
            setattr(mod, a, type(a, (), {"Goal": type("Goal", (), {}),
                                         "Request": type("Request", (), {})}))
        sys.modules[name] = mod
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


def test_full_speed_retime_is_a_no_op():
    traj = _traj([0.0, 1.0, 2.0])
    BlendedPathExecutor.retime(traj, 1.0)
    assert [p.t for p in traj.joint_trajectory.points] == [0.0, 1.0, 2.0]
