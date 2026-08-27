"""Designer-server behaviour that is hard to reproduce by hand.

Covers the cases where a mistake is expensive rather than merely visible: a
rejected design destroying the last good one, two browsers racing to move the
same arm, and a failed trace reporting success.
"""

import json
import os
import threading
import types

import pytest

from avatar_challenge.designer_server import (              # noqa: E402
    DesignerServer, FAILED, SUCCEEDED, TRACING)
from avatar_challenge.shapes_io import load_shapes           # noqa: E402


GOOD = {"shapes": [{
    "name": "sq",
    "vertices": [[0, 0], [0, 0.1], [0.1, 0.1], [0.1, 0]],
    "closed": True,
    "start_pose": {"position": [0.3, -0.05, 0.25], "rpy": [0, 0, 0]},
}]}


class FakeLogger:
    def __init__(self):
        self.msgs = []

    def info(self, m): self.msgs.append(("info", m))
    def warn(self, m): self.msgs.append(("warn", m))
    def error(self, m): self.msgs.append(("error", m))
    def debug(self, m): pass


class FakeNode:
    """Just enough node for the server to drive."""

    def __init__(self, fail_on=None):
        self._log = FakeLogger()
        self.traced = []
        self.fail_on = fail_on
        self.markers = 0
        self.cleared = 0
        self.homed = []
        self.home_on_start = True
        self.shapes = []

    def get_logger(self): return self._log
    def publish_markers(self): self.markers += 1
    def go_home(self): self.homed.append(len(self.traced))
    def clear_actual(self): self.cleared += 1
    def create_subscription(self, *a, **k): return None

    def trace_shape(self, shape):
        if self.fail_on and shape.name == self.fail_on:
            raise RuntimeError(f"[{shape.name}] execution aborted")
        self.traced.append(shape.name)


def make_server(tmp_path, node=None):
    node = node or FakeNode()
    srv = DesignerServer(node, page_path=str(tmp_path / "page.html"),
                         config_path=str(tmp_path / "cfg" / "shapes.json"))
    return srv, node


# --------------------------------------------------------------------------
# a rejected design must not destroy the last good one
# --------------------------------------------------------------------------

@pytest.mark.parametrize("bad", [
    {"shapes": [{"name": "b", "vertices": [[0.05, 0], [0.1, 0]],
                 "start_pose": {"position": [0.3, 0, 0.25]}}]},   # not at origin
    {"nope": 1},                                                  # no shapes key
])
def test_invalid_design_leaves_the_saved_file_untouched(tmp_path, bad):
    srv, _ = make_server(tmp_path)
    srv._run(GOOD)
    saved = open(srv.config_path).read()

    with pytest.raises(Exception):
        srv._run(bad)

    assert open(srv.config_path).read() == saved, "a rejected design overwrote the good one"
    assert load_shapes(srv.config_path)[0].name == "sq"


def test_no_staging_file_is_left_behind_after_a_rejection(tmp_path):
    srv, _ = make_server(tmp_path)
    srv._run(GOOD)
    with pytest.raises(Exception):
        srv._run({"shapes": []})
    leftovers = [f for f in os.listdir(os.path.dirname(srv.config_path))
                 if f.endswith(".staged")]
    assert leftovers == []


def test_every_submission_starts_from_the_ready_pose(tmp_path):
    """Otherwise the second trace begins wherever the first one stopped, and the
    Cartesian solver seeds its IK from that arbitrary configuration."""
    srv, node = make_server(tmp_path)
    srv._run(GOOD)
    srv._run(GOOD)
    assert node.homed == [0, 1], "homing did not happen once before each trace"
    assert node.traced == ["sq", "sq"]


def test_homing_is_not_repeated_between_shapes_in_one_request(tmp_path):
    two = {"shapes": [dict(GOOD["shapes"][0], name="a"),
                      dict(GOOD["shapes"][0], name="b")]}
    srv, node = make_server(tmp_path)
    srv._run(two)
    assert len(node.homed) == 1, "homing between shapes was measured as worse"


def test_a_valid_design_does_replace_the_file(tmp_path):
    srv, node = make_server(tmp_path)
    srv._run(GOOD)
    second = json.loads(json.dumps(GOOD))
    second["shapes"][0]["name"] = "renamed"
    srv._run(second)
    assert load_shapes(srv.config_path)[0].name == "renamed"
    assert node.traced == ["sq", "renamed"]


# --------------------------------------------------------------------------
# a failing trace must report failure, and must not continue
# --------------------------------------------------------------------------

def test_failure_part_way_through_stops_and_does_not_report_success(tmp_path):
    two = {"shapes": [
        dict(GOOD["shapes"][0], name="first"),
        dict(GOOD["shapes"][0], name="second"),
    ]}
    node = FakeNode(fail_on="second")
    srv, _ = make_server(tmp_path, node)
    with pytest.raises(RuntimeError, match="aborted"):
        srv._run(two)
    assert node.traced == ["first"], "the run continued past a failure"


@pytest.mark.parametrize("name", [5, ""])
def test_invalid_names_are_rejected_before_the_arm_moves(tmp_path, name):
    node = FakeNode()
    srv, _ = make_server(tmp_path, node)
    with pytest.raises(ValueError, match="name"):
        srv._run({"shapes": [dict(GOOD["shapes"][0], name=name)]})
    assert node.traced == []


# --------------------------------------------------------------------------
# only the drawn outline is recorded, not the travel to it
# --------------------------------------------------------------------------

class _FakeChain:
    """FK that simply returns whatever world point the test asks for."""

    def __init__(self, point):
        self.actuated = [types.SimpleNamespace(name="j1")]
        self.point = point

    def fk(self, q):
        import numpy as np
        T = np.eye(4)
        T[:3, 3] = self.point
        return T


def _joint_msg():
    return types.SimpleNamespace(name=["j1"], position=[0.0])


@pytest.mark.parametrize("height,recorded", [
    (0.0, True),        # on the plane: drawing
    (0.03, False),      # hover height: travelling
    (0.20, False),      # mid-approach, far above
])
def test_only_on_plane_samples_are_recorded(tmp_path, height, recorded):
    import numpy as np
    srv, _ = make_server(tmp_path)
    srv._frame = types.SimpleNamespace(
        rotation=np.eye(3), position=np.array([0.0, 0.0, 0.0]))
    srv._recording = True
    srv._chain = _FakeChain([0.05, 0.02, height])
    srv._on_joints(_joint_msg())
    assert (len(srv._path) == 1) is recorded


def test_recorded_points_are_the_in_plane_coordinates(tmp_path):
    import numpy as np
    srv, _ = make_server(tmp_path)
    srv._frame = types.SimpleNamespace(
        rotation=np.eye(3), position=np.array([0.30, -0.05, 0.25]))
    srv._recording = True
    srv._chain = _FakeChain([0.35, 0.01, 0.25])
    srv._on_joints(_joint_msg())
    assert srv._path == [[50.0, 60.0]]


def _drain(srv):
    """Run one queued job the way spin() does, without a ROS loop."""
    job = srv.jobs.get_nowait()
    try:
        if job.abandoned:
            return job
        try:
            job.result = srv._run(job.payload)
        except Exception as exc:                            # noqa: BLE001
            srv._state, srv._error = FAILED, str(exc)
            job.result = {"ok": False, "error": str(exc)}
    finally:
        with srv._lock:
            srv._reserved = False
        job.done.set()
    return job


# --------------------------------------------------------------------------
# only one trace at a time
# --------------------------------------------------------------------------

def test_only_one_request_can_reserve_the_robot(tmp_path):
    """Two browsers pressing Send at once must not both drive the arm."""
    srv, _ = make_server(tmp_path)
    first = srv.reserve(GOOD)
    second = srv.reserve(GOOD)
    assert first is not None
    assert second is None, "a second request was accepted while one was pending"
    assert srv.is_busy()


def test_concurrent_reservations_admit_exactly_one(tmp_path):
    """Race many threads at the same instant through a barrier."""
    srv, _ = make_server(tmp_path)
    n = 16
    barrier = threading.Barrier(n)
    granted = []
    lock = threading.Lock()

    def attempt():
        barrier.wait(timeout=5)
        job = srv.reserve(GOOD)
        if job is not None:
            with lock:
                granted.append(job)

    threads = [threading.Thread(target=attempt) for _ in range(n)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=5)

    assert len(granted) == 1, f"{len(granted)} of {n} requests were admitted"
    assert srv.jobs.qsize() == 1


def test_reservation_is_released_after_the_worker_runs(tmp_path):
    srv, node = make_server(tmp_path)
    job = srv.reserve(GOOD)
    assert srv.is_busy()
    _drain(srv)
    assert not srv.is_busy()
    assert job.done.is_set()
    assert node.traced == ["sq"]

    # and the robot can be claimed again
    assert srv.reserve(GOOD) is not None


def test_abandoned_job_never_moves_the_robot(tmp_path):
    """A request whose client gave up waiting must not execute later."""
    srv, node = make_server(tmp_path)
    job = srv.reserve(GOOD)
    srv.abandon(job)
    _drain(srv)
    assert node.traced == [], "a timed-out request still drove the arm"
    assert not srv.is_busy()


# --------------------------------------------------------------------------
# progress must not claim success after a failure
# --------------------------------------------------------------------------

def test_progress_reports_failure_not_one_hundred_percent(tmp_path):
    node = FakeNode(fail_on="sq")
    srv, _ = make_server(tmp_path, node)
    srv.reserve(GOOD)
    _drain(srv)
    snap = srv.progress_snapshot()
    assert snap["state"] == FAILED
    assert snap["fraction"] < 1.0, "a failed trace reported 100%"
    assert snap["error"] and "aborted" in snap["error"]
    assert snap["tracing"] is False


def test_progress_reaches_one_hundred_percent_only_on_success(tmp_path):
    srv, _ = make_server(tmp_path)
    assert srv.progress_snapshot()["state"] == "idle"
    assert srv.progress_snapshot()["fraction"] == 0.0
    srv.reserve(GOOD)
    _drain(srv)
    snap = srv.progress_snapshot()
    assert snap["state"] == SUCCEEDED
    assert snap["fraction"] == 1.0


def test_failure_state_survives_for_the_ui_to_read(tmp_path):
    node = FakeNode(fail_on="sq")
    srv, _ = make_server(tmp_path, node)
    srv.reserve(GOOD)
    _drain(srv)
    for _ in range(3):
        assert srv.progress_snapshot()["state"] == FAILED
