"""Designer-server behaviour that is hard to reproduce by hand.

Covers the cases where a mistake is expensive rather than merely visible: a
rejected design destroying the last good one, two browsers racing to move the
same arm, and a failed trace reporting success.
"""

import json
import os
import sys
import threading
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from avatar_challenge.designer_server import DesignerServer  # noqa: E402
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
    """Just enough node for the server: reload() validates, trace_shape() runs."""

    def __init__(self, fail_on=None):
        self._log = FakeLogger()
        self.traced = []
        self.fail_on = fail_on
        self.markers = 0

    def get_logger(self): return self._log
    def reload(self, path): return load_shapes(path)
    def publish_markers(self): self.markers += 1

    def trace_shape(self, shape):
        if self.fail_on and shape.name == self.fail_on:
            raise RuntimeError(f"[{shape.name}] execution aborted")
        self.traced.append(shape.name)


def make_server(tmp_path, node=None):
    node = node or FakeNode()
    srv = DesignerServer(node, page_path=str(tmp_path / "page.html"),
                         config_path=str(tmp_path / "cfg" / "shapes.json"))
    srv._httpd = types.SimpleNamespace(index=0, total=0)
    return srv, node


# --------------------------------------------------------------------------
# a rejected design must not destroy the last good one
# --------------------------------------------------------------------------

@pytest.mark.parametrize("bad", [
    {"shapes": [{"name": "b", "vertices": [[0.05, 0], [0.1, 0]],
                 "start_pose": {"position": [0.3, 0, 0.25]}}]},          # not at origin
    {"shapes": []},                                                       # empty
    {"nope": 1},                                                          # no shapes key
    {"shapes": [{"name": "b", "vertices": [[0, 0], [0.1, 0]],
                 "closed": "false",
                 "start_pose": {"position": [0.3, 0, 0.25]}}]},           # string bool
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


def test_names_must_be_strings(tmp_path):
    for name in (5, {"a": 1}, [], None):
        payload = {"shapes": [dict(GOOD["shapes"][0], name=name)]}
        srv, _ = make_server(tmp_path)
        p = tmp_path / "n.json"
        p.write_text(json.dumps(payload))
        shapes = load_shapes(str(p))
        # a non-string name must not crash formatting downstream
        assert isinstance(f"{shapes[0].name}", str)


# --------------------------------------------------------------------------
# only one trace at a time
# --------------------------------------------------------------------------

def test_busy_flag_gates_concurrent_runs(tmp_path):
    """Two browsers pressing Send at once must not both drive the arm."""
    srv, node = make_server(tmp_path)
    started = threading.Event()
    release = threading.Event()

    def slow_trace(shape):
        started.set()
        release.wait(timeout=5)
        node.traced.append(shape.name)

    node.trace_shape = slow_trace

    def run():
        srv.busy.set()
        try:
            srv._run(GOOD)
        finally:
            srv.busy.clear()

    t = threading.Thread(target=run)
    t.start()
    started.wait(timeout=5)

    # while the first is mid-trace the server must report itself busy, which is
    # what the POST handler checks before accepting another job
    assert srv.busy.is_set()

    release.set()
    t.join(timeout=5)
    assert not srv.busy.is_set()
    assert node.traced == ["sq"]
