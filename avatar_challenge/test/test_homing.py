"""Homing retries must distinguish "it said no" from "it never answered".

A rejection means nothing was started, so retrying is safe and is what gets past
a controller that is still activating. A timeout means the arm's state is
unknown -- the request may have been received and acted on -- so re-commanding
it is the one thing that must not happen automatically.
"""

import types

import pytest

from avatar_challenge.shape_tracer_node import (  # noqa: E402
    PlannerRejected, ServiceTimeout, ShapeTracerNode)


class FakeLogger:
    def __init__(self):
        self.warns = []

    def warn(self, m):
        self.warns.append(m)

    def info(self, m):
        pass

    def error(self, m):
        pass


def make_node(call_results):
    """A node with _call stubbed to yield the given outcomes in order."""
    node = ShapeTracerNode.__new__(ShapeTracerNode)
    node.home_joints = [0.0] * 7
    node._log = FakeLogger()
    node.get_logger = lambda: node._log
    node.joint_plan_cli = object()
    node.exec_cli = object()
    node.calls = []

    outcomes = list(call_results)

    def fake_call(client, request, description):
        node.calls.append(description)
        outcome = outcomes.pop(0) if outcomes else None
        if isinstance(outcome, Exception):
            raise outcome
        return types.SimpleNamespace(success=True)

    node._call = fake_call
    return node


def test_a_rejection_is_retried(monkeypatch):
    monkeypatch.setattr("avatar_challenge.shape_tracer_node.time.sleep", lambda s: None)
    # plan ok, exec rejected, then plan ok, exec ok
    node = make_node([None, PlannerRejected("exec_plan(home): planner reported failure"),
                      None, None])
    node.go_home(attempts=3, delay=0)
    assert len(node.calls) == 4, node.calls
    assert node.get_logger().warns, "a retried rejection should be reported"


def test_a_timeout_is_not_retried(monkeypatch):
    monkeypatch.setattr("avatar_challenge.shape_tracer_node.time.sleep", lambda s: None)
    node = make_node([ServiceTimeout("exec_plan(home): service call timed out")])

    with pytest.raises(ServiceTimeout):
        node.go_home(attempts=4, delay=0)

    # exactly one attempt: the arm may already be moving, so it must not be
    # commanded again on its own
    assert len(node.calls) == 1, node.calls
    assert node.get_logger().warns == []


def test_a_rejection_that_never_clears_eventually_raises(monkeypatch):
    monkeypatch.setattr("avatar_challenge.shape_tracer_node.time.sleep", lambda s: None)
    node = make_node([PlannerRejected("no")] * 6)

    with pytest.raises(PlannerRejected):
        node.go_home(attempts=3, delay=0)
    assert len(node.calls) == 3
