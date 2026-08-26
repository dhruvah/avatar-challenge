"""Let the tests import the package without a ROS installation.

`geometry.py`, `shapes_io.py` and `kinematics.py` are plain numpy, but
`blended_path.py` and `designer_server.py` import ROS message types at module
level. Under `colcon test` those are real; run directly with pytest they are
not, so stand-ins are installed only when the real ones cannot be imported.

Each test module previously carried its own copy of this, gated on whether
`rclpy` was already in `sys.modules` -- which made the result depend on import
order and broke when the whole directory was collected at once.
"""

import importlib
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def _missing(name):
    try:
        importlib.import_module(name)
        return False
    except Exception:
        return True


def _stub_rclpy():
    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: True
    rclpy.spin_until_future_complete = lambda node, future, timeout_sec=None: None
    sys.modules["rclpy"] = rclpy

    action = types.ModuleType("rclpy.action")
    action.ActionClient = object
    sys.modules["rclpy.action"] = action

    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kw: types.SimpleNamespace(**kw)
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    sys.modules["rclpy.qos"] = qos


def _stub_moveit():
    class CartesianRequest:
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
                is_diff=False,
                joint_state=types.SimpleNamespace(name=[], position=[]))

    class Goal:
        def __init__(self):
            self.trajectory = None

    sys.modules.setdefault("moveit_msgs", types.ModuleType("moveit_msgs"))
    action = types.ModuleType("moveit_msgs.action")
    action.ExecuteTrajectory = type("ExecuteTrajectory", (), {"Goal": Goal})
    sys.modules["moveit_msgs.action"] = action
    srv = types.ModuleType("moveit_msgs.srv")
    srv.GetCartesianPath = type("GetCartesianPath", (), {"Request": CartesianRequest})
    sys.modules["moveit_msgs.srv"] = srv


def _stub_msgs():
    for pkg, cls in (("sensor_msgs", "JointState"), ("std_msgs", "String")):
        base = types.ModuleType(pkg)
        msg = types.ModuleType(f"{pkg}.msg")
        setattr(msg, cls, type(cls, (), {}))
        base.msg = msg
        sys.modules[pkg] = base
        sys.modules[f"{pkg}.msg"] = msg


if _missing("rclpy"):
    _stub_rclpy()
if _missing("moveit_msgs.srv"):
    _stub_moveit()
if _missing("sensor_msgs.msg"):
    _stub_msgs()
