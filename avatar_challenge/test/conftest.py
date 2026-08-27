"""Let the tests import the package without a ROS installation.

`geometry.py`, `shapes_io.py` and `kinematics.py` are plain numpy, but
`blended_path.py` and `designer_server.py` import ROS message types at module
level.

The stand-ins below are installed *unconditionally*, not only when ROS is
absent. These are unit tests of pure logic -- re-timing arithmetic, goal
cancellation, request reservation -- and they should behave identically whether
or not a ROS installation happens to be on the path. Stubbing only when the real
packages are missing made the suite pass under plain pytest and fail under
`colcon test`, where the real message types reject the plain objects the tests
pass as waypoints.
"""

import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def _stub_rclpy():
    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: True
    rclpy.spin_until_future_complete = lambda node, future, timeout_sec=None: None
    sys.modules["rclpy"] = rclpy

    action = types.ModuleType("rclpy.action")
    action.ActionClient = object
    sys.modules["rclpy.action"] = action

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = type("Node", (), {"__init__": lambda self, *a, **k: None})
    sys.modules["rclpy.node"] = node_mod
    rclpy.node = node_mod

    param = types.ModuleType("rclpy.parameter")
    param.Parameter = type("Parameter", (), {"Type": types.SimpleNamespace(STRING="string")})
    sys.modules["rclpy.parameter"] = param

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


class _Msg:
    """Stand-in message: accepts whatever fields the caller sets or passes."""

    def __init__(self, **fields):
        self.__dict__.update(fields)


def _stub_msgs():
    """Message types the node imports at module level. They are only used as
    data holders here, so an empty class is enough."""
    spec = {
        "sensor_msgs.msg": ["JointState"],
        "std_msgs.msg": ["String"],
        "geometry_msgs.msg": ["Pose", "Point", "PoseStamped"],
        "visualization_msgs.msg": ["Marker", "MarkerArray"],
        "xarm_msgs.srv": ["PlanPose", "PlanSingleStraight", "PlanExec", "PlanJoint"],
    }
    for path, names in spec.items():
        pkg, sub = path.split(".")
        base = sys.modules.setdefault(pkg, types.ModuleType(pkg))
        mod = types.ModuleType(path)
        for n in names:
            setattr(mod, n, type(n, (), {"Request": _Msg}))
        setattr(base, sub, mod)
        sys.modules[path] = mod


def _stub_moveit_msgs_extra():
    msg = types.ModuleType("moveit_msgs.msg")
    msg.DisplayTrajectory = type("DisplayTrajectory", (), {})
    sys.modules["moveit_msgs.msg"] = msg
    srv = sys.modules["moveit_msgs.srv"]
    srv.GetPositionIK = type("GetPositionIK", (), {"Request": _Msg})


_stub_rclpy()
_stub_moveit()
_stub_msgs()
_stub_moveit_msgs_extra()
