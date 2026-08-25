"""ROS2 node that drives the xArm7 (via xarm_planner's MoveIt-backed services)
to trace a list of 2D shapes in 3D space, and publishes the intended outline
to RViz for visual comparison against the arm's actual path.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Pose, Point, PoseStamped
from moveit_msgs.srv import GetPositionIK
from visualization_msgs.msg import Marker, MarkerArray
from xarm_msgs.srv import PlanPose, PlanSingleStraight, PlanExec

from avatar_challenge.geometry import build_shape_waypoints, Waypoint
from avatar_challenge.shapes_io import load_shapes, ShapeDef


def _to_pose_msg(position, quaternion) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        float(v) for v in quaternion
    )
    return pose


class ShapeTracerNode(Node):
    def __init__(self):
        super().__init__("shape_tracer_node")

        self.declare_parameter("shapes_file", "")
        self.declare_parameter("lift_height", 0.03)
        self.declare_parameter("arc_segments", 16)
        self.declare_parameter("closed", True)
        self.declare_parameter("service_timeout_sec", 20.0)

        self.lift_height = self.get_parameter("lift_height").value
        self.arc_segments = self.get_parameter("arc_segments").value
        self.default_closed = self.get_parameter("closed").value
        self.service_timeout = self.get_parameter("service_timeout_sec").value

        shapes_file = self.get_parameter("shapes_file").value
        if not shapes_file:
            self.get_logger().fatal("Required parameter 'shapes_file' was not set")
            raise SystemExit(1)

        self.pose_plan_cli = self.create_client(PlanPose, "xarm_pose_plan")
        self.straight_plan_cli = self.create_client(PlanSingleStraight, "xarm_straight_plan")
        self.exec_cli = self.create_client(PlanExec, "xarm_exec_plan")
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        # Latched: RViz usually subscribes after the node has already published.
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "shape_tracer/target_shapes",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        for name, cli in [
            ("xarm_pose_plan", self.pose_plan_cli),
            ("xarm_straight_plan", self.straight_plan_cli),
            ("xarm_exec_plan", self.exec_cli),
            ("/compute_ik", self.ik_cli),
        ]:
            self.get_logger().info(f"Waiting for service '{name}'...")
            if not cli.wait_for_service(timeout_sec=self.service_timeout):
                self.get_logger().fatal(f"Service '{name}' did not become available")
                raise SystemExit(1)

        self.shapes = load_shapes(shapes_file, default_closed=self.default_closed)
        self.get_logger().info(f"Loaded {len(self.shapes)} shape(s) from {shapes_file}")

    # -- service call helpers -------------------------------------------------

    def _call(self, client, request, description):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout)
        if future.result() is None:
            raise RuntimeError(f"{description}: service call timed out / failed")
        if not future.result().success:
            raise RuntimeError(f"{description}: planner reported failure")
        return future.result()

    def _plan_and_exec_free(self, position, quaternion, description):
        req = PlanPose.Request()
        req.target = _to_pose_msg(position, quaternion)
        self._call(self.pose_plan_cli, req, f"pose_plan({description})")
        self._call(self.exec_cli, PlanExec.Request(wait=True), f"exec_plan({description})")

    def _plan_and_exec_straight(self, position, quaternion, description):
        req = PlanSingleStraight.Request()
        req.target = _to_pose_msg(position, quaternion)
        self._call(self.straight_plan_cli, req, f"straight_plan({description})")
        self._call(self.exec_cli, PlanExec.Request(wait=True), f"exec_plan({description})")

    # -- preflight reachability -------------------------------------------------

    def _is_reachable(self, waypoint: Waypoint) -> bool:
        req = GetPositionIK.Request()
        req.ik_request.group_name = "xarm7"
        req.ik_request.ik_link_name = "link_eef"
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1
        stamped = PoseStamped()
        stamped.header.frame_id = "world"
        stamped.pose = _to_pose_msg(waypoint.position, waypoint.quaternion)
        req.ik_request.pose_stamped = stamped

        future = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout)
        result = future.result()
        return result is not None and result.error_code.val == 1

    def check_reachable(self, shape: ShapeDef, waypoints):
        """Reject a shape whose waypoints have no IK solution, before moving.

        The planner's own failure surfaces only once the arm is already
        mid-shape, which leaves it parked in a half-drawn pose. Checking every
        waypoint up front turns that into an actionable error naming the exact
        points that are out of the workspace.
        """
        unreachable = [i for i, wp in enumerate(waypoints) if not self._is_reachable(wp)]
        if unreachable:
            shown = ", ".join(
                f"wp{i} at ({waypoints[i].position[0]:.3f}, "
                f"{waypoints[i].position[1]:.3f}, {waypoints[i].position[2]:.3f})"
                for i in unreachable[:5]
            )
            raise RuntimeError(
                f"[{shape.name}] {len(unreachable)}/{len(waypoints)} waypoints have no "
                f"collision-free IK solution and are outside the xArm7 workspace: {shown}"
                + (" ..." if len(unreachable) > 5 else "")
            )
        self.get_logger().info(f"[{shape.name}] all {len(waypoints)} waypoints reachable")

    # -- shape execution --------------------------------------------------------

    def trace_shape(self, shape: ShapeDef):
        waypoints = build_shape_waypoints(
            vertices=shape.vertices,
            position=shape.position,
            rpy=shape.rpy,
            closed=shape.closed,
            lift_height=self.lift_height,
            arc_segments=self.arc_segments,
        )
        self.check_reachable(shape, waypoints)
        self.get_logger().info(f"[{shape.name}] tracing {len(waypoints)} waypoints")

        # First waypoint is always the pen-up "hover" pose above the start
        # vertex; approach it with a free-space plan since the arm may be
        # coming from anywhere (e.g. the previous shape's hover point).
        first = waypoints[0]
        self._plan_and_exec_free(first.position, first.quaternion, f"{shape.name}:approach")

        for i, wp in enumerate(waypoints[1:], start=1):
            desc = f"{shape.name}:wp{i}" + (" (hover)" if wp.is_travel else "")
            self._plan_and_exec_straight(wp.position, wp.quaternion, desc)

    def trace_all(self):
        self.publish_markers()
        for shape in self.shapes:
            self.trace_shape(shape)
        self.get_logger().info("All shapes traced.")

    # -- RViz visualization (bonus) ----------------------------------------------

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, shape in enumerate(self.shapes):
            waypoints = build_shape_waypoints(
                vertices=shape.vertices,
                position=shape.position,
                rpy=shape.rpy,
                closed=shape.closed,
                lift_height=0.0,  # outline only, no hover points
                arc_segments=self.arc_segments,
            )
            marker = Marker()
            marker.header.frame_id = "link_base"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "shape_tracer_targets"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.003
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.1, 0.9, 0.2, 1.0)
            marker.pose.orientation.w = 1.0
            for wp in waypoints:
                if wp.is_travel:
                    continue
                pt = Point()
                pt.x, pt.y, pt.z = (float(v) for v in wp.position)
                marker.points.append(pt)
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = ShapeTracerNode()
        # Give RViz a moment to subscribe before the first (latched-less) publish.
        time.sleep(1.0)
        node.trace_all()
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001 - surface any failure clearly and exit non-zero
        if node is not None:
            node.get_logger().error(f"shape_tracer_node failed: {exc}")
        else:
            print(f"shape_tracer_node failed during startup: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
