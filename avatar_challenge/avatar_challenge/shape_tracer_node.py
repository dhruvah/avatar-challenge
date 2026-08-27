"""ROS2 node that drives the xArm7 (via xarm_planner's MoveIt-backed services)
to trace a list of 2D shapes in 3D space, and publishes the intended outline
to RViz for visual comparison against the arm's actual path.
"""

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Pose, Point, PoseStamped
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from xarm_msgs.srv import PlanPose, PlanSingleStraight, PlanExec, PlanJoint

from avatar_challenge.blended_path import BlendedPathExecutor
from avatar_challenge.geometry import build_shape_waypoints, Waypoint
from avatar_challenge.kinematics import Chain
from avatar_challenge.shapes_io import load_shapes, ShapeDef


# Below this Yoshikawa measure the arm is close enough to a singularity that
# small tool motions need large joint speeds. Calibrated from a workspace sweep
# -- see tools/workspace_quality.py and the README.
SINGULARITY_WARN = 0.010

# Most points the live-progress endpoint will return per poll.
PROGRESS_PATH_CAP = 400


class IKServiceError(RuntimeError):
    """/compute_ik failed to answer -- distinct from a pose being unreachable."""


class ServiceTimeout(RuntimeError):
    """A service did not answer. The arm's state is unknown: the request may
    have been received and acted on, so this must never be retried blindly."""


class PlannerRejected(RuntimeError):
    """A service answered and said no. Nothing was started, so retrying is safe."""


def _to_pose_msg(position, quaternion) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        float(v) for v in quaternion
    )
    return pose


class ShapeTracerNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__("shape_tracer_node", **node_kwargs)

        self.declare_parameter("shapes_file", "")
        self.declare_parameter("lift_height", 0.03)
        self.declare_parameter("arc_segments", 16)
        self.declare_parameter("closed", True)
        self.declare_parameter("service_timeout_sec", 120.0)
        self.declare_parameter("blend", True)
        self.declare_parameter("blend_max_step", 0.005)
        # A known, natural ready configuration: base straight ahead, elbow bent,
        # zero wrist roll. Starting from it makes the approach repeatable and
        # gives the Cartesian solver a sane IK seed.
        self.declare_parameter("home_joints", [0.0, -0.5706, 0.0, 0.5039, 0.0, 1.0745, 0.0])
        self.declare_parameter("home_on_start", True)
        # The target and actual-path markers are latched, which means RViz holds
        # them only while their publisher is alive. If the node exits as soon as
        # it finishes tracing, the outlines vanish from RViz at exactly the
        # moment there is something to look at.
        self.declare_parameter("hold_after_trace", True)

        self.lift_height = self.get_parameter("lift_height").value
        self.arc_segments = self.get_parameter("arc_segments").value
        self.default_closed = self.get_parameter("closed").value
        self.service_timeout = self.get_parameter("service_timeout_sec").value
        self.blend = self.get_parameter("blend").value
        self.home_joints = list(self.get_parameter("home_joints").value)
        self.home_on_start = self.get_parameter("home_on_start").value
        self.hold_after_trace = self.get_parameter("hold_after_trace").value

        shapes_file = self.get_parameter("shapes_file").value

        self.pose_plan_cli = self.create_client(PlanPose, "xarm_pose_plan")
        self.straight_plan_cli = self.create_client(PlanSingleStraight, "xarm_straight_plan")
        self.exec_cli = self.create_client(PlanExec, "xarm_exec_plan")
        self.joint_plan_cli = self.create_client(PlanJoint, "xarm_joint_plan")
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        # Latched: RViz usually subscribes after the node has already published.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(
            MarkerArray, "shape_tracer/target_shapes", latched)
        # RViz's MotionPlanning display animates whatever appears here.
        self.display_pub = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 1)
        # Where the tool actually went, from joint states through our own FK, so
        # target and actual can be compared in one view.
        self.actual_pub = self.create_publisher(
            MarkerArray, "shape_tracer/actual_path", latched)

        self._chain = None
        self._recording = False
        self._actual = []
        # Executed paths for every shape so far. The publisher is latched with
        # depth 1, so each message must carry the whole set -- otherwise an RViz
        # that connects late sees only the shape that happened to finish last.
        self._traced_paths = []
        self.create_subscription(String, "/robot_description", self._on_urdf, latched)

        for name, cli in [
            ("xarm_pose_plan", self.pose_plan_cli),
            ("xarm_straight_plan", self.straight_plan_cli),
            ("xarm_exec_plan", self.exec_cli),
            ("xarm_joint_plan", self.joint_plan_cli),
            ("/compute_ik", self.ik_cli),
        ]:
            self.get_logger().info(f"Waiting for service '{name}'...")
            if not cli.wait_for_service(timeout_sec=self.service_timeout):
                self.get_logger().fatal(f"Service '{name}' did not become available")
                raise SystemExit(1)

        # /compute_ik seeds from the state we hand it. Under `ros2 launch` this
        # node comes up alongside move_group, before the controllers publish any
        # joint state -- an empty seed makes IK fail and every waypoint look
        # unreachable. Block until a real joint state has arrived.
        self._joint_state = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._await_joint_state()

        self.blender = None
        if self.blend:
            self.blender = BlendedPathExecutor(
                self,
                max_step=self.get_parameter("blend_max_step").value,
                timeout=self.service_timeout,
            )
            if not self.blender.wait_for_servers():
                self.get_logger().fatal(
                    "blend=true but /compute_cartesian_path or /execute_trajectory "
                    "never became available"
                )
                raise SystemExit(1)

        if not shapes_file:
            self.get_logger().fatal("Required parameter 'shapes_file' was not set")
            raise SystemExit(1)
        self.shapes = load_shapes(shapes_file, default_closed=self.default_closed)
        self.get_logger().info(f"Loaded {len(self.shapes)} shape(s) from {shapes_file}")

        # Re-timing happens on the Cartesian trajectory, which only the blended
        # backend produces; xarm_planner's services expose no speed control.
        if not self.blend:
            slowed = [s.name for s in self.shapes if s.speed < 1.0]
            if slowed:
                self.get_logger().warn(
                    f"blend=false, so 'speed' is ignored for: {', '.join(slowed)}")

    # -- service call helpers -------------------------------------------------

    def _call(self, client, request, description):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout)
        if future.result() is None:
            raise ServiceTimeout(f"{description}: service call timed out")
        if not future.result().success:
            raise PlannerRejected(f"{description}: planner reported failure")
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

    def _approach(self, waypoint, shape_name):
        """Move to the hover pose above the first vertex.

        A straight line is tried first: it is deterministic and reads as a
        single clean move. The free-space fallback is planned by RRTConnect,
        which is sampling-based -- it finds a way around obstacles but wanders,
        and produces a different path on every run. So it is used only when the
        straight line genuinely cannot be planned.
        """
        try:
            self._plan_and_exec_straight(waypoint.position, waypoint.quaternion,
                                         f"{shape_name}:approach(straight)")
            return
        except RuntimeError as exc:
            self.get_logger().warn(
                f"[{shape_name}] straight-line approach unavailable ({exc}); "
                f"falling back to the sampling-based planner"
            )
        self._plan_and_exec_free(waypoint.position, waypoint.quaternion,
                                 f"{shape_name}:approach(free)")

    def go_home(self, attempts: int = 4, delay: float = 2.0):
        """Drive to the configured ready pose in joint space.

        The free-space approach is planned by RRTConnect, which is sampling-based:
        from an arbitrary start it produces a different, often meandering path
        every run. Starting each shape from the same known configuration makes
        that approach short, repeatable, and easy to watch. It also gives the
        Cartesian solver a sane seed instead of whatever contorted branch sits
        near the previous shape's end.

        Retried because this is the first motion after launch, and the joint
        trajectory controller can still be activating when the planner's
        services are already answering -- the execute is then rejected for a
        reason that resolves itself a second later.

        Only an outright rejection is retried. A timeout means the service never
        answered, so it is unknown whether the motion was started; re-commanding
        an arm that may already be moving is exactly the wrong response, and it
        is raised immediately instead.
        """
        last = None
        for attempt in range(1, attempts + 1):
            try:
                req = PlanJoint.Request()
                req.target = [float(v) for v in self.home_joints]
                self._call(self.joint_plan_cli, req, "joint_plan(home)")
                self._call(self.exec_cli, PlanExec.Request(wait=True), "exec_plan(home)")
                return
            except PlannerRejected as exc:
                last = exc
                if attempt < attempts:
                    self.get_logger().warn(
                        f"homing attempt {attempt}/{attempts} rejected ({exc}); "
                        f"the controller may still be starting -- retrying")
                    time.sleep(delay)
        raise last

    # -- robot state ------------------------------------------------------------

    def _on_urdf(self, msg: String):
        if self._chain is not None:
            return
        try:
            self._chain = Chain.from_urdf(msg.data)
            self.get_logger().info(
                f"Kinematic chain loaded ({len(self._chain.actuated)} actuated "
                f"joints); singularity reporting enabled"
            )
        except Exception as exc:  # noqa: BLE001 - overlay is optional, never fatal
            self.get_logger().warn(f"could not parse /robot_description: {exc}")

    def _on_joint_state(self, msg: JointState):
        if msg.name and msg.position:
            self._joint_state = msg
            if self._recording and self._chain is not None:
                idx = {n: i for i, n in enumerate(msg.name)}
                try:
                    q = [msg.position[idx[j.name]] for j in self._chain.actuated]
                except KeyError:
                    return
                self._actual.append(self._chain.fk(q)[:3, 3].copy())

    def _await_joint_state(self):
        self.get_logger().info("Waiting for /joint_states...")
        deadline = time.time() + self.service_timeout
        while self._joint_state is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._joint_state is None:
            self.get_logger().fatal(
                "No /joint_states received; cannot seed IK or verify reachability"
            )
            raise SystemExit(1)
        self.get_logger().info(
            f"Got /joint_states ({len(self._joint_state.name)} joints)"
        )

    # -- preflight reachability -------------------------------------------------

    def _is_reachable(self, waypoint: Waypoint) -> bool:
        """True if IK solved. Raises IKServiceError if IK never answered.

        A service that times out is a very different problem from a pose that is
        genuinely out of reach, and conflating them sends debugging in the wrong
        direction -- so an absent response raises rather than reporting the point
        as unreachable.
        """
        req = GetPositionIK.Request()
        req.ik_request.group_name = "xarm7"
        req.ik_request.ik_link_name = "link_eef"
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1
        req.ik_request.robot_state.joint_state = self._joint_state
        stamped = PoseStamped()
        stamped.header.frame_id = "world"
        stamped.pose = _to_pose_msg(waypoint.position, waypoint.quaternion)
        req.ik_request.pose_stamped = stamped

        future = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout)
        result = future.result()
        if result is None:
            raise IKServiceError(
                "/compute_ik did not respond within "
                f"{self.service_timeout}s -- is move_group running and healthy?"
            )
        return result.error_code.val == 1

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
        self._approach(first, shape.name)

        if self.blender is not None:
            # Everything after the approach -- descend, trace, lift -- goes out
            # as a single Cartesian path so the arm never stops at a vertex.
            poses = [_to_pose_msg(wp.position, wp.quaternion) for wp in waypoints[1:]]
            desc = f"{shape.name}:blended"
            trajectory, fraction = self.blender.plan(poses, desc)
            self.blender.retime(trajectory, shape.speed)
            self.report_trajectory(shape, trajectory, fraction, len(poses))
            self.publish_planned(trajectory)
            self._actual = []
            self._recording = True
            try:
                self.blender.execute(trajectory, desc)
            finally:
                self._recording = False
                self.publish_actual(shape)
            return

        for i, wp in enumerate(waypoints[1:], start=1):
            desc = f"{shape.name}:wp{i}" + (" (hover)" if wp.is_travel else "")
            self._plan_and_exec_straight(wp.position, wp.quaternion, desc)

    def trace_all(self):
        """Trace every loaded shape, stopping at the first failure.

        Recovery policy on failure: log exactly which shape failed and why, and
        do NOT start the next one. We deliberately do not command a "safe lift"
        afterwards -- if a trace aborted, the controller's actual state is
        unknown, and blindly commanding a motion from an unknown state is more
        dangerous than stopping. Recovery is a deliberate operator action:
        re-run, which begins by homing to a known configuration.
        """
        self.publish_markers()
        self.clear_actual()
        if self.home_on_start:
            self.get_logger().info("Moving to the ready pose before tracing")
            self.go_home()
        for index, shape in enumerate(self.shapes):
            try:
                self.trace_shape(shape)
            except Exception as exc:  # noqa: BLE001 - report, then stop
                remaining = len(self.shapes) - index - 1
                self.get_logger().error(
                    f"[{shape.name}] failed: {exc}. Stopping; {remaining} shape(s) "
                    f"not attempted. The arm is left where it stopped -- re-run to "
                    f"recover via the ready pose."
                )
                raise
        self.get_logger().info("All shapes traced.")

    def report_trajectory(self, shape, trajectory, fraction, n_poses):
        pts = trajectory.joint_trajectory.points
        dur = pts[-1].time_from_start
        seconds = dur.sec + dur.nanosec * 1e-9
        msg = (f"[{shape.name}] blended {n_poses} waypoints into one trajectory "
               f"({len(pts)} points, {seconds:.2f}s at {shape.speed*100:.0f}% speed, "
               f"{fraction*100:.1f}% of path)")
        if self._chain is not None:
            w = [self._chain.manipulability(p.positions) for p in pts]
            msg += f"; manipulability min={min(w):.4f} mean={sum(w)/len(w):.4f}"
            if min(w) < SINGULARITY_WARN:
                self.get_logger().warn(
                    f"[{shape.name}] passes close to a singularity "
                    f"(manipulability {min(w):.4f} < {SINGULARITY_WARN}); expect "
                    f"large joint speeds for small tool motion"
                )
        self.get_logger().info(msg)

    def publish_planned(self, trajectory):
        msg = DisplayTrajectory()
        msg.model_id = "UF_ROBOT"
        if self._joint_state is not None:
            msg.trajectory_start.joint_state = self._joint_state
        msg.trajectory.append(trajectory)
        self.display_pub.publish(msg)

    # -- RViz visualization (bonus) ----------------------------------------------

    def publish_actual(self, shape):
        """Publish the executed path for every shape traced so far.

        One marker id per shape, and the whole set in every message: the
        publisher is latched with depth 1, so a partial message would leave an
        RViz that connects late showing only the most recent shape.
        """
        if self._actual:
            index = self.shapes.index(shape) if shape in self.shapes else len(self._traced_paths)
            self._traced_paths.append((index, list(self._actual)))
        if not self._traced_paths:
            return

        arr = MarkerArray()
        arr.markers.append(self._delete_all("shape_tracer_actual"))
        for index, path in self._traced_paths:
            marker = Marker()
            marker.header.frame_id = "link_base"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "shape_tracer_actual"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.002
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                0.95, 0.45, 0.15, 1.0)
            marker.pose.orientation.w = 1.0
            for p in path:
                pt = Point()
                pt.x, pt.y, pt.z = (float(v) for v in p)
                marker.points.append(pt)
            arr.markers.append(marker)
        self.actual_pub.publish(arr)

    @staticmethod
    def _delete_all(namespace):
        marker = Marker()
        marker.header.frame_id = "link_base"
        marker.ns = namespace
        marker.action = Marker.DELETEALL
        return marker

    def clear_actual(self):
        """Drop executed-path outlines from a previous run."""
        self._traced_paths = []
        arr = MarkerArray()
        arr.markers.append(self._delete_all("shape_tracer_actual"))
        self.actual_pub.publish(arr)

    def publish_markers(self):
        marker_array = MarkerArray()
        # The publisher is latched, so a shorter design would otherwise leave
        # the previous run's extra outlines on screen forever. DELETEALL first,
        # in the same message, so a late subscriber still gets a clean set.
        marker_array.markers.append(self._delete_all("shape_tracer_targets"))
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
        node.trace_all()
        if node.hold_after_trace:
            node.get_logger().info(
                "Tracing complete. Holding so RViz keeps the outlines on screen "
                "-- press Ctrl-C to exit.")
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
