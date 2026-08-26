"""Single-trajectory (blended) execution via MoveIt's Cartesian path service.

The per-edge `xarm_planner` services plan and execute one straight segment at a
time, so the arm decelerates to a full stop at every vertex. Handing MoveIt the
*whole* waypoint list in one `/compute_cartesian_path` call instead yields a
single continuously time-parameterized `RobotTrajectory`, which the controller
tracks without stopping -- the corners get rounded by the interpolation and
time parameterization rather than by an explicit corner-blend geometry pass.
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath

PLANNING_GROUP = "xarm7"
EEF_LINK = "link_eef"
PLANNING_FRAME = "world"


class BlendedPathExecutor:
    """Plans and executes a list of waypoints as one continuous trajectory."""

    def __init__(self, node, max_step: float, min_fraction: float, timeout: float):
        self._node = node
        self._max_step = max_step
        self._min_fraction = min_fraction
        self._timeout = timeout
        self._plan_cli = node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._exec_cli = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")

    def wait_for_servers(self) -> bool:
        return self._plan_cli.wait_for_service(
            timeout_sec=self._timeout
        ) and self._exec_cli.wait_for_server(timeout_sec=self._timeout)

    def plan(self, poses, description: str):
        req = GetCartesianPath.Request()
        req.header.frame_id = PLANNING_FRAME
        req.group_name = PLANNING_GROUP
        req.link_name = EEF_LINK
        req.waypoints = list(poses)
        req.max_step = self._max_step
        # 0 disables the joint-space jump check; the shapes are small and fully
        # inside the workspace, and the check spuriously truncates paths that
        # pass near the wrist's redundant configurations.
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.start_state.is_diff = True

        future = self._plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self._timeout)
        result = future.result()
        if result is None:
            raise RuntimeError(f"{description}: /compute_cartesian_path timed out")
        if result.fraction < self._min_fraction:
            raise RuntimeError(
                f"{description}: Cartesian path only {result.fraction * 100:.1f}% "
                f"complete (need >= {self._min_fraction * 100:.0f}%)"
            )
        return result.solution, result.fraction

    @staticmethod
    def retime(trajectory, speed: float):
        """Uniformly slow a trajectory to `speed` x its planned rate, in place.

        GetCartesianPath has no velocity-scaling field in Humble, so scaling has
        to happen after planning. Stretching every timestamp by the same factor
        leaves the geometric path untouched and only changes how fast it is
        traversed; velocities scale with 1/k and accelerations with 1/k^2 to stay
        consistent with the new timing.
        """
        if speed >= 1.0:
            return trajectory
        k = 1.0 / speed
        for point in trajectory.joint_trajectory.points:
            total = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            scaled = total * k
            sec = int(scaled)
            nsec = int(round((scaled - sec) * 1e9))
            # rounding can land exactly on a full second; builtin_interfaces
            # requires nanosec < 1e9, so carry rather than emit an invalid stamp
            if nsec >= 1_000_000_000:
                sec += 1
                nsec -= 1_000_000_000
            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nsec
            point.velocities = [v / k for v in point.velocities]
            point.accelerations = [a / (k * k) for a in point.accelerations]
        return trajectory

    def execute(self, trajectory, description: str):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = self._exec_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=self._timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{description}: /execute_trajectory rejected the goal")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=self._timeout)
        outcome = result_future.result()
        if outcome is None:
            # Letting this node exit does NOT stop the controller -- the arm would
            # keep tracing after the process is gone. Ask the server to cancel and
            # give it a bounded window to acknowledge before giving up.
            self._node.get_logger().error(
                f"{description}: execution result timed out; cancelling the goal"
            )
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self._node, cancel_future, timeout_sec=5.0)
            if cancel_future.result() is None:
                raise RuntimeError(
                    f"{description}: execution timed out AND the cancel request was "
                    f"not acknowledged -- the arm may still be moving"
                )
            raise RuntimeError(f"{description}: trajectory execution timed out (goal cancelled)")
        if outcome.result.error_code.val != 1:
            raise RuntimeError(
                f"{description}: trajectory execution failed "
                f"(MoveItErrorCode {outcome.result.error_code.val})"
            )

    def plan_and_execute(self, poses, description: str, speed: float = 1.0):
        trajectory, fraction = self.plan(poses, description)
        self.retime(trajectory, speed)
        points = len(trajectory.joint_trajectory.points)
        duration = trajectory.joint_trajectory.points[-1].time_from_start
        self._node.get_logger().info(
            f"{description}: blended {len(poses)} waypoints into one trajectory "
            f"({points} points, {duration.sec + duration.nanosec * 1e-9:.2f}s "
            f"at {speed * 100:.0f}% speed, {fraction * 100:.1f}% of path)"
        )
        self.execute(trajectory, description)
