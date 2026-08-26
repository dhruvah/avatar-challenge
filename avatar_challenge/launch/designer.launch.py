from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("xarm_moveit_config"),
                    "launch",
                    "xarm7_moveit_fake.launch.py",
                ]
            )
        ),
    )

    # xarm_planner/launch/_robot_planner.launch.py starts xarm_planner_node,
    # which wraps MoveGroupInterface behind simple ROS2 services
    # (xarm_pose_plan / xarm_straight_plan / xarm_exec_plan) that our
    # shape_tracer_node calls to plan and execute each shape's edges.
    xarm_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("xarm_planner"), "launch", "_robot_planner.launch.py"]
            )
        ),
        launch_arguments={
            "dof": "7",
            "robot_type": "xarm",
            "hw_ns": "xarm",
            "add_gripper": "false",
        }.items(),
    )

    # The designer replaces the one-shot tracer; everything below it in
    # start.launch.py (MoveIt, RViz, xarm_planner) is identical.
    designer_node = Node(
        package="avatar_challenge",
        executable="designer_server_node.py",
        name="shape_tracer_node",
        output="screen",
        parameters=[
            {
                "shapes_file": PathJoinSubstitution(
                    [FindPackageShare("avatar_challenge"), "config", "shapes.json"]
                ),
                "lift_height": 0.03,
                "arc_segments": 16,
                "closed": True,
                "service_timeout_sec": 120.0,
                "blend": True,
                "blend_max_step": 0.005,
                "port": 8080,
                # Loopback only; this moves a robot.
                "bind_address": "127.0.0.1",
            }
        ],
    )

    return LaunchDescription(
        [
            xarm_moveit_launch,
            xarm_planner_launch,
            designer_node,
        ]
    )
