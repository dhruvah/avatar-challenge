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

    shapes_file = PathJoinSubstitution(
        [FindPackageShare("avatar_challenge"), "config", "shapes.json"]
    )

    shape_tracer_node = Node(
        package="avatar_challenge",
        executable="shape_tracer_node.py",
        name="shape_tracer_node",
        output="screen",
        parameters=[
            {
                "shapes_file": shapes_file,
                "lift_height": 0.03,
                "arc_segments": 16,
                "closed": True,
                "service_timeout_sec": 120.0,
                # blend=True plans each shape as a single continuous Cartesian
                # trajectory; set False to fall back to stop-at-every-vertex
                # per-edge moves (exact corners, jerkier motion).
                "blend": True,
                "blend_max_step": 0.005,
                "blend_min_fraction": 0.99,
                "blend": True,
                "blend_max_step": 0.005,
                "blend_min_fraction": 0.99,
            }
        ],
    )

    return LaunchDescription(
        [
            xarm_moveit_launch,
            xarm_planner_launch,
            shape_tracer_node,
        ]
    )
