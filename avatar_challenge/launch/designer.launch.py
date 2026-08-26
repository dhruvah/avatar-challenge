from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def moveit_parameters(context):
    """The parameters RViz needs to show MoveIt's planned-path displays.

    RViz's MotionPlanning plugin loads the robot model itself, so it needs the
    semantic description and kinematics as its own parameters -- without them it
    logs "Unable to parse SRDF" and the planned trajectory never appears. The
    stock MoveIt launch passes these to the RViz it starts; we start our own, so
    we have to build the same configuration here.
    """
    config = MoveItConfigsBuilder(
        context=context,
        dof=7,
        robot_type="xarm",
        prefix="",
        hw_ns="xarm",
        limited=True,
        attach_to="world",
        attach_xyz='"0 0 0"',
        attach_rpy='"0 0 0"',
    ).to_moveit_configs().to_dict()
    return {
        key: config[key]
        for key in (
            "robot_description",
            "robot_description_semantic",
            "robot_description_kinematics",
            "robot_description_planning",
            "planning_pipelines",
        )
        if key in config
    }


def launch_setup(context, *args, **kwargs):
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
        launch_arguments={"show_rviz": "false"}.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("avatar_challenge"), "rviz", "shape_tracer.rviz"]
            ),
        ],
        parameters=[moveit_parameters(context)],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
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

    return [
        xarm_moveit_launch,
        rviz_node,
        xarm_planner_launch,
        designer_node,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
