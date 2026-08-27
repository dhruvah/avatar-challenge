from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    # show_rviz:=false suppresses the stock MoveIt RViz so we can start our own
    # with a layout that already has the target and actual-path displays in it.
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

    shapes_file = LaunchConfiguration("shapes_file")

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
                "hold_after_trace": True,
            }
        ],
    )

    return [
        xarm_moveit_launch,
        rviz_node,
        xarm_planner_launch,
        shape_tracer_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "shapes_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("avatar_challenge"), "config", "shapes.json"]
            ),
            description="Shape list to trace. Defaults to the packaged samples; "
                        "pass an absolute path to trace your own without rebuilding.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
