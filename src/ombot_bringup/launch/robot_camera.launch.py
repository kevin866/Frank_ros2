from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Args
    declared_arguments = [
        DeclareLaunchArgument("launch_zed", default_value="true"),
        DeclareLaunchArgument("start_control", default_value="true", description="Start controller_manager."),
        DeclareLaunchArgument("start_jsb", default_value="true", description="Spawn joint_state_broadcaster."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    # URDF via xacro
    xacro_file = PathJoinSubstitution([FindPackageShare("ombot_description"), "urdf", "ombot.urdf.xacro"])
    robot_description_content = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # Controller YAML
    controllers_yaml = PathJoinSubstitution([FindPackageShare("ombot_bringup"), "config", "ombot_controller.yaml"])

    # robot_state_publisher (needs /joint_states from JSB)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    # controller_manager (optional via flag)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml, {"validate_interfaces": False}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_control")),
    )

    # spawn joint_state_broadcaster after controller_manager starts
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_jsb")),
    )

    # ZED 2i (no TFs)
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"])
        ]),
        launch_arguments={
            "camera_model": "zed2",
            "camera_name": "zed",
            "publish_tf": "false",
            "publish_map_tf": "false",
            "enable_depth": "true",
            "enable_point_cloud": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_zed")),
    )

    return LaunchDescription(declared_arguments + [
        rsp,
        control_node,
        jsb_spawner,
        zed_launch,
    ])
