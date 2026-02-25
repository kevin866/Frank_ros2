from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess, TimerAction, LogInfo, EmitEvent, DeclareLaunchArgument
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import datetime


def generate_launch_description():
    # -------------------------
    # Args
    # -------------------------
    duration_arg = DeclareLaunchArgument(
        "duration",
        default_value="180.0",
        description="Seconds to run calibration + bag recording before shutdown.",
    )
    bag_prefix_arg = DeclareLaunchArgument(
        "bag_prefix",
        default_value="joint_friction_calib_run",
        description="Prefix for rosbag output folder.",
    )

    duration = LaunchConfiguration("duration")
    bag_prefix = LaunchConfiguration("bag_prefix")

    # Timestamped bag folder to avoid 'already exists'
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = [bag_prefix, "_", stamp]

    # -------------------------
    # robot_description via xacro
    # -------------------------
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("ombot_description"),
                "urdf",
                "arm",
                "ombot_arm.urdf.xacro",
            ]),
            " ",
            "robot_name:=ombot",
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # -------------------------
    # Controllers YAML
    # -------------------------
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ombot_bringup"),
            "config",
            "ombot_controller.yaml",
        ]
    )

    # -------------------------
    # Nodes
    # -------------------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    calib_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_friction_calib_controller", "-c", "/controller_manager"],
        output="screen",
    )

    delay_calib_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[calib_controller_spawner],
        )
    )

    # -------------------------
    # Rosbag record
    # -------------------------
    rosbag_record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "-o", *bag_name,
            "--storage", "sqlite3",
            "/joint_states",
            # Optional extras (uncomment if useful)
            # "/tf",
            # "/tf_static",
        ],
        output="screen",
    )

    # -------------------------
    # Stop after duration
    # -------------------------
    stop_recording = TimerAction(
        period=duration,
        actions=[
            LogInfo(msg=["Stopping calibration + rosbag after ", duration, " seconds..."]),
            EmitEvent(event=Shutdown(reason="Calibration recording complete")),
        ],
    )

    return LaunchDescription([
        duration_arg,
        bag_prefix_arg,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_calib_after_jsb,
        rosbag_record,
        stop_recording,
    ])
