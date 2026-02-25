from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess, TimerAction, LogInfo, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # --- URDF via xacro -> robot_description ---
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

    # --- Controller YAML ---
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ombot_bringup"),
            "config",
            "ombot_controller.yaml",
        ]
    )

    # --- Nodes ---
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

    # Update this name to whatever you used in ombot_controller.yaml under controller_manager.ros__parameters
    vel_excitation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_velocity_excitation_controller", "-c", "/controller_manager"],
        output="screen",
    )

    delay_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[vel_excitation_controller_spawner],
        )
    )

    # --- Rosbag recorder via CLI ---
    rosbag_record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "-o", "joint_velocity_excitation_run",
            "--storage", "sqlite3",
            "/joint_velocity_excitation_controller/qdot_cmd",
            "/joint_states",
        ],
        output="screen",
    )

    stop_recording = TimerAction(
        period=50.0,   # run for 50 seconds
        actions=[
            LogInfo(msg="Stopping rosbag recording..."),
            EmitEvent(event=Shutdown(reason="Recording complete"))
        ]
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_controller_after_jsb,
        rosbag_record,
        stop_recording,
    ]

    return LaunchDescription(nodes)
