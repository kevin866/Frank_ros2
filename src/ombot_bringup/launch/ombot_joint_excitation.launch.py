from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # --- URDF via xacro -> robot_description (XML string, not a path) ---
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
            "ombot_controller.yaml",   # must define joint_excitation_controller
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

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    excitation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_excitation_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Ensure we spawn the excitation controller *after* JSB is up
    delay_excitation_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[excitation_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_excitation_after_jsb,
    ]

    return LaunchDescription(nodes)
