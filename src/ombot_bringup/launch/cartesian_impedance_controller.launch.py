from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    ]

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

    # --- RViz config (optional) ---
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ombot_description"), "ombot/rviz", "view_robot.rviz"]
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

    gui = LaunchConfiguration("gui")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn your NEW Cartesian controller
    cartesian_impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_impedance_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Start order: spawn JSB first, then Cartesian impedance
    delay_cartesian_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[cartesian_impedance_controller_spawner],
        )
    )

    # Optional: RViz after JSB (or after controller if you prefer)
    delay_rviz_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_cartesian_after_jsb,
        # delay_rviz_after_jsb,  # uncomment if you want RViz auto-start
    ]

    return LaunchDescription(declared_arguments + nodes)
