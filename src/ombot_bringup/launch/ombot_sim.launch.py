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
        DeclareLaunchArgument("gui", default_value="true",
                              description="Start RViz2."),
    ]

    # Sim URDF: replaces hardware plugins with mock_components/GenericSystem
    sim_urdf = PathJoinSubstitution([
        FindPackageShare("ombot_description"),
        "urdf",
        "ombot_sim.urdf.xacro",   # <-- see note below
    ])
    robot_description_content = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", sim_urdf]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("ombot_bringup"), "config", "ombot_controller.yaml",
    ])
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ombot_description"), "ombot/rviz", "view_robot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml, {"validate_interfaces": False}],
        output="screen",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_impedance_controller", "-c", "/controller_manager"],
        output="screen",
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    gui = LaunchConfiguration("gui")
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    delay_rviz_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner, on_exit=[rviz_node],
        )
    )
    delay_impedance_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner, on_exit=[impedance_controller_spawner],
        )
    )
    delay_gripper_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner, on_exit=[gripper_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_impedance_after_jsb,
        delay_gripper_after_jsb,
        delay_rviz_after_jsb,
    ]
    return LaunchDescription(declared_arguments + nodes)