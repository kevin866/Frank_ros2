from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # --- Args ---
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    ]

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
            # optionally add args if your xacro uses them:
            " ",
            "robot_name:=ombot",   # <-- pass arg here
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # --- Controller YAML ---
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ombot_bringup"),
            "config",
            "ombot_controller.yaml",   # make sure this includes your GravityCancelController
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
        # IMPORTANT: give the robot_description (URDF XML) to controller_manager, too
        parameters=[robot_description, robot_controllers],
        output="both",
        # You do NOT need to remap robot_description; it's a parameter, not a topic.
        # remappings=[("~/robot_description", "/robot_description")],  # <-- remove
        # arguments=["--ros-args", "--log-level", "controller_manager:=info"],
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

    # Swap to your gravity controller name from YAML (e.g., "gravity_cancel_controller")
    gravity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gravity_cancel_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Start order: controller_manager + RSP come up, then spawn JSB, then gravity controller, then RViz.
    delay_rviz_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_gravity_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gravity_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_gravity_after_jsb,
        delay_rviz_after_jsb,
    ]

    return LaunchDescription(declared_arguments + nodes)
