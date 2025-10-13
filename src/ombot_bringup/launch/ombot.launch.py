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

    # --- URDF via xacro for base ---
    base_urdf = PathJoinSubstitution([
        FindPackageShare("ombot_description"),
        "urdf",
        "ombot.urdf.xacro",
    ])
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            base_urdf,
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # --- Controller YAML ---
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("ombot_bringup"),
        "config",
        "ombot_controller.yaml",
    ])

    # --- RViz config (optional) ---
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ombot_description"), "ombot/rviz", "view_robot.rviz"]
    )

    # --- Nodes ---
    #  controller_manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml,{'validate_interfaces': False}],
        output="screen",
    )

    # control_node = Node(
    #     package='controller_manager', executable='ros2_control_node',
    #     parameters=[
    #         {'robot_description': robot_description},
    #         controllers_yaml,
    #         {'validate_interfaces': False}
    #     ],
    #     output='screen'
    # )


    # Base robot_state_publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    mecanum_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_controller"],
        output="screen",
    )
    impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_impedance_controller", "-c", "/controller_manager"],
        output="screen",
    )
    # RViz
    gui = LaunchConfiguration("gui")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
    # Event handlers
    delay_rviz_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    delay_impedance_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[impedance_controller_spawner],
        )
    )

    ee2base = Node(
        package='ombot_controller',
        executable='ee_to_base',
        name='ee_to_base',
        parameters=[{
            'use_stamped_out': True,        # set False if /mecanum_controller/reference expects Twist
            'kp_lin': 0.35,
            'kp_yaw': 1.0,
            'vmax': 0.5,
            'wmax': 1.0,
            'lpf_alpha': 0.4,
            'deadband_m': 0.01,
            'deadband_yaw': 0.02,
            'watchdog_ms': 150,
            'zero_on_startup': True,
            'enable': True,
            'publish_rate_hz': 50.0,
        }],)
    
    # Launch all nodes
    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        mecanum_controller_spawner,
        delay_impedance_after_jsb,
        delay_rviz_after_jsb,
        # ee2base,
    ]
    return LaunchDescription(declared_arguments + nodes)
