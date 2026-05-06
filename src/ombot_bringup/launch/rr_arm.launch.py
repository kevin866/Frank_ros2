# bringup_rr_with_publisher.launch.py

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch arguments ─────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ── Robot description ─────────────────────────────────────────────────────
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('ombot_description'),
        'urdf', 'ombot.urdf.xacro'
    ])
    robot_description_content = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }],
        output='screen'
    )

    # ── ros2_control node ─────────────────────────────────────────────────────
    ctrl_yaml = PathJoinSubstitution([
        get_package_share_directory('ombot_bringup'),
        'config', 'ombot_controller.yaml'
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description_content}, ctrl_yaml],
    )

    # ── Controller spawners ───────────────────────────────────────────────────
    jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--activate', '-c', '/controller_manager'],
        output='screen'
    )

    imp = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_impedance_controller', '--activate', '-c', '/controller_manager'],
        output='screen'
    )

    rr = Node(
        package='controller_manager', executable='spawner',
        arguments=['resolved_rate_controller', '--activate', '-c', '/controller_manager'],
        parameters=[ctrl_yaml],
        output='screen'
    )

    # Chain: JSB → Impedance → ResolvedRate
    chain_imp_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[imp])
    )
    chain_rr_after_imp = RegisterEventHandler(
        OnProcessExit(target_action=imp, on_exit=[rr])
    )

    # ── EE Twist publisher (safe bounded oscillator) ─────────────────────────
    ee_twist_publisher = Node(
        package='ombot_coordination',        # ← change to whichever package owns the script
        executable='ee_twist_oscillate',     # ← must match entry_point in setup.py
        name='ee_twist_oscillate',
        output='screen',
        parameters=[{
            'topic':    '/resolved_rate_controller/ee_twist',
            'frame_id': 'base_link',
            'amp':      0.02,    # m/s peak velocity
            'freq':     0.05,    # Hz  (one cycle per 20 s)
            'x_limit':  0.02,    # m   max EE displacement from start
            'rate_hz':  50.0,
            'base_link': 'link_1',
            'tip_link':  'link_6',
            'joints': [
                'joint_1', 'joint_2', 'joint_3',
                'joint_4', 'joint_5', 'joint_6',
            ],
        }]
    )

    # Start publisher only after RR controller is active
    start_publisher_after_rr = RegisterEventHandler(
        OnProcessExit(target_action=rr, on_exit=[ee_twist_publisher])
    )

    # Shutdown when ros2_control_node exits
    end_when_control_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[Shutdown(reason='ros2_control_node exited')]
        )
    )

    # ── LaunchDescription ─────────────────────────────────────────────────────
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        robot_state_publisher,
        control_node,

        jsb,
        chain_imp_after_jsb,
        chain_rr_after_imp,

        start_publisher_after_rr,
        end_when_control_exits,
    ])