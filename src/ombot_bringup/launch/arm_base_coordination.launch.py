# bringup_with_bag.launch.py
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, ExecuteProcess,
    RegisterEventHandler, Shutdown
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Common args ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    bag_prefix   = LaunchConfiguration('bag_prefix',   default='ombot_run')
    storage      = LaunchConfiguration('storage',      default='sqlite3')    # or 'sqlite3'
    compress     = LaunchConfiguration('compress',     default='zstd')    # or 'none'
    qos_path     = LaunchConfiguration('qos_overrides', default='/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml')       # optional: path to qos.yaml
    split_size   = LaunchConfiguration('max_bag_size', default=str(1024*1024*1024))  # 1 GiB
    split_secs   = LaunchConfiguration('max_bag_secs', default='600')     # 10 minutes

    # --- Build robot_description from URDF/Xacro ---
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
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'robot_description': robot_description_content}],
        output='screen'
    )

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

    # --- Controllers (chain activation) ---
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
    rr  = Node(
        package='controller_manager', executable='spawner',
        arguments=['resolved_rate_controller', '--activate', '-c', '/controller_manager'],
        output='screen'
    )
    mecanum_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['mecanum_controller', '--activate', '-c', '/controller_manager'],
        output='screen'
    )

    # Chain: JSB -> Impedance -> ResolvedRate
    chain_imp_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[imp])
    )
    chain_rr_after_imp = RegisterEventHandler(
        OnProcessExit(target_action=imp, on_exit=[rr])
    )

    # --- Coordinator ---
    coordinator = Node(
        package='ombot_coordination',
        executable='arm_base_coordinator',
        name='arm_base_coordinator',
        output='screen',
        parameters=[{
            'inputs_in_world': True,
            'world_frame_name': 'world',
            'base_pose_topic': '/vrpn_mocap/RigidBody_1/pose',
            'ee_pose_topic':   '/vrpn_mocap/RigidBody_2/pose',

            'use_offset_goal': True,
            'offset_frame': 'base',
            'offset_xyz': [1.5, 0.0, 0.0],

            'ee_twist_topic': '/resolved_rate_controller/ee_twist',

            # ✅ IMPORTANT: match MecanumDriveController's subscriber (TwistStamped)
            'cmd_vel_topic':  '/mecanum_controller/reference',

            'base_is_holonomic': False,
            'k_heading': 1.5,
            'kp_lin': 3.0, 'kd_lin': 0.3,
            'kp_ang': 2.0, 'kd_ang': 0.25,
            'k_ori_weight': 0.5,
            'ee_lin_limit': 0.15, 'ee_ang_limit': 0.6,
            'base_lin_limit': 0.6, 'base_ang_limit': 1.2,
            'blend_mid_distance': 0.8, 'blend_slope': 6.0,
            'max_reach': 0.8,
            'd_retract_enter': 0.25, 'd_retract_exit': 0.35,
            'vel_lpf_alpha': 0.9,
            'slew_base': 4.0,
            'slew_arm_lin': 0.5,
            'slew_arm_ang': 1.5,
            'base_marker_offset_xyz': [0.0, 0.0, 0.0],
            'base_marker_offset_rpy': [0.0, 0.0, 0.0],
            'k_d': 3.0,
            'd_mid': 1.80,
        }]
    )

    # Start coordinator only after resolved_rate_controller is active
    start_coord_after_rr = RegisterEventHandler(
        OnProcessExit(target_action=rr, on_exit=[coordinator])
    )

    # --- rosbag2 recorder ---
    topics_to_record = [
        '/mecanum_controller/reference',     # base ref (TwistStamped)
        '/resolved_rate_controller/ee_twist',
        '/vrpn_mocap/RigidBody_1/pose',
        '/vrpn_mocap/RigidBody_2/pose',
        '/goal_pose',
        '/joint_states'
    ]
    bag_cmd_final = [
        'ros2', 'bag', 'record', *topics_to_record,
        '--output', LaunchConfiguration('bag_prefix'),
        '--storage', LaunchConfiguration('storage'),
        '--compression-mode', 'file',
        '--compression-format', LaunchConfiguration('compress'),
        '--max-bag-size', LaunchConfiguration('max_bag_size'),
        '--max-bag-duration', LaunchConfiguration('max_bag_secs'),
        '--qos-profile-overrides-path', LaunchConfiguration('qos_overrides')
    ]
    bag_record = ExecuteProcess(cmd=bag_cmd_final, output='screen')

    # Start bag after resolved_rate_controller (same moment as coordinator)
    start_bag_after_rr = RegisterEventHandler(
        OnProcessExit(target_action=rr, on_exit=[bag_record])
    )

    end_when_control_exits = RegisterEventHandler(
        OnProcessExit(target_action=control_node, on_exit=[Shutdown(reason='ros2_control_node exited')])
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',  default_value='false'),
        DeclareLaunchArgument('bag_prefix',    default_value='ombot_run'),
        DeclareLaunchArgument('storage',       default_value='sqlite3'),
        DeclareLaunchArgument('compress',      default_value='zstd'),
        DeclareLaunchArgument('qos_overrides', default_value='/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml'),
        DeclareLaunchArgument('max_bag_size',  default_value=str(1024*1024*1024)),
        DeclareLaunchArgument('max_bag_secs',  default_value='600'),

        robot_state_publisher,
        control_node,

        # Controllers: mecanum can start anytime; chain the arm controllers
        mecanum_spawner,
        jsb,
        chain_imp_after_jsb,
        chain_rr_after_imp,

        # Start coord + bag once RR is active
        start_coord_after_rr,
        start_bag_after_rr,

        end_when_control_exits,
    ])
