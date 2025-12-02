# bringup_wb_with_bag.launch.py

import math
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess,
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
    storage      = LaunchConfiguration('storage',      default='sqlite3')
    compress     = LaunchConfiguration('compress',     default='zstd')
    qos_path     = LaunchConfiguration(
        'qos_overrides',
        default='/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml'
    )
    split_size   = LaunchConfiguration('max_bag_size', default=str(1024*1024*1024))  # 1 GiB
    split_secs   = LaunchConfiguration('max_bag_secs', default='600')                # 10 min

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
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }],
        output='screen'
    )

    # --- ros2_control node + controllers YAML ---
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

    # --- Controllers (spawn chain) ---
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

    wb_rr = Node(
        package='controller_manager', executable='spawner',
        arguments=['wb_resolved_rate_controller', '--activate', '-c', '/controller_manager'],
        output='screen'
    )

    mecanum_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['mecanum_controller', '--activate', '-c', '/controller_manager'],
        output='screen'
    )

    goal_from_offset = Node(
        package='ombot_coordination',
        executable='goal_from_base_offset_latched',
        name='goal_from_base_offset_latched',
        output='screen',
        parameters=[{
            'base_pose_topic': '/vrpn_mocap/RigidBody_1/pose',
            'goal_pose_topic': '/goal_pose',
            'offset_xyz': [0.1, 0.0, 0.0],   # set your desired offset here (world frame)
            'latch': True,                   # True = latch once, False = follow base
        }]
    )


    # Chain: JSB -> Impedance -> WholeBodyResolvedRate
    chain_imp_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[imp])
    )
    chain_wb_after_imp = RegisterEventHandler(
        OnProcessExit(target_action=imp, on_exit=[wb_rr])
    )

    # --- Whole-body task commander (Python) ---
    # This node just publishes desired EE twist in base_link frame
    wb_task_commander = Node(
        package='ombot_coordination',
        executable='whole_body_task_commander',
        name='whole_body_task_commander',
        output='screen',
        parameters=[{
            # Poses (in world frame)
            'base_pose_topic': '/vrpn_mocap/RigidBody_1/pose',
            'ee_pose_topic':   '/ee_pose',
            'goal_pose_topic': '/goal_pose',

            # Twist out -> must match controller's "~ee_twist" topic expansion
            'ee_twist_topic': '/wb_resolved_rate_controller/ee_twist',

            # Simple task-space gains (tune as needed)
            'kp_pos': 1.0,
            'kp_rot': 1.0,
            'kd_pos': 0.0,
            'kd_rot': 0.0,

            # Velocity caps
            'max_lin': 0.2,   # m/s
            'max_ang': 0.7,   # rad/s
        }]
    )

    # Start commander only after wb_resolved_rate_controller is active
    start_commander_after_wb = RegisterEventHandler(
        OnProcessExit(target_action=wb_rr, on_exit=[wb_task_commander])
    )

    # --- rosbag2 recorder ---
    topics_to_record = [
        '/mecanum_controller/reference',         # base ref (TwistStamped)
        '/wb_resolved_rate_controller/ee_twist', # desired EE twist
        '/vrpn_mocap/RigidBody_1/pose',
        '/vrpn_mocap/RigidBody_2/pose',
        '/goal_pose',
        '/joint_states',
        '/ee_pose'
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

    # Start bag when whole-body controller is active (same time as commander)
    start_bag_after_wb = RegisterEventHandler(
        OnProcessExit(target_action=wb_rr, on_exit=[bag_record])
    )

    # Shutdown when ros2_control_node exits
    end_when_control_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[Shutdown(reason='ros2_control_node exited')]
        )
    )

    return LaunchDescription([
        # Launch args
        DeclareLaunchArgument('use_sim_time',  default_value='false'),
        DeclareLaunchArgument('bag_prefix',    default_value='ombot_run1'),
        DeclareLaunchArgument('storage',       default_value='sqlite3'),
        DeclareLaunchArgument('compress',      default_value='zstd'),
        DeclareLaunchArgument(
            'qos_overrides',
            default_value='/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml'
        ),
        DeclareLaunchArgument('max_bag_size',  default_value=str(1024*1024*1024)),
        DeclareLaunchArgument('max_bag_secs',  default_value='600'),

        # Core nodes
        robot_state_publisher,
        control_node,

        # Controllers: mecanum can start anytime; chain the arm controllers
        mecanum_spawner,
        jsb,
        chain_imp_after_jsb,
        chain_wb_after_imp,

        goal_from_offset,

        # Start commander + bag once WB controller is active
        start_commander_after_wb,
        start_bag_after_wb,

        end_when_control_exits,
    ])
