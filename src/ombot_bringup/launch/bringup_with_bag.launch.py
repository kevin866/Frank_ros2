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
    storage      = LaunchConfiguration('storage',      default='mcap')    # or 'sqlite3'
    compress     = LaunchConfiguration('compress',     default='zstd')    # or 'none'
    qos_path     = LaunchConfiguration('qos_overrides', default='')       # optional: path to qos.yaml
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

    # --- Core nodes ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}],
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

    mecanum_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_controller', '-c', '/controller_manager'],
        output='screen'
    )

    # --- Coordination test nodes ---
    goal_from_offset = Node(
        package='ombot_coordination',
        executable='goal_from_base_offset_latched',
        name='goal_from_offset',
        parameters=[{
            'offset_x': 0.5,
            'publish_rate_hz': 20.0,
            'mode': 'latch'
        }],
        remappings=[
            ('/vrpn_mocap/RigidBody_1/pose', '/vrpn_mocap/RigidBody_1/pose'),
            ('/goal_pose', '/goal_pose'),
        ],
        output='screen',
        respawn=True
    )

    base_p_on_x = Node(
        package='ombot_coordination',
        executable='base_p_on_x',
        parameters=[{'kx': 3.0, 'vmax': 5.0, 'flip_forward': False}],
        output='screen'
    )

    arm_zero_twist = Node(
        package='ombot_coordination',
        executable='arm_zero_twist',
        name='arm_zero_twist',
        output='screen'
    )

    start_tests = TimerAction(period=2.0, actions=[goal_from_offset, base_p_on_x, arm_zero_twist])

    # --- rosbag2 recorder (records only your 5 topics) ---
    topics_to_record = [
        '/goal_pose',
        '/mecanum_controller/reference',
        '/resolved_rate_controller/ee_twist',
        '/vrpn_mocap/RigidBody_1/pose',
        '/vrpn_mocap/RigidBody_2/pose'
    ]

    bag_cmd = [
        'ros2', 'bag', 'record',
        *topics_to_record,
        '--output', bag_prefix,
        '--storage', storage,
        '--compression-mode', 'file',
        '--compression-format', compress,
        '--max-bag-size', split_size,
        '--max-bag-duration', split_secs,
    ]
    # Optionally add QoS overrides if provided
    # (use this if VRPN or other sources are best_effort and you’ve seen QoS mismatch)
    # Example qos.yaml shown below.
    bag_cmd_with_qos = bag_cmd + ['--qos-profile-overrides-path', qos_path]
    bag_cmd_final = ['bash', '-c',
        'if [ -n "$QOS" ]; then ' +
        '  ros2 bag record ' + ' '.join(topics_to_record) + ' ' +
        '--output "$PREFIX" --storage "$STO" --compression-mode file --compression-format "$CMP" ' +
        '--max-bag-size "$SZ" --max-bag-duration "$DUR" ' +
        '--qos-profile-overrides-path "$QOS" ; ' +
        'else ' +
        '  ros2 bag record ' + ' '.join(topics_to_record) + ' ' +
        '--output "$PREFIX" --storage "$STO" --compression-mode file --compression-format "$CMP" ' +
        '--max-bag-size "$SZ" --max-bag-duration "$DUR" ; ' +
        'fi'
    ]

    bag_record = ExecuteProcess(
        cmd=bag_cmd_final,
        additional_env={
            'PREFIX': bag_prefix.perform(None),
            'STO': storage.perform(None),
            'CMP': compress.perform(None),
            'SZ': split_size.perform(None),
            'DUR': split_secs.perform(None),
            'QOS': qos_path.perform(None),
        },
        output='screen'
    )

    # Start recorder after controllers spawn (small delay helps discovery)
    delayed_bag = TimerAction(period=3.0, actions=[bag_record])

    # Optional: shut down the whole launch when the control node exits
    end_when_control_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[Shutdown(reason='ros2_control_node exited')]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('bag_prefix',   default_value='ombot_run'),
        DeclareLaunchArgument('storage',      default_value='mcap'),
        DeclareLaunchArgument('compress',     default_value='zstd'),
        DeclareLaunchArgument('qos_overrides', default_value=''),
        DeclareLaunchArgument('max_bag_size', default_value=str(1024*1024*1024)),
        DeclareLaunchArgument('max_bag_secs', default_value='600'),

        robot_state_publisher,
        control_node,
        mecanum_spawner,
        start_tests,

        delayed_bag,
        end_when_control_exits,
    ])
