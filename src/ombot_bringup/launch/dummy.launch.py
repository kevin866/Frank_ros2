# bringup_with_bag.launch.py
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, ExecuteProcess,
    RegisterEventHandler, Shutdown
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ---- Arguments (defaults set ONLY here) ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    bag_prefix   = LaunchConfiguration('bag_prefix')
    storage      = LaunchConfiguration('storage')
    compress     = LaunchConfiguration('compress')
    qos_path     = LaunchConfiguration('qos_overrides')
    split_size   = LaunchConfiguration('max_bag_size')
    split_secs   = LaunchConfiguration('max_bag_secs')

    # ---- Robot description ----
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('ombot_description'),
        'urdf', 'ombot.urdf.xacro'
    ])
    robot_description_content = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf_file]),
        value_type=str
    )

    # ---- Core nodes ----
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
        parameters=[{'robot_description': robot_description_content}, ctrl_yaml],
        output='screen'
    )

    mecanum_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_controller', '-c', '/controller_manager'],
        output='screen'
    )

    # ---- Coordination test nodes ----
    goal_from_offset = Node(
        package='ombot_coordination',
        executable='goal_from_base_offset_latched',
        name='goal_from_offset',
        parameters=[{'offset_x': 1.5, 'publish_rate_hz': 20.0, 'mode': 'latch'}],
        # remappings can be omitted if identical
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

    # ---- rosbag2 recorder: your 5 topics ----
    topics_to_record = [
        '/goal_pose',
        '/mecanum_controller/reference',
        '/resolved_rate_controller/ee_twist',
        '/vrpn_mocap/RigidBody_1/pose',
        '/vrpn_mocap/RigidBody_2/pose',
    ]

    # Run when qos_overrides == '' (no QoS file)
    bag_record_no_qos = ExecuteProcess(
        condition=UnlessCondition(
            PythonExpression(["'", qos_path, "' != ''"])
        ),
        cmd=[
            'ros2', 'bag', 'record',
            *topics_to_record,
            '--output', bag_prefix,
            '--storage', storage,
            '--compression-mode', 'file',
            '--compression-format', compress,
            '--max-bag-size', split_size,
            '--max-bag-duration', split_secs,
        ],
        output='screen'
    )

    # Run when qos_overrides != '' (QoS file provided)
    bag_record_with_qos = ExecuteProcess(
        condition=IfCondition(
            PythonExpression(["'", qos_path, "' != ''"])
        ),
        cmd=[
            'ros2', 'bag', 'record',
            *topics_to_record,
            '--output', bag_prefix,
            '--storage', storage,
            '--compression-mode', 'file',
            '--compression-format', compress,
            '--max-bag-size', split_size,
            '--max-bag-duration', split_secs,
            '--qos-profile-overrides-path', qos_path,
        ],
        output='screen'
    )

    delayed_bag_no_qos   = TimerAction(period=0.5, actions=[bag_record_no_qos])
    delayed_bag_with_qos = TimerAction(period=0.5, actions=[bag_record_with_qos])

    # Optional: end launch when ros2_control_node exits
    end_when_control_exits = RegisterEventHandler(
        OnProcessExit(target_action=control_node, on_exit=[Shutdown(reason='ros2_control_node exited')])
    )

    return LaunchDescription([
        # ----- Declare args with baked-in defaults -----
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('bag_prefix',   default_value='my_test_run'),
        DeclareLaunchArgument('storage',      default_value='sqlite3'),
        DeclareLaunchArgument('compress',     default_value='zstd'),
        DeclareLaunchArgument('qos_overrides', default_value='/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml'),
        DeclareLaunchArgument('max_bag_size', default_value=str(1024*1024*1024)),
        DeclareLaunchArgument('max_bag_secs', default_value='600'),

        # ----- Nodes -----
        robot_state_publisher,
        control_node,
        mecanum_spawner,
        start_tests,

        # ----- Recorder (one of these runs depending on qos_overrides) -----
        delayed_bag_no_qos,
        delayed_bag_with_qos,

        end_when_control_exits,
    ])
