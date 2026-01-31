# rr_with_bag.launch.py
#
# Bring up ombot arm controllers (JSB -> impedance -> resolved_rate),
# optionally start RViz and resolved_rate_tester, and start a rosbag
# recorder AFTER the resolved_rate_controller is active (same pattern
# as your bringup_wb_with_bag.launch.py).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # -----------------------------
    # Launch args (same style as your bag launch)
    # -----------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    bag_prefix   = LaunchConfiguration('bag_prefix',   default='rr_run1')
    storage      = LaunchConfiguration('storage',      default='sqlite3')
    qos_path     = LaunchConfiguration(
        'qos_overrides',
        default='/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml'
    )
    split_size   = LaunchConfiguration('max_bag_size', default=str(1024*1024*1024))  # 1 GiB
    split_secs   = LaunchConfiguration('max_bag_secs', default='600')                # 10 min

    gui          = LaunchConfiguration('gui', default='false')
    chained      = LaunchConfiguration('chained', default='false')

    run_rr_tester     = LaunchConfiguration('run_rr_tester', default='true')
    rr_tester_topic   = LaunchConfiguration('rr_tester_topic', default='/resolved_rate_controller/ee_twist')
    rr_tester_vz      = LaunchConfiguration('rr_tester_vz', default='0.04')
    rr_tester_frame   = LaunchConfiguration('rr_tester_frame', default='world')
    rr_tester_rate_hz = LaunchConfiguration('rr_tester_rate_hz', default='50.0')

    controllers_yaml = LaunchConfiguration(
        'controllers_yaml',
        default=PathJoinSubstitution([
            get_package_share_directory('ombot_bringup'),
            'config', 'ombot_controller.yaml'
        ])
    )

    # -----------------------------
    # robot_description (xacro)
    # -----------------------------
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('ombot_description'),
        'urdf', 'arm', 'ombot_arm.urdf.xacro'
    ])

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ', urdf_file,
            ' ', 'robot_name:=ombot'
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content}, controllers_yaml],
        output='screen'
    )

    # -----------------------------
    # Controllers: JSB -> impedance -> resolved_rate
    # -----------------------------
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--activate', '-c', '/controller_manager'],
        output='screen'
    )

    imp_name = 'joint_impedance_controller'
    rr_name  = 'resolved_rate_controller'

    imp_spawner_chained = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[imp_name, '--activate-as', 'chained', '-c', '/controller_manager'],
        output='screen',
        condition=IfCondition(chained)
    )

    imp_spawner_normal = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[imp_name, '--activate', '-c', '/controller_manager'],
        output='screen',
        condition=UnlessCondition(chained)
    )

    rr_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[rr_name, '--activate', '-c', '/controller_manager'],
        output='screen'
    )

    chain_imp_after_jsb_chained = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[imp_spawner_chained])
    )
    chain_imp_after_jsb_normal = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[imp_spawner_normal])
    )
    chain_rr_after_imp_chained = RegisterEventHandler(
        OnProcessExit(target_action=imp_spawner_chained, on_exit=[rr_spawner])
    )
    chain_rr_after_imp_normal = RegisterEventHandler(
        OnProcessExit(target_action=imp_spawner_normal, on_exit=[rr_spawner])
    )

    # -----------------------------
    # RViz (optional)
    # -----------------------------
    rviz_config = PathJoinSubstitution([
        get_package_share_directory('ombot_description'),
        'ombot', 'rviz', 'view_robot.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(gui)
    )
    start_rviz_after_rr = RegisterEventHandler(
        OnProcessExit(target_action=rr_spawner, on_exit=[rviz_node])
    )

    # -----------------------------
    # resolved_rate_tester (optional) — start AFTER RR is active
    # -----------------------------
    rr_tester_node = Node(
        package='ombot_coordination',
        executable='resolved_rate_tester',
        name='resolved_rate_tester',
        output='screen',
        parameters=[{
            'topic': rr_tester_topic,
            'vz': rr_tester_vz,
            'frame_id': rr_tester_frame,
            'rate_hz': rr_tester_rate_hz,
        }],
        condition=IfCondition(run_rr_tester)
    )
    start_tester_after_rr = RegisterEventHandler(
        OnProcessExit(target_action=rr_spawner, on_exit=[rr_tester_node])
    )

    # -----------------------------
    # rosbag2 recorder (start AFTER RR is active)
    # -----------------------------
    topics_to_record = [
        '/resolved_rate_controller/ee_twist',  # command going into RR
        '/ee_pose',                            # feedback
        '/joint_states',                       # joints
        # optional debugging:
        # '/tf', '/tf_static',
        # '/controller_manager/robot_description',
    ]

    bag_cmd = [
        'ros2', 'bag', 'record', *topics_to_record,
        '--output', bag_prefix,
        '--storage', storage,
        '--max-bag-size', split_size,
        '--max-bag-duration', split_secs,
        '--qos-profile-overrides-path', qos_path
    ]

    bag_record = ExecuteProcess(cmd=bag_cmd, output='screen')
    start_bag_after_rr = RegisterEventHandler(
        OnProcessExit(target_action=rr_spawner, on_exit=[bag_record])
    )

    # -----------------------------
    # Shutdown when ros2_control_node exits
    # -----------------------------
    end_when_control_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[Shutdown(reason='ros2_control_node exited')]
        )
    )

    # -----------------------------
    # LaunchDescription
    # -----------------------------
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',  default_value='false'),
        DeclareLaunchArgument('gui',           default_value='false'),
        DeclareLaunchArgument('controllers_yaml', default_value=PathJoinSubstitution([
            get_package_share_directory('ombot_bringup'),
            'config', 'ombot_controller.yaml'
        ])),
        DeclareLaunchArgument('chained',       default_value='false'),

        DeclareLaunchArgument('bag_prefix',    default_value='rr_run1'),
        DeclareLaunchArgument('storage',       default_value='sqlite3'),
        DeclareLaunchArgument(
            'qos_overrides',
            default_value='/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml'
        ),
        DeclareLaunchArgument('max_bag_size',  default_value=str(1024*1024*1024)),
        DeclareLaunchArgument('max_bag_secs',  default_value='600'),

        DeclareLaunchArgument('run_rr_tester', default_value='true'),
        DeclareLaunchArgument('rr_tester_topic', default_value='/resolved_rate_controller/ee_twist'),
        DeclareLaunchArgument('rr_tester_vz', default_value='0.04'),
        DeclareLaunchArgument('rr_tester_frame', default_value='world'),
        DeclareLaunchArgument('rr_tester_rate_hz', default_value='50.0'),

        # Core nodes
        rsp_node,
        control_node,

        # Controllers
        jsb,
        chain_imp_after_jsb_chained,
        chain_imp_after_jsb_normal,
        chain_rr_after_imp_chained,
        chain_rr_after_imp_normal,

        # Start bag + tester (and RViz) after RR is active
        start_bag_after_rr,
        # start_tester_after_rr,
        start_rviz_after_rr,

        end_when_control_exits,
    ])
