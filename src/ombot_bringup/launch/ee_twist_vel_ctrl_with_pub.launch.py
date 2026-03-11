# ee_twist_vel_ctrl_with_pub.launch.py
#
# Bring up ros2_control + arm controllers (JSB -> impedance -> EeTwistVelocityController),
# then start the Python Twist publisher AFTER the EE velocity controller is active.
# Optionally start RViz and rosbag recorder after controller activation (same pattern as your rr_with_bag).
#
# Assumptions:
# - Your C++ controller name in YAML is: ee_twist_velocity_controller
# - Your Python node executable is: ee_twist_cmd_publisher (from the script I wrote)
# - The controller listens on twist_topic (default /ee_twist_cmd)
#
# Customize package names at the top if needed.

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
    # Launch args
    # -----------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    gui          = LaunchConfiguration('gui', default='false')
    chained      = LaunchConfiguration('chained', default='false')

    # Bag args (optional)
    record_bag   = LaunchConfiguration('record_bag', default='false')
    bag_prefix   = LaunchConfiguration('bag_prefix', default='ee_vel_run1')
    storage      = LaunchConfiguration('storage', default='sqlite3')
    qos_path     = LaunchConfiguration(
        'qos_overrides',
        default='/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml'
    )
    split_size   = LaunchConfiguration('max_bag_size', default=str(1024*1024*1024))  # 1 GiB
    split_secs   = LaunchConfiguration('max_bag_secs', default='600')                # 10 min

    # Publisher args
    run_pub      = LaunchConfiguration('run_pub', default='true')
    pub_topic    = LaunchConfiguration('pub_topic', default='/ee_twist_cmd')
    pub_frame    = LaunchConfiguration('pub_frame', default='base_link')
    pub_rate_hz  = LaunchConfiguration('pub_rate_hz', default='50.0')
    pub_mode     = LaunchConfiguration('pub_mode', default='const')  # const|circle|yaw|zero

    pub_vx       = LaunchConfiguration('pub_vx', default='0.05')
    pub_vy       = LaunchConfiguration('pub_vy', default='0.0')
    pub_vz       = LaunchConfiguration('pub_vz', default='0.0')
    pub_wx       = LaunchConfiguration('pub_wx', default='0.0')
    pub_wy       = LaunchConfiguration('pub_wy', default='0.0')
    pub_wz       = LaunchConfiguration('pub_wz', default='0.0')

    pub_vxy      = LaunchConfiguration('pub_vxy', default='0.05')
    pub_freq     = LaunchConfiguration('pub_freq', default='0.10')
    pub_wz_amp   = LaunchConfiguration('pub_wz_amp', default='0.6')

    # Controllers YAML (must include ee_twist_velocity_controller config)
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
    # Controllers: JSB -> impedance -> EE velocity controller
    # -----------------------------
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--activate', '-c', '/controller_manager'],
        output='screen'
    )

    imp_name = 'joint_impedance_controller'
    ee_vel_name = 'ee_twist_velocity_controller'

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

    ee_vel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[ee_vel_name, '--activate', '-c', '/controller_manager'],
        output='screen'
    )

    chain_imp_after_jsb_chained = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[imp_spawner_chained])
    )
    chain_imp_after_jsb_normal = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[imp_spawner_normal])
    )
    chain_ee_vel_after_imp_chained = RegisterEventHandler(
        OnProcessExit(target_action=imp_spawner_chained, on_exit=[ee_vel_spawner])
    )
    chain_ee_vel_after_imp_normal = RegisterEventHandler(
        OnProcessExit(target_action=imp_spawner_normal, on_exit=[ee_vel_spawner])
    )

    # -----------------------------
    # Python Twist publisher (start AFTER EE vel controller is active)
    # -----------------------------
    # Change package/executable if you put the script elsewhere:
    #   package='ombot_coordination', executable='ee_twist_cmd_publisher'
    twist_pub_node = Node(
        package='ombot_coordination',
        executable='ee_twist_cmd_publisher',
        name='ee_twist_cmd_publisher',
        output='screen',
        parameters=[{
            'topic': pub_topic,
            'frame_id': pub_frame,
            'rate_hz': pub_rate_hz,
            'mode': pub_mode,
            'vx': pub_vx, 'vy': pub_vy, 'vz': pub_vz,
            'wx': pub_wx, 'wy': pub_wy, 'wz': pub_wz,
            'vxy': pub_vxy,
            'freq': pub_freq,
            'wz_amp': pub_wz_amp,
        }],
        condition=IfCondition(run_pub)
    )

    start_pub_after_ee_vel = RegisterEventHandler(
        OnProcessExit(target_action=ee_vel_spawner, on_exit=[twist_pub_node])
    )

    # -----------------------------
    # RViz (optional) — start AFTER EE vel controller is active
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
    start_rviz_after_ee_vel = RegisterEventHandler(
        OnProcessExit(target_action=ee_vel_spawner, on_exit=[rviz_node])
    )

    # -----------------------------
    # rosbag2 recorder (optional) — start AFTER EE vel controller is active
    # -----------------------------
    topics_to_record = [
        '/joint_states',
        pub_topic,  # the twist command topic
        # add more if you want:
        # '/tf', '/tf_static',
        # '/ee_pose',
    ]

    bag_cmd = [
        'ros2', 'bag', 'record', *topics_to_record,
        '--output', bag_prefix,
        '--storage', storage,
        '--max-bag-size', split_size,
        '--max-bag-duration', split_secs,
        '--qos-profile-overrides-path', qos_path
    ]

    bag_record = ExecuteProcess(cmd=bag_cmd, output='screen', condition=IfCondition(record_bag))
    start_bag_after_ee_vel = RegisterEventHandler(
        OnProcessExit(target_action=ee_vel_spawner, on_exit=[bag_record])
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('chained', default_value='false'),
        DeclareLaunchArgument('controllers_yaml', default_value=PathJoinSubstitution([
            get_package_share_directory('ombot_bringup'),
            'config', 'ombot_controller.yaml'
        ])),

        DeclareLaunchArgument('run_pub', default_value='true'),
        DeclareLaunchArgument('pub_topic', default_value='/ee_twist_cmd'),
        DeclareLaunchArgument('pub_frame', default_value='base_link'),
        DeclareLaunchArgument('pub_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('pub_mode', default_value='const'),

        DeclareLaunchArgument('pub_vx', default_value='-0.05'),
        DeclareLaunchArgument('pub_vy', default_value='0.0'),
        DeclareLaunchArgument('pub_vz', default_value='0.0'),
        DeclareLaunchArgument('pub_wx', default_value='0.0'),
        DeclareLaunchArgument('pub_wy', default_value='0.0'),
        DeclareLaunchArgument('pub_wz', default_value='0.0'),
        DeclareLaunchArgument('pub_vxy', default_value='0.05'),
        DeclareLaunchArgument('pub_freq', default_value='0.10'),
        DeclareLaunchArgument('pub_wz_amp', default_value='0.6'),

        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('bag_prefix', default_value='ee_vel_run1'),
        DeclareLaunchArgument('storage', default_value='sqlite3'),
        DeclareLaunchArgument('qos_overrides',
                              default_value='/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml'),
        DeclareLaunchArgument('max_bag_size', default_value=str(1024*1024*1024)),
        DeclareLaunchArgument('max_bag_secs', default_value='600'),

        # Core nodes
        rsp_node,
        control_node,

        # Controllers
        jsb,
        chain_imp_after_jsb_chained,
        chain_imp_after_jsb_normal,
        chain_ee_vel_after_imp_chained,
        chain_ee_vel_after_imp_normal,

        # Start pub + bag + RViz after EE vel controller is active
        start_pub_after_ee_vel,
        start_bag_after_ee_vel,
        start_rviz_after_ee_vel,

        end_when_control_exits,
    ])