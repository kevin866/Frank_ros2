# bringup_qp_moveaway_zed.launch.py

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess,
    RegisterEventHandler, Shutdown, IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Common args ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    bag_prefix   = LaunchConfiguration('bag_prefix',   default='ombot_run1')
    storage      = LaunchConfiguration('storage',      default='sqlite3')
    qos_path     = LaunchConfiguration(
        'qos_overrides',
        default='/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml'
    )
    split_size   = LaunchConfiguration('max_bag_size', default=str(1024*1024*1024))  # 1 GiB
    split_secs   = LaunchConfiguration('max_bag_secs', default='600')                # 10 min

    # ZED + moveaway args
    launch_zed   = LaunchConfiguration('launch_zed', default='true')
    depth_topic  = LaunchConfiguration('depth_topic', default='/zed/zed_node/depth/depth_registered')
    cmd_topic    = LaunchConfiguration('cmd_topic',   default='/wb_cmd')

    trigger_dist = LaunchConfiguration('trigger_dist', default='0.10')
    clear_dist   = LaunchConfiguration('clear_dist',   default='0.30')
    v_back       = LaunchConfiguration('v_back',       default='0.10')

    publish_rate = LaunchConfiguration('publish_rate', default='50.0')
    base_frame   = LaunchConfiguration('base_frame',   default='base_link')

    # NOTE: keep roi fixed here to avoid DOUBLE_ARRAY typing issues from launch substitutions
    roi = [0.40, 0.60, 0.55, 0.80]

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

    # --- ZED launch (your exact style) ---
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"])
        ]),
        launch_arguments={
            "camera_model": "zed2",
            "camera_name": "zed",
            "publish_tf": "false",
            "publish_map_tf": "false",
            "enable_depth": "true",
            "enable_point_cloud": "true",
        }.items(),
        condition=IfCondition(launch_zed),
    )

    optitrack_tf = Node(
        package="ombot_coordination",
        executable="optitrack_tf_pub",
        output="screen",
        parameters=[{
            "pose_topic": "/vrpn_mocap/RigidBody_1/pose",
            "world_frame": "world",
            "base_frame": "base_link",
            "use_planar": True,
            "axis_fix": "xzy",
            "yaw_offset": 0.0,
            "alpha_pos": 0.2,
            "alpha_yaw": 0.2,
        }]
    )

    goal_commander = Node(
        package="ombot_coordination",
        executable="goal_commander",
        output="screen",
        parameters=[{
            "world_frame": "world",
            "topic": "/goal_pose",
            "rate_hz": 5.0,
            'goal': "1.2 1.0 0.5 0.0",
        }]
    )

    # ---------------- Moveaway CMD publisher ----------------
    # This is your installed console_script: depth_move_away_cmd_publisher
    depth_moveaway_cmd = Node(
        package="ombot_coordination",
        executable="depth_move_away_cmd_publisher",
        name="depth_move_away_cmd_publisher",
        output="screen",
        parameters=[{
            "cmd_topic": cmd_topic,
            "base_frame": base_frame,
            "depth_topic": depth_topic,
            "publish_rate": publish_rate,
            "trigger_dist": trigger_dist,
            "clear_dist": clear_dist,
            "v_back": v_back,
            "roi": roi,
        }],
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

    wb_qp = Node(
        package='controller_manager', executable='spawner',
        arguments=['wb_qp_controller', '--activate', '-c', '/controller_manager'],
        parameters=[ctrl_yaml],
        output='screen'
    )

    mecanum_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['mecanum_controller', '--activate', '-c', '/controller_manager'],
        output='screen'
    )

    # Chain: JSB -> Impedance -> WB QP
    chain_imp_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[imp])
    )
    chain_wb_after_imp = RegisterEventHandler(
        OnProcessExit(target_action=imp, on_exit=[wb_qp])
    )

    # Start moveaway cmd publisher only after wb_qp is active
    start_moveaway_after_wb = RegisterEventHandler(
        OnProcessExit(target_action=wb_qp, on_exit=[depth_moveaway_cmd])
    )

    # --- rosbag2 recorder ---
    topics_to_record = [
        '/mecanum_controller/reference',
        '/wb_cmd',
        '/joint_states',
        '/ee_pose',
        '/vrpn_mocap/RigidBody_1/pose',

        # Depth topics for debugging/demo
        '/zed/zed_node/depth/depth_registered',
    ]

    bag_cmd_final = [
        'ros2', 'bag', 'record', *topics_to_record,
        '--output', bag_prefix,
        '--storage', storage,
        '--max-bag-size', split_size,
        '--max-bag-duration', split_secs,
        '--qos-profile-overrides-path', qos_path
    ]

    bag_record = ExecuteProcess(cmd=bag_cmd_final, output='screen')

    # Start bag when whole-body controller is active
    start_bag_after_wb = RegisterEventHandler(
        OnProcessExit(target_action=wb_qp, on_exit=[bag_record])
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
        DeclareLaunchArgument(
            'qos_overrides',
            default_value='/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml'
        ),
        DeclareLaunchArgument('max_bag_size',  default_value=str(1024*1024*1024)),
        DeclareLaunchArgument('max_bag_secs',  default_value='600'),

        DeclareLaunchArgument('launch_zed',    default_value='true'),
        DeclareLaunchArgument('depth_topic',   default_value='/zed/zed_node/depth/depth_registered'),
        DeclareLaunchArgument('cmd_topic',     default_value='/wb_cmd'),
        DeclareLaunchArgument('publish_rate',  default_value='50.0'),
        DeclareLaunchArgument('base_frame',    default_value='base_link'),
        DeclareLaunchArgument('trigger_dist',  default_value='0.40'),
        DeclareLaunchArgument('clear_dist',    default_value='0.60'),
        DeclareLaunchArgument('v_back',        default_value='0.1'),

        # Core nodes
        robot_state_publisher,
        control_node,
        optitrack_tf,
        # goal_commander,


        # Camera
        zed_launch,


        # Controllers
        mecanum_spawner,
        jsb,
        chain_imp_after_jsb,
        chain_wb_after_imp,

        # Moveaway + bag after WB QP
        start_moveaway_after_wb,
        start_bag_after_wb,

        end_when_control_exits,
    ])
