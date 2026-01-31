# bringup_qp_moveaway_goto_zed.launch.py

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

    # ZED + cmd args
    launch_zed   = LaunchConfiguration('launch_zed', default='true')
    depth_topic  = LaunchConfiguration('depth_topic', default='/zed/zed_node/depth/depth_registered')
    cmd_topic    = LaunchConfiguration('cmd_topic',   default='/wb_cmd')

    publish_rate = LaunchConfiguration('publish_rate', default='50.0')

    # Frames for TF + cmd
    base_frame   = LaunchConfiguration('base_frame', default='base_link')
    world_frame  = LaunchConfiguration('world_frame', default='world')  # set to 'odom' if no optitrack

    # Obstacle thresholds
    trigger_dist = LaunchConfiguration('trigger_dist', default='0.40')
    clear_dist   = LaunchConfiguration('clear_dist',   default='0.60')
    v_back       = LaunchConfiguration('v_back',       default='0.10')

    # Depth validity / robustness
    min_valid_depth   = LaunchConfiguration('min_valid_depth', default='0.35')
    max_valid_depth   = LaunchConfiguration('max_valid_depth', default='5.0')
    depth_percentile  = LaunchConfiguration('depth_percentile', default='10.0')

    # Return tolerances
    return_tol_xy   = LaunchConfiguration('return_tol_xy', default='0.03')
    return_tol_yaw  = LaunchConfiguration('return_tol_yaw', default='0.08')

    # Go-to goal
    goal_x   = LaunchConfiguration('goal_x', default='1.0')
    goal_y   = LaunchConfiguration('goal_y', default='0.0')
    goal_yaw = LaunchConfiguration('goal_yaw', default='0.0')

    # Go-to gains/limits
    kp_xy = LaunchConfiguration('kp_xy', default='1.0')
    kp_yaw = LaunchConfiguration('kp_yaw', default='1.0')
    max_v = LaunchConfiguration('max_v', default='0.3')
    max_w = LaunchConfiguration('max_w', default='1.0')

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

    # --- ZED launch (same as yours) ---
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

    # --- OptiTrack TF (provides world->base_link) ---
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

    # ---------------- MoveAway + GoTo CMD publisher ----------------
    # NOTE: Update executable to match your installed console_script name.
    # Example: "moveaway_goto_cmd" or "depth_move_away_goto_cmd_publisher" etc.
    moveaway_goto_cmd = Node(
        package="ombot_coordination",
        executable="moveaway_goto_cmd",  # <-- CHANGE if your entry point name is different
        name="moveaway_goto_cmd",
        output="screen",
        parameters=[{
            "cmd_topic": cmd_topic,
            "depth_topic": depth_topic,
            "publish_rate": publish_rate,

            "world_frame": world_frame,
            "base_frame": base_frame,

            "roi": roi,
            "min_valid_depth": min_valid_depth,
            "max_valid_depth": max_valid_depth,
            "depth_percentile": depth_percentile,

            "trigger_dist": trigger_dist,
            "clear_dist": clear_dist,
            "v_back": v_back,

            "return_tol_xy": return_tol_xy,
            "return_tol_yaw": return_tol_yaw,

            "goal_x": goal_x,
            "goal_y": goal_y,
            "goal_yaw": goal_yaw,

            "kp_xy": kp_xy,
            "kp_yaw": kp_yaw,
            "max_v": max_v,
            "max_w": max_w,
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

    # Start cmd publisher only after wb_qp is active
    start_cmd_after_wb = RegisterEventHandler(
        OnProcessExit(target_action=wb_qp, on_exit=[moveaway_goto_cmd])
    )

    # --- rosbag2 recorder ---
    topics_to_record = [
        '/mecanum_controller/reference',
        '/wb_cmd',
        '/joint_states',
        '/ee_pose',
        "/vrpn_mocap/RigidBody_1/pose",

        # Depth topics for debugging/demo
        '/zed/zed_node/depth/depth_registered',
        '/zed/zed_node/point_cloud/cloud_registered',
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

        DeclareLaunchArgument('world_frame',   default_value='world'),
        DeclareLaunchArgument('base_frame',    default_value='base_link'),

        DeclareLaunchArgument('trigger_dist',  default_value='0.40'),
        DeclareLaunchArgument('clear_dist',    default_value='0.60'),
        DeclareLaunchArgument('v_back',        default_value='0.10'),

        DeclareLaunchArgument('min_valid_depth',  default_value='0.35'),
        DeclareLaunchArgument('max_valid_depth',  default_value='5.0'),
        DeclareLaunchArgument('depth_percentile', default_value='10.0'),

        DeclareLaunchArgument('return_tol_xy',  default_value='0.03'),
        DeclareLaunchArgument('return_tol_yaw', default_value='0.08'),

        DeclareLaunchArgument('goal_x',   default_value='1.5'),
        DeclareLaunchArgument('goal_y',   default_value='0.5'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('kd_xy',   default_value='0.0'),
        DeclareLaunchArgument('kd_yaw',  default_value='0.0'),

        DeclareLaunchArgument('kp_xy',  default_value='0.2'),
        DeclareLaunchArgument('kp_yaw', default_value='0.2'),
        DeclareLaunchArgument('max_v',  default_value='0.3'),
        DeclareLaunchArgument('max_w',  default_value='0.2'),

        DeclareLaunchArgument('k_reach',  default_value='1.1'),


        # Core nodes
        robot_state_publisher,
        control_node,

        # World TF
        optitrack_tf,

        # Camera
        zed_launch,

        # Controllers
        mecanum_spawner,
        jsb,
        chain_imp_after_jsb,
        chain_wb_after_imp,

        # MoveAway+GoTo + bag after WB QP
        start_cmd_after_wb,
        start_bag_after_wb,

        end_when_control_exits,
    ])
