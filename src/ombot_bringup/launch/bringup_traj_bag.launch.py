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


    dual_traj_generator = Node(
        package="ombot_coordination",
        executable="dual_traj_generator",
        name="dual_traj_generator",
        output="screen",
        parameters=[{
            "world_frame": "world",
            "base_frame": "base_link",
            "rate_hz": 100.0,
            "T_base": 2.0,
            "T_ee": 2.0,
            "Kp_track": 15.0,
            "Kyaw": 15.0,
            "Kd_track": 5.0,
            "Kd_yaw": 5.0,
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

    # wb_rr = Node(
    #     package='controller_manager', executable='spawner',
    #     arguments=['wb_resolved_rate_controller', '--activate', '-c', '/controller_manager'],
    #     parameters=[ctrl_yaml],
    #     output='screen'
    # )
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

    # goal_from_offset = Node(
    #     package='ombot_coordination',
    #     executable='goal_from_base_offset_latched',
    #     name='goal_from_base_offset_latched',
    #     output='screen',
    #     parameters=[{
    #         'base_pose_topic': '/vrpn_mocap/RigidBody_1/pose',
    #         'goal_pose_topic': '/goal_pose',
    #         'offset_xyz': [1.0, 0.0, 0.0],   # set your desired offset here (world frame)
    #         'latch': True,                   # True = latch once, False = follow base
    #     }]
    # )


    # Chain: JSB -> Impedance -> WholeBodyResolvedRate
    chain_imp_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[imp])
    )
    chain_wb_after_imp = RegisterEventHandler(
        OnProcessExit(target_action=imp, on_exit=[wb_qp])
    )

    # start_traj_after_goal = RegisterEventHandler(
    #     OnProcessExit(target_action=goal_commander, on_exit=[dual_traj_generator])
    # )


    # --- Whole-body task commander (Python) ---
    # This node just publishes desired EE twist in base_link frame
    wb_task_commander = Node(
        package="ombot_coordination",
        executable="whole_body_task_commander",
        name="whole_body_task_commander",
        output="screen",
        parameters=[{
            # Frames for TF lookup
            "world_frame": "world",
            "base_frame": "base_link",

            # Poses
            # IMPORTANT: /ee_pose should be in base_link (FK output). If it's in world, you'll need to transform it too.
            "ee_pose_topic":   "/ee_pose",
            "goal_pose_topic": "/goal_pose",

            # Optional trajectory refs (keep if you're using them)
            "use_traj": True,
            # (These are hardcoded in your code right now as /ee_desired_pose and /ee_desired_twist.
            #  If you later parameterize them, add them here.)

            # Twist out -> must match controller's "~ee_twist" topic expansion
            # "ee_twist_topic": "/wb_resolved_rate_controller/desired_twist",
            "wb_cmd_topic": "/wb_cmd",


            # Gains
            "kp_pos": 2.0,
            "kp_rot": 1.0,   # consider 0.0 initially until frames are verified
            "kd_pos": 0.2,
            "kd_rot": 0.1,

            # Velocity caps (real robot: start smaller)
            "max_lin": 3.5,  # m/s (suggested safer start than 1.0)
            "max_ang": 1.0,  # rad/s
        }],
    )


    # Start commander only after wb_resolved_rate_controller is active
    start_commander_after_wb = RegisterEventHandler(
        OnProcessExit(target_action=wb_qp, on_exit=[wb_task_commander])
    )

    # --- rosbag2 recorder ---
    # topics_to_record = [
    #     '/mecanum_controller/reference',         # base ref (TwistStamped)
    #     '/wb_resolved_rate_controller/ee_twist', # desired EE twist
    #     '/vrpn_mocap/RigidBody_1/pose',
    #     '/vrpn_mocap/RigidBody_2/pose',
    #     '/goal_pose',
    #     '/joint_states',
    #     '/ee_pose'
    # ]
    topics_to_record = [
        '/mecanum_controller/reference',
        '/wb_cmd',
        '/vrpn_mocap/RigidBody_1/pose',
        '/vrpn_mocap/RigidBody_2/pose',
        '/goal_pose',
        '/base_goal_pose',
        '/base_desired_twist',
        '/ee_desired_pose',
        '/ee_desired_twist',
        '/joint_states',
        '/ee_pose'
    ]



    bag_cmd_final = [
        'ros2', 'bag', 'record', *topics_to_record,
        '--output', LaunchConfiguration('bag_prefix'),
        '--storage', LaunchConfiguration('storage'),
        '--max-bag-size', LaunchConfiguration('max_bag_size'),
        '--max-bag-duration', LaunchConfiguration('max_bag_secs'),
        '--qos-profile-overrides-path', LaunchConfiguration('qos_overrides')
    ]


    bag_record = ExecuteProcess(cmd=bag_cmd_final, output='screen')

    # Start bag when whole-body controller is active (same time as commander)
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
        # DeclareLaunchArgument('compress',      default_value='zstd'),
        DeclareLaunchArgument(
            'qos_overrides',
            default_value='/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml'
        ),
        DeclareLaunchArgument('max_bag_size',  default_value=str(1024*1024*1024)),
        DeclareLaunchArgument('max_bag_secs',  default_value='600'),

        # Core nodes
        robot_state_publisher,
        control_node,

        optitrack_tf,
        goal_commander,
        dual_traj_generator,

        # Controllers: mecanum can start anytime; chain the arm controllers
        mecanum_spawner,
        jsb,
        chain_imp_after_jsb,
        chain_wb_after_imp,

        # goal_from_offset,

        # Start commander + bag once WB controller is active
        # start_commander_after_wb,
        start_bag_after_wb,

        end_when_control_exits,
    ])

