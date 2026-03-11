# bringup_wb_with_bag_vel.launch.py
#
# Same as bringup_wb_with_bag.launch.py but uses ee_twist_velocity_controller
# (joint velocity control) instead of resolved_rate_controller (torque control).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown
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

    # CHANGED: use ee_twist_velocity_controller (velocity interface)
    ee_vel = Node(
        package='controller_manager', executable='spawner',
        arguments=['ee_twist_velocity_controller', '--activate', '-c', '/controller_manager'],
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
            'base_pose_topic': '/vrpn_mocap/RigidBody_2/pose',
            'goal_pose_topic': '/goal_pose',
            'offset_xyz': [1.8, 0.0, 0.0],
            # 'home_position': [0.246, 0.00, 0.485],
            'home_position': [0.0, 0.00, 0.0],
            'latch': True,
        }]
    )

    # Chain: JSB -> Impedance -> EE_Vel
    chain_imp_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[imp])
    )
    chain_ee_vel_after_imp = RegisterEventHandler(
        OnProcessExit(target_action=imp, on_exit=[ee_vel])
    )

    # --- Split commander (Python) ---
    # CHANGED: publish EE twist to the topic your ee_twist_velocity_controller subscribes to.
    # In my publisher/controller defaults this is /ee_twist_cmd (TwistStamped).
    split_commander = Node(
        package='ombot_coordination',
        executable='split_commander',
        name='split_commander',
        output='screen',
        parameters=[{
            # Inputs
            'base_pose_topic': '/vrpn_mocap/RigidBody_1/pose',
            'ee_pose_topic':   '/ee_pose',
            'goal_pose_topic': '/goal_pose',

            # Outputs (CHANGED)
            'ee_twist_topic':  '/ee_twist_cmd',
            'base_cmd_topic':  '/mecanum_controller/reference',

            # Option B gains
            'k1': 1.5,
            'k3': 1.5,
            'k1d': 0.1,
            'k3d': 0.1,
            'k2': 0.0,
            'k2d': 0.0,

            'stow_point_b': [0.246, 0.00, 0.485],

            'kp_pos': 1.0,
            'kp_rot': 0.0,
            'kd_pos': 0.0,
            'kd_rot': 0.0,

            'max_lin': 0.15,
            'max_ang': 0.7,

            'max_base_lin': 0.5,
            'max_base_ang': 0.2,
        }]
    )

    # Start commander only after EE velocity controller is active
    start_commander_after_ee = RegisterEventHandler(
        OnProcessExit(target_action=ee_vel, on_exit=[split_commander])
    )

    # --- rosbag2 recorder ---
    topics_to_record = [
        '/mecanum_controller/reference',
        '/ee_twist_cmd',                    # CHANGED
        '/vrpn_mocap/RigidBody_1/pose',
        '/vrpn_mocap/RigidBody_2/pose',
        '/goal_pose',
        '/joint_states',
        '/ee_pose',
        '/debug/e1',
        '/debug/e2',
        '/debug/e3'
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

    # Start bag after EE velocity controller is active
    start_bag_after_ee = RegisterEventHandler(
        OnProcessExit(target_action=ee_vel, on_exit=[bag_record])
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

        # Core nodes
        robot_state_publisher,
        control_node,

        # Controllers
        mecanum_spawner,
        jsb,
        chain_imp_after_jsb,
        chain_ee_vel_after_imp,

        goal_from_offset,

        # Start commander + bag once EE velocity controller is active
        start_commander_after_ee,
        start_bag_after_ee,

        end_when_control_exits,
    ])