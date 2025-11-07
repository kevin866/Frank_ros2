from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Combined URDF (base + arm)
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('ombot_description'), 'urdf', 'ombot.urdf.xacro'
    ])
    robot_description_content = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf_file]),
        value_type=str
    )

    # Core bringup
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}],
        output='screen'
    )
    ctrl_yaml = PathJoinSubstitution([
        get_package_share_directory('ombot_bringup'), 'config', 'ombot_controller.yaml'
    ])
    control = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content}, ctrl_yaml],
        output='screen'
    )

    # Spawners (one controller manager for ALL controllers)
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster', '-c', '/controller_manager', '--activate'],
               output='screen')

    imp = Node(package='controller_manager', executable='spawner',
               arguments=['joint_impedance_controller', '-c', '/controller_manager', '--activate'],
               output='screen')

    rr  = Node(package='controller_manager', executable='spawner',
               arguments=['resolved_rate_controller', '-c', '/controller_manager', '--activate'],
               output='screen')

    mec = Node(package='controller_manager', executable='spawner',
               arguments=['mecanum_controller', '-c', '/controller_manager'],
               output='screen')

    # World-frame goal: latch base.x + offset once
    goal_from_offset = Node(
        package='ombot_coordination', executable='goal_from_base_offset_latched',
        name='goal_from_offset',
        parameters=[{'offset_x': -1.0}],   # change to -5.0 etc.
        remappings=[
            ('/vrpn_mocap/RigidBody_1/pose', '/vrpn_mocap/RigidBody_1/pose'),
            ('/goal_pose', '/goal_pose'),
        ],
        output='screen'
    )

#     # Base-only P controller (already publishes TwistStamped to /mecanum_controller/reference)
#     base_p_on_x = Node(
#         package='ombot_coordination', executable='base_p_on_x',
#         name='base_p_on_x',
#         parameters=[{'kx': 0.8, 'vmax': 0.6, 'tol': 0.03, 'min_v': 0.12, 'flip_forward': False}],
#         output='screen'
#     )

    base_p_on_x = Node(
        package='ombot_coordination',
        executable='base_p_on_x',
        parameters=[{'kx': 5.0, 'vmax': 8.0, 'flip_forward': False}],
        # no remap for cmd_vel anymore
        output='screen'
    )

    # Arm+Base coordinator — we use it for the ARM ONLY here.
    # Remap its /cmd_vel to a dummy topic so it won't fight base_p_on_x.
    coordinator = Node(
        package='ombot_coordination', executable='arm_base_coordinator',
        name='arm_base_coordinator',
        parameters=[
            {'inputs_in_world': True},
            {'world_frame_name': 'world'},
            {'base_pose_topic': '/vrpn_mocap/RigidBody_1/pose'},
            {'ee_pose_topic':   '/vrpn_mocap/RigidBody_2/pose'},
            {'goal_pose_topic': '/goal_pose'},

            # Let coordinator compute correctly for holonomic base,
            # but its base output is remapped away below.
            {'base_is_holonomic': True},

            # Arm twist topic (what your resolved-rate controller reads)
            {'ee_twist_topic': '/resolved_rate_controller/ee_twist'},

            # Limits & gains (safe starters)
            {'kp_lin': 1.2}, {'kd_lin': 0.3},
            {'kp_ang': 1.0}, {'kd_ang': 0.25},
            {'ee_lin_limit': 0.15}, {'ee_ang_limit': 0.6},

            # Blending still runs (alpha), but base command is discarded by remap.
            {'blend_mid_distance': 0.8}, {'blend_slope': 6.0},
            {'d_retract_enter': 0.25}, {'d_retract_exit': 0.35},
        ],
        remappings=[
            # IMPORTANT: dump base cmd so this node does not drive the base now
            ('/cmd_vel', '/arm_base_coordinator/cmd_vel_dump'),
        ],
        output='screen'
    )

    start_after_ctrl = TimerAction(
        period=2.0,
        actions=[goal_from_offset, base_p_on_x, coordinator]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rsp, control, jsb, imp, rr, mec,
        start_after_ctrl
    ])
