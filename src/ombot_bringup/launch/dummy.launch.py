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

    # Spawners
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

    # Arm+Base coordinator — now drives BOTH base and arm with internal XYZ offset goal
    coordinator = Node(
        package='ombot_coordination', executable='arm_base_coordinator',
        name='arm_base_coordinator',
        parameters=[
            # Frames & inputs
            {'inputs_in_world': True},                   # OptiTrack/world frame
            {'world_frame_name': 'world'},
            {'base_pose_topic': '/vrpn_mocap/RigidBody_1/pose'},
            {'ee_pose_topic':   '/vrpn_mocap/RigidBody_2/pose'},

            # Internal offset-goal (no /goal_pose needed)
            {'use_offset_goal': True},
            {'offset_frame': 'base'},                    # 'base' (relative to base heading) or 'world'
            {'offset_xyz': [0.5, 0.0, 0.0]},            # change offset here (x,y,z) meters

            # Command topics
            {'ee_twist_topic': '/resolved_rate_controller/ee_twist'},  # arm twist
            {'cmd_vel_topic':  '/cmd_vel'},                             # base twist

            # Base model
            {'base_is_holonomic': False},               # True if fully holonomic; False for diff-drive
            {'k_heading': 1.5},

            # Gains & limits
            {'kp_lin': 1.2}, {'kd_lin': 0.3},
            {'kp_ang': 1.0}, {'kd_ang': 0.25},
            {'k_ori_weight': 0.5},
            {'ee_lin_limit': 0.15}, {'ee_ang_limit': 0.6},
            {'base_lin_limit': 0.30}, {'base_ang_limit': 0.80},

            # Blending & retract
            {'blend_mid_distance': 0.8}, {'blend_slope': 6.0},
            {'max_reach': 0.8},
            {'d_retract_enter': 0.25}, {'d_retract_exit': 0.35},

            # Filtering/slew
            {'vel_lpf_alpha': 0.6},
            {'slew_base': 1.0},
            {'slew_arm_lin': 0.5},
            {'slew_arm_ang': 1.5},

            # Mocap marker → base (adjust if needed; keep π flips in params, not code)
            {'base_marker_offset_xyz': [0.0, 0.0, 0.0]},
            {'base_marker_offset_rpy': [0.0, 0.0, 0.0]},
        ],
        output='screen'
    )

    # Start coordinator a bit after controllers come up
    start_after_ctrl = TimerAction(period=2.0, actions=[coordinator])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rsp, control, jsb, imp, rr, mec,
        start_after_ctrl
    ])
