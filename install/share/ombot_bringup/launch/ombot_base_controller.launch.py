from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue




def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('ombot_description'),
        'urdf', 'ombot.urdf.xacro'
    ])

    # Build the URDF string from xacro, force it to be a string param
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            urdf_file,
            # ' ', 'robot_name:=ombot',  # <- add xacro args here if needed
        ]),
        value_type=str
    )

    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }],
        output='screen'
    )


    # ros2_control node
    ctrl_yaml = PathJoinSubstitution([get_package_share_directory('ombot_bringup'), 'config', 'ombot_controller.yaml'])
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'use_sim_time': use_sim_time},
                    {'robot_description': robot_description_content},
                    ctrl_yaml],
        output='screen'
    )

    # Spawners
    jsb_spawner = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', '-c', '/controller_manager'])

    vel_spawner = Node(package='controller_manager', executable='spawner',
                       arguments=['wheel_velocity_controller', '-c', '/controller_manager'])

    # Kinematics/Odom
    kinod = Node(
        package='ombot_base_kinematics',
        executable='mecanum_kinod',
        parameters=[{
            'wheel_radius': 0.0762,
            'Lx': 0.20, 'Ly': 0.17,
            'cmd_topic': '/wheel_velocity_controller/commands',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True
        }],
        remappings=[
            # If you use twist_mux or different cmd topic, remap here.
            ('cmd_vel', '/cmd_vel')
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        robot_description,
        control_node,
        jsb_spawner,
        vel_spawner,
        kinod
    ])
