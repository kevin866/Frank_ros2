from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue  # <-- import this

def generate_launch_description():
    desc = get_package_share_directory('ombot_description')
    br   = get_package_share_directory('ombot_bringup')

    urdf = PathJoinSubstitution([desc, 'urdf', 'ombot.urdf.xacro'])

    # Build URDF string via xacro
    robot_description_content = Command([FindExecutable(name='xacro'), ' ', urdf])

    # Force it to be treated as a plain string, not YAML
    robot_description = ParameterValue(robot_description_content, value_type=str)

    controllers_yaml = PathJoinSubstitution([br, 'config', 'ombot_controller.yaml'])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     output='screen',
        #     parameters=[
        #         {'robot_description': robot_description},  
        #         controllers_yaml,
        #         {'validate_interfaces': False}
        #     ],
        # ),
        Node(
            package='controller_manager', 
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_yaml,
                {'validate_interfaces': False}
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['wheel_velocity_controller'],
            output='screen',
        ),
    ])
