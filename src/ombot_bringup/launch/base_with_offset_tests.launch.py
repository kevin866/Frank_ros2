from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution,
    Command, FindExecutable
)
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # --- Common arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # --- Build robot_description from URDF/Xacro ---
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('ombot_description'),
        'urdf', 'ombot.urdf.xacro'
    ])
    robot_description_content = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf_file]),
        value_type=str
    )

    # --- Nodes: robot_state_publisher + controller manager ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }],
        output='screen'
    )

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

    mecanum_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_controller', '-c', '/controller_manager'],
        output='screen'
    )

    # --- Coordination test nodes (from ombot_coordination) ---
    goal_from_offset = Node(
        package='ombot_coordination',
        executable='goal_from_base_offset_latched',   # <-- match your setup.py entry point
        name='goal_from_offset',
        parameters=[{
            'offset_x': -0.5,            # meters
            'publish_rate_hz': 20.0,     # republish goal with fresh timestamps
            'mode': 'latch'              # default behavior (one-shot latch)
        }],
        # RigidBody_1 is the base mocap topic, RigidBody_2 would be the arm if needed.
        remappings=[
            ('/vrpn_mocap/RigidBody_1/pose', '/vrpn_mocap/RigidBody_2/pose'),  # base
            ('/goal_pose', '/goal_pose'),
        ],
        output='screen',
        respawn=True                     # optional: auto-restart if it crashes
    )

    base_p_on_x = Node(
        package='ombot_coordination',
        executable='base_p_on_x',
        parameters=[{'kx': 3.0, 'vmax': 5.0, 'flip_forward': False}],
        # no remap for cmd_vel anymore
        output='screen'
    )

    arm_zero_twist = Node(
        package='ombot_coordination',
        executable='arm_zero_twist',
        name='arm_zero_twist',
        output='screen'
    )

    # Wait ~2 seconds to ensure the mecanum controller is alive before sending commands
    start_tests = TimerAction(
        period=2.0,
        actions=[goal_from_offset, base_p_on_x, arm_zero_twist]
    )

    # --- Launch description ---
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        robot_state_publisher,
        control_node,
        mecanum_spawner,
        start_tests
    ])

