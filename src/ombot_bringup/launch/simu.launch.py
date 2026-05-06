from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            "launch_zed",
            default_value="true",
            description="Launch ZED2 camera.",
        ),
    ]

    # --- URDF via xacro ---
    base_urdf = PathJoinSubstitution([
        FindPackageShare("ombot_description"),
        "urdf",
        "ombot.urdf.xacro",
    ])
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            base_urdf,
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # --- RViz config ---
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ombot_description"), "ombot/rviz", "view_robot.rviz"]
    )

    # --- Nodes ---
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # --- ZED Camera ---
    launch_zed = LaunchConfiguration("launch_zed")
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2',
        }.items(),
        condition=IfCondition(launch_zed),
    )

    return LaunchDescription(declared_arguments + [
        robot_state_pub_node,
        joint_state_publisher_gui_node,
        rviz_node,
        zed_camera_launch,
    ])