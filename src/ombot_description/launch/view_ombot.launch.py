# view_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("start_rviz", default_value="false",
                              description="Start RViz2."),
        DeclareLaunchArgument("start_jsp_gui", default_value="false",
                              description="Start joint_state_publisher_gui."),
        DeclareLaunchArgument("description_pkg", default_value="ombot_description",
                              description="Package containing the URDF/xacro and meshes."),
        DeclareLaunchArgument("xacro_path", default_value="urdf/ombot.urdf.xacro",
                              description="Relative path to the xacro inside description_pkg."),
        DeclareLaunchArgument("rviz_rel_path", default_value="ombot/rviz/view_robot.rviz",
                              description="Relative path to RViz config inside description_pkg."),
    ]

    start_rviz = LaunchConfiguration("start_rviz")
    start_jsp_gui = LaunchConfiguration("start_jsp_gui")
    description_pkg = LaunchConfiguration("description_pkg")
    xacro_path = LaunchConfiguration("xacro_path")
    rviz_rel_path = LaunchConfiguration("rviz_rel_path")

    urdf_file = PathJoinSubstitution([FindPackageShare(description_pkg), xacro_path])
    robot_description_content = Command([FindExecutable(name="xacro"), " ", urdf_file])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_pkg), rviz_rel_path])

    # GUI sliders (only if requested)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(start_jsp_gui),
    )

    # Headless joint state publisher (publishes zeros by default)
    joint_state_publisher_headless_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{
            "rate": 50.0,
            "publish_default_positions": False  # <- key: publishes zero for each non-fixed joint from URDF
        }],
        condition=UnlessCondition(start_jsp_gui),  # run headless when GUI is off
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(declared_arguments + [
        joint_state_publisher_gui_node,
        joint_state_publisher_headless_node,
        robot_state_publisher_node,
        rviz_node,
    ])
