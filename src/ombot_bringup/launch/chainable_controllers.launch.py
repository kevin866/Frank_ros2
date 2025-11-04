from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="false", description="Start RViz2."),
        DeclareLaunchArgument(
            "controllers_yaml",
            default_value=PathJoinSubstitution([FindPackageShare("ombot_bringup"), "config", "ombot_controller.yaml"]),
            description="YAML containing resolved_rate_controller and joint_impedance_controller",
        ),
        DeclareLaunchArgument(
            "chained",
            default_value="false",
            description="If true, activate impedance controller as 'chained'.",
        ),
    ]

    gui = LaunchConfiguration("gui")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    chained = LaunchConfiguration("chained")

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("ombot_description"), "urdf", "arm", "ombot_arm.urdf.xacro"]),
            " ", "robot_name:=ombot"
        ]),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="both",
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--activate"],
        output="screen",
    )

    rr_name = "resolved_rate_controller"
    imp_name = "joint_impedance_controller"

    # Impedance spawner in two variants
    imp_spawner_chained = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[imp_name, "-c", "/controller_manager", "--activate-as", "chained"],
        output="screen",
        condition=IfCondition(chained),
    )
    imp_spawner_normal = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[imp_name, "-c", "/controller_manager", "--activate"],
        output="screen",
        condition=UnlessCondition(chained),
    )

    rr_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[rr_name, "-c", "/controller_manager", "--activate"],
        output="screen",
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("ombot_description"), "ombot", "rviz", "view_robot.rviz"
    ])
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_config], condition=IfCondition(gui), output="screen"
    )

    # Sequence: JSB → Impedance → RR → RViz
    after_jsb_start_imp_chained = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[imp_spawner_chained])
    )
    after_jsb_start_imp_normal = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[imp_spawner_normal])
    )
    after_imp_chained_start_rr = RegisterEventHandler(
        OnProcessExit(target_action=imp_spawner_chained, on_exit=[rr_spawner])
    )
    after_imp_normal_start_rr = RegisterEventHandler(
        OnProcessExit(target_action=imp_spawner_normal, on_exit=[rr_spawner])
    )
    after_rr_start_rviz = RegisterEventHandler(
        OnProcessExit(target_action=rr_spawner, on_exit=[rviz_node])
    )

    nodes = [
        control_node,
        rsp_node,
        jsb_spawner,
        after_jsb_start_imp_chained,
        after_jsb_start_imp_normal,
        after_imp_chained_start_rr,
        after_imp_normal_start_rr,
        after_rr_start_rviz,
    ]
    return LaunchDescription(declared_arguments + nodes)