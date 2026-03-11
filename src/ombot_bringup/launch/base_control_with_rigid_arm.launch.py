from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # -----------------------------
    # Launch arguments
    # -----------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    bag_prefix = LaunchConfiguration("bag_prefix")
    storage = LaunchConfiguration("storage")
    qos_path = LaunchConfiguration("qos_overrides")
    split_size = LaunchConfiguration("max_bag_size")
    split_secs = LaunchConfiguration("max_bag_secs")

    # -----------------------------
    # Full robot_description
    # -----------------------------
    urdf_file = PathJoinSubstitution([
        get_package_share_directory("ombot_description"),
        "urdf",
        "ombot.urdf.xacro",
    ])

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file,
        ]),
        value_type=str,
    )

    robot_description = {
        "robot_description": robot_description_content,
        "use_sim_time": use_sim_time,
    }

    # -----------------------------
    # Controller YAML
    # -----------------------------
    ctrl_yaml = PathJoinSubstitution([
        get_package_share_directory("ombot_bringup"),
        "config",
        "ombot_controller.yaml",
    ])

    # -----------------------------
    # Optional RViz config
    # -----------------------------
    rviz_config_file = PathJoinSubstitution([
        get_package_share_directory("ombot_description"),
        "ombot",
        "rviz",
        "view_robot.rviz",
    ])

    # -----------------------------
    # Core nodes
    # -----------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ctrl_yaml],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="log",
        condition=IfCondition(gui),
    )

    # -----------------------------
    # Controller spawners
    # -----------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--activate",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    mecanum_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_controller",
            "--activate",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    cartesian_impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cartesian_impedance_controller",
            "--activate",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # -----------------------------
    # Coordination / test nodes
    # -----------------------------
    goal_from_offset = Node(
        package="ombot_coordination",
        executable="goal_from_base_offset_latched",
        name="goal_from_offset",
        parameters=[{
            "publish_rate_hz": 20.0,
            "mode": "latch",
            "offset_xyz": [0.0, 0.5, 0.0],
        }],
        remappings=[
            ("/goal_pose", "/goal_pose"),
        ],
        output="screen",
        respawn=True,
    )

    base_p_on_x = Node(
        package="ombot_coordination",
        executable="base_p_on_x",
        name="base_p_on_x",
        parameters=[{
            "kx": 0.1,
            "vmax": 0.1,
            "flip_forward": False,
        }],
        output="screen",
    )

    arm_zero_twist = Node(
        package="ombot_coordination",
        executable="arm_zero_twist",
        name="arm_zero_twist",
        output="screen",
    )

    # -----------------------------
    # rosbag recorder
    # -----------------------------
    topics_to_record = [
        "/mecanum_controller/reference",
        "/goal_pose",
        "/joint_states",
        "/ee_pose",
        "/vrpn_mocap/RigidBody_1/pose",
        "/vrpn_mocap/RigidBody_2/pose",
        "/vrpn_mocap/RigidBody_3/pose",
    ]

    # Add command topics here only if your Cartesian impedance controller
    # actually uses them. Uncomment / adjust as needed:
    # topics_to_record += [
    #     "/ee_twist_cmd",
    #     "/cartesian_impedance_controller/target_pose",
    # ]

    bag_cmd = [
        "ros2", "bag", "record",
        *topics_to_record,
        "--output", bag_prefix,
        "--storage", storage,
        "--max-bag-size", split_size,
        "--max-bag-duration", split_secs,
        "--qos-profile-overrides-path", qos_path,
    ]

    bag_record = ExecuteProcess(
        cmd=bag_cmd,
        output="screen",
    )

    # -----------------------------
    # Startup sequencing
    # -----------------------------
    # JSB -> mecanum
    delay_mecanum_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_spawner],
        )
    )

    # mecanum -> Cartesian impedance
    delay_cartesian_after_mecanum = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mecanum_spawner,
            on_exit=[cartesian_impedance_controller_spawner],
        )
    )

    # Optional RViz after JSB
    delay_rviz_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Start coordination nodes after Cartesian impedance is active
    start_tests_after_cartesian = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=cartesian_impedance_controller_spawner,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        goal_from_offset,
                        base_p_on_x,
                        arm_zero_twist,
                    ],
                )
            ],
        )
    )

    # Start rosbag after Cartesian impedance is active
    start_bag_after_cartesian = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=cartesian_impedance_controller_spawner,
            on_exit=[bag_record],
        )
    )

    # Shutdown launch if ros2_control exits
    end_when_control_exits = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[Shutdown(reason="ros2_control_node exited")],
        )
    )

    # -----------------------------
    # Launch description
    # -----------------------------
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically.",
        ),
        DeclareLaunchArgument(
            "bag_prefix",
            default_value="ombot_cartesian_run1",
            description="Output rosbag folder prefix.",
        ),
        DeclareLaunchArgument(
            "storage",
            default_value="sqlite3",
            description="Rosbag storage backend.",
        ),
        DeclareLaunchArgument(
            "qos_overrides",
            default_value="/home/frank/frank_ws/src/ombot_bringup/config/qos.yaml",
            description="Path to rosbag QoS override YAML.",
        ),
        DeclareLaunchArgument(
            "max_bag_size",
            default_value=str(1024 * 1024 * 1024),
            description="Split bag when size exceeds this many bytes.",
        ),
        DeclareLaunchArgument(
            "max_bag_secs",
            default_value="600",
            description="Split bag when duration exceeds this many seconds.",
        ),

        robot_state_publisher,
        control_node,

        joint_state_broadcaster_spawner,
        delay_mecanum_after_jsb,
        delay_cartesian_after_mecanum,
        delay_rviz_after_jsb,

        start_tests_after_cartesian,
        start_bag_after_cartesian,

        end_when_control_exits,
    ])