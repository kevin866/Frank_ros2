from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── launch arguments ──────────────────────────────────────────────────────
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/zed/zed_node/rgb/image_rect_color',
        description='ZED image topic for face detector'
    )
    display_debug_arg = DeclareLaunchArgument(
        'display_debug',
        default_value='true',
        description='Publish annotated debug image from face detector'
    )
    model_size_arg = DeclareLaunchArgument(
        'model_size',
        default_value='tiny.en',
        description='Whisper model size'
    )
    vad_aggressiveness_arg = DeclareLaunchArgument(
        'vad_aggressiveness',
        default_value='1',
        description='WebRTC VAD aggressiveness 0-3'
    )
    launch_zed_arg = DeclareLaunchArgument(
        'launch_zed',
        default_value='true',
        description='Launch ZED camera and robot hardware'
    )
    start_control_arg = DeclareLaunchArgument(
        'start_control',
        default_value='true',
        description='Start controller_manager'
    )
    start_jsb_arg = DeclareLaunchArgument(
        'start_jsb',
        default_value='true',
        description='Spawn joint_state_broadcaster'
    )
    launch_base_arg = DeclareLaunchArgument(
        'launch_base',
        default_value='true',
        description='Launch mecanum base controller'
    )

    # ── include robot_camera.launch.py (ZED + RSP + controller_manager) ───────
    robot_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ombot_bringup'), 'launch', 'robot_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'launch_zed':    LaunchConfiguration('launch_zed'),
            'start_control': LaunchConfiguration('start_control'),
            'start_jsb':     LaunchConfiguration('start_jsb'),
            'use_sim_time':  'false',
        }.items(),
    )

    # ── include base controller launch ────────────────────────────────────────
    base_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ombot_bringup'), 'launch', 'ombot_base_controller.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('launch_base')),
    )

    # ── frank nodes ───────────────────────────────────────────────────────────
    face_detector_node = Node(
        package='frank_vision',
        executable='face_detector_node',
        name='face_detector_node',
        output='screen',
        parameters=[{
            'image_topic':   LaunchConfiguration('image_topic'),
            'display_debug': LaunchConfiguration('display_debug'),
            'score_threshold': 0.85,
        }]
    )

    stt_node = Node(
        package='frank_audio',
        executable='stt_node',
        name='stt_node',
        output='screen',
        parameters=[{
            'model_size':         LaunchConfiguration('model_size'),
            'vad_aggressiveness': LaunchConfiguration('vad_aggressiveness'),
            'device':             'cpu',
            'mic_device_index':   0,
            'native_sample_rate': 44100,
        }]
    )

    behavior_manager_node = Node(
        package='frank_behaviors',
        executable='behavior_manager_node',
        name='behavior_manager_node',
        output='screen',
    )

    return LaunchDescription([
        # args
        image_topic_arg,
        display_debug_arg,
        model_size_arg,
        vad_aggressiveness_arg,
        launch_zed_arg,
        start_control_arg,
        start_jsb_arg,
        launch_base_arg,
        # hardware + camera
        robot_camera_launch,
        # base controller
        base_controller_launch,
        # perception + audio + behavior
        face_detector_node,
        stt_node,
        behavior_manager_node,
    ])