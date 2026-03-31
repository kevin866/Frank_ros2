from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ),
        launch_arguments={
            'camera_model': 'zed2',
        }.items()
    )

    face_detector_node = Node(
        package='frank_vision',
        executable='face_detector_node',
        output='screen',
        parameters=[{
            'image_topic': '/zed/zed_node/rgb/image_rect_color',
            'display_debug': True,
            'score_threshold': 0.75,
            'nms_threshold': 0.3,
            'top_k': 5000,
        }]
    )

    return LaunchDescription([
        zed_camera_launch,
        face_detector_node,
    ])