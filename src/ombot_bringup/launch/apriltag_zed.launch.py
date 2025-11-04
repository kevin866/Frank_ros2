# apriltag_zed.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            parameters=[{
                "tag_family": "tag36h11",
                "tag_size": 0.10,                 # meters, match your print
                "publish_tf": True,
                "camera_frame": "zed_left_camera_optical_frame",  # IMPORTANT
            }],
            remappings=[
                ("image", "/zed/zed_node/left/image_rect_color"),
                ("camera_info", "/zed/zed_node/left/camera_info"),
            ],
        )
    ])
