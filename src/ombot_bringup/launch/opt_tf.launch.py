from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    optitrack_tf = Node(
        package="ombot_coordination",
        executable="optitrack_tf_pub",
        output="screen",
        parameters=[{
            "pose_topic": "/vrpn_mocap/RigidBody_1/pose",
            "world_frame": "world",
            "base_frame": "base_link",
            "use_planar": True,
            "axis_fix": "xzy",
            "yaw_offset": 0.0,
            "alpha_pos": 0.2,
            "alpha_yaw": 0.2,
        }]
    )

    goal_commander = Node(
        package="ombot_coordination",
        executable="goal_commander",
        output="screen",
        parameters=[{
            "world_frame": "world",
            "topic": "/goal_pose",
            "rate_hz": 5.0,
            'goal': "0.8 0.2 0.5 0.0",
        }]
    )

    return LaunchDescription([
        optitrack_tf,
        goal_commander,
    ])
