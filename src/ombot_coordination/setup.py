from setuptools import setup

package_name = 'ombot_coordination'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages',
                 ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='kevin@example.com',
    description='Coordination and simple test nodes for OMBot',
    license='BSD',
    entry_points={
        'console_scripts': [
            'arm_base_coordinator = ombot_coordination.arm_base_coordinator:main',
            'goal_from_base_offset_latched = ombot_coordination.goal_from_base_offset_latched:main',
            'base_p_on_x = ombot_coordination.base_p_on_x:main',
            'arm_zero_twist = ombot_coordination.arm_zero_twist:main',
            'set_null_kp = ombot_coordination.set_null_kp:main',
            'whole_body_task_commander = ombot_coordination.whole_body_task_commander:main',
            "split_commander = ombot_coordination.split_commander:main",
            'ee_trajectory_generator = ombot_coordination.ee_trajectory_generator:main',
            'optitrack_tf_pub = ombot_coordination.optitrack_tf_pub:main',
            'goal_commander = ombot_coordination.goal_commander:main',
            'dual_traj_generator = ombot_coordination.dual_traj_generator:main',
            'bell_curve_turn_publisher = ombot_coordination.bell_curve_turn_publisher:main',
            'whole_body_cmd_publisher = ombot_coordination.whole_body_cmd_publisher:main',
            'depth_move_away_cmd_publisher = ombot_coordination.depth_move_away_cmd_publisher:main',
            'moveaway_goto_cmd = ombot_coordination.moveaway_goto_cmd:main',
            'resolved_rate_tester = ombot_coordination.resolved_rate_tester:main',
            'ee_twist_cmd_publisher = ombot_coordination.ee_twist_cmd_publisher:main'
        ],
    },
)
