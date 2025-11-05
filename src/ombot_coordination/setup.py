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
            'goal_from_base_offset = ombot_coordination.goal_from_base_offset:main',
            'base_p_on_x = ombot_coordination.base_p_on_x:main',
            'arm_zero_twist = ombot_coordination.arm_zero_twist:main',
            'set_null_kp = ombot_coordination.set_null_kp:main',
        ],
    },
)
