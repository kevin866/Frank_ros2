from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'frank_behaviors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='kevin@example.com',
    description='Behavior nodes for Frank interaction demo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_intent_node = frank_behaviors.text_intent_node:main',
            'behavior_manager_node = frank_behaviors.behavior_manager_node:main',
        ],
    },
)
