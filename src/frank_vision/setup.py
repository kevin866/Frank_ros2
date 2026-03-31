from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'frank_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frank',
    maintainer_email='frank@example.com',
    description='Vision nodes for Frank interaction demo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detector_node = frank_vision.face_detector_node:main',
        ],
    },
)