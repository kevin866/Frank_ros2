from setuptools import setup

package_name = 'frank_audio'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frank',
    maintainer_email='frank@todo.todo',
    description='TODO',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stt_node = frank_audio.stt_node:main',
            'test_whisper = frank_audio.test_whisper:main',  # add this
        ],
    },
)