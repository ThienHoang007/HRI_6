from glob import glob
from setuptools import setup

package_name = 'gesture_turtlebot4'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Gesture controlled TurtleBot 4 behavior using MediaPipe and OpenCV.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gesture_recognition_node = gesture_turtlebot4.gesture_recognition_node:main',
            'robot_behavior_node = gesture_turtlebot4.robot_behavior_node:main',
        ],
    },
)
