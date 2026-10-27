from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('gesture_turtlebot4'))
    default_params = str(package_share / 'config' / 'params.yaml')

    params_file = LaunchConfiguration('params_file')
    camera_index = LaunchConfiguration('camera_index')
    model_path = LaunchConfiguration('model_path')
    show_window = LaunchConfiguration('show_window')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='ROS 2 parameter file',
        ),
        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
            description='OpenCV camera index',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Path to gesture_recognizer.task; empty uses package share/models',
        ),
        DeclareLaunchArgument(
            'show_window',
            default_value='true',
            description='Show the OpenCV webcam window',
        ),
        Node(
            package='gesture_turtlebot4',
            executable='gesture_recognition_node',
            name='gesture_recognition_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'camera_index': camera_index,
                    'model_path': model_path,
                    'show_window': show_window,
                },
            ],
        ),
        Node(
            package='gesture_turtlebot4',
            executable='robot_behavior_node',
            name='robot_behavior_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
