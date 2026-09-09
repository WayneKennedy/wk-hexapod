"""
Launch file for Hexapod perception nodes

The RealSense D435i (started by realsense_slam.launch.py) supplies the RGB
stream, so by default only face recognition is started here, subscribed to the
RealSense colour image.

Usage:
  ros2 launch hexapod_perception perception.launch.py
  ros2 launch hexapod_perception perception.launch.py image_topic:=/some/other/image
  ros2 launch hexapod_perception perception.launch.py use_camera_node:=true   # legacy Pi Camera node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('hexapod_perception')
    config_file = os.path.join(pkg_dir, 'config', 'perception.yaml')

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/camera/color/image_raw',
        description='RGB image topic consumed by face recognition',
    )
    use_camera_node_arg = DeclareLaunchArgument(
        'use_camera_node',
        default_value='false',
        description='Start the legacy Pi Camera node (requires picamera2)',
    )

    return LaunchDescription([
        image_topic_arg,
        use_camera_node_arg,

        Node(
            package='hexapod_perception',
            executable='camera_node',
            name='camera_node',
            parameters=[config_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_camera_node')),
        ),

        Node(
            package='hexapod_perception',
            executable='face_recognition_node',
            name='face_recognition_node',
            parameters=[config_file],
            remappings=[('camera/image_raw', LaunchConfiguration('image_topic'))],
            output='screen',
        ),
    ])
