"""
Launch file for Hexapod hardware drivers

Starts all hardware interface nodes:
- IMU driver
- Servo driver
- Battery monitor
- LED controller
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    hardware_pkg = get_package_share_directory('hexapod_hardware')

    # Config file path
    config_file = os.path.join(hardware_pkg, 'config', 'hardware.yaml')

    return LaunchDescription([
        # IMU Driver
        Node(
            package='hexapod_hardware',
            executable='imu_driver',
            name='imu_driver',
            parameters=[config_file],
            output='screen',
        ),

        # Servo Driver
        Node(
            package='hexapod_hardware',
            executable='servo_driver',
            name='servo_driver',
            parameters=[config_file],
            output='screen',
        ),

        # Battery Monitor
        Node(
            package='hexapod_hardware',
            executable='battery_monitor',
            name='battery_monitor',
            parameters=[config_file],
            output='screen',
        ),

        # LED Controller
        Node(
            package='hexapod_hardware',
            executable='led_controller',
            name='led_controller',
            parameters=[config_file],
            output='screen',
        ),
    ])
