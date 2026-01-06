"""
Launch file for Hexapod hardware drivers

Starts all hardware interface nodes:
- IMU driver
- Servo driver
- Battery monitor
- LED controller
- Range finder driver
- Buzzer controller
- Power indicator (LED status based on battery)
- Startup sequence (safe servo initialization)
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

        # Range Finder Driver
        Node(
            package='hexapod_hardware',
            executable='range_finder_driver',
            name='range_finder_driver',
            parameters=[config_file],
            output='screen',
        ),

        # Buzzer Controller
        Node(
            package='hexapod_hardware',
            executable='buzzer_controller',
            name='buzzer_controller',
            parameters=[config_file],
            output='screen',
        ),

        # Power Indicator (LED status based on battery)
        Node(
            package='hexapod_hardware',
            executable='power_indicator',
            name='power_indicator',
            parameters=[config_file],
            output='screen',
        ),

        # Startup Sequence (safe servo initialization)
        Node(
            package='hexapod_hardware',
            executable='startup_sequence',
            name='startup_sequence',
            parameters=[config_file],
            output='screen',
        ),
    ])
