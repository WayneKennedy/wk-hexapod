"""
Launch file for Hexapod Controller

Starts the body-centric controller standalone (no servo_driver): it drives the
servos directly. The full robot uses robot.launch.py instead.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    controller_pkg = get_package_share_directory('hexapod_controller')

    # Config file path
    config_file = os.path.join(controller_pkg, 'config', 'body_params.yaml')

    # Launch arguments
    balance_enabled = DeclareLaunchArgument(
        'balance_enabled',
        default_value='false',
        description='Enable IMU balance on startup'
    )

    gait_type = DeclareLaunchArgument(
        'gait_type',
        default_value='tripod',
        description='Default gait type (tripod or wave)'
    )

    direct_hardware = DeclareLaunchArgument(
        'direct_hardware',
        default_value='true',
        description='Controller drives the PCA9685 itself (standalone, no servo_driver)'
    )

    return LaunchDescription([
        balance_enabled,
        gait_type,
        direct_hardware,

        # Hexapod Controller
        Node(
            package='hexapod_controller',
            executable='controller',
            name='hexapod_controller',
            parameters=[
                config_file,
                {
                    'balance.enabled': LaunchConfiguration('balance_enabled'),
                    'gait.default': LaunchConfiguration('gait_type'),
                    'hardware.direct': LaunchConfiguration('direct_hardware'),
                }
            ],
            output='screen',
        ),
    ])
