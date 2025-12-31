"""
Main launch file for Hexapod Robot

Starts all robot components:
- Robot state publisher (URDF/TF)
- Hardware drivers (IMU, battery, range finder)
- Locomotion controller
- Optionally: Camera for perception

Usage:
  ros2 launch hexapod_bringup robot.launch.py
  ros2 launch hexapod_bringup robot.launch.py use_camera:=true
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    bringup_pkg = get_package_share_directory('hexapod_bringup')
    hardware_pkg = get_package_share_directory('hexapod_hardware')

    # Config file paths
    hardware_config = os.path.join(hardware_pkg, 'config', 'hardware.yaml')
    robot_config = os.path.join(bringup_pkg, 'config', 'robot.yaml')
    urdf_file = os.path.join(bringup_pkg, 'urdf', 'hexapod.urdf')

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch arguments
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Launch camera node for perception'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Run in simulation mode (no hardware)'
    )

    return LaunchDescription([
        use_camera_arg,
        use_sim_arg,

        # ===== Robot State Publisher (URDF/TF) =====

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),

        # ===== Hardware Drivers =====

        # IMU Driver
        Node(
            package='hexapod_hardware',
            executable='imu_driver',
            name='imu_driver',
            parameters=[hardware_config],
            output='screen',
        ),

        # Battery Monitor
        Node(
            package='hexapod_hardware',
            executable='battery_monitor',
            name='battery_monitor',
            parameters=[hardware_config],
            output='screen',
        ),

        # Range Finder Driver
        Node(
            package='hexapod_hardware',
            executable='range_finder_driver',
            name='range_finder_driver',
            parameters=[hardware_config],
            output='screen',
        ),

        # ===== Locomotion Controller =====

        Node(
            package='hexapod_controller',
            executable='controller',
            name='hexapod_controller',
            parameters=[robot_config] if os.path.exists(robot_config) else [],
            output='screen',
        ),

        # ===== Optional: Camera =====

        # Camera node (only if use_camera:=true)
        # Node(
        #     package='hexapod_perception',
        #     executable='camera_node',
        #     name='camera_node',
        #     condition=IfCondition(LaunchConfiguration('use_camera')),
        #     output='screen',
        # ),
    ])
