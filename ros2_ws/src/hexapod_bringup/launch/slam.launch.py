"""
SLAM launch file for Hexapod Robot

Starts slam_toolbox for mapping. Requires:
- Robot to be running (robot.launch.py)
- A laser scan source (lidar or range_to_laserscan)

Usage:
  ros2 launch hexapod_bringup slam.launch.py
  ros2 launch hexapod_bringup slam.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    bringup_pkg = get_package_share_directory('hexapod_bringup')

    # Config file paths
    slam_params = os.path.join(bringup_pkg, 'config', 'slam_params.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params,
        description='Full path to slam_toolbox params file'
    )

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    # Optional: Range to LaserScan converter for ultrasonic sensor
    # This creates a fake single-beam "scan" from the range finder
    # Very limited but allows basic obstacle marking
    range_to_laserscan_node = Node(
        package='depthimage_to_laserscan',  # or custom node
        executable='range_to_laserscan_node',
        name='range_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'range_topic': '/range_finder/range',
            'scan_topic': '/scan',
            'min_range': 0.02,
            'max_range': 3.0,
        }],
        # Comment out if using real lidar
        condition=None,  # Always disabled for now - needs custom node
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_arg,
        slam_toolbox_node,
        # range_to_laserscan_node,  # Enable when converter is available
    ])
