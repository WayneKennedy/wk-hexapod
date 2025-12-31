"""
Navigation launch file for Hexapod Robot

Starts Nav2 navigation stack with hexapod-specific parameters.
Requires robot.launch.py to be running first.

Usage:
  ros2 launch hexapod_bringup navigation.launch.py
  ros2 launch hexapod_bringup navigation.launch.py map:=/path/to/map.yaml
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    bringup_pkg = get_package_share_directory('hexapod_bringup')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # Config file paths
    nav2_params = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file (empty for SLAM mode)'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Full path to Nav2 params file'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the nav2 stack'
    )

    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'map': LaunchConfiguration('map'),
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        params_file_arg,
        autostart_arg,
        nav2_bringup,
    ])
