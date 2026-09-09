"""
Navigation launch file for Hexapod Robot

Starts the Nav2 navigation servers (planner, controller, behaviors, BT
navigator, waypoint follower, velocity smoother) with hexapod-specific
parameters. Localization and the /map come from RTAB-Map
(realsense_slam.launch.py), so Nav2's own map server and AMCL are not started.

Requires robot.launch.py and realsense_slam.launch.py to be running.

Usage:
  ros2 launch hexapod_bringup navigation.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory('hexapod_bringup')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
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

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        nav2_navigation,
    ])
