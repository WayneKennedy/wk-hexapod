#!/usr/bin/env python3
"""
Autonomous Behavior Launch File for Hexapod Robot

Launches all autonomy nodes plus SLAM/perception:
- RealSense D435i camera + RTAB-Map SLAM
- slam_monitor: Monitors RTAB-Map localization status
- look_around: Action server for head/body sweep
- frontier_explorer: Frontier-based exploration
- mission_server: External mission command interface
- autonomy_manager: Central state machine coordinator
- web_dashboard: Mission control web interface

Usage:
  ros2 launch hexapod_autonomy autonomy.launch.py
  ros2 launch hexapod_autonomy autonomy.launch.py mission_timeout:=120.0
  ros2 launch hexapod_autonomy autonomy.launch.py slam:=false  # Skip SLAM
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    autonomy_pkg = get_package_share_directory('hexapod_autonomy')
    bringup_pkg = get_package_share_directory('hexapod_bringup')
    perception_pkg = get_package_share_directory('hexapod_perception')
    config_file = os.path.join(autonomy_pkg, 'config', 'autonomy_params.yaml')

    # Check if RTAB-Map database exists (determines localization vs mapping mode)
    rtabmap_db_path = os.path.expanduser('~/.ros/rtabmap.db')
    map_exists = os.path.exists(rtabmap_db_path)

    # Declare launch arguments
    mission_timeout_arg = DeclareLaunchArgument(
        'mission_timeout',
        default_value='60.0',
        description='Seconds to wait for external mission before auto-exploring'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Launch RTAB-Map SLAM with RealSense camera'
    )

    dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='true',
        description='Launch web dashboard for mission control'
    )

    # Include RealSense + RTAB-Map SLAM (conditional)
    # Starts in localization mode if map exists, mapping mode otherwise
    realsense_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            bringup_pkg, '/launch/realsense_slam.launch.py'
        ]),
        launch_arguments={
            'localization': 'true' if map_exists else 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    # SLAM Monitor - monitors RTAB-Map localization status
    slam_monitor_node = Node(
        package='hexapod_autonomy',
        executable='slam_monitor',
        name='slam_monitor',
        parameters=[config_file],
        output='screen',
    )

    # Look Around - action server for visual feature gathering
    look_around_node = Node(
        package='hexapod_autonomy',
        executable='look_around',
        name='look_around',
        parameters=[config_file],
        output='screen',
    )

    # Frontier Explorer - frontier-based exploration
    frontier_explorer_node = Node(
        package='hexapod_autonomy',
        executable='frontier_explorer',
        name='frontier_explorer',
        parameters=[config_file],
        output='screen',
    )

    # Mission Server - external command interface
    mission_server_node = Node(
        package='hexapod_autonomy',
        executable='mission_server',
        name='mission_server',
        parameters=[config_file],
        output='screen',
    )

    # Autonomy Manager - central state machine
    autonomy_manager_node = Node(
        package='hexapod_autonomy',
        executable='autonomy_manager',
        name='autonomy_manager',
        parameters=[
            config_file,
            {'mission_timeout_sec': LaunchConfiguration('mission_timeout')},
        ],
        output='screen',
    )

    # Web Dashboard for mission control UI
    web_dashboard_node = Node(
        package='hexapod_perception',
        executable='web_dashboard',
        name='web_dashboard',
        output='screen',
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    return LaunchDescription([
        # Launch arguments
        mission_timeout_arg,
        slam_arg,
        dashboard_arg,

        # SLAM (optional, enabled by default)
        realsense_slam_launch,

        # Autonomy nodes
        slam_monitor_node,
        look_around_node,
        frontier_explorer_node,
        mission_server_node,
        autonomy_manager_node,

        # Web dashboard (optional, enabled by default)
        web_dashboard_node,
    ])
