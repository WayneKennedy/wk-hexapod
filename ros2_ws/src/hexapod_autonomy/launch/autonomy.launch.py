#!/usr/bin/env python3
"""
Autonomous Behavior Launch File for Hexapod Robot

Launches the autonomy nodes plus navigation:
- Nav2 navigation servers; the global costmap, built from the head's sonar
  sweeps, is the map (navigation.launch.py)
- frontier_explorer: Frontier-based exploration
- mission_server: External mission command interface
- autonomy_manager: Central state machine coordinator
- web_dashboard: Mission control web interface

The LookAround action (head survey) is served by head_controller, which
robot.launch.py starts with the drivers.

Usage:
  ros2 launch hexapod_autonomy autonomy.launch.py
  ros2 launch hexapod_autonomy autonomy.launch.py mission_timeout:=120.0
  ros2 launch hexapod_autonomy autonomy.launch.py nav:=false   # Skip Nav2
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    autonomy_pkg = get_package_share_directory('hexapod_autonomy')
    bringup_pkg = get_package_share_directory('hexapod_bringup')
    config_file = os.path.join(autonomy_pkg, 'config', 'autonomy_params.yaml')

    # Declare launch arguments
    mission_timeout_arg = DeclareLaunchArgument(
        'mission_timeout',
        default_value='60.0',
        description='Seconds to wait for external mission before auto-exploring'
    )

    dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='true',
        description='Launch web dashboard for mission control'
    )

    nav_arg = DeclareLaunchArgument(
        'nav',
        default_value='true',
        description='Launch Nav2 (required for exploration and navigate missions)'
    )

    # Nav2 navigation servers (planner, controller, behaviors) and the static
    # map -> odom identity
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            bringup_pkg, '/launch/navigation.launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration('nav')),
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
        dashboard_arg,
        nav_arg,

        # Nav2 (optional, enabled by default)
        navigation_launch,

        # Autonomy nodes
        frontier_explorer_node,
        mission_server_node,
        autonomy_manager_node,

        # Web dashboard (optional, enabled by default)
        web_dashboard_node,
    ])
