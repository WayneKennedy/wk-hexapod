"""
SLAM launch file for Hexapod Robot: slam_toolbox (online async) on the lidar.

Builds /map from /scan and gait odometry and publishes map -> odom. Included
by navigation.launch.py (DEC-32), so it is part of the boot stack; run it by
hand only against a stack started with autonomy:=false.

Requires robot.launch.py (lidar:=true) to be running.

Usage:
  ros2 launch hexapod_bringup slam.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory('hexapod_bringup'), 'config', 'slam_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=slam_params,
        description='Full path to the slam_toolbox params file'
    )

    # slam_toolbox's own launch: a lifecycle node, configured and activated on start.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([params_file_arg, slam])
