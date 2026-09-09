"""
RealSense D435i + RTAB-Map SLAM launch file

Starts:
- Intel RealSense D435i camera
- RTAB-Map for visual SLAM
- Depth to LaserScan conversion for Nav2 costmap

Usage:
  ros2 launch hexapod_bringup realsense_slam.launch.py
  ros2 launch hexapod_bringup realsense_slam.launch.py localization:=true
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    bringup_pkg = get_package_share_directory('hexapod_bringup')

    # Config file paths
    realsense_config = os.path.join(bringup_pkg, 'config', 'realsense.yaml')
    rtabmap_config = os.path.join(bringup_pkg, 'config', 'rtabmap.yaml')

    # Launch arguments
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Run in localization mode (use existing map)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Mapping works in ~/.ros/rtabmap.db (from rtabmap.yaml) and starts fresh
    # each run. Localization loads a deliberately saved map (scripts/save-map.sh).
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.expanduser('~/.hexapod/maps/rtabmap.db'),
        description='RTAB-Map database used in localization mode'
    )

    # RealSense D435i camera node
    # realsense-ros prefixes topics with <namespace>/<node name>, so streams
    # appear as /camera/camera/color/image_raw etc.
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[realsense_config],
        output='screen',
    )

    # Depth to LaserScan for Nav2 obstacle layer
    # Converts depth image to fake 2D laser scan
    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('depth_camera_info', '/camera/camera/aligned_depth_to_color/camera_info'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'scan_height': 10,  # pixels to use for scan
            'scan_time': 0.033,
            'range_min': 0.2,
            'range_max': 3.0,
            'output_frame': 'camera_depth_frame',
        }],
        output='screen',
    )

    # RTAB-Map SLAM node (mapping mode)
    rtabmap_slam_node = Node(
        condition=UnlessCondition(LaunchConfiguration('localization')),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('odom', '/odom'),
        ],
        arguments=['--delete_db_on_start'],  # Fresh map each time in mapping mode
    )

    # RTAB-Map localization node (use existing map)
    rtabmap_localization_node = Node(
        condition=IfCondition(LaunchConfiguration('localization')),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'database_path': LaunchConfiguration('database_path'),
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
            },
        ],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('odom', '/odom'),
        ],
    )

    # RTAB-Map visualization (optional, for debugging)
    # rtabmap_viz_node = Node(
    #     package='rtabmap_viz',
    #     executable='rtabmap_viz',
    #     name='rtabmap_viz',
    #     output='screen',
    #     parameters=[rtabmap_config],
    #     remappings=[
    #         ('rgb/image', '/camera/camera/color/image_raw'),
    #         ('rgb/camera_info', '/camera/camera/color/camera_info'),
    #         ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
    #     ],
    # )

    return LaunchDescription([
        localization_arg,
        use_sim_time_arg,
        database_path_arg,
        realsense_node,
        depthimage_to_laserscan_node,
        rtabmap_slam_node,
        rtabmap_localization_node,
    ])
