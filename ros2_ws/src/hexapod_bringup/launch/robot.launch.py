"""
Main launch file for Hexapod Robot

Starts all robot components:
- Robot state publisher (URDF/TF)
- Hardware drivers (servo, IMU + orientation filter, battery, LED, buzzer)
- Power indicator and startup sequence
- Locomotion controller
- Optionally: Autonomous behavior system (SLAM, Nav2, exploration, dashboard)

Usage:
  ros2 launch hexapod_bringup robot.launch.py
  ros2 launch hexapod_bringup robot.launch.py autonomy:=true
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
    controller_config = os.path.join(
        get_package_share_directory('hexapod_controller'), 'config', 'body_params.yaml')
    urdf_file = os.path.join(bringup_pkg, 'urdf', 'hexapod.urdf')

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Run in simulation mode (no hardware)'
    )

    autonomy_arg = DeclareLaunchArgument(
        'autonomy',
        default_value='false',
        description='Enable autonomous behavior system'
    )

    mission_timeout_arg = DeclareLaunchArgument(
        'mission_timeout',
        default_value='60.0',
        description='Seconds to wait for mission before auto-exploring (when autonomy:=true)'
    )

    return LaunchDescription([
        use_sim_arg,
        autonomy_arg,
        mission_timeout_arg,

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

        # IMU orientation filter: imu_driver publishes raw gyro/accel on
        # imu/data_raw; the controller's yaw fusion needs a quaternion on imu/data.
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'fixed_frame': 'base_link',
            }],
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


        # Servo Driver
        Node(
            package='hexapod_hardware',
            executable='servo_driver',
            name='servo_driver',
            parameters=[hardware_config],
            output='screen',
        ),

        # LED Controller
        Node(
            package='hexapod_hardware',
            executable='led_controller',
            name='led_controller',
            parameters=[hardware_config],
            output='screen',
        ),

        # Buzzer Controller
        Node(
            package='hexapod_hardware',
            executable='buzzer_controller',
            name='buzzer_controller',
            parameters=[hardware_config],
            output='screen',
        ),

        # Power Indicator (battery status on LEDs)
        Node(
            package='hexapod_hardware',
            executable='power_indicator',
            name='power_indicator',
            parameters=[hardware_config],
            output='screen',
        ),

        # Startup Sequence (safe servo initialization)
        Node(
            package='hexapod_hardware',
            executable='startup_sequence',
            name='startup_sequence',
            parameters=[hardware_config],
            output='screen',
        ),

        # ===== Locomotion Controller =====

        Node(
            package='hexapod_controller',
            executable='controller',
            name='hexapod_controller',
            parameters=[controller_config],
            output='screen',
        ),


        # ===== Optional: Autonomous Behavior =====

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('hexapod_autonomy'),
                '/launch/autonomy.launch.py'
            ]),
            launch_arguments={
                'mission_timeout': LaunchConfiguration('mission_timeout'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('autonomy')),
        ),
    ])
