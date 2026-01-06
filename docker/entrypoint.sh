#!/bin/bash
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source RealSense ROS wrapper if built
if [ -f /opt/realsense_ros/install/setup.bash ]; then
    source /opt/realsense_ros/install/setup.bash
fi

# Source RTAB-Map ROS wrapper if built
if [ -f /opt/rtabmap_ros/install/setup.bash ]; then
    source /opt/rtabmap_ros/install/setup.bash
fi

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Set ROS 2 domain ID (change if needed to avoid conflicts)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Configure DDS for local network discovery
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

exec "$@"
