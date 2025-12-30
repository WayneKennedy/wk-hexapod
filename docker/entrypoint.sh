#!/bin/bash
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Set ROS 2 domain ID (change if needed to avoid conflicts)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Configure DDS for local network discovery
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

exec "$@"
