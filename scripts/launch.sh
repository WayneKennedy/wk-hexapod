#!/bin/bash
# Set the environment, source ROS 2 Jazzy and the workspace, then launch the robot stack.
# Used by systemd/hexapod.service (which adds autonomy:=true) and for manual starts:
#   scripts/launch.sh                     # drivers + controllers (robot.launch.py defaults)
#   scripts/launch.sh autonomy:=true      # full stack, as at boot
#   scripts/launch.sh hardware.launch.py  # any other hexapod_bringup launch file
# Follows wk-robotics docs/common.md -> "Robot startup is familial".
set -e
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# The environment is set here only (not in the unit), and *before* sourcing ROS: Jazzy's
# ros_environment hook sets ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET if it is unset, so a
# default applied afterwards never takes effect.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# SUBNET, stated explicitly: this is the behaviour the robot has always had (Jazzy's
# default), so remote tools on the LAN can see its graph. The family's choice of range is
# still open (wk-robotics docs/common.md, the discovery finding).
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export GPIOZERO_PIN_FACTORY="${GPIOZERO_PIN_FACTORY:-lgpio}"

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/ros2_ws/install/setup.bash"

LAUNCH_FILE="robot.launch.py"
if [[ "${1:-}" == *.launch.py ]]; then
    LAUNCH_FILE="$1"
    shift
fi

exec ros2 launch hexapod_bringup "$LAUNCH_FILE" "$@"
