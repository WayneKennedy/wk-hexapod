#!/bin/bash
# Source ROS 2 Jazzy and the workspace, then launch the robot stack.
# Used by systemd/hexapod.service and handy for manual starts:
#   scripts/launch.sh                     # full stack, autonomy on
#   scripts/launch.sh autonomy:=false     # hardware + controller only
#   scripts/launch.sh hardware.launch.py  # any other hexapod_bringup launch file
set -e
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/ros2_ws/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export GPIOZERO_PIN_FACTORY="${GPIOZERO_PIN_FACTORY:-lgpio}"

LAUNCH_FILE="robot.launch.py"
if [[ "${1:-}" == *.launch.py ]]; then
    LAUNCH_FILE="$1"
    shift
fi

if [[ "$LAUNCH_FILE" == "robot.launch.py" ]] && [[ "$*" != *autonomy:=* ]]; then
    set -- autonomy:=true "$@"
fi

exec ros2 launch hexapod_bringup "$LAUNCH_FILE" "$@"
