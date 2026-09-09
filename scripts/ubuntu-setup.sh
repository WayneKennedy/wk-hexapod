#!/bin/bash
#
# Native setup for the hexapod on Ubuntu Server 24.04 (Raspberry Pi 5).
# Installs ROS 2 Jazzy plus every apt/pip dependency, configures hardware
# interfaces, and builds the workspace. Idempotent: safe to re-run.
#
# Usage: sudo ./scripts/ubuntu-setup.sh [--skip-build] [--skip-pip] [--dry-run]
#
set -e

SKIP_BUILD=false
SKIP_PIP=false
DRY_RUN=false
CONFIG_FILE="/boot/firmware/config.txt"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ACTUAL_USER="${SUDO_USER:-$USER}"

for arg in "$@"; do
    case $arg in
        --skip-build) SKIP_BUILD=true ;;
        --skip-pip)   SKIP_PIP=true ;;
        --dry-run)    DRY_RUN=true ;;
        -h|--help)    sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "Unknown option: $arg" >&2; exit 1 ;;
    esac
done

log()  { echo -e "\033[0;32m[INFO]\033[0m $1"; }
warn() { echo -e "\033[1;33m[WARN]\033[0m $1"; }
run()  { if $DRY_RUN; then echo "  [dry-run] $*"; else "$@"; fi; }

if [[ $EUID -ne 0 ]] && ! $DRY_RUN; then
    echo "Run with sudo" >&2; exit 1
fi

# ---------------------------------------------------------------------------
log "Step 1: /boot/firmware/config.txt (I2C 400kHz, SPI, safe GPIO defaults)"
ensure_config_line() {
    local line="$1"
    if grep -qF "$line" "$CONFIG_FILE"; then
        log "  present: $line"
    else
        log "  adding:  $line"
        $DRY_RUN || echo "$line" >> "$CONFIG_FILE"
    fi
}
if grep -q "^dtparam=i2c_arm=on,i2c_arm_baudrate=400000" "$CONFIG_FILE"; then
    log "  I2C already at 400kHz"
elif grep -q "^dtparam=i2c_arm=on" "$CONFIG_FILE"; then
    log "  raising I2C baud rate to 400kHz (servo response)"
    $DRY_RUN || sed -i 's/^dtparam=i2c_arm=on.*/dtparam=i2c_arm=on,i2c_arm_baudrate=400000/' "$CONFIG_FILE"
else
    ensure_config_line "dtparam=i2c_arm=on,i2c_arm_baudrate=400000"
fi
ensure_config_line "dtparam=spi=on"
# Buzzer (GPIO 17) floats high and sounds continuously if nothing drives it.
ensure_config_line "gpio=17=op,dl"
# Servo power enable (GPIO 4, low = enabled): keep servos off until the driver runs.
ensure_config_line "gpio=4=op,dh"

# ---------------------------------------------------------------------------
log "Step 2: ROS 2 apt repository"
if [[ ! -f /etc/apt/sources.list.d/ros2.sources ]] && ! ls /etc/apt/sources.list.d/ros2* >/dev/null 2>&1; then
    run apt-get install -y -qq curl
    V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F\" '{print $4}')
    run curl -sL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.noble_all.deb"
    run dpkg -i /tmp/ros2-apt-source.deb
else
    log "  already configured"
fi

# ---------------------------------------------------------------------------
log "Step 3: apt packages (ROS 2 Jazzy, Nav2, RTAB-Map, RealSense, Python libs)"
if dpkg-query -W -f='${Version}\n' '*' 2>/dev/null | grep -q -E 'rpt|deb12'; then
    warn "  Raspberry Pi OS (bookworm) packages are installed on this host."
    warn "  They conflict with ROS packages. See docs/operations.md, 'Foreign packages'."
fi
APT_PACKAGES=(
    ros-jazzy-ros-base ros-dev-tools python3-colcon-common-extensions python3-rosdep python3-vcstool
    ros-jazzy-robot-state-publisher ros-jazzy-rmw-fastrtps-cpp ros-jazzy-imu-filter-madgwick
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup
    ros-jazzy-rtabmap-ros ros-jazzy-realsense2-camera ros-jazzy-depthimage-to-laserscan
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-diagnostic-updater
    ros-jazzy-foxglove-bridge ros-jazzy-pcl-ros ros-jazzy-laser-geometry
    python3-gpiozero python3-lgpio python3-spidev python3-smbus python3-numpy python3-opencv python3-flask
    python3-pip python3-dev build-essential cmake libopenblas-dev liblapack-dev
    i2c-tools gpiod
)
run apt-get update -qq
run apt-get install -y -qq --no-install-recommends "${APT_PACKAGES[@]}"

# ---------------------------------------------------------------------------
log "Step 4: udev rules (RealSense as non-root)"
if [[ ! -f /etc/udev/rules.d/99-realsense-libusb.rules ]]; then
    run curl -sL -o /etc/udev/rules.d/99-realsense-libusb.rules \
        https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
    run udevadm control --reload-rules
    run udevadm trigger
else
    log "  present"
fi

# ---------------------------------------------------------------------------
log "Step 5: user groups for $ACTUAL_USER"
for grp in i2c spi gpio dialout video plugdev; do
    if getent group "$grp" >/dev/null && ! id -nG "$ACTUAL_USER" | grep -qw "$grp"; then
        run usermod -aG "$grp" "$ACTUAL_USER"
        log "  added to $grp"
    fi
done

# ---------------------------------------------------------------------------
if $SKIP_PIP; then
    log "Step 6: pip packages skipped"
else
    log "Step 6: pip packages (system interpreter; dlib build takes ~20 min)"
    run sudo -u "$ACTUAL_USER" pip3 install --break-system-packages -r "$REPO_DIR/requirements.txt"
fi

# ---------------------------------------------------------------------------
log "Step 7: rosdep"
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || run rosdep init
run sudo -u "$ACTUAL_USER" rosdep update

# ---------------------------------------------------------------------------
if $SKIP_BUILD; then
    log "Step 8: workspace build skipped"
else
    log "Step 8: build workspace"
    run sudo -u "$ACTUAL_USER" bash -c "source /opt/ros/jazzy/setup.bash && cd '$REPO_DIR/ros2_ws' && \
        rosdep install --from-paths src --ignore-src -y -r && colcon build --symlink-install"
fi

# ---------------------------------------------------------------------------
echo ""
log "Setup complete."
log "  Reboot if config.txt or group membership changed."
log "  Start manually:   scripts/launch.sh"
log "  Install service:  systemd/install.sh   (auto-start on boot)"
log "  Verify I2C:       i2cdetect -y 1   (expect 0x40 0x41 0x48 0x68)"
