#!/bin/bash
#
# Ubuntu Server Setup Script for Hexapod (Raspberry Pi 5)
#
# This script configures hardware interfaces and installs dependencies
# for the Freenove Big Hexapod Robot on Ubuntu Server 24.04.
#
# Usage: sudo ./ubuntu-setup.sh [options]
#
# Options:
#   --camera-port PORT   Camera port: cam0 or cam1 (default: cam0)
#   --camera-model MODEL Camera model: ov5647 or imx219 (default: ov5647)
#   --skip-reboot        Don't prompt for reboot at end
#   --dry-run            Show what would be done without making changes
#

set -e

# Defaults
CAMERA_PORT="cam0"
CAMERA_MODEL="ov5647"
SKIP_REBOOT=false
DRY_RUN=false
CONFIG_FILE="/boot/firmware/config.txt"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --camera-port)
            CAMERA_PORT="$2"
            shift 2
            ;;
        --camera-model)
            CAMERA_MODEL="$2"
            shift 2
            ;;
        --skip-reboot)
            SKIP_REBOOT=true
            shift
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        -h|--help)
            head -20 "$0" | tail -n +2 | sed 's/^# //' | sed 's/^#//'
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Check if running as root
if [[ $EUID -ne 0 ]] && [[ "$DRY_RUN" == "false" ]]; then
    log_error "This script must be run as root (use sudo)"
    exit 1
fi

# Validate camera options
if [[ "$CAMERA_PORT" != "cam0" && "$CAMERA_PORT" != "cam1" ]]; then
    log_error "Invalid camera port: $CAMERA_PORT (must be cam0 or cam1)"
    exit 1
fi

if [[ "$CAMERA_MODEL" != "ov5647" && "$CAMERA_MODEL" != "imx219" ]]; then
    log_error "Invalid camera model: $CAMERA_MODEL (must be ov5647 or imx219)"
    exit 1
fi

log_info "Ubuntu Server Setup for Hexapod"
log_info "================================"
log_info "Camera: $CAMERA_MODEL on $CAMERA_PORT"
echo ""

if [[ "$DRY_RUN" == "true" ]]; then
    log_warn "DRY RUN MODE - no changes will be made"
    echo ""
fi

# Backup config.txt
backup_config() {
    if [[ -f "$CONFIG_FILE" ]]; then
        BACKUP="${CONFIG_FILE}.bak.$(date +%Y%m%d_%H%M%S)"
        if [[ "$DRY_RUN" == "false" ]]; then
            cp "$CONFIG_FILE" "$BACKUP"
            log_info "Backed up config to $BACKUP"
        else
            log_info "[DRY RUN] Would backup config to $BACKUP"
        fi
    fi
}

# Update config.txt setting
update_config() {
    local pattern="$1"
    local replacement="$2"
    local description="$3"

    if grep -q "$pattern" "$CONFIG_FILE" 2>/dev/null; then
        if [[ "$DRY_RUN" == "false" ]]; then
            sed -i "s|$pattern|$replacement|" "$CONFIG_FILE"
        fi
        log_info "$description"
    else
        log_warn "Pattern not found: $pattern"
    fi
}

# Add line to config.txt if not present
add_config() {
    local line="$1"
    local after="$2"
    local description="$3"

    if ! grep -q "^${line}$" "$CONFIG_FILE" 2>/dev/null; then
        if [[ "$DRY_RUN" == "false" ]]; then
            if [[ -n "$after" ]]; then
                sed -i "/$after/a $line" "$CONFIG_FILE"
            else
                echo "$line" >> "$CONFIG_FILE"
            fi
        fi
        log_info "$description"
    else
        log_info "$description (already set)"
    fi
}

# Step 1: Backup config
log_info "Step 1: Backing up configuration"
backup_config

# Step 2: Configure I2C with fast baud rate
log_info "Step 2: Configuring I2C"

if grep -q "dtparam=i2c_arm=on,i2c_arm_baudrate=400000" "$CONFIG_FILE" 2>/dev/null; then
    log_info "I2C already configured with 400kHz baud rate"
elif grep -q "dtparam=i2c_arm=on" "$CONFIG_FILE" 2>/dev/null; then
    update_config \
        "dtparam=i2c_arm=on$" \
        "dtparam=i2c_arm=on,i2c_arm_baudrate=400000" \
        "Updated I2C baud rate to 400kHz (faster servo response)"
else
    add_config "dtparam=i2c_arm=on,i2c_arm_baudrate=400000" "dtparam=audio" \
        "Added I2C with 400kHz baud rate"
fi

# Step 3: Ensure SPI is enabled (for WS2812 LEDs on Pi 5)
log_info "Step 3: Configuring SPI"

if grep -q "^dtparam=spi=on" "$CONFIG_FILE" 2>/dev/null; then
    log_info "SPI already enabled"
else
    add_config "dtparam=spi=on" "dtparam=i2c_arm" \
        "Enabled SPI (required for WS2812 LEDs on Pi 5)"
fi

# Step 4: Configure camera
log_info "Step 4: Configuring camera ($CAMERA_MODEL on $CAMERA_PORT)"

# Disable auto-detect
if grep -q "camera_auto_detect=1" "$CONFIG_FILE" 2>/dev/null; then
    update_config \
        "camera_auto_detect=1" \
        "camera_auto_detect=0" \
        "Disabled camera auto-detect"
fi

# Remove any existing camera overlay for our model
if [[ "$DRY_RUN" == "false" ]]; then
    sed -i "/^dtoverlay=${CAMERA_MODEL}/d" "$CONFIG_FILE"
fi

# Add camera overlay
CAMERA_OVERLAY="dtoverlay=${CAMERA_MODEL},${CAMERA_PORT}"
add_config "$CAMERA_OVERLAY" "camera_auto_detect" \
    "Added camera overlay: $CAMERA_OVERLAY"

# Step 5: Install system packages
log_info "Step 5: Installing system packages"

PACKAGES=(
    i2c-tools
    python3-smbus
    python3-dev
    python3-pip
    python3-venv
    python3-lgpio
    libcap-dev
    ffmpeg
)

if [[ "$DRY_RUN" == "false" ]]; then
    apt-get update -qq
    apt-get install -y "${PACKAGES[@]}"
    log_info "Installed: ${PACKAGES[*]}"
else
    log_info "[DRY RUN] Would install: ${PACKAGES[*]}"
fi

# Step 6: Camera setup notes
log_info "Step 6: Camera setup"

# Note: picamera2 PPA (ppa:r41k0u/python3-simplejpeg) does not support Ubuntu 24.04 (noble)
# Options:
#   1. For ROS 2: Use ros2_camera_node or similar ROS 2 camera packages
#   2. For direct Python: Build libcamera from Raspberry Pi fork (complex)
#   3. Wait for Ubuntu 25.04+ where camera support is improved
#
# See: https://github.com/raspberrypi/picamera2/issues/1337

log_warn "picamera2 not available via PPA for Ubuntu 24.04"
log_info "For ROS 2: camera will be handled via ros2_camera_node"
log_info "libcamera tools should work: rpicam-hello, rpicam-still, etc."

# Install libcamera tools if available
if [[ "$DRY_RUN" == "false" ]]; then
    apt-get install -y libcamera-tools 2>/dev/null || log_warn "libcamera-tools not available"
fi

# Step 7: Set up user permissions
log_info "Step 7: Configuring user permissions"

# Get the actual user (not root)
ACTUAL_USER="${SUDO_USER:-$USER}"

if [[ "$ACTUAL_USER" != "root" ]]; then
    GROUPS_TO_ADD=(i2c dialout gpio)

    for grp in "${GROUPS_TO_ADD[@]}"; do
        if getent group "$grp" > /dev/null 2>&1; then
            if [[ "$DRY_RUN" == "false" ]]; then
                usermod -aG "$grp" "$ACTUAL_USER" 2>/dev/null || true
            fi
            log_info "Added $ACTUAL_USER to group: $grp"
        else
            log_warn "Group $grp does not exist"
        fi
    done
else
    log_warn "Running as root without SUDO_USER set, skipping group setup"
fi

# Step 8: Create Python virtual environment and install dependencies
log_info "Step 8: Python virtual environment and dependencies"

REPO_PATH="/home/${ACTUAL_USER}/Code/wk-hexapi"
VENV_PATH="${REPO_PATH}/.venv"

if [[ "$ACTUAL_USER" != "root" ]] && [[ -d "$REPO_PATH" ]]; then
    if [[ ! -d "$VENV_PATH" ]]; then
        if [[ "$DRY_RUN" == "false" ]]; then
            sudo -u "$ACTUAL_USER" python3 -m venv --system-site-packages "$VENV_PATH"
            log_info "Created virtual environment at $VENV_PATH"
        else
            log_info "[DRY RUN] Would create venv at $VENV_PATH"
        fi
    else
        log_info "Virtual environment already exists at $VENV_PATH"
    fi

    # Install Python dependencies
    if [[ -f "${REPO_PATH}/requirements.txt" ]]; then
        if [[ "$DRY_RUN" == "false" ]]; then
            sudo -u "$ACTUAL_USER" "${VENV_PATH}/bin/pip" install --upgrade pip
            sudo -u "$ACTUAL_USER" "${VENV_PATH}/bin/pip" install -r "${REPO_PATH}/requirements.txt"
            log_info "Installed Python dependencies from requirements.txt"
        else
            log_info "[DRY RUN] Would install dependencies from requirements.txt"
        fi
    fi
fi

# Summary
echo ""
log_info "================================"
log_info "Setup complete!"
echo ""
log_info "Configuration summary:"
echo "  - I2C: enabled at 400kHz"
echo "  - SPI: enabled"
echo "  - Camera: $CAMERA_MODEL on $CAMERA_PORT"
echo ""
log_info "After reboot, verify with:"
echo "  sudo i2cdetect -y 1        # Check I2C devices"
echo "  libcamera-hello --list     # Check camera"
echo "  ls /dev/spidev*            # Check SPI"
echo ""

if [[ "$DRY_RUN" == "false" ]]; then
    log_warn "A reboot is required for changes to take effect."

    if [[ "$SKIP_REBOOT" == "false" ]]; then
        read -p "Reboot now? [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            log_info "Rebooting..."
            reboot
        else
            log_info "Remember to reboot manually: sudo reboot"
        fi
    fi
fi
