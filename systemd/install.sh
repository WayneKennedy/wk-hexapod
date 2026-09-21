#!/bin/bash
# Install the hexapod systemd service for auto-start on boot.
# Fills in the repo path and user from the current checkout.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"

if [[ ! -f "$REPO_DIR/ros2_ws/install/setup.bash" ]]; then
    echo "Workspace not built. Run: cd $REPO_DIR/ros2_ws && colcon build --symlink-install" >&2
    exit 1
fi

echo "Installing hexapod-buzzer-guard.service (holds the shield buzzer silent)"
sudo cp "$SCRIPT_DIR/hexapod-buzzer-guard.service" /etc/systemd/system/hexapod-buzzer-guard.service
sudo systemctl daemon-reload
sudo systemctl enable --now hexapod-buzzer-guard.service

echo "Enabling systemd-time-wait-sync (bounded to 90 s so an offline boot still starts)"
sudo mkdir -p /etc/systemd/system/systemd-time-wait-sync.service.d
sudo cp "$SCRIPT_DIR/time-wait-sync-timeout.conf" \
    /etc/systemd/system/systemd-time-wait-sync.service.d/timeout.conf
sudo systemctl daemon-reload
sudo systemctl enable systemd-time-wait-sync.service

# Linger, so logind never counts the service user as logged out. Without it, RemoveIPC=yes
# (Ubuntu's default) deletes the user's /dev/shm when the last SSH session closes, taking
# Fast DDS's shared-memory transport with it: nodes keep running, same-host delivery stops.
# wk-robotics docs/common.md, Robot startup is familial, rule 10 (2026-09-21).
echo "Enabling linger for $RUN_USER"
sudo loginctl enable-linger "$RUN_USER"

echo "Installing hexapod.service (user=$RUN_USER, repo=$REPO_DIR)"
sed -e "s|__USER__|$RUN_USER|g" \
    -e "s|__REPO_DIR__|$REPO_DIR|g" \
    -e "s|__HOME__|$RUN_HOME|g" \
    "$SCRIPT_DIR/hexapod.service" | sudo tee /etc/systemd/system/hexapod.service > /dev/null

sudo systemctl daemon-reload
sudo systemctl enable hexapod.service

echo ""
echo "Service installed and enabled."
echo ""
echo "Commands:"
echo "  sudo systemctl start hexapod    # Start now"
echo "  sudo systemctl stop hexapod     # Stop (servos relax)"
echo "  sudo systemctl status hexapod   # Check status"
echo "  sudo systemctl disable hexapod  # Disable auto-start"
echo "  journalctl -u hexapod -f        # View logs"
echo ""
echo "Buzzer: hexapod-buzzer-guard keeps GPIO 17 low at all times."
echo "  To allow beeps: set buzzer.enabled: true in hexapod_hardware/config/hardware.yaml"
echo "  and: sudo systemctl disable --now hexapod-buzzer-guard"
