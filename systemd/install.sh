#!/bin/bash
# Install hexapod systemd service for auto-start on boot

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_FILE="$SCRIPT_DIR/hexapod.service"

echo "Installing hexapod systemd service..."

# Copy service file
sudo cp "$SERVICE_FILE" /etc/systemd/system/hexapod.service

# Reload systemd
sudo systemctl daemon-reload

# Enable service (start on boot)
sudo systemctl enable hexapod.service

echo ""
echo "Service installed and enabled."
echo ""
echo "Commands:"
echo "  sudo systemctl start hexapod    # Start now"
echo "  sudo systemctl stop hexapod     # Stop"
echo "  sudo systemctl status hexapod   # Check status"
echo "  sudo systemctl disable hexapod  # Disable auto-start"
echo "  journalctl -u hexapod -f        # View logs"
