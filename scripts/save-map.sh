#!/bin/bash
# Save the current RTAB-Map working database as the robot's map.
# On the next boot the stack starts in localization mode against this map.
#
# Usage: scripts/save-map.sh [name]     (default name: rtabmap)
#
# Stop the stack first so RTAB-Map has flushed the database:
#   sudo systemctl stop hexapod
set -e
WORK_DB="$HOME/.ros/rtabmap.db"
MAP_DIR="$HOME/.hexapod/maps"
NAME="${1:-rtabmap}"

if systemctl is-active --quiet hexapod || pgrep -x rtabmap >/dev/null; then
    echo "RTAB-Map is running; stop the stack first (sudo systemctl stop hexapod)" >&2
    exit 1
fi
[[ -s "$WORK_DB" ]] || { echo "No working database at $WORK_DB" >&2; exit 1; }

mkdir -p "$MAP_DIR"
if [[ -f "$MAP_DIR/$NAME.db" ]]; then
    cp "$MAP_DIR/$NAME.db" "$MAP_DIR/$NAME.db.bak"
    echo "Previous map kept as $MAP_DIR/$NAME.db.bak"
fi
cp "$WORK_DB" "$MAP_DIR/$NAME.db"
echo "Saved $MAP_DIR/$NAME.db ($(du -h "$MAP_DIR/$NAME.db" | cut -f1))"
[[ "$NAME" == "rtabmap" ]] && echo "The stack will localize against it on next start. Delete it to map again."
