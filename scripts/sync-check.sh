#!/usr/bin/env bash
# Check that this checkout and the robot's are in sync (DEC-33): both clean, both at the
# same commit, and that commit is origin/main. Run on the workstation before anything is
# started or restarted on the robot. It changes nothing on either side.
#
#   scripts/sync-check.sh [ssh-host]     # defaults to $HEXAPOD_HOST without its port
#
# HEXAPOD_REPO is the checkout's path on the robot (default Code/wk-hexapod, from $HOME).
# Exit 0: in sync. Exit 1: not in sync. Exit 2: could not check.

set -uo pipefail

HOST="${1:-${HEXAPOD_HOST:-}}"
HOST="${HOST%%:*}"
if [ -z "$HOST" ]; then
    echo "usage: $0 [ssh-host]   (or set HEXAPOD_HOST)" >&2
    exit 2
fi
REMOTE_DIR="${HEXAPOD_REPO:-Code/wk-hexapod}"
cd "$(dirname "$0")/.." || exit 2

git fetch -q origin || { echo "could not fetch origin" >&2; exit 2; }
origin_head=$(git rev-parse origin/main)
local_head=$(git rev-parse HEAD)
local_dirty=$(git status --porcelain | wc -l)

robot=$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$HOST" \
    "cd $REMOTE_DIR && git rev-parse HEAD && git status --porcelain | wc -l") \
    || { echo "could not read the robot's checkout" >&2; exit 2; }
robot_head=$(sed -n 1p <<<"$robot")
robot_dirty=$(sed -n 2p <<<"$robot")

printf '%-12s %s\n' origin/main "$origin_head"
printf '%-12s %s  %s uncommitted\n' workstation "$local_head" "$local_dirty"
printf '%-12s %s  %s uncommitted\n' robot "$robot_head" "$robot_dirty"

if [ "$local_head" = "$origin_head" ] && [ "$robot_head" = "$origin_head" ] \
    && [ "$local_dirty" -eq 0 ] && [ "$robot_dirty" -eq 0 ]; then
    echo "IN SYNC"
    exit 0
fi
echo "NOT IN SYNC: do not start or restart anything on the robot (DEC-33)"
exit 1
