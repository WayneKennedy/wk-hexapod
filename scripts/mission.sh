#!/bin/bash
# Send mission commands to the robot's mission API from any machine with curl.
# The API is served by the web dashboard node (hexapod_perception/web_dashboard).
#
# Usage:
#   scripts/mission.sh [-H host[:port]] state
#   scripts/mission.sh [-H host[:port]] start [explore|patrol|navigate|return_home] [timeout_sec]
#   scripts/mission.sh [-H host[:port]] stop [--return-home]
#   scripts/mission.sh [-H host[:port]] status        # full dashboard status (battery, faces, ...)
#
# Host defaults to $HEXAPOD_HOST, then spid:8080.
set -e

HOST="${HEXAPOD_HOST:-spid:8080}"
if [[ "${1:-}" == "-H" ]]; then
    HOST="$2"; shift 2
fi
[[ "$HOST" == *:* ]] || HOST="$HOST:8080"
BASE="http://$HOST"

cmd="${1:-state}"; shift || true
case "$cmd" in
    state)
        curl -sS "$BASE/api/autonomy/state"; echo ;;
    status)
        curl -sS "$BASE/status"; echo ;;
    start)
        type="${1:-explore}"; timeout="${2:-0}"
        curl -sS -X POST "$BASE/api/mission/start" -H 'Content-Type: application/json' \
            -d "{\"mission_type\": \"$type\", \"timeout_sec\": $timeout}"; echo ;;
    stop)
        rh=false; [[ "${1:-}" == "--return-home" ]] && rh=true
        curl -sS -X POST "$BASE/api/mission/stop" -H 'Content-Type: application/json' \
            -d "{\"return_home\": $rh}"; echo ;;
    *)
        sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
