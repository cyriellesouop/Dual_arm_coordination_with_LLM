#!/usr/bin/env bash
# scripts/jetson_scripts/stop_jetsons.sh
# Kill rtsp_server.py on each Jetson and close the local tmux session.
#
# Usage:
#   ./scripts/stop_jetsons.sh [ip0] [ip1] ...
#
# Same IP resolution as run_jetsons_tmux.sh — pass IPs as args or set JETSON_IPS.

set -e

JETSON_USER="${JETSON_USER:-ssl-nano}"
SESSION=jetsons

if [[ $# -gt 0 ]]; then
    JETSON_IPS=("$@")
elif [[ -n "${JETSON_IPS}" ]]; then
    read -ra JETSON_IPS <<< "${JETSON_IPS}"
else
    JETSON_IPS=(
        "192.168.2.22"
        "192.168.2.23"
        "192.168.2.25"
        "192.168.2.26"
    )
fi

for IP in "${JETSON_IPS[@]}"; do
    echo "Stopping rtsp_server.py on ${IP}..."
    ssh "${JETSON_USER}@${IP}" "pkill -f rtsp_server.py || true" 2>/dev/null && \
        echo "  stopped" || echo "  not running (or SSH failed)"
done

tmux kill-session -t "$SESSION" 2>/dev/null && \
    echo "tmux session '$SESSION' closed." || \
    echo "tmux session '$SESSION' was not running."
