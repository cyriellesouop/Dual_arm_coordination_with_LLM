#!/usr/bin/env bash
# scripts/run_jetsons_tmux.sh
# SSH into each Jetson Nano and start the RTSP server.
# Each stream gets its own tmux window (not a pane) — navigate with:
#   Ctrl-b [0-3]   jump to window by index
#   Ctrl-b n / p   next / previous window
#   Ctrl-b d       detach  (streams keep running)
#   Ctrl-b [       scroll  (q to exit)
#
# Usage:
#   ./scripts/run_jetsons_tmux.sh [ip0] [ip1] [ip2] [ip3]
#
# If no IPs are passed, JETSON_IPS env var is used (space-separated).
# Falls back to the defaults below — edit them to match your LAN.
#
# Per-Jetson env knobs (set before calling this script):
#   JETSON_USER        SSH username on the Jetsons     (default: ssl-nano)
#   JETSON_REPO_PATH   path to this repo on the Jetson (default: ~/rtsp_streaming)
#   SENSOR_ID          CSI sensor-id passed to rtsp_server.py (default: 0, same for all)
#
# Prerequisites on each Jetson (run once):
#   sudo apt-get install -y python3-gi gir1.2-gst-rtsp-server-1.0

set -e

# default settings
JETSON_USER="${JETSON_USER:-ssl-nano}"
JETSON_REPO_PATH="${JETSON_REPO_PATH:-/home/${JETSON_USER}/rtsp_streaming}"
SENSOR_ID="${SENSOR_ID:-0}"
SESSION=jetsons

# input jetson ips
if [[ $# -gt 0 ]]; then
    JETSON_IPS=("$@")
elif [[ -n "${JETSON_IPS}" ]]; then
    read -ra JETSON_IPS <<< "${JETSON_IPS}"
else
    # default IPs
    JETSON_IPS=(
        "192.168.2.22"   # stream 0
        "192.168.2.23"   # stream 1
        "192.168.2.25"   # stream 2
        "192.168.2.26"   # stream 3
    )
fi

  #     "192.168.2.22"   # stream 0
   #     "192.168.2.23"   # stream 1
    #    "192.168.2.27"   # stream 2
     #   "192.168.2.28"   # stream 3

NUM_JETSONS=${#JETSON_IPS[@]}

# kill stale sessions
tmux kill-session -t "$SESSION" 2>/dev/null || true

# first window to create session
IP="${JETSON_IPS[0]}"
tmux new-session -d -s "$SESSION" -n "stream_0 | ${IP}"
tmux send-keys -t "${SESSION}:0" \
    "ssh -t ${JETSON_USER}@${IP} 'python3 ${JETSON_REPO_PATH}/rtsp_server.py --sensor-id ${SENSOR_ID}'; exec bash" \
    Enter

# make additional windows
for (( i=1; i<NUM_JETSONS; i++ )); do
    IP="${JETSON_IPS[$i]}"
    tmux new-window -t "$SESSION" -n "stream_${i} | ${IP}"
    tmux send-keys -t "${SESSION}:${i}" \
        "ssh -t ${JETSON_USER}@${IP} 'python3 ${JETSON_REPO_PATH}/rtsp_server.py --sensor-id ${SENSOR_ID}'; exec bash" \
        Enter
done

# focus first window
tmux select-window -t "${SESSION}:0"
tmux attach -t "$SESSION"
