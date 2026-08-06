#!/usr/bin/env bash
# jetson/run_rtsp_server.sh
# Grant Hillman 4/25/2026
# 
# Wrapper to launch rtsp_server.py on a device over local network.
#
# Prerequisites (run once per Jetson):
#   sudo apt-get install -y python3-gi gir1.2-gst-rtsp-server-1.0
#
# Usage:
#   ./run_rtsp_server.sh
#   SENSOR_ID=1 WIDTH=640 HEIGHT=480 ./run_rtsp_server.sh

set -e

SENSOR_ID="${SENSOR_ID:-0}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-4000000}"
PORT="${PORT:-8554}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "${SCRIPT_DIR}/rtsp_server.py" \
    --sensor-id "${SENSOR_ID}" \
    --width     "${WIDTH}"     \
    --height    "${HEIGHT}"    \
    --fps       "${FPS}"       \
    --bitrate   "${BITRATE}"   \
    --port      "${PORT}"
