#!/usr/bin/env bash
# scripts/jetson_scripts/deploy_jetsons.sh
# Push the jetson/ directory to each Jetson Nano via rsync.
# Only transfers files that have changed — safe to run repeatedly.
#
# Usage:
#   ./scripts/jetson_scripts/deploy_jetsons.sh [ip0] [ip1] ...
#
# If no IPs are passed, JETSON_IPS env var is used (space-separated).
# Falls back to the defaults below.
#
# Knobs:
#   JETSON_USER        SSH username      (default: user)
#   JETSON_DEST        destination path  (default: ~/rtsp_streaming)

set -e
trap 'echo "ERROR: deploy_jetsons.sh failed on line ${LINENO}" >&2' ERR

JETSON_USER="${JETSON_USER:-ssl-nano}"
JETSON_DEST="${JETSON_DEST:-/home/${JETSON_USER}/rtsp_streaming}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${SCRIPT_DIR}/../jetson/"

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
    echo "-> ${JETSON_USER}@${IP}:${JETSON_DEST}/"
    ssh "${JETSON_USER}@${IP}" "mkdir -p ${JETSON_DEST}"
    rsync -az --checksum \
        "${SRC}" \
        "${JETSON_USER}@${IP}:${JETSON_DEST}/"
done

echo "Done."
