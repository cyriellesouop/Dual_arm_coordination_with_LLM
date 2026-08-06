#!/usr/bin/env bash
# scripts/jetson_scripts/setup_jetson_keys.sh
# Copy your SSH public key to each Jetson so that deploy_jetsons.sh and
# run_jetsons_tmux.sh work without password prompts.
#
# Run once per machine. Requires sshpass:
#   sudo apt-get install -y sshpass
#
# Usage:
#   ./scripts/setup_jetson_keys.sh [ip0] [ip1] ...
#
# If no IPs are passed, falls back to JETSON_IPS env var then the defaults below.
# Set the password via JETSON_PASSWORD env var or enter it when prompted.
#
# Knobs:
#   JETSON_USER        SSH username    (default: ssl-nano)
#   JETSON_PASSWORD    SSH password    (prompted if not set)

set -e

JETSON_USER="${JETSON_USER:-ssl-nano}"

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

if ! command -v sshpass &>/dev/null; then
    echo "ERROR: sshpass not found. Install it with: sudo apt-get install -y sshpass" >&2
    exit 1
fi

if [[ -z "${JETSON_PASSWORD}" ]]; then
    read -rs -p "Jetson password for ${JETSON_USER}: " JETSON_PASSWORD
    echo
fi

if [[ ! -f ~/.ssh/id_ed25519.pub ]] && [[ ! -f ~/.ssh/id_rsa.pub ]]; then
    echo "No SSH public key found. Generating one..."
    ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
fi

for IP in "${JETSON_IPS[@]}"; do
    echo "-> copying key to ${JETSON_USER}@${IP}"
    sshpass -p "${JETSON_PASSWORD}" ssh-copy-id \
        -o StrictHostKeyChecking=no \
        "${JETSON_USER}@${IP}"
done

echo "Done. SSH key auth is set up for all Jetsons."
