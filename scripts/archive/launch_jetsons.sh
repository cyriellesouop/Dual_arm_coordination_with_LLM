#!/usr/bin/env bash
# scripts/launch_jetsons.sh
#
# Usage:
#   ./scripts/launch_jetsons.sh              # interactive: opens tmux with one pane per Jetson
#   ./scripts/launch_jetsons.sh --detach     # headless: fires SSH commands in background (used by Docker)
#
# requirements: sudo apt install tmux sshpass
# each local must allow sudo with no password (add username ALL=(ALL) NOPASSWD: ALL to sudo visudo on each jetson)

DETACH=false
[[ "$1" == "--detach" ]] && DETACH=true

# sconfig
SSH_USER="ssl-nano"
PROJECT_DIR="auro-final-project"
SESSION_NAME="jetson_controller"
PASSWORD="ssl-nano"
JETSONS=(
    "192.168.1.22"
    "192.168.1.23"
    "192.168.1.27"
    "192.168.1.28"
)

REMOTE_PATH="/home/${SSH_USER}/${PROJECT_DIR}"

#tmux set -g mouse on # add tmux mouse support

# command to execute on the remote Jetson
REMOTE_PATH="/home/${SSH_USER}/${PROJECT_DIR}"

SCRIPT="
export LAN_ID=0;
cd ${REMOTE_PATH} || exit;

sudo docker run -it --rm \
    --runtime nvidia \
    --network host \
    -e ROS_DOMAIN_ID=\$LAN_ID \
    -v /tmp:/tmp \
    -v ${REMOTE_PATH}:/auro-final-project \
    -w /auro-final-project \
    dustynv/ros:foxy-ros-base-l4t-r32.7.1 \
    bash -c 'source install/setup.bash; ros2 launch csi_jetson_pkg csi_jetson.launch.py; exec bash'
"

# clean up remote docker containers on exit
cleanup() {
    echo "Cleaning up remote sessions..."
    for ip in "${JETSONS[@]}"; do
        sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no "${SSH_USER}@${ip}" \
            "docker ps -q | xargs -r docker stop" 2>/dev/null &
    done
    wait
}
trap cleanup EXIT INT TERM

# kill existing tmux terminals
tmux kill-session -t "$SESSION_NAME" 2>/dev/null

# start new session on first jetson
# -t flag is included in the SSH command to provide the TTY for sudo
tmux new-session -d -s "$SESSION_NAME" -n "Main" \
    "sshpass -p '$PASSWORD' ssh -t -o StrictHostKeyChecking=no ${SSH_USER}@${JETSONS[0]} \"$SCRIPT\""

# loop through remaining Jetsons and create horizontal splits
for i in "${!JETSONS[@]}"; do
    if [ "$i" -gt 0 ]; then
        tmux split-window -h -t "$SESSION_NAME" \
            "sshpass -p '$PASSWORD' ssh -t -o StrictHostKeyChecking=no ${SSH_USER}@${JETSONS[$i]} \"$SCRIPT\""
    fi
done

# even out panes
tmux select-layout -t "$SESSION_NAME" even-horizontal
tmux attach-session -t "$SESSION_NAME"