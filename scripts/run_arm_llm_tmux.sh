#!/usr/bin/env bash
# scripts/run_arm_llm_tmux.sh
# Starts the full arm-side pipeline on the laptop in a single tmux session.
# All windows run inside the auro-server Docker container.
#
#   Window 0 (llm)    — LLM node
#   Window 1 (kortex) — Kortex hardware driver (start first)
#   Window 2 (moveit) — MoveIt2 + RViz        (start after kortex is up)
#   Window 3 (arm)    — arm_controller         (waits 20s for MoveIt2)
#
# Usage:
#   ./scripts/run_arm_llm_tmux.sh [robot_ip]
#   robot_ip defaults to 192.168.1.10
#
# Prerequisites:
#   - Ollama running on the host:  ollama serve
#   - Desktop running run_perception_control_tmux.sh on the same ROS_DOMAIN_ID
#
# Tmux keys:
#   Ctrl-b 0-3  switch window
#   Ctrl-b d    detach  (container keeps running)
#   Ctrl-b [    scroll  (q to exit)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROBOT_IP="${1:-192.168.1.10}"
LAN_ID=0
IMAGE=auro-server:latest
CONTAINER=auro-laptop

# one sudo call — open docker socket so windows don't need sudo for docker exec
sudo chmod 666 /var/run/docker.sock

xhost +local:docker 2>/dev/null || true

# generate CycloneDDS config (bind to physical interfaces, skip docker bridges)
PHYS_IFACES=$(ip -4 addr show | awk '
  /^[0-9]+:/ { gsub(/:/, ""); iface = $2 }
  /inet / && iface != "lo" && iface !~ /^(docker|veth|br-)/ { print iface }
')
CYCLONE_XML="$SCRIPT_DIR/../cyclonedds_local.xml"
{
  echo '<?xml version="1.0" encoding="UTF-8" ?>'
  echo '<CycloneDDS xmlns="https://cdds.io/config"'
  echo '            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"'
  echo '            xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">'
  echo '  <Domain id="any">'
  echo '    <General>'
  echo '      <Interfaces>'
  for iface in $PHYS_IFACES; do
    echo "        <NetworkInterface name=\"${iface}\" multicast=\"true\"/>"
  done
  echo '      </Interfaces>'
  echo '    </General>'
  if [[ -n "${CYCLONE_PEERS}" ]]; then
    echo '    <Discovery>'
    echo '      <Peers>'
    for peer in ${CYCLONE_PEERS}; do
      echo "        <Peer address=\"${peer}\"/>"
    done
    echo '      </Peers>'
    echo '      <ParticipantIndex>auto</ParticipantIndex>'
    echo '    </Discovery>'
  fi
  echo '  </Domain>'
  echo '</CycloneDDS>'
} > "$CYCLONE_XML"
echo "CycloneDDS interfaces: $(echo $PHYS_IFACES | tr '\n' ' ')"
[[ -n "${CYCLONE_PEERS}" ]] && echo "CycloneDDS peers: ${CYCLONE_PEERS}"

# build image if not built
if ! sudo docker image inspect "$IMAGE" > /dev/null 2>&1; then
  echo "Building $IMAGE ..."
  sudo docker build -f "$SCRIPT_DIR/../Dockerfile.server" -t "$IMAGE" "$SCRIPT_DIR/.."
fi

# remove stale containers
sudo docker rm -f "$CONTAINER" 2>/dev/null || true

# start container
echo "Starting container $CONTAINER ..."
sudo docker run -d \
  --name "$CONTAINER" \
  --network host \
  -e DISPLAY="${DISPLAY}" \
  -e ROS_DOMAIN_ID="${LAN_ID}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/auro-final-project/cyclonedds_local.xml \
  -e PULSE_SERVER=unix:"${XDG_RUNTIME_DIR:-/run/user/1000}"/pulse/native \
  -v "${XDG_RUNTIME_DIR:-/run/user/1000}"/pulse/native:"${XDG_RUNTIME_DIR:-/run/user/1000}"/pulse/native \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$SCRIPT_DIR/..":/auro-final-project \
  -w /auro-final-project \
  --privileged \
  "$IMAGE" \
  sleep infinity

# rebuild workspace in container
echo "Building workspace inside container..."
sudo docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /kortex_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  cd /auro-final-project
  colcon build \
    --packages-select llm_node_pkg kinova_pick_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -10
"

# check ollama
if ! curl -sf http://localhost:11434 > /dev/null 2>&1; then
  echo ""
  echo "WARNING: Ollama is not running.  Start it with:  ollama serve"
  echo ""
fi

# write pane scripts
sudo docker exec "$CONTAINER" bash -c "
cat > /tmp/run_kortex.sh << 'SCRIPT'
source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash
export ROS_DOMAIN_ID=${LAN_ID}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch kortex_bringup gen3_lite.launch.py \
  robot_ip:=${ROBOT_IP} use_fake_hardware:=false launch_rviz:=false
exec bash
SCRIPT

cat > /tmp/run_moveit.sh << 'SCRIPT'
source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash
export ROS_DOMAIN_ID=${LAN_ID}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
  robot_ip:=${ROBOT_IP}
exec bash
SCRIPT

cat > /tmp/run_llm.sh << 'SCRIPT'
source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash
source /auro-final-project/install/setup.bash
export ROS_DOMAIN_ID=${LAN_ID}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run llm_node_pkg llm_node --ros-args -p use_voice:=false -p ollama_model:=llama3.2:3b
exec bash
SCRIPT

cat > /tmp/run_arm.sh << 'SCRIPT'
source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash
source /auro-final-project/install/setup.bash
export ROS_DOMAIN_ID=${LAN_ID}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo 'Waiting 20s for MoveIt2 to finish initialising...'
sleep 20
echo 'Setting joint trajectory controller goal timeout to 30s...'
ros2 param set /joint_trajectory_controller constraints.goal_time 30.0
echo 'Starting arm controller...'
#ros2 run kinova_pick_planner arm_controller_demo
#Instead of running demo data, run real pipeline with ros data
#ros2 run kinova_pick_planner arm_controller_ros
ros2 run kinova_pick_planner arm_controller
exec bash
SCRIPT

chmod +x /tmp/run_kortex.sh /tmp/run_moveit.sh /tmp/run_llm.sh /tmp/run_arm.sh
"

# kill existing sessions
tmux kill-session -t laptop 2>/dev/null || true

# build tmux sessions
tmux new-session -d -s laptop -n llm \
  "docker exec -it ${CONTAINER} bash /tmp/run_llm.sh; exec bash"

tmux new-window -t laptop -n kortex \
  "docker exec -it ${CONTAINER} bash /tmp/run_kortex.sh; exec bash"

tmux new-window -t laptop -n moveit \
  "docker exec -it ${CONTAINER} bash /tmp/run_moveit.sh; exec bash"

tmux new-window -t laptop -n arm \
  "docker exec -it ${CONTAINER} bash /tmp/run_arm.sh; exec bash"

tmux select-window -t laptop:0

tmux bind-key -T prefix k \
  confirm-before -p "Kill all laptop processes? (y/n)" \
  "run-shell 'docker stop ${CONTAINER} 2>/dev/null || true; tmux kill-session -t laptop'"

tmux attach -t laptop
