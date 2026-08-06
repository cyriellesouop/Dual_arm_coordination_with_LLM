#!/usr/bin/env bash
# scripts/run_arm_llm_tmux_mac.sh
# Mac (Docker Desktop / Apple Silicon) equivalent of run_arm_llm_tmux.sh.
#
# Differences from the Linux version:
#   - No `sudo docker` / docker.sock chmod (Docker Desktop's CLI doesn't need it)
#   - No `--network host` (Docker Desktop for Mac doesn't support it); the
#     container uses bridge networking. Outbound connections to the Kinova
#     arm (robot_ip) and to Ollama on the host still work fine over the bridge.
#   - Ollama runs on the host, so the container reaches it via
#     OLLAMA_HOST=http://host.docker.internal:11434 instead of localhost
#     (see src/llm_node_pkg/llm_node_pkg/llm_core.py, which reads OLLAMA_HOST)
#   - CycloneDDS interface discovery (`ip -4 addr show`) runs *inside* the
#     container, since macOS has no `ip` command
#   - MoveIt2 + RViz have no XQuartz/X11 path on macOS (stuck at OpenGL
#     1.4/1.5, RViz's Ogre renderer needs 2.1+). Instead this renders on a
#     virtual display (Xvfb) inside the container and shares it over VNC via
#     a noVNC bridge on the host, same approach as run_docker_server_mac.sh
#   - tmux runs *inside* the container (the host doesn't need tmux installed)
#
# Usage:
#   ./scripts/run_arm_llm_tmux_mac.sh [robot_ip]
#   robot_ip defaults to 192.168.2.12
#
# Prerequisites:
#   - Ollama running on the host:  ollama serve
#   - Desktop running run_perception_control_tmux_mac.sh (or the Linux
#     equivalent) on the same ROS_DOMAIN_ID
#
# Tmux keys (once attached):
#   Ctrl-b 0-3  switch window (llm / kortex / moveit / arm)
#   Ctrl-b d    detach  (container keeps running)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROBOT_IP="${1:-192.168.2.12}"
LAN_ID=0
IMAGE=auro-server:latest
CONTAINER=auro-laptop
NOVNC_DIR=/tmp/novnc
NOVNC_VENV=/tmp/novnc_venv

echo "==> Checking Docker Desktop..."
if ! docker info >/dev/null 2>&1; then
  echo "Starting Docker Desktop..."
  open -a Docker
  for i in $(seq 1 24); do
    docker info >/dev/null 2>&1 && break
    sleep 5
  done
  docker info >/dev/null 2>&1 || { echo "Docker did not start in time."; exit 1; }
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "Image $IMAGE not found. Build it first:"
  echo "  docker build -f Dockerfile.server -t $IMAGE ."
  exit 1
fi

if ! curl -sf http://localhost:11434 >/dev/null 2>&1; then
  echo ""
  echo "WARNING: Ollama is not running on the host.  Start it with:  ollama serve"
  echo ""
fi

# remove stale container
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

echo "==> Starting container $CONTAINER ..."
docker run -d \
  --name "$CONTAINER" \
  --network host \
  -e ROS_DOMAIN_ID="${LAN_ID}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/auro-final-project/cyclonedds_local.xml \
  -e OLLAMA_HOST=http://host.docker.internal:11434 \
  -v "$SCRIPT_DIR/..":/auro-final-project \
  -w /auro-final-project \
  "$IMAGE" \
  sleep infinity >/dev/null

# iproute2/xvfb/x11vnc are baked into the auro-server image (see Dockerfile.server)
# so this never needs apt at runtime -- the robot LAN (192.168.2.x) has no
# internet/DNS access, only the perception/arm containers' host machine does.
docker exec "$CONTAINER" bash -c "command -v ip >/dev/null" || {
  echo "ERROR: 'ip' not found in $IMAGE. Rebuild the image (it now includes iproute2):"
  echo "  docker build -f Dockerfile.server -t $IMAGE ."
  exit 1
}

# generate CycloneDDS config *inside* the container (macOS has no `ip` command)
echo "==> Generating CycloneDDS config..."
docker exec "$CONTAINER" bash -c "
  PHYS_IFACES=\$(ip -4 addr show | awk '
    /^[0-9]+:/ { gsub(/:/, \"\"); iface = \$2; sub(/@.*/, \"\", iface) }
    /inet / && iface != \"lo\" && iface !~ /^(docker|veth|br-)/ { print iface }
  ')
  {
    echo '<?xml version=\"1.0\" encoding=\"UTF-8\" ?>'
    echo '<CycloneDDS xmlns=\"https://cdds.io/config\"'
    echo '            xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"'
    echo '            xsi:schemaLocation=\"https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd\">'
    echo '  <Domain id=\"any\">'
    echo '    <General>'
    echo '      <Interfaces>'
    for iface in \$PHYS_IFACES; do
      echo \"        <NetworkInterface name=\\\"\${iface}\\\" multicast=\\\"true\\\"/>\"
    done
    echo '      </Interfaces>'
    echo '    </General>'
    if [ -n \"${CYCLONE_PEERS:-}\" ]; then
      echo '    <Discovery>'
      echo '      <Peers>'
      for peer in ${CYCLONE_PEERS:-}; do
        echo \"        <Peer address=\\\"\${peer}\\\"/>\"
      done
      echo '      </Peers>'
      echo '      <ParticipantIndex>auto</ParticipantIndex>'
      echo '    </Discovery>'
    fi
    echo '  </Domain>'
    echo '</CycloneDDS>'
  } > /auro-final-project/cyclonedds_local.xml
  echo 'CycloneDDS interfaces:' \$PHYS_IFACES
"
[[ -n "${CYCLONE_PEERS:-}" ]] && echo "CycloneDDS peers: ${CYCLONE_PEERS}"

docker exec "$CONTAINER" bash -c "command -v Xvfb >/dev/null && command -v x11vnc >/dev/null" || {
  echo "ERROR: Xvfb/x11vnc not found in $IMAGE. Rebuild the image (it now includes them):"
  echo "  docker build -f Dockerfile.server -t $IMAGE ."
  exit 1
}

echo "==> Starting virtual display (:99) and VNC server..."
docker exec -d "$CONTAINER" bash -c "Xvfb :99 -screen 0 1600x900x24 > /tmp/xvfb.log 2>&1"
sleep 2
docker exec -d "$CONTAINER" bash -c "x11vnc -display :99 -noshm -nopw -forever -shared -listen 0.0.0.0 -rfbport 5900 > /tmp/x11vnc.log 2>&1"
sleep 2

echo "==> Setting up noVNC bridge on host (one-time)..."
if [ ! -d "$NOVNC_VENV" ]; then
  python3 -m venv "$NOVNC_VENV"
  "$NOVNC_VENV/bin/pip" install --quiet websockify
fi
if [ ! -d "$NOVNC_DIR" ]; then
  git clone --quiet --depth 1 https://github.com/novnc/noVNC.git "$NOVNC_DIR"
fi

pkill -f "websockify --web $NOVNC_DIR 6081" 2>/dev/null || true
sleep 1
nohup "$NOVNC_VENV/bin/python" -m websockify --web "$NOVNC_DIR" 6081 127.0.0.1:5900 \
  > /tmp/websockify_laptop.log 2>&1 &
disown
sleep 2

# rebuild workspace in container
echo "==> Building workspace inside container..."
docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /kortex_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  cd /auro-final-project
  colcon build \
    --packages-select llm_node_pkg kinova_pick_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -10
"

# write pane scripts
docker exec "$CONTAINER" bash -c "
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
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
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
ros2 run llm_node_pkg llm_node --ros-args -p use_voice:=false -p ollama_model:=llama3.2:3b -p ollama_host:=\${OLLAMA_HOST}
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
ros2 run kinova_pick_planner arm_controller
exec bash
SCRIPT

chmod +x /tmp/run_kortex.sh /tmp/run_moveit.sh /tmp/run_llm.sh /tmp/run_arm.sh
"

# build tmux session inside the container
docker exec "$CONTAINER" tmux new-session -d -s laptop -n llm     'bash /tmp/run_llm.sh'
docker exec "$CONTAINER" tmux new-window  -t laptop   -n kortex  'bash /tmp/run_kortex.sh'
docker exec "$CONTAINER" tmux new-window  -t laptop   -n moveit  'bash /tmp/run_moveit.sh'
docker exec "$CONTAINER" tmux new-window  -t laptop   -n arm     'bash /tmp/run_arm.sh'
docker exec "$CONTAINER" tmux select-window -t laptop:0

echo ""
echo "==> RViz available at: http://localhost:6081/vnc.html?autoconnect=true&resize=scale"
open "http://localhost:6081/vnc.html?autoconnect=true&resize=scale"
echo ""
echo "==> Attaching to tmux session 'laptop' (Ctrl-b d to detach)..."
docker exec -it "$CONTAINER" tmux attach -t laptop
