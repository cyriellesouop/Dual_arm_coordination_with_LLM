#!/usr/bin/env bash
# scripts/helpers/run_docker_server_mac.sh
#
# Mac (Apple Silicon / Docker Desktop) equivalent of run_docker_server.sh.
#
# XQuartz/X11 forwarding does not work for RViz on macOS: XQuartz's indirect
# GLX implementation is stuck at OpenGL 1.4/1.5 and RViz's Ogre renderer needs
# 2.1+. Instead this script renders everything inside the container on a
# virtual display (Xvfb) using Mesa software rendering, then shares that
# display over VNC. A noVNC bridge (websockify) turns the VNC feed into a
# plain browser tab, so no native macOS VNC client is required.
#
# Prerequisites (one-time):
#   - Docker Desktop installed and running
#   - auro-server:latest image already built:
#       docker build -f Dockerfile.server -t auro-server:latest .
#
# Usage:
#   ./scripts/helpers/run_docker_server_mac.sh [robot_ip]
#   robot_ip defaults to 192.168.1.10 (irrelevant in fake-hardware mode, but
#   still a required launch argument).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROBOT_IP="${1:-192.168.2.12}"
CONTAINER=auro-dev
IMAGE=auro-server:latest
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

echo "==> (Re)starting container $CONTAINER..."
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
docker run -d --name "$CONTAINER" \
  -p 5900:5900 \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$PROJECT_ROOT":/auro-final-project \
  -w /auro-final-project \
  "$IMAGE" sleep infinity >/dev/null

# Xvfb/x11vnc are baked into the auro-server image (see Dockerfile.server)
docker exec "$CONTAINER" bash -c "command -v Xvfb >/dev/null && command -v x11vnc >/dev/null" || {
  echo "Xvfb/x11vnc not found in $IMAGE. Rebuild the image (it now includes them):"
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

# Kill any previous websockify bridge before starting a fresh one.
pkill -f "websockify --web $NOVNC_DIR" 2>/dev/null || true
sleep 1
nohup "$NOVNC_VENV/bin/python" -m websockify --web "$NOVNC_DIR" 6080 127.0.0.1:5900 \
  > /tmp/websockify.log 2>&1 &
disown
sleep 2

echo "==> Launching Kortex driver (fake hardware)..."
docker exec -d "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /kortex_ws/install/setup.bash
  export DISPLAY=:99
  ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=${ROBOT_IP} use_fake_hardware:=true fake_sensor_commands:=true launch_rviz:=false > /tmp/kortex.log 2>&1
"

echo "==> Waiting for /joint_states..."
docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /kortex_ws/install/setup.bash
  for i in \$(seq 1 20); do
    ros2 topic list 2>/dev/null | grep -q joint_states && exit 0
    sleep 3
  done
  exit 1
" || { echo "Kortex driver did not come up in time; check /tmp/kortex.log inside the container."; exit 1; }

echo "==> Launching MoveIt2 + RViz..."
docker exec -d "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /kortex_ws/install/setup.bash
  export DISPLAY=:99
  export LIBGL_ALWAYS_SOFTWARE=1
  ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=${ROBOT_IP} use_fake_hardware:=true launch_rviz:=true > /tmp/moveit.log 2>&1
"
sleep 15

echo "==> Rebuilding kinova_pick_planner and adding table + simulated objects..."
docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /kortex_ws/install/setup.bash
  cd /auro-final-project
  pip install --no-cache-dir -U packaging --quiet
  colcon build --packages-select kinova_pick_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
"
docker exec -d "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /kortex_ws/install/setup.bash
  source /auro-final-project/install/setup.bash
  export DISPLAY=:99
  ros2 run kinova_pick_planner arm_controller_demo > /tmp/arm_controller.log 2>&1
"

echo "==> Opening RViz in your browser..."
open "http://localhost:6080/vnc.html?autoconnect=true&resize=scale"

echo ""
echo "Done. If RViz doesn't show the table, add Displays > Add > PlanningScene in RViz."
echo "Debug logs inside the container: /tmp/kortex.log /tmp/moveit.log /tmp/arm_controller.log"
echo "To get a shell in the running container: docker exec -it $CONTAINER bash"
