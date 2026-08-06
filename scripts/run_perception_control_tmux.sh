#!/usr/bin/env bash
# scripts/run_perception_control_tmux.sh
# Starts the full AURO perception + controller pipeline in a single tmux session.
#
# Usage:
#   ./scripts/run_perception_control_tmux.sh           # uses stream_ids 0,1
#   ./scripts/run_perception_control_tmux.sh 0,1,2,3   # use more Jetson streams
#
# Tmux keys:
#   Ctrl-b 0-3  : jump to pane (bridge / detector / localizer / controller)
#   Ctrl-b n/p  : next / prev pane
#   Ctrl-b d    : detach  (container keeps running)
#   Ctrl-b [    : scroll mode  (q to exit)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
STREAM_IDS="${1:-0,1}"
LAN_ID=0
IMAGE=auro-server:latest
CONTAINER=auro-pipeline

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

# build image if missing
if ! sudo docker image inspect "$IMAGE" > /dev/null 2>&1; then
  echo "Building $IMAGE ..."
  sudo docker build -f "$SCRIPT_DIR/../Dockerfile.server" -t "$IMAGE" "$SCRIPT_DIR/.."
fi

# remove stale containers
sudo docker rm -f "$CONTAINER" 2>/dev/null || true

# detect GPU
GPU_FLAGS=""
nvidia-smi &>/dev/null && GPU_FLAGS="--gpus all"

# start container
echo "Starting container $CONTAINER ..."
sudo docker run -d \
  --name "$CONTAINER" \
  --network host \
  $GPU_FLAGS \
  -e DISPLAY="${DISPLAY}" \
  -e ROS_DOMAIN_ID="${LAN_ID}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/auro-final-project/cyclonedds_local.xml \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$SCRIPT_DIR/..":/auro-final-project \
  -w /auro-final-project \
  --privileged \
  "$IMAGE" \
  sleep infinity

# rebuild workspace
echo "Building workspace inside container..."
sudo docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /kortex_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  cd /auro-final-project
  colcon build \
    --packages-select jetson_bridge overhead_perception auro_controller llm_node_pkg kinova_pick_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -10
"





# per pane launch 
sudo docker exec "$CONTAINER" bash -c "
cat > /tmp/run_bridge.sh << 'SCRIPT'
source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash
source /auro-final-project/install/setup.bash
export ROS_DOMAIN_ID=${LAN_ID}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch jetson_bridge camera_bridge.launch.py stream_ids:=${STREAM_IDS}
exec bash
SCRIPT

cat > /tmp/run_detector.sh << 'SCRIPT'
source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash
source /auro-final-project/install/setup.bash
export ROS_DOMAIN_ID=${LAN_ID}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch overhead_perception detector_only.launch.py stream_ids:=${STREAM_IDS}
exec bash
SCRIPT

cat > /tmp/run_localizer.sh << 'SCRIPT'
source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash
source /auro-final-project/install/setup.bash
export ROS_DOMAIN_ID=${LAN_ID}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch overhead_perception object_localization.launch.py stream_ids:=${STREAM_IDS}
exec bash
SCRIPT

cat > /tmp/run_controller.sh << 'SCRIPT'
source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash
source /auro-final-project/install/setup.bash
export ROS_DOMAIN_ID=${LAN_ID}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch auro_controller controller.launch.py enable_arm:=true
exec bash
SCRIPT

chmod +x /tmp/run_bridge.sh /tmp/run_detector.sh /tmp/run_localizer.sh /tmp/run_controller.sh
"

# build tmux session
sudo docker exec "$CONTAINER" \
  tmux new-session -d -s auro -n bridge     'bash /tmp/run_bridge.sh'
sudo docker exec "$CONTAINER" \
  tmux new-window  -t auro   -n detector    'bash /tmp/run_detector.sh'
sudo docker exec "$CONTAINER" \
  tmux new-window  -t auro   -n localizer   'bash /tmp/run_localizer.sh'
sudo docker exec "$CONTAINER" \
  tmux new-window  -t auro   -n controller  'bash /tmp/run_controller.sh'

sudo docker exec -it "$CONTAINER" tmux attach -t auro
