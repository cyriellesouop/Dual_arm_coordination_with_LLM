#!/usr/bin/env bash
# scripts/helpers/run_docker_server.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

export LAN_ID=0

# Build CycloneDDS config with all physical interfaces (excludes lo, docker*, veth*, br-*).
# Without this, CycloneDDS may latch onto docker0 and ros2 topic list hangs.
PHYS_IFACES=$(ip -4 addr show | awk '
  /^[0-9]+:/ { gsub(/:/, ""); iface = $2 }
  /inet / && iface != "lo" && iface !~ /^(docker|veth|br-)/ { print iface }
')

echo "CycloneDDS interfaces: $(echo $PHYS_IFACES | tr '\n' ' ')"

CYCLONE_XML="$PROJECT_ROOT/cyclonedds_local.xml"
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
[[ -n "${CYCLONE_PEERS}" ]] && echo "CycloneDDS peers: ${CYCLONE_PEERS}"

xhost +local:docker

IMAGE=auro-server:latest

if ! sudo docker image inspect "$IMAGE" > /dev/null 2>&1; then
  echo "Building $IMAGE..."
  sudo docker build -f "$PROJECT_ROOT/Dockerfile.server" -t "$IMAGE" "$PROJECT_ROOT"
fi

# detect GPU
GPU_FLAGS=""
nvidia-smi &>/dev/null && GPU_FLAGS="--gpus all"

sudo docker run -it --rm \
  --network host \
  $GPU_FLAGS \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=$LAN_ID \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/auro-final-project/cyclonedds_local.xml \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PROJECT_ROOT":/auro-final-project \
  -w /auro-final-project \
  --privileged \
  "$IMAGE" \
  bash -c "source /auro-final-project/scripts/helpers/setup_workspace.sh && exec bash"
