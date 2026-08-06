#!/usr/bin/env bash
# scripts/run_docker_server.sh

export LAN_ID=0

xhost +local:docker

IMAGE=auro-server:latest

if ! sudo docker image inspect "$IMAGE" > /dev/null 2>&1; then
  echo "Building $IMAGE..."
  sudo docker build -f "$(dirname "$0")/../Dockerfile.server" -t "$IMAGE" "$(dirname "$0")/.."
fi

sudo docker run -it \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=$LAN_ID \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/auro-final-project \
  -w /auro-final-project \
  --privileged \
  "$IMAGE" \
  bash -c "source /auro-final-project/scripts/setup_workspace.sh && exec bash"