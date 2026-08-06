#!/usr/bin/env bash
# scripts/run_docker_jetson.sh

export LAN_ID=0 # should match all devices on same LAN

sudo docker run -it --rm \
    --runtime nvidia \
    --network host \
    -e ROS_DOMAIN_ID=$LAN_ID \
    -v /tmp:/tmp \
    -v $(pwd):/auro-final-project \
    -w /auro-final-project \
    dustynv/ros:foxy-ros-base-l4t-r32.7.1

