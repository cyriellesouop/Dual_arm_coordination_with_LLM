#!/bin/bash
# Source the kortex underlay, build project packages, and source the result.
# Run automatically on container start via run_docker_server.sh.

set -e

source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash

cd /auro-final-project

echo "[setup] Building project workspace..."
colcon build \
  --packages-select jetson_bridge overhead_perception auro_controller llm_node_pkg kinova_pick_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source /auro-final-project/install/setup.bash
echo "[setup] Workspace ready."
