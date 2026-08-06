#!/usr/bin/env bash
# setup.sh — Install dependencies and build kinova_pick_planner
#
# Usage:
#   cd <your_kinova_ws>
#   ./src/kinova_pick_planner/setup.sh
#
# Supports: Ubuntu 22.04 (Humble) / Ubuntu 24.04 (Jazzy)

set -e

info()  { echo -e "\033[1;34m[setup]\033[0m $*"; }
ok()    { echo -e "\033[1;32m[setup]\033[0m $*"; }
warn()  { echo -e "\033[1;33m[setup]\033[0m $*"; }
err()   { echo -e "\033[1;31m[setup]\033[0m $*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# Detect ROS2 distro
# ---------------------------------------------------------------------------
if [ -z "$ROS_DISTRO" ]; then
    for distro in jazzy humble; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            source "/opt/ros/$distro/setup.bash"
            break
        fi
    done
fi

if [ -z "$ROS_DISTRO" ]; then
    err "No ROS2 installation found. Install ROS2 Jazzy or Humble first."
fi

info "Detected ROS2 distro: $ROS_DISTRO"

# ---------------------------------------------------------------------------
# Find workspace root (look for src/ directory)
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Walk up to find workspace root (directory containing src/)
WS_ROOT="$SCRIPT_DIR"
while [ "$WS_ROOT" != "/" ]; do
    if [ -d "$WS_ROOT/src" ] && [ "$(basename "$WS_ROOT")" != "src" ]; then
        break
    fi
    WS_ROOT="$(dirname "$WS_ROOT")"
done

if [ ! -d "$WS_ROOT/src" ]; then
    err "Could not find workspace root. Run from within your colcon workspace."
fi

info "Workspace root: $WS_ROOT"
cd "$WS_ROOT"

# ---------------------------------------------------------------------------
# Install system dependencies
# ---------------------------------------------------------------------------
info "Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-controller-manager \
    > /dev/null 2>&1 || warn "Some apt packages may not be available"
ok "System dependencies installed."

# ---------------------------------------------------------------------------
# Install pymoveit2 (from source if not already present)
# ---------------------------------------------------------------------------
if python3 -c "from pymoveit2 import MoveIt2" 2>/dev/null; then
    info "pymoveit2 already available."
else
    if [ ! -d "$WS_ROOT/src/pymoveit2" ]; then
        info "Cloning pymoveit2..."
        cd "$WS_ROOT/src"
        git clone https://github.com/AndrejOrsula/pymoveit2.git
        cd "$WS_ROOT"
    fi
    info "Building pymoveit2..."
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    colcon build --packages-select pymoveit2 --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
    ok "pymoveit2 installed."
fi

# ---------------------------------------------------------------------------
# Install ros2_kortex (Kinova drivers) if not present
# ---------------------------------------------------------------------------
if [ -d "$WS_ROOT/src/ros2_kortex" ]; then
    info "ros2_kortex already present."
else
    warn "ros2_kortex not found in workspace."
    warn "Clone it manually if you need the robot driver:"
    warn "  cd $WS_ROOT/src"
    warn "  git clone https://github.com/Kinovarobotics/ros2_kortex.git"
    warn "  vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.${ROS_DISTRO}.repos"
    warn "  cd $WS_ROOT && rosdep install --ignore-src --from-paths src -y -r"
fi

# ---------------------------------------------------------------------------
# Install rosdep dependencies
# ---------------------------------------------------------------------------
info "Installing rosdep dependencies..."
rosdep install --ignore-src --from-paths src -y -r 2>/dev/null || warn "Some rosdep dependencies may be missing"

# ---------------------------------------------------------------------------
# Build kinova_pick_planner
# ---------------------------------------------------------------------------
info "Building kinova_pick_planner..."
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
fi
colcon build --packages-select kinova_pick_planner
source install/setup.bash
ok "kinova_pick_planner built."

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo
echo "============================================================"
ok "Setup complete!"
echo
echo "  ROS2 distro: $ROS_DISTRO"
echo "  Workspace:   $WS_ROOT"
echo
echo "  Quick start:"
echo "    # Terminal 1: Launch MoveIt2 + robot"
echo "    source $WS_ROOT/install/setup.bash"
echo "    ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \\"
echo "        robot_ip:=192.168.1.10 use_fake_hardware:=false"
echo
echo "    # Terminal 2: Set controller tolerance"
echo "    ros2 param set /joint_trajectory_controller constraints.goal_time 30.0"
echo
echo "    # Terminal 3: Run arm controller"
echo "    source $WS_ROOT/install/setup.bash"
echo "    ros2 run kinova_pick_planner arm_controller"
echo
echo "  See README.md for full documentation."
echo "============================================================"
