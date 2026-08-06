# Dockerfile for kinova_pick_planner
# Supports both Humble and Jazzy via build arg
#
# Build:
#   docker build -t kinova_pick_planner .
#   docker build --build-arg ROS_DISTRO=humble -t kinova_pick_planner .
#
# Run (needs host networking for ROS2 DDS discovery):
#   docker run -it --net=host kinova_pick_planner
#
# Run with specific command:
#   docker run -it --net=host kinova_pick_planner ros2 run kinova_pick_planner arm_controller

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-controller-manager \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ws
RUN mkdir -p src

# Clone and build pymoveit2
RUN cd src && git clone https://github.com/AndrejOrsula/pymoveit2.git
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-select pymoveit2 --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Copy kinova_pick_planner package
COPY . src/kinova_pick_planner/

# Install rosdep dependencies
RUN apt-get update && \
    rosdep install --ignore-src --from-paths src -y -r 2>/dev/null; \
    rm -rf /var/lib/apt/lists/*

# Build kinova_pick_planner
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source install/setup.bash && \
    colcon build --packages-select kinova_pick_planner"

# Source workspace on entry
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
