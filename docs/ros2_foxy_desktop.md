# Installing ROS 2 Foxy on a Server / Desktop Machine


ROS 2 does not support cross-distro communication reliably. The Jetson Nano
runs `dustynv/ros:foxy-ros-base-l4t-r32.7.1`, so this machine must also run
Foxy.

---

## Step 1: Verify configuration

### 1. Confirm OS

```bash
lsb_release -a
```

`osrf/ros:foxy-desktop` is based on **Ubuntu 20.04 (Focal)**. Running this
container on Ubuntu 20.04, 22.04, or 24.04 host machines is fine. Docker
handles the OS mismatch. The host just needs to be x86_64.

### 2. Confirm Docker is installed

```bash
docker --version
```

If missing:
```bash
sudo apt-get update
sudo apt-get install -y docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker $USER   # allows running docker without sudo (re-login required)
```

## Step 2: Docker image

### 1. Pull the image

```bash
docker pull osrf/ros:foxy-desktop
```

### 2. Verify the image pulled correctly

```bash
docker images | grep foxy
```

Expected size is roughly 2–3 GB. If suspiciously small, repull.

---

## Step 3: Run the container

Run `scripts/run_docker_server.sh` (from repo root) to start the container. The script includes the necessary Docker flags for display forwarding and host networking.

---

## Step 5: View a stream

```bash
# within the docker environment (at repo root)
colcon build

source install/setup.bash

ros2 run csi_jetson_pkg csi_subscriber_node \
--ros-args \
-p stream_id:=0 # change stream as needed
```
This will display the Jetson's video feed using OpenCV.