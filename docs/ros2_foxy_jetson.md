# Installing ROS 2 Foxy on the Jetson Nano 4GB Developer Kit

## Step 1: Verify configuration
### 1. Confirm your exact L4T version

```bash
head -n 1 /etc/nv_tegra_release
```

This should list **7.1** or **6.1** depending on the L4T version installed. The docker image pulled should match the system L4T (check the [NVIDIA Jetpack Archive](https://developer.nvidia.com/embedded/jetpack-archive)).

### 2. Confirm CUDA version

```bash
cat /usr/local/cuda/version.txt
```

Expected: `CUDA Version 10.2.300`. If CUDA is not installed, you will most likely need to reflash the Jetson with the correct SD card image from NVIDIA.

### 3. Confirm Docker is installed

```bash
docker --version
```

Docker comes with the default Jetson Jetpack SD card image. 
If missing, I recommend reflashing the Jetson with the supported image. But for quickly installing:
```bash
sudo apt-get update
sudo apt-get install -y docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker $USER   # allows running docker without sudo (re-login required)
```

### 4. Confirm the Docker runtime is NVIDIA

```bash
docker info | grep -i runtime
```

Expected output should include `nvidia` in the runtimes list. If missing:
```bash
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```
---
## Step 2: Docker image installation
### 1. Pull the correct base image

Check out these [ROS2 docker images](https://hub.docker.com/r/dustynv/ros/tags?page_size=&ordering=&name=r32.7.1) provided by NVIDIA. Some are intended for SLAM or pytorch, but I am just using the base ROS2 image below since it contains the necessary OpenCV libraries built for the Jetson.

Substitute `r32.X.X` with your confirmed L4T revision from the check above. The command below is for JetPack 4.6.1 (L4T r32.7.1).

```bash
docker pull dustynv/ros:foxy-ros-base-l4t-r32.7.1
```

This may take 10-20 minutes to pull depending on wifi speed.

### 2. Verify the image pulled correctly

```bash
docker images | grep foxy
```

Check the image size. Mine was roughly 2-3 GB after I pulled. If it is suspiciously small you may have to repull the image (after verifying you pusing OpenCV.ulled the correct tag).

---
## Step 3: Run docker & publish video streams
Run `scripts/run_docker_jetson.sh` to run the container. The shell script also includes necessary docker configurations for running ROS2 nodes within this project.


```bash
# within the docker environment (at repo root)
colcon build

source install/setup.bash

ros2 launch csi_jetson_pkg csi_jetson.launch.py # configure csi_params.yaml as needed in src/csi_jetson_pkg/config
```
This will publish the Jetson's video feed as a compressed jpeg over LAN.
