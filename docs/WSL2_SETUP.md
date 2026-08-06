# WSL2 Desktop Setup Guide

How to get a Windows PC running WSL2 connected to the Jetsons on the LAN and viewing the overhead perception streams.

---

## Prerequisites

- Windows 10/11 with WSL2 installed
- Ubuntu 22.04 distro in WSL2
- ROS2 Humble installed inside WSL2
- LAN cable connecting PC to the same switch as the Jetsons
- WiFi for internet access

---

## 1. Enable Mirrored Networking

This makes WSL2 share Windows' network interfaces (both WiFi and LAN visible inside WSL).

Add to `C:\Users\<you>\.wslconfig`:
```ini
[wsl2]
networkingMode=mirrored
```

Then restart WSL from PowerShell:
```powershell
wsl --shutdown
```

---

## 2. Fix Interface Metrics on Windows (one-time)

When you plug in the LAN cable, Windows may set it as the default route and break internet access. Fix this by raising the Ethernet metric so WiFi stays preferred.

Open PowerShell as Administrator and check your interface names:
```powershell
Get-NetIPInterface | Select-Object InterfaceAlias, InterfaceMetric, AddressFamily | Where-Object AddressFamily -eq 'IPv4'
```

Find the Ethernet interface name (e.g. `Ethernet 5`) and set its metric high:
```powershell
Set-NetIPInterface -InterfaceAlias "Ethernet 5" -InterfaceMetric 100
```

Replace `"Ethernet 5"` with whatever your interface is actually called. This persists across reboots. After this, WiFi handles internet and Ethernet handles Jetson traffic automatically.

---

## 3. Windows Firewall Rule

The Jetsons send UDP packets for ROS2 DDS discovery. Windows Firewall blocks these by default.

Open PowerShell as Administrator:
```powershell
New-NetFirewallRule -DisplayName "ROS2 Jetsons" -Direction Inbound -Protocol UDP -RemoteAddress 192.168.1.0/24 -Action Allow
```

---

## 4. WSL2 Environment Setup

Inside WSL2, add the following to `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
source ~/auro-final-project/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jesse/cyclone_jetson.xml
```

Create `~/cyclone_jetson.xml`:
```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" multicast="true"/>
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer address="192.168.1.23"/>
        <Peer address="192.168.1.24"/>
        <Peer address="192.168.1.28"/>
        <Peer address="192.168.1.29"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

> Note: `eth0` is the LAN interface name inside WSL2 with mirrored networking. Update the Jetson IP addresses if they change.

Install CycloneDDS (must match what the Jetsons use):
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

---

## 5. Install Python Dependencies

```bash
sudo apt install python3-pip
pip3 install "numpy<2"
pip3 install "opencv-python==4.9.0.80"
pip3 install ultralytics
```

The `numpy<2` pin is required because `cv_bridge` in Humble was compiled against NumPy 1.x and crashes with NumPy 2+.

---

## 6. Build the Workspace

```bash
cd ~/auro-final-project
source /opt/ros/humble/setup.bash
colcon build --packages-select overhead_perception_pkg csi_jetson_pkg
source install/setup.bash
```

---

## 7. Open the Viewer

Open a WSL2 terminal from Windows Terminal (click the dropdown → Ubuntu-22.04), then:

```bash
view-overhead
```

This opens a 2×2 tiled window showing all 4 camera streams with YOLO bounding boxes overlaid. The window renders on the Windows desktop via WSLg.

Options:
```bash
view-overhead stream_ids:=0,1          # show only 2 cameras
view-overhead tile_width:=1280 tile_height:=720   # larger tiles
```

---

## Verify ROS2 is Seeing the Jetsons

```bash
ros2 daemon start
ros2 topic list
```

You should see topics like `/video/stream_0/compressed_jpeg`, `/perception/stream_0/detections_2d`, etc.

If topics are missing, check:
1. `ROS_DOMAIN_ID` matches on all devices (should be `0`)
2. LAN cable is plugged in
3. Jetsons are running (`ros2 topic list` from a Jetson should show the same topics)
