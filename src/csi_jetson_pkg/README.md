# csi_jetson_pkg

**Not used in the current implementation.** This package is kept as a reference for a ROS2-on-Jetson approach explored during development and later replaced.

---

## Background

An early version of the camera pipeline ran ROS2 directly on each Jetson Nano, publishing CSI frames as ROS2 topics over the network to the server. This required installing ROS2 Humble on Jetson OS (Ubuntu 20.04 / L4T), which has no official binary packages and requires a full build from source.

The current system avoids ROS2 on the Jetsons entirely. Each Jetson runs `jetson/rtsp_server.py` (plain Python, no ROS), and `jetson_bridge` on the server opens the RTSP streams and publishes them as ROS2 topics. This approach only needs `python3-gi` and standard GStreamer packages on the Jetson side, all available as apt binaries.

---

## Nodes (unused)

### `csi_publisher_node`

Runs on a Jetson Nano. Captures from a CSI camera via `nvarguscamerasrc` (Jetson multimedia API) and publishes JPEG-compressed frames to `/video/stream_{id}/compressed_jpeg`.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `camera_id` | int | `0` | Sensor ID passed to `nvarguscamerasrc` |
| `stream_id` | int | `0` | Used to name the output topic |
| `width` | int | `1280` | Capture width |
| `height` | int | `720` | Capture height |
| `fps` | int | `30` | Capture frame rate |
| `jpeg_quality` | int | `95` | JPEG encode quality (0-100) |

### `camera_bridge_node`

Runs on the server. Subscribes to `/stream_{id}/compressed_jpeg`, decodes JPEG frames with OpenCV, and republishes as `sensor_msgs/Image` to `/video/stream_{id}/raw`. Serves the same role as `jetson_bridge/camera_bridge_node` in the current implementation.

### `csi_subscriber_node`

Displays one or more `/video/stream_{id}/raw` topics in a tiled OpenCV window at 30 Hz. Functionally equivalent to `rtsp_viewer`, which is the package used in the current system.

---

## Launch files (unused)

| File | Description |
|---|---|
| `launch/csi_jetson.launch.py` | Starts `csi_publisher_node` on a Jetson |
| `launch/camera_bridge.launch.py` | Starts `camera_bridge_node` on the server |
| `launch/csi_viewer.launch.py` | Starts `csi_subscriber_node` for display |
