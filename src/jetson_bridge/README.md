# jetson_bridge

ROS2 Humble package that runs on the server/laptop. Opens one RTSP stream per Jetson Nano, decodes H.264 with GStreamer, and publishes each stream as `sensor_msgs/Image` over CycloneDDS. The Jetsons run Python RTSP server (`jetson/rtsp_server.py`).

---

## Node: `camera_bridge_node`

Spawns one background thread per stream. Each thread opens a GStreamer `rtspsrc` pipeline via `cv2.VideoCapture(..., cv2.CAP_GSTREAMER)`, reads frames continuously, and publishes them as `sensor_msgs/Image` (bgr8). On read failure the capture is released and the thread reconnects automatically after 3 s.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `stream_ids` | string | `"0"` | Comma-separated stream IDs |
| `rtsp_urls` | string | `""` | Comma-separated RTSP URLs, one per stream ID in the same order |
| `output_prefix` | string | `"/rtsp/stream_"` | Topic prefix; full topic = `{output_prefix}{id}/raw` |
| `rtspsrc_latency` | int | `50` | GStreamer `rtspsrc` jitter buffer in ms (`0` = minimum latency) |

### Published topics

| Topic | Type | QoS |
|---|---|---|
| `/rtsp/stream_{id}/raw` | `sensor_msgs/Image` (bgr8) | BEST_EFFORT, KEEP_LAST, depth 1 |

One topic per stream ID. No subscriptions — the node pulls from RTSP directly.

---

## GStreamer pipeline

One pipeline per stream:

```
rtspsrc location=<url> latency=<ms> protocols=tcp
  ! rtph264depay ! h264parse
  ! avdec_h264 max-threads=2
  ! videoconvert
  ! video/x-raw, format=BGR
  ! appsink drop=true max-buffers=1 sync=false
```

- `protocols=tcp` — avoids UDP fragmentation on LAN.
- `appsink drop=true max-buffers=1 sync=false` — always delivers the freshest frame; stale frames are dropped if the publisher falls behind.

---

## Launch

### Launch file

```bash
ros2 launch jetson_bridge camera_bridge.launch.py \
  stream_ids:="0,1,2,3" \
  rtsp_urls:="rtsp://192.168.1.22:8554/stream0,rtsp://192.168.1.23:8554/stream0,rtsp://192.168.1.24:8554/stream0,rtsp://192.168.1.25:8554/stream0"
```

### Direct

```bash
ros2 run jetson_bridge camera_bridge_node \
  --ros-args \
  -p stream_ids:="0,1" \
  -p rtsp_urls:="rtsp://192.168.1.22:8554/stream0,rtsp://192.168.1.23:8554/stream0" \
  -p rtspsrc_latency:=50
```

---

## Configuration

Edit `config/camera_bridge_config.yaml` to set Jetson IPs before launching:

```yaml
camera_bridge_node:
  ros__parameters:
    stream_ids: "0,1,2,3"
    rtsp_urls: "rtsp://192.168.1.22:8554/stream0,rtsp://192.168.1.23:8554/stream0,rtsp://192.168.1.24:8554/stream0,rtsp://192.168.1.25:8554/stream0"
    output_prefix: '/rtsp/stream_'
    rtspsrc_latency: 50
```

---

## Dependencies

ROS2 packages: `rclpy`, `sensor_msgs`, `cv_bridge`

GStreamer packages (included in `Dockerfile.server`):

| Package | Provides |
|---|---|
| `gstreamer1.0-plugins-good` | `rtspsrc`, `rtph264depay` |
| `gstreamer1.0-libav` | `avdec_h264` |
| `gstreamer1.0-plugins-base/bad/ugly` | core elements, `videoconvert` |

OpenCV must be built with GStreamer support. Verify:

```bash
python3 -c "import cv2; [print(l) for l in cv2.getBuildInformation().split('\n') if 'GStreamer' in l]"
# Expected: GStreamer:  YES (x.x.x)
```

---

## Jetson setup

Each Jetson runs `jetson/rtsp_server.py` (no ROS, no Docker). Install once per Jetson:

```bash
sudo apt-get install -y python3-gi gir1.2-gst-rtsp-server-1.0
```

Start all Jetsons from the laptop:

```bash
./scripts/run_jetsons_tmux.sh 192.168.1.22 192.168.1.23 192.168.1.24 192.168.1.25
```

Each Jetson serves `rtsp://<jetson_ip>:8554/stream<sensor_id>` using `nvv4l2h264enc` (hardware H.264, UltraFast preset) via `gi.repository.GstRtspServer`.
