# rtsp_viewer

ROS2 Humble package that displays live camera streams in a single tiled OpenCV window. Subscribes to compressed image topics published by `jetson_bridge` and renders all streams side by side at up to 30 Hz. Useful for monitoring feeds without needing RViz or additional tooling.

---

## Node: `rtsp_viewer_node`

Subscribes to one `CompressedImage` topic per stream ID and renders all frames in a grid. The grid dimensions are computed automatically from the number of streams (e.g. 4 streams become a 2x2 grid). Streams that have not yet published show a grey placeholder tile until the first frame arrives.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `stream_ids` | int[] | `[0, 1, 2, 3]` | Stream IDs to display |
| `input_prefix` | string | `"/rtsp/stream_"` | Topic prefix; full topic = `{input_prefix}{id}/compressed` |
| `tile_width` | int | `640` | Width of each tile in pixels |
| `tile_height` | int | `360` | Height of each tile in pixels |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/rtsp/stream_{id}/compressed` | `sensor_msgs/CompressedImage` | Compressed frames from `camera_bridge_node` |

---

## Launch

```bash
ros2 launch rtsp_viewer rtsp_viewer.launch.py
```

Override tile size at launch:

```bash
ros2 launch rtsp_viewer rtsp_viewer.launch.py tile_width:=1280 tile_height:=720
```

---

## Configuration

Edit `config/rtsp_viewer_config.yaml` to set stream IDs before launching:

```yaml
rtsp_viewer:
  ros__parameters:
    stream_ids: [0, 1, 2, 3]
    input_prefix: '/rtsp/stream_'
    tile_width:  640
    tile_height: 360
```

`stream_ids` should match the `stream_ids` set in `jetson_bridge/config/camera_bridge_config.yaml`. Streams not listed here will not be shown even if they are publishing.

---

## Dependencies

ROS2 packages: `rclpy`, `sensor_msgs`

Python: `opencv-python`, `numpy`

Requires a display. Running headless (no `DISPLAY` set) will fail when the node tries to open the OpenCV window. Inside Docker, forward `DISPLAY` and mount `/tmp/.X11-unix` — see `Dockerfile.server` for the pattern.
