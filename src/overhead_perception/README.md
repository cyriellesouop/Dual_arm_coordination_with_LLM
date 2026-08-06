# overhead_perception

ROS2 Humble package that runs on the server. Subscribes to raw image topics from `jetson_bridge`, runs YOLOv11 object detection on each stream, detects ArUco markers to derive camera extrinsics, and triangulates 2D detections into 3D object positions using DLT. Publishes object locations to `auro_controller`.

---

## Node: `detector_node`

Loads a YOLOv11 model and runs inference on each subscribed image stream at a configurable rate. One subscription and one publisher per stream ID.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `stream_ids` | string | `"0,1"` | Comma-separated stream IDs to subscribe to |
| `model_path` | string | `"yolo11m.pt"` | Path to YOLO model weights |
| `confidence_threshold` | float | `0.5` | Minimum detection confidence |
| `max_fps` | float | `5.0` | Maximum inference rate per stream |
| `device` | string | `"auto"` | Inference device: `"auto"`, `"cpu"`, or `"cuda:0"` |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/rtsp/stream_{id}/raw` | `sensor_msgs/Image` (bgr8) | Raw frames from `camera_bridge_node` |

### Published topics

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/perception/stream_{id}/detections_2d` | `vision_msgs/Detection2DArray` | BEST_EFFORT, depth 1 | 2D bounding boxes + class labels per stream |

---

## Node: `aruco_localizer_node`

Detects ArUco markers on the reference camera stream to compute camera-to-world extrinsics. Uses a known marker layout and `solvePnP` to establish the coordinate frame origin. Publishes per-object positions in the ArUco marker frame and the marker landmark map.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ref_stream_id` | int | `1` | Stream ID of the reference camera (where markers are visible) |
| `aruco_dict` | string | `"DICT_4X4_50"` | ArUco dictionary name |
| `marker_size` | float | `0.097` | Physical marker side length (m) |
| `marker_ids` | string | `"0,1,2,3"` | Comma-separated marker IDs present in the scene |
| `origin_id` | int | `0` | Marker ID that defines the coordinate origin |
| `update_rate` | float | `1.0` | Rate to re-estimate extrinsics (Hz) |
| `stale_timeout_sec` | float | `3.0` | Seconds since a marker was last detected before it's treated as stale — the origin marker going stale pauses `/perception/objects/aruco_id_0`; a stale landmark marker is dropped from `/perception/aruco/landmarks` |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/rtsp/stream_{ref_id}/raw` | `sensor_msgs/Image` (bgr8) | Reference camera frames |
| `/perception/objects/cam1` | `std_msgs/String` (JSON) | 3D object positions from `object_localizer_node` |

### Published topics

| Topic | Type | Description |
|---|---|---|
| `/perception/objects/aruco_id_0` | `std_msgs/String` (JSON) | Object positions in ArUco origin frame |
| `/perception/aruco/landmarks` | `std_msgs/String` (JSON) | Detected marker poses in camera frame |

---

## Node: `object_localizer_node`

Collects `Detection2DArray` messages from all streams, pairs detections across cameras by class label, and triangulates 3D positions using DLT. Requires a stereo calibration YAML (from `camera_calibration`) that provides camera projection matrices.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `stream_ids` | string | `"0,1"` | Comma-separated stream IDs to fuse |
| `calibration_file` | string | `""` | Path to stereo calibration YAML from `camera_calibration` |
| `min_cameras` | int | `2` | Minimum number of cameras that must see an object to triangulate |
| `publish_rate` | float | `5.0` | Output publish rate (Hz) |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/perception/stream_{id}/detections_2d` | `vision_msgs/Detection2DArray` | Per-stream detections from `detector_node` |

### Published topics

| Topic | Type | Description |
|---|---|---|
| `/perception/objects/cam1` | `std_msgs/String` (JSON) | Triangulated 3D object positions (camera frame) |

---

## Node: `debug_overlay_node`

Live multi-camera debug viewer. Tiles all configured streams into one OpenCV window, drawing YOLO boxes (reused from `detector_node`'s published detections), live per-camera ArUco marker detection (run independently on every configured stream, not just the reference stream `aruco_localizer_node` uses), and a workspace-boundary polyline connecting the configured marker IDs. Flags any camera that's missing an expected marker in red, and marks whichever stream is `ref_stream_id` since that's the one camera whose marker visibility actually drives the production ArUco frame/table bounds today. A visual complement to `ros2 run rqt_image_view rqt_image_view` for diagnosing dropped markers/objects, e.g. after the table is repositioned.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `stream_ids` | string | `"0,1,2,3"` | Comma-separated stream IDs to tile |
| `ref_stream_id` | int | `1` | Stream ID used by `aruco_localizer_node`; its tile is marked "REFERENCE" |
| `aruco_dict` | string | `"DICT_4X4_50"` | ArUco dictionary name |
| `marker_ids` | string | `"0,1,2,3"` | Comma-separated marker IDs expected in the scene |
| `aruco_detect_rate` | float | `8.0` | Rate to re-run ArUco detection per camera (Hz), independent of the 30Hz display refresh |
| `tile_width` | int | `640` | Per-camera tile width in the display grid (px) |
| `tile_height` | int | `360` | Per-camera tile height in the display grid (px) |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/rtsp/stream_{id}/raw` | `sensor_msgs/Image` (bgr8) | Raw frames from `camera_bridge_node`, one per tiled stream |
| `/perception/stream_{id}/detections_2d` | `vision_msgs/Detection2DArray` | Per-stream detections from `detector_node` |

Run with `ros2 launch overhead_perception debug_overlay.launch.py`.

---

## Launch

### Full perception pipeline (detector + localizer + ArUco)

```bash
ros2 launch overhead_perception object_localization.launch.py \
  stream_ids:="0,1" \
  calibration_file:=calibration/stereo_0_1.yaml
```

### Detector only

```bash
ros2 launch overhead_perception detector_only.launch.py \
  stream_ids:="0,1" \
  model_path:=yolo11m.pt \
  confidence_threshold:=0.5
```

### Debug overlay viewer

```bash
ros2 launch overhead_perception debug_overlay.launch.py stream_ids:="0,1,2,3"
```

---

## Configuration

| File | Controls |
|---|---|
| `config/detector.yaml` | YOLO model path, confidence threshold, max fps, device |
| `config/aruco_localizer_config.yaml` | Marker dictionary, size, IDs, origin marker, update rate |
| `config/object_localizer_config.yaml` | Stream IDs, calibration file path, min cameras, publish rate |
| `config/debug_overlay_config.yaml` | Stream IDs, reference stream, marker dictionary/IDs, tile size |

Example `config/detector.yaml`:

```yaml
detector_node:
  ros__parameters:
    stream_ids: "0,1"
    model_path: "yolo11m.pt"
    confidence_threshold: 0.5
    max_fps: 5.0
    device: "auto"
```

---

## Dependencies

ROS2 packages: `rclpy`, `sensor_msgs`, `vision_msgs`, `cv_bridge`

Python: `ultralytics` (YOLOv11), `opencv-python`, `numpy<2`

> **Note:** `ros-humble-cv-bridge` is compiled against the NumPy 1.x ABI. `numpy>=2` causes a silent import error. The `Dockerfile.server` pins `numpy<2`.
