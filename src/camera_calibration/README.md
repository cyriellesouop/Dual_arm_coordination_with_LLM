# camera_calibration

ROS2 Humble package for intrinsic and stereo camera calibration. Subscribes to live image topics from `jetson_bridge`, collects calibration frames, and saves YAML files to `calibration/` that are loaded at runtime by the perception pipeline. Run once per camera setup; not required during normal operation.

---

## Node: `intrinsic_aruco_calibration_node`

Collects frames from a single camera stream and computes intrinsic parameters (camera matrix, distortion coefficients) using a ChArUco board. Saves results to `calibration/intrinsic_<stream_id>.yaml`.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `stream_id` | int | `0` | Stream ID to subscribe to |
| `squares_x` | int | `11` | ChArUco board columns |
| `squares_y` | int | `8` | ChArUco board rows |
| `square_length` | float | `0.030` | Chessboard square side length (m) |
| `marker_length` | float | `0.022` | ArUco marker side length (m) |
| `aruco_dict` | string | `"DICT_5X5_100"` | ArUco dictionary name |
| `min_samples` | int | `25` | Minimum accepted frames before calibrating |
| `output_dir` | string | `"calibration"` | Directory to write output YAML |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/rtsp/stream_{id}/raw` | `sensor_msgs/Image` (bgr8) | Input frames from `camera_bridge_node` |

---

## Node: `intrinsic_checkerboard_calibration_node`

Same as above but uses a standard checkerboard pattern instead of ChArUco.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `stream_id` | int | `0` | Stream ID to subscribe to |
| `pattern_cols` | int | `6` | Interior corner columns |
| `pattern_rows` | int | `8` | Interior corner rows |
| `square_size` | float | `0.029` | Square side length (m) |
| `min_samples` | int | `10` | Minimum accepted frames before calibrating |
| `output_dir` | string | `"calibration"` | Directory to write output YAML |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/rtsp/stream_{id}/raw` | `sensor_msgs/Image` (bgr8) | Input frames from `camera_bridge_node` |

---

## Node: `stereo_aruco_calibration_node`

Collects synchronised frame pairs from two streams and computes stereo extrinsics (rotation matrix `R`, translation vector `T`) using a ChArUco board. Saves results to `calibration/stereo_<left_id>_<right_id>.yaml`. Requires intrinsic YAMLs for both cameras.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `stream_id_left` | int | `0` | Left stream ID |
| `stream_id_right` | int | `1` | Right stream ID |
| `intrinsic_yaml_left` | string | `""` | Path to left camera intrinsic YAML |
| `intrinsic_yaml_right` | string | `""` | Path to right camera intrinsic YAML |
| `squares_x` | int | `12` | ChArUco board columns |
| `squares_y` | int | `9` | ChArUco board rows |
| `square_length` | float | `0.1` | Chessboard square side length (m) |
| `marker_length` | float | `0.075` | ArUco marker side length (m) |
| `aruco_dict` | string | `"DICT_4X4_100"` | ArUco dictionary name |
| `min_samples` | int | `25` | Minimum synchronised frame pairs before calibrating |
| `min_corner_displacement` | float | `20.0` | Minimum pixel displacement between accepted samples |
| `max_sync_delta_ms` | float | `50.0` | Maximum timestamp difference for frame pairing (ms) |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/rtsp/stream_{left_id}/raw` | `sensor_msgs/Image` (bgr8) | Left camera frames |
| `/rtsp/stream_{right_id}/raw` | `sensor_msgs/Image` (bgr8) | Right camera frames |

---

## Launch

### Intrinsic calibration (ChArUco)

```bash
ros2 launch camera_calibration intrinsic_aruco_calibration.launch.py \
  stream_id:=0 \
  squares_x:=11 squares_y:=8 \
  square_length:=0.030 marker_length:=0.022
```

Repeat for each stream:

```bash
ros2 launch camera_calibration intrinsic_aruco_calibration.launch.py stream_id:=1
ros2 launch camera_calibration intrinsic_aruco_calibration.launch.py stream_id:=2
ros2 launch camera_calibration intrinsic_aruco_calibration.launch.py stream_id:=3
```

### Intrinsic calibration (checkerboard)

```bash
ros2 launch camera_calibration intrinsic_calibration.launch.py \
  stream_id:=0 pattern_cols:=6 pattern_rows:=8 square_size:=0.029
```

### Stereo calibration

```bash
ros2 launch camera_calibration stereo_aruco_calibration.launch.py \
  stream_id_left:=0 stream_id_right:=1 \
  intrinsic_yaml_left:=calibration/intrinsic_0.yaml \
  intrinsic_yaml_right:=calibration/intrinsic_1.yaml
```

---

## Post-calibration: generating the combined YAML

After running stereo calibration for all camera pairs, run `calibration/convert.py` from the repo root to combine the individual stereo YAMLs into `calibration/camera_calibration_nodes.yaml`. This unified file is what `object_localizer_node` loads at runtime.

```bash
python3 calibration/convert.py
# or specify a custom output path:
python3 calibration/convert.py --out calibration/camera_calibration_nodes.yaml
```

The script reads the stereo YAML for each non-origin stream (streams 0, 2, 3 relative to the stream-1 world origin), inverts the stereo extrinsics to get each camera's pose in the world frame, and writes a single YAML with intrinsics and extrinsics for all four cameras.

Stream 1 is hardcoded as the world origin (identity rotation, zero translation). The intrinsics for all cameras are taken from the jointly-calibrated values inside each stereo file rather than from standalone per-camera files, which keeps the intrinsics consistent with the R/T from the same optimization.

This step is required after every new stereo calibration. The perception pipeline will not have correct camera geometry until `camera_calibration_nodes.yaml` is regenerated.

---

## Configuration

Edit `config/intrinsic_aruco_calibration_config.yaml` or `config/stereo_aruco_calibration_config.yaml` to set board dimensions and output paths before launching:

```yaml
intrinsic_aruco_calibration_node:
  ros__parameters:
    squares_x: 11
    squares_y: 8
    square_length: 0.030
    marker_length: 0.022
    aruco_dict: "DICT_5X5_100"
    min_samples: 25
    output_dir: "calibration"
```

Output YAMLs are written to `calibration/` at the workspace root and loaded by `object_localizer_node` at runtime.

---

## Dependencies

ROS2 packages: `rclpy`, `sensor_msgs`, `cv_bridge`

Python: `opencv-python` (with ArUco support), `numpy<2`

> **Note:** `ros-humble-cv-bridge` is compiled against the NumPy 1.x ABI. `numpy>=2` causes a silent import error. The `Dockerfile.server` pins `numpy<2`.
