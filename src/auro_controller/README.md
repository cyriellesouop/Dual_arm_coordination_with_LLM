# auro_controller

Central orchestration node for the AURO robotic pick-and-place system.

---

## Role in the system

The controller sits between the perception stack and the arm:

- `overhead_perception` publishes `/overhead/objects/info` and `/overhead/aruco/pose`
- `auro_controller` validates, transforms, and forwards objects to `llm_node_pkg` via `/detected_objects`
- `llm_node_pkg` sends the user's selection back via `/selected_object`
- the controller publishes `/arm_status` for the LLM and drives the arm via MoveIt2

The controller:
1. Receives raw detections from `object_localizer_node` (already in the correct frame, either `world` or `table_origin` depending on ArUco visibility).
2. Optionally transforms centroids to `base_link` (only needed once arm calibration is done, see below).
3. Validates each object is physically on the table, dropping out-of-bounds objects with a `WARN`.
4. Publishes the clean list to the LLM node, which asks the user which object to pick.
5. Executes the Kinova arm pick, lift, place, release, and home sequence once the LLM selection arrives.
6. Resets and waits for the next scene.

---

## Topics

### Subscribed

| Topic | Type | Publisher | Message format |
|---|---|---|---|
| `/overhead/objects/info` | `std_msgs/String` | `object_localizer_node` | JSON array (see below) |
| `/selected_object` | `std_msgs/String` | `llm_node` | `{"name": "bottle", "x": 0.3, "y": 0.1, "z": 0.04}` |
| `/arm_stop` | `std_msgs/Bool` | `llm_node` | `true` = emergency stop |

`/overhead/objects/info` format (one entry per detected object):
```json
[
  {
    "label":    "water_bottle",
    "centroid": [0.12, -0.05, 0.04],
    "radius_m": 0.035,
    "cameras":  [0, 1],
    "hull_pts": 84,
    "color":    [0.13, 0.59, 0.95],
    "frame":    "table_origin"
  }
]
```

> **`frame` field**: `object_localizer_node` sets this to `"table_origin"` when the ArUco marker is visible in both cameras, and `"world"` otherwise. The controller reads this per-object and uses it as the source frame for any coordinate transform.

### Published

| Topic | Type | Subscriber | Message format |
|---|---|---|---|
| `/detected_objects` | `std_msgs/String` | `llm_node` | JSON array (see below) |
| `/arm_status` | `std_msgs/String` | `llm_node` | `"ready"` or `"moving"` |
| `/controller/object_markers` | `visualization_msgs/MarkerArray` | RViz | Sphere markers at object centroids (2 s lifetime) |

`/detected_objects` format (validated objects forwarded to the LLM):
```json
[
  {"name": "water_bottle", "x": 0.12, "y": -0.05, "z": 0.04, "type": "water_bottle"},
  {"name": "cup",          "x": 0.30, "y":  0.10, "z": 0.06, "type": "coffee_cup"}
]
```

> The `"type"` field is extra context for the arm planner; `llm_node` only reads `name/x/y/z` and ignores it.

---

## Coordinate frames and the ArUco calibration path

The system uses three frames:

| Frame | Origin | Set by |
|---|---|---|
| `world` | Top-left camera optical centre | Fixed at startup by `object_localizer_node` |
| `table_origin` | ArUco marker corner on the table | Published dynamically by `aruco_tf_node` (TF + `/overhead/aruco/pose`) |
| `base_link` | Kinova arm base | Published by the Kinova MoveIt2 driver |

### Current state (what works now)

The perception stack (`overhead_perception`) already handles the `world` to `table_origin` transform entirely. When the ArUco marker is visible, all object centroids are automatically re-expressed in `table_origin` before they reach the controller. No work is needed in the controller for this step.

The controller's `FrameTransformer` is currently in pass-through mode (`use_tf: false`, all offsets 0). Objects arrive in either `world` or `table_origin` and are forwarded to the LLM in whichever frame they arrived in.

### What still needs to happen (table_origin to base_link)

To send accurate pick poses to the arm, the controller needs to know where `table_origin` is relative to `base_link`. This requires a one-time physical measurement:

Step 1: measure the transform. With the robot and camera rig both powered on and the ArUco marker on the table, read the ArUco pose from `/overhead/aruco/pose` (gives `table_origin` in `world` frame). Measure the physical offset from the ArUco marker corner to the Kinova `base_link` origin with a ruler or calibration tool. This gives you a 6-DOF rigid transform: `table_origin` to `base_link` (translation + rotation).

Step 2: broadcast it as a static TF. Add to `controller.launch.py` (or a dedicated calibration launch file):

```bash
ros2 run tf2_ros static_transform_publisher \
    --x TX --y TY --z TZ \
    --roll 0 --pitch 0 --yaw YAW \
    --frame-id table_origin --child-frame-id base_link
```

Replace `TX TY TZ YAW` with the measured values.

Step 3: enable TF lookup in the controller. In `config/controller_params.yaml`:

```yaml
use_tf: true
perception_frame: "table_origin"
ground_frame: "base_link"
```

The controller will then automatically look up the live TF and transform every centroid into `base_link` before the arm plans its motion. No other code changes needed.

> Until this calibration is done, the controller passes coordinates through unchanged. Pick poses will be in `table_origin` or `world` frame, which is meaningless to the arm, so keep `enable_arm: false`.

---

## Parameters (`config/controller_params.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `enable_arm` | `false` | Set `true` to enable Kinova arm execution. Requires MoveIt2 running. |
| `use_tf` | `false` | Set `true` to use live TF lookup. Requires the `table_origin` to `base_link` static TF above. |
| `perception_frame` | `"world"` | Expected source frame. Only used in static-offset mode. |
| `ground_frame` | `"base_link"` | Target frame for arm planning. Only used in static-offset mode. |
| `offset_x/y/z` | `0.0` | Static shift (metres). Only apply if not using TF. |
| `offset_yaw` | `0.0` | Static Z rotation (radians). Only apply if not using TF. |
| `table_center_x/y` | `0.0` | Table centre in ground frame (metres). |
| `table_half_x` | `1.0` | Half-length of table along X (metres). |
| `table_half_y` | `0.6` | Half-width of table along Y (metres). |
| `table_z_min/max` | `-0.05 / 0.60` | Valid Z range for objects on the table. |
| `place_x/y/z` | `0.25 / 0.25 / 0.05` | Drop-off location in `base_link` (metres). Tune after arm calibration. |
| `debounce_min_sec` | `1.0` | Min seconds between identical object-list republications. |

---

## Running

### In Docker (recommended)

The controller is launched as part of the main `auro` service in the repo-root `docker-compose.yml`, alongside `overhead_perception`, `llm_node_pkg`, and `ollama`. See the repo-root [README](../../README.md#quick-start--full-system-in-docker) for full setup.

```bash
# From the repo root
sudo docker-compose up
```

Default command (from `docker-compose.yml`):
```
ros2 launch auro_controller system.launch.py enable_llm:=false
```

To override at compose-time (e.g., enable the LLM node for integrated typed-input testing):
```bash
# Edit the `command:` line in docker-compose.yml, or run one-off:
sudo docker-compose run --rm auro ros2 launch auro_controller system.launch.py enable_llm:=true
```

Launch args:

| Arg | Default | Effect |
|---|---|---|
| `enable_llm` | `true` (in system.launch.py) | Start the LLM node here. Set `false` when a separate voice laptop runs it. |
| `enable_arm` | `false` | Execute Kinova moves. Requires MoveIt2 running separately. |
| `use_tf` | `false` | Use live TF for `table_origin` to `base_link`. Requires calibration (see above). |
| `use_voice` | `false` | Microphone input for the LLM. Default is typed input. |

Inspecting the running container:

```bash
# Watch the topic the LLM would see
docker exec -it $(docker ps -qf name=auro) ros2 topic echo /detected_objects

# Drop into a shell
docker exec -it $(docker ps -qf name=auro) bash
```

> RViz runs inside the container but needs `DISPLAY` forwarded from the host. The compose file mounts `/tmp/.X11-unix` already. If RViz fails to open, the controller and perception still run fine; you can `ros2 topic echo` from another terminal.

### Native (no Docker), perception only (validate the pipeline, no arm)

```bash
# Terminal 1: perception (suppress its own RViz since controller owns it)
ros2 launch overhead_perception object_localization.launch.py rviz:=false

# Terminal 2: controller + RViz
ros2 launch auro_controller controller.launch.py

# Terminal 3: watch what reaches the LLM
ros2 topic echo /detected_objects

# Terminal 4: watch the raw frame field from perception
ros2 topic echo /overhead/objects/info
```

### Software-only test (no cameras, no arm)

```bash
# Terminal 1: controller
ros2 launch auro_controller controller.launch.py

# Terminal 2: fake objects
ros2 run auro_controller mock_perception_pub

# Terminal 3: verify output
ros2 topic echo /detected_objects
```

### Full system (arm enabled, after calibration)

```bash
# Start MoveIt2 + Kinova driver first, then:
ros2 launch auro_controller controller.launch.py enable_arm:=true
```

---

## State machine

- **WAITING_FOR_OBJECTS**: idle, waiting for `/overhead/objects/info` with at least one valid object
- **OBJECTS_PUBLISHED**: published to `/detected_objects`, waiting for the LLM to respond with `/selected_object`
- **PICKING**: arm moving to the selected object *(planned, requires arm integration)*
- **PLACING**: arm moving to the drop-off location *(planned, requires arm integration)*
- **RETURNING_HOME**: arm returning to home pose, then resets to WAITING_FOR_OBJECTS *(planned)*
- **STOPPED**: `/arm_stop` received at any point, requires manual reset to resume

---

## Open TODOs

### Blocking before arm integration

- [ ] Measure `table_origin` to `base_link` transform and add static TF to launch file (see calibration path above).
- [ ] Set `use_tf: true`, `perception_frame: "table_origin"` in `controller_params.yaml` after calibration.
- [ ] Set `enable_arm: true` once MoveIt2 + Kinova driver are confirmed running.
- [ ] Set `place_x / place_y / place_z` to the real drop-off location relative to `base_link`.
- [ ] Verify `TABLE_POSITION` in `kinova_pick_planner/kinova_pick_planner.py` matches the physical table in `base_link` frame (used for collision avoidance).

### Nice to have

- [ ] Tune `HOME_X / HOME_Y / HOME_Z` constants at the top of `controller_node.py` after real arm testing.
- [ ] Tune `GRIPPER_PROFILES` (position and effort per object type) after testing with real objects.
- [ ] Extend `LABEL_TO_OBJECT_TYPE` as new YOLO classes are added to the perception model.
- [ ] Add active trajectory cancellation when `/arm_stop` arrives (currently sets a flag but does not cancel a mid-flight MoveIt2 trajectory).

---

## Package layout

```
src/auro_controller/
├── auro_controller/
│   ├── controller_node.py       Main orchestration node
│   ├── frame_transform.py       Coordinate-frame converter (pass-through / static offset / tf2)
│   └── mock_perception_pub.py   Fake /overhead/objects/info publisher for offline testing
├── config/
│   ├── controller_params.yaml   All tunable parameters
│   └── controller.rviz          RViz config (includes perception topics + arm model)
├── launch/
│   └── controller.launch.py     Starts controller node + RViz + static TF anchors
└── README.md                    This file
```

---

## Build

```bash
cd ~/auro-final-project
colcon build --packages-select auro_controller
source install/setup.bash
```
