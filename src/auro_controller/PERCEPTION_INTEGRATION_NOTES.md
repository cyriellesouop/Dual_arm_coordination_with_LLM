# Perception Integration Notes

**For:** Overhead-perception team (overhead_perception package)  
**From:** Controller team (auro_controller package)  
**Status:** Items below are NOT yet implemented in perception; controller has workarounds for now.

---

## What the controller currently assumes

The controller subscribes to `/overhead/objects/info` and expects this JSON format:

```json
[
  {
    "label":    "water_bottle",
    "centroid": [x, y, z],
    "radius_m": 0.035,
    "cameras":  [0, 2],
    "hull_pts": 120,
    "color":    [0.13, 0.59, 0.95]
  }
]
```

This matches what `object_localizer_node.py` already publishes. **No changes needed for basic operation.**

---

## Asks / TODOs for the perception team

### 1. Machine-readable table bounds topic  *(Priority: Medium)*

**Current state:** The table boundary is only published as a visual `MarkerArray` on `/overhead/scene/table` for RViz. The controller cannot read it programmatically.

**What we need:** A `std_msgs/String` topic (or a custom message) that publishes the table bounds as JSON:

```
Topic: /overhead/scene/table_bounds
Type:  std_msgs/String
Rate:  1 Hz (latched is fine)

Payload:
{
  "center":       [0.0, 0.0, 0.0],   # table centre in the perception frame
  "half_extents": [1.0, 0.6, 0.3]   # half-width along X, Y; max object height
}
```

**Why:** Right now the controller uses hardcoded parameters (`table_half_x`, `table_half_y`, etc.) in `controller_params.yaml`. If the table bounds are published, the controller can subscribe to them and auto-update with no manual YAML edits when the table moves.

**Controller side ready:** The controller already has a `table_half_x/y/z` parameter system. Subscribing to `table_bounds` instead would be a ~20-line addition once the topic exists.

---

### 2. Ground-frame TF broadcast  *(Priority: High)*

**Current state:** Perception publishes centroids in a frame whose origin is the top-left camera (currently called `"world"` in the MarkerArray header). There is no TF transform from this frame to the Kinova `base_link` frame.

**What we need:** A static TF broadcast from `overhead_world` (or whatever the camera-rig frame is named) to `base_link`:

```bash
ros2 run tf2_ros static_transform_publisher \
    --x TX --y TY --z TZ \
    --roll 0 --pitch 0 --yaw YAW \
    --frame-id base_link \
    --child-frame-id world
```

This should ideally go in a launch file (e.g., `camera_calibration_pkg`) once the extrinsic calibration is measured.

**Why:** The controller's `FrameTransformer` has a complete `tf2` implementation already. Once the transform is broadcasting, flip `use_tf: true` in `config/controller_params.yaml` and the controller uses live TF automatically.

**Until then:** Set `offset_x/y/z/yaw` in `controller_params.yaml` to manually approximate the transform.

---

### 3. Coordinate frame documentation  *(Priority: Medium)*

**What we need to know:**
- What is the exact origin of the current `"world"` frame? (Top-left camera optical centre? Top-left camera body? Something else?)
- What are the axis directions? (Is +X pointing into the scene, to the right, etc.?)
- Does the frame move if the camera rig is repositioned?

**Why:** Without this, setting correct `offset_x/y/z/yaw` values in the YAML is guesswork. Even a quick comment in `object_localizer_node.py` would help.

---

### 4. Confidence / staleness filtering  *(Priority: Low)*

**Optional enhancement:** If the YOLO model has low confidence on a detection, it would help to include the score in `/overhead/objects/info`:

```json
{ "label": "apple", "centroid": [...], "score": 0.87, ... }
```

The controller could then filter out low-confidence detections (`score < 0.5`) before forwarding to the LLM, preventing the robot from trying to pick phantom objects.

The controller already passes through unknown keys silently, so adding `score` to the message would not break anything.

---

## What the controller does NOT need from perception

- The raw camera images (handled internally by perception).
- The voxel hull data (`hull_pts`). The controller ignores this field.
- The colour data (`color`). The controller ignores this field.
- RViz markers. The controller only reads `/overhead/objects/info`.
