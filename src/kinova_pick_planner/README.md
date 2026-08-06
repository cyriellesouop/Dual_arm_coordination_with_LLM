# kinova_pick_planner

Pick-and-place controller for the Kinova Gen3 Lite robot arm with MoveIt2 and LLM voice interface integration.

## Topics

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|-----------|------------|-------------|
| `/detected_objects` | `std_msgs/String` | Camera node | LLM node | JSON array of detected objects |
| `/selected_object` | `std_msgs/String` | LLM node | Arm controller | JSON of user's selection |
| `/arm_status` | `std_msgs/String` | Arm controller | LLM node | `"moving"` or `"ready"` |
| `/arm_stop` | `std_msgs/Bool` | LLM node | Arm controller | `true` = emergency stop |

### Message Formats

**`/detected_objects`** (JSON array):
```json
[
  {"name": "red ball", "x": 0.40, "y": 0.05, "z": 0.0, "type": "foam_ball"},
  {"name": "coffee cup", "x": 0.376, "y": 0.15, "z": 0.0, "type": "coffee_cup"}
]
```

- `x`, `y`: position in robot's `base_link` frame (meters)
- `z`: height of object base above table surface (usually 0.0)
- `type`: one of `foam_ball`, `coffee_cup`, `water_bottle`, `small_box`

**`/selected_object`** (JSON object):
```json
{"name": "red ball", "x": 0.40, "y": 0.05, "z": 0.0}
```

## Requirements

- Ubuntu 22.04 + ROS2 Humble, OR Ubuntu 24.04 + ROS2 Jazzy
- Kinova Gen3 Lite with ros2_kortex driver
- MoveIt2
- pymoveit2

## Installation

### Setup script (recommended)

```bash
cd <your_kinova_workspace>
# Copy kinova_pick_planner into src/
cp -r kinova_pick_planner src/

# Run setup
chmod +x src/kinova_pick_planner/setup.sh
./src/kinova_pick_planner/setup.sh
```

### Manual installation

```bash
cd <your_kinova_workspace>

# Clone pymoveit2 if not present
cd src
git clone https://github.com/AndrejOrsula/pymoveit2.git
cd ..

# Install dependencies
rosdep install --ignore-src --from-paths src -y -r

# Build
source /opt/ros/<distro>/setup.bash
colcon build --packages-select pymoveit2 kinova_pick_planner
source install/setup.bash
```

### Docker

```bash
cd src/kinova_pick_planner
docker build -t kinova_pick_planner .
docker run -it --net=host kinova_pick_planner
```

## Running

### Terminal 1: MoveIt2 + Robot

```bash
source <workspace>/install/setup.bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
    robot_ip:=192.168.1.10 use_fake_hardware:=false
```

### Terminal 2: Set controller tolerance (run once after MoveIt2 starts)

```bash
ros2 param set /joint_trajectory_controller constraints.goal_time 30.0
```

### Terminal 3: LLM Node (if using voice interface)

```bash
source <workspace>/install/setup.bash
ros2 run llm_node_pkg llm_node --ros-args -p use_voice:=false -p ollama_model:=llama3.2:3b
```

### Terminal 4: Arm Controller

```bash
source <workspace>/install/setup.bash
ros2 run kinova_pick_planner arm_controller
```

### Testing without LLM

Publish a selection manually:
```bash
ros2 topic pub --once /selected_object std_msgs/String \
  '{"data": "{\"name\": \"red ball\", \"x\": 0.40, \"y\": 0.05, \"z\": 0.0, \"type\": \"foam_ball\"}"}'
```

### Testing without camera

The `arm_controller` entry point publishes dummy objects by default. For real camera integration, the camera node publishes to `/detected_objects` and the arm controller subscribes automatically.

## Configuration

Key parameters are in `kinova_pick_planner.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `BASE_HEIGHT_ABOVE_TABLE` | 0.05m | Height of robot mounting plate |
| `MAX_VELOCITY` | 0.3 | Motion velocity scaling (0-1) |
| `MAX_ACCELERATION` | 0.3 | Motion acceleration scaling (0-1) |
| `PLANNING_ATTEMPTS` | 15 | Number of planning attempts |

Object dimensions and grasp strategies are defined in `OBJECT_TYPES` and `GRASP_STRATEGIES` dictionaries. Adjust these to match your actual objects.

Place location is configured in `arm_controller.py`:
```python
PLACE_POSITION = {
    "x": 0.30,      # 30cm in front of robot
    "y": -0.15,     # 15cm to the right
    "z_above_table": 0.05,  # 5cm above table
}
```

## Robot Coordinate System

```
        +Z (up)
         |
         |_____ +Y (left from robot's perspective)
        /
       /
      +X (forward, away from robot)

  Robot base at origin (0, 0, 0)
  Table surface at z = -0.05 (5cm below base)
```

## Motion Strategy

- Free-space moves (home to approach, lift to place): joint-space planning via pymoveit2
- Precision moves (approach to grasp, grasp to lift): Cartesian straight-line paths
- Retry logic: 3 attempts + wiggle recovery + home reset fallback
- Collision avoidance: table, keepout zone, obstacle objects in MoveIt2 planning scene

## Troubleshooting

"Joint states are not available yet" -- normal on first move after startup. Resolves within 1 second.

Planning failures (99999) -- known MoveIt2 Jazzy bug that masks real errors. Usually means the arm is in a bad state. Use the Kinova web GUI to clear faults and send to home.

`goal_time` parameter -- must be set to 30.0 after every MoveIt2 restart:
```bash
ros2 param set /joint_trajectory_controller constraints.goal_time 30.0
```

**Gripper stalls** — Gripper commands use `time.sleep()` instead of `wait_until_executed()` due to pymoveit2 compatibility.

## Files

| File | Description |
|------|-------------|
| `kinova_pick_planner.py` | Core planner: motion, collisions, gripper, grasp computation |
| `arm_controller.py` | State machine: LLM integration, pick-place sequence |
| `setup.sh` | Automated dependency installation |
| `package.xml` | ROS2 package manifest |
| `setup.py` | Python package configuration |
