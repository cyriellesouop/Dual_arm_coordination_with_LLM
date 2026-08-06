# Per-Arm Dual-Bringup Sequence

Status as of this writing: **validated in fake-hardware mode, in a scratch/throwaway configuration.
Nothing here is wired into a permanent launch file or committed config yet** — this document is the
record of what was proven to work, so the next step (making it reproducible) has something to build on.

## Context / why this approach

Goal: run two Kinova Gen3 Lite arms side by side for a dual-arm coordination project. Two approaches
were considered:

1. **Shared URDF/TF tree with a `prefix` on every joint/link name** (`arm_a_joint_1`, `arm_b_joint_1`,
   ...), one shared MoveIt2 planning scene, real-time geometric collision-awareness between arms.
   Kinova's `ros2_kortex` driver has real infrastructure for this (`kortex_dual_robots.xacro`,
   `gen3_dual.launch.py`, `prefix_1`/`prefix_2` args) — but investigating it surfaced several
   undocumented gaps specific to Gen3 Lite (`gen3_dual.launch.py` hardcodes `description_file:
   gen3.xacro`, its `gripper_1`/`gripper_2` args reject `gen3_lite_2f`, and the Gen3 Lite
   `ros2_controllers.yaml` has no `${prefix}` placeholders needed for the two arms' controllers to
   resolve correctly). A working fix was drafted (see `docker/kortex_overrides/
   gen3_lite_ros2_controllers_prefixed.yaml`) but **shelved, not deleted** — kept as a parked
   artifact in case a future pass wants real shared-scene collision-avoidance.

2. **Two fully independent single-arm stacks** (the existing, already-trusted single-arm launch
   files, unmodified), each on its own `ROS_DOMAIN_ID` for total isolation, coordinated only through
   a small, explicitly-bridged set of topics via the `domain_bridge` ROS2 package. Lower risk, no
   vendor-internals dependency, faster to get working reliably. **This is the approach being pursued.**
   Safety between arms is handled at the software/policy level (task-sequencing mutual exclusion +
   static keepout collision geometry) rather than via a live shared planning scene — see the project
   README section on this for the reasoning.

This document covers proving approach 2's core mechanism: that two arms on separate ROS domains are
genuinely isolated from each other by default, and that `domain_bridge` can selectively relay just the
topics we choose across that isolation boundary.

## Repo files touched in this pass

- **`Dockerfile.server`** — added `ros-humble-domain-bridge` to the apt install list (durable; survives
  image rebuilds).
- **`docker/kortex_overrides/gen3_lite_ros2_controllers_prefixed.yaml`** — parked artifact from the
  shelved shared-URDF approach (see above). Not used by anything currently.
- **This file.**

## Prerequisites

- Container `auro-laptop` (built from `Dockerfile.server`, image `auro-server:latest`) running:
  ```bash
  docker ps -a --filter name=auro-laptop
  # if stopped: docker start auro-laptop
  # if missing: see Dockerfile.server build instructions in the README
  ```
- `ros-humble-domain-bridge` installed in the container (already added to `Dockerfile.server`; if
  working in an older container built before this change, install manually:
  `docker exec auro-laptop apt-get install -y ros-humble-domain-bridge`).

## Step-by-step sequence

### 1. Confirm `domain_bridge` is available

```bash
docker exec auro-laptop apt-get update -qq
docker exec auro-laptop apt-cache policy ros-humble-domain-bridge
```
Confirms the package exists in the apt index and reports installed vs. candidate version, without
installing anything. Result: candidate `0.5.0-1jammy...` was available, not yet installed.

### 2. Install it

```bash
docker exec auro-laptop apt-get install -y ros-humble-domain-bridge
```
Installed live in the running container for immediate testing. Also added as a line in
`Dockerfile.server`'s apt install block (next to the other `ros-humble-*` packages) so it's part of
the image on every rebuild, not just this one running container.

### 3. Launch arm A on domain 0

```bash
docker exec -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp auro-laptop bash -c "
source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash
ros2 launch kortex_bringup gen3_lite.launch.py \
  robot_ip:=192.168.1.10 use_fake_hardware:=true launch_rviz:=false
"
```
- `-e ROS_DOMAIN_ID=0`: scopes this one process to DDS domain 0. Passed per-`docker exec` call, not
  set globally on the container.
- `-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`: matches the DDS implementation used elsewhere in this
  project.
- `use_fake_hardware:=true`: simulates joint states instead of connecting to real hardware over the
  network — no physical robot is touched by this sequence.
- `launch_rviz:=false`: headless; verification here was done via `ros2 node list`/`topic list`, not
  visually.
- `robot_ip:=192.168.1.10`: **required even in fake-hardware mode** — `gen3_lite.launch.py` has no
  default for this argument (confirmed via `ros2 launch kortex_bringup gen3_lite.launch.py
  --show-args`, which lists `robot_ip` with no `(default: ...)` line, unlike every other argument).
  First launch attempt without it failed: `Included launch description missing required argument
  'robot_ip'`. The IP value itself is never used when `use_fake_hardware:=true`, any placeholder works.

Run as a background process — a working hardware-bringup `ros2 launch` never exits on its own.

Result: `controller_manager` loaded and activated all four controllers
(`joint_trajectory_controller`, `gen3_lite_2f_gripper_controller`, `twist_controller`,
`joint_state_broadcaster`) with no errors.

### 4. Launch arm B on domain 1

Identical command, `-e ROS_DOMAIN_ID=1`, `robot_ip:=192.168.1.11`. Same successful result.

### 5. Verify domain isolation

```bash
docker exec -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp auro-laptop \
  bash -c "source /opt/ros/humble/setup.bash && ros2 node list"
# repeat with ROS_DOMAIN_ID=1
```
`ros2 node list` reports what's visible from the domain the command itself is joined to. Ran once per
domain.

Result: domain 0 showed exactly arm A's 6 nodes; domain 1 showed exactly arm B's 6 identically-named
nodes. Neither list showed the other domain's nodes — confirms `ROS_DOMAIN_ID` genuinely isolates the
two stacks by default, with zero extra configuration.

### 6. Learn `domain_bridge`'s config format

```bash
docker exec auro-laptop bash -c "find / -iname '*.yaml' -path '*domain_bridge*'"
docker exec auro-laptop ros2 run domain_bridge domain_bridge --help
docker exec auro-laptop cat /opt/ros/humble/share/domain_bridge/examples/example_bridge_config.yaml
```
Located the package's own installed example rather than guessing the YAML schema. Key fields: `name`,
`from_domain`, `to_domain`, and a `topics:` map where each entry has a `type` and optional
`bidirectional` / `remap` / `qos` overrides.

### 7. Write a throwaway test config and run the bridge

Written by hand to `/tmp/test_bridge_config.yaml` **inside the container** (not part of this repo —
see the file-location table earlier in this doc):

```yaml
name: test_bridge
from_domain: 0
to_domain: 1
topics:
  test_coord:
    type: std_msgs/msg/String
    bidirectional: True
```

```bash
docker exec -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp auro-laptop \
  bash -c "source /opt/ros/humble/setup.bash && ros2 run domain_bridge domain_bridge /tmp/test_bridge_config.yaml"
```
A throwaway topic name (`test_coord`) was used deliberately, to isolate "does the bridge mechanism
work at all" from "are the real coordination topics configured correctly" — one variable at a time.
No `-e ROS_DOMAIN_ID` was set for this command: `domain_bridge` doesn't use the ambient env var, it
reads `from_domain`/`to_domain` from the YAML directly and manages both DDS contexts itself.

### 8. First test attempt — failed

```bash
# domain 1:
timeout 8 ros2 topic echo /test_coord std_msgs/msg/String
# domain 0:
timeout 6 ros2 topic pub -r 2 /test_coord std_msgs/msg/String "data: 'hello from domain 0'"
```
`-r 2`: republish twice a second so there's a real window to be caught, rather than one message that
might race the subscriber's readiness. `timeout N`: bounds each command, since neither `pub -r` nor
`echo` exits on its own.

Result: echo caught nothing, timed out (exit 124). Initial suspicion was Python stdout buffering
hiding log output (the bridge's own log file was empty despite `ps aux` confirming the process was
alive) — this turned out to be a red herring, not the actual cause.

### 9. Diagnose the real cause

```bash
docker exec auro-laptop bash -c "ros2 topic info /test_coord -v"
```
Run on domain 0, after `ros2 topic list` on both domains confirmed the `test_coord` topic entry
existed on both (so the bridge's nodes were correctly present — the question was why no data moved
through them).

Result: showed the bridge's own subscriber on domain 0 requesting `Durability: TRANSIENT_LOCAL`.
`ros2 topic pub`'s CLI default durability is `VOLATILE`, and DDS will not match a `VOLATILE`
publisher to a subscriber specifically requesting `TRANSIENT_LOCAL` — so the test messages were never
received by the bridge at all, independent of any buffering question.

### 10. Fix and confirm

```bash
timeout 5 ros2 topic pub -r 2 --qos-durability transient_local --qos-reliability reliable \
  /test_coord std_msgs/msg/String "data: 'hello with matching qos'"
```
Added `--qos-durability transient_local` to make the publisher's QoS compatible with what the
bridge's subscriber requires.

Result: the domain-1 echo printed `data: hello with matching qos` repeatedly — confirmed crossing the
bridge. It also printed an earlier "hello from domain 0 retry" message retroactively:
`TRANSIENT_LOCAL` durability means a publisher retains its last N samples (`KEEP_LAST` depth 10 here)
for any subscriber that joins later, so the bridge had cached that earlier message once discovery
caught up, and delivered it the moment a compatible subscriber (the new echo) appeared.

### 11. Confirm isolation still holds for non-bridged topics

```bash
# domain 0:
timeout 3 ros2 topic pub --once /test_unbridged std_msgs/msg/String "data: 'should stay on domain 0'"
# domain 1:
ros2 topic list | grep test_unbridged
```
Published on a topic name never mentioned in the bridge config, then checked whether it leaked into
domain 1's topic list.

Result: `NOT PRESENT` on domain 1 — confirmed nothing crosses except what's explicitly configured.

### 12. Clean up

```bash
docker exec auro-laptop bash -c "pkill -f 'domain_bridge /tmp/test_bridge_config.yaml'"
```
Killed only the throwaway test bridge process (matched by its exact command-line text). Both arms'
real control processes (`ros2_control_node`, confirmed via `ps aux` to still be running) were left
untouched.

## Current state after this sequence

- Arm A: running, `ROS_DOMAIN_ID=0`, fake hardware, all controllers active.
- Arm B: running, `ROS_DOMAIN_ID=1`, fake hardware, all controllers active.
- No `domain_bridge` process currently running (test one was killed; nothing permanent set up yet).
- `/tmp/test_bridge_config.yaml` still exists inside the container but is not tracked anywhere and
  will be lost on container removal.

## Known gotchas for whoever picks this up next

1. `robot_ip` is a required argument on `gen3_lite.launch.py` even with `use_fake_hardware:=true` —
   always pass a placeholder value.
2. `domain_bridge`'s subscribers default to `TRANSIENT_LOCAL` durability. Anything publishing into a
   bridged topic from a plain `ros2 topic pub` needs `--qos-durability transient_local
   --qos-reliability reliable` to be received. Real ROS nodes using default `rclpy`/`rclcpp` publisher
   QoS (usually `RELIABLE`/`VOLATILE` or matching whatever the subscriber requests) may or may not hit
   this — check compatibility with `ros2 topic info -v` if a bridged topic seems to silently drop
   messages.
3. `domain_bridge` process stdout appeared empty even while running correctly — don't use "no log
   output" as a sign it's broken; check `ps aux` and `ros2 node list`/`topic info -v` instead.

## Not yet done (next steps)

- Write the *real* `domain_bridge` config for actual coordination topics (`arm_a_status`,
  `arm_b_status`, `arm_a_stop`, `arm_b_stop`, and any agent command topics), committed into the repo
  (e.g. `config/domain_bridge/`) rather than hand-typed into `/tmp` each session.
- Static per-arm keepout collision boxes in `kinova_pick_planner.py` (Day 3–5 of the project plan).
- Duplicate `arm_controller.py` into two namespaced instances, one per domain.
