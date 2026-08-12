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

### 13. Attempted RViz — discovered the container had no display support

```bash
docker inspect auro-laptop --format '{{range .Config.Env}}{{println .}}{{end}}' | grep -i display
docker inspect auro-laptop --format '{{range .Mounts}}{{.Source}} -> {{.Destination}}{{println}}{{end}}' | grep -i x11
```
Checked whether the running container had a `DISPLAY` env var or the `/tmp/.X11-unix` socket mounted,
before attempting to launch `rviz2` (which fails immediately without both). Result: neither was
present — this container had been created earlier purely for headless file inspection, not GUI use.
Docker doesn't allow adding env vars or mounts to an already-running container, so this meant
recreating it.

### 14. Recreate the container with X11 support

```bash
docker rm -f auro-laptop
xhost +local:docker

docker run -d --name auro-laptop \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$(pwd)":/auro-final-project \
  -w /auro-final-project \
  auro-server:latest \
  sleep infinity
```
`docker rm -f` stops and removes the container in one step (this killed the two running arm
bringups — expected, they're stateless fake-hardware processes, trivially relaunched). `xhost
+local:docker` grants the host's X server permission to accept connections from local Docker
containers — without it, the container can have `DISPLAY` set correctly and still be refused a
connection. The new `docker run` matches what `run_arm_llm_tmux.sh` already does for the same reason.

### 15. Rebuild the image before continuing

Recreating the container surfaced a second gap: `ros-humble-domain-bridge` (step 2) had only ever
been `apt-get install`ed live on the *old*, now-deleted container — it was added to
`Dockerfile.server`'s source text, but the image itself was never rebuilt, so the fresh container
didn't have it either.

```bash
docker build -f Dockerfile.server -t auro-server:latest .
```
Full rebuild (reclones `ros2_kortex`, rebuilds `kortex_ws`, reinstalls all apt packages) — took
roughly 5 minutes. One benign warning (`pip`'s dependency resolver flagging a `setuptools` version
conflict with `colcon-core`) — not fatal, build completed successfully.

Then recreated the container again from the rebuilt image (same command as step 14), and confirmed:
```bash
docker exec auro-laptop bash -c "dpkg -l | grep domain-bridge"
# ii  ros-humble-domain-bridge   0.5.0-1jammy...
```
`domain_bridge` now present without any manual install step — it survives container recreation going
forward, since it's baked into the image itself.

### 16. Relaunch both arms with RViz and visually confirm

Same launch commands as steps 3–4, with `launch_rviz:=true` instead of `false`, and `-e
DISPLAY="$DISPLAY"` added to each `docker exec`. Both launches logged `OpenGl version: 4.5 (GLSL
4.5)` with no display-connection errors — confirmed the X11 forwarding path end-to-end.

**Visually confirmed**: two separate RViz windows opened, one per domain, each showing its own Gen3
Lite arm rendered in a static default pose (expected — fake hardware holds its initial joint state
until a trajectory is actually commanded; no motion has been issued yet).

### 17. Bring up MoveIt2 per arm

`kortex_bringup` (steps 3–4/16) only starts the `ros2_control` driver layer — no motion planning
capability. Actual pick/place motion (what `kinova_pick_planner`/`arm_controller` will need) requires
`move_group`, from `kinova_gen3_lite_moveit_config`'s `robot.launch.py`, run *alongside* the existing
driver on each domain:

```bash
docker exec -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e DISPLAY="$DISPLAY" auro-laptop bash -c "
source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=192.168.1.10 use_fake_hardware:=true launch_rviz:=true
"
# repeated for domain 1, robot_ip:=192.168.1.11
```
Confirmed this launch file only starts `move_group` + RViz + a static TF publisher — it does **not**
start its own `ros2_control_node`, so it attaches cleanly to the already-running driver rather than
conflicting with it. Both came up with `You can start planning now!` and RViz reporting `Ready to
take commands for planning group arm`. Only benign warnings (no depth sensor for octomap — expected,
no camera; a minor SRDF end-effector parent-group warning that doesn't block planning).

### 18. First real Plan & Execute attempt — three overlapping issues surfaced at once

Using the MotionPlanning panel's interactive marker to set a new goal pose, then `Plan` (succeeded,
`Global Status: Ok`) then `Execute`: command showed **`Failed`**, but the arm visibly moved to the
goal anyway. Chasing this surfaced three separate things — worth reading in order, since the first
two turned out to be red herrings for *this specific* failure, not for the project in general:

**a) High CPU load** (`docker stats` showed 1037%) — two *original* `kortex_bringup` RViz windows
(`view_robot.rviz`, redundant now that MoveIt2's own RViz was open) were each burning ~300% CPU.
Killed both (`docker exec auro-laptop kill 192 410`), load dropped to ~695%. Real and worth fixing
(see step 19), but not the actual cause of the `Execute` failure.

**b) `ros2 param set /joint_trajectory_controller constraints.goal_time ...` returned `Node not
found`**, and separately `ros2 node list` on domain 0 stopped showing the driver's own nodes
(`/controller_manager`, `/joint_trajectory_controller`, etc.) even though `ps aux` confirmed the
underlying process was still alive. Root cause, found by tailing the original driver launch log:
```
tev: ddsi_udp_conn_write to udp/10.138.164.112:33765 failed with retcode -1
```
— a continuous stream of these, on **both** arms, toward the same handful of stale peer ports.
These driver processes had been running for hours (since step 3/4); the host's network state (WiFi
reconnect, sleep/wake, DHCP renewal — this is a laptop) had changed underneath them, and CycloneDDS
was still trying to reach peer addresses discovered at startup that no longer resolved. This explained
*both* symptoms: the final "trajectory succeeded" acknowledgment from controller back to `move_group`
was lost over the broken connection (→ `Failed` in the UI) while the actual `FollowJointTrajectory`
goal still reached the controller and executed (→ the arm visibly moved anyway).

**c) `goal_time: 0.0` in `ros2_controllers.yaml`** — separately documented in
`kinova_pick_planner/README.md`'s own Troubleshooting section: zero tolerance for finishing a
trajectory even slightly late, which trips easily at low `Accel. Scaling`. This is a real, general
gotcha independent of (b) — needs setting after *every* MoveIt2 restart, on *both* arms now:
```bash
ros2 param set /joint_trajectory_controller constraints.goal_time 30.0
```

### 19. Clean restart of both arms

Given (b), the fix wasn't "set the param harder" — it was restarting the driver so it re-establishes
DDS transport state against the *current* network config:
```bash
docker exec auro-laptop pkill -f "ros2_control_node|robot_state_publisher|move_group|rviz2|static_transform_publisher"
# pkill missed ros2_control_node and robot_state_publisher specifically (see gotcha 6 below) —
# killed the remaining 4 PIDs explicitly:
docker exec auro-laptop kill -9 46 48 264 266
```
Relaunched `kortex_bringup` **headless** this time (`launch_rviz:=false` — the MoveIt2 RViz already
shows the robot model, no need for the redundant, CPU-heavy second window) for both arms, then
MoveIt2 with RViz for both arms (same commands as steps 3–4 and 17). Confirmed clean: no DDS write
errors in either driver's fresh log, `ros2 param set ... goal_time 30.0` returned `Set parameter
successful` on both domains, and `ros2 node list` on domain 0 showed the full graph again (all 6
driver nodes + all of MoveIt2's nodes together). CPU dropped to ~672% (down from 1037%) with only 2
RViz windows total instead of 4.

### 20. Plan & Execute retested — succeeded, but appeared to "never stop"

Re-ran Plan & Execute on arm A. Status showed `Executed` (not `Failed` — confirms step 18c's fix
worked), but the visible robot model appeared to keep sliding continuously and never settle. Verified
at the data level before assuming anything was actually wrong:
```bash
# sampled /joint_states twice, 3 seconds apart
ros2 topic echo /joint_states --once
```
Position values were identical across both samples — the real (simulated) robot state was frozen, not
moving. Cross-checked `move_group`'s log: a single, clean
`Execute request accepted → Starting trajectory execution → Goal request accepted!` →
(~16s later) `Controller ... successfully finished → Completed trajectory execution with status
SUCCEEDED → Execute request success!` — one execution, no repeated goal submissions, no loop.

**Actual cause**: RViz's `MotionPlanning` display has a `Planned Path` sub-display with a `Loop
Animation` option, on by default in the generated `moveit.rviz` config — it continuously replays a
semi-transparent ghost of the last planned trajectory, purely as a local visualization, completely
decoupled from live robot state. Confirmed by comparing screenshots: the opaque, correctly-colored
arm model stayed in the same position across all of them; only the semi-transparent gray ghost moved.
**Fix**: `Displays` panel → `MotionPlanning` → `Planned Path` → uncheck `Loop Animation`.

## Current state after this sequence

- Arm A: running, `ROS_DOMAIN_ID=0`, fake hardware, all controllers active, MoveIt2 (`move_group`) up,
  one RViz window (MoveIt2's `moveit.rviz`, `Loop Animation` unchecked), `goal_time` set to 30.0.
  Plan & Execute confirmed genuinely working end-to-end.
- Arm B: same, `ROS_DOMAIN_ID=1`, `robot_ip:=192.168.1.11`.
- Container `auro-laptop` recreated from a freshly rebuilt `auro-server:latest` image — `DISPLAY`/X11
  and `domain_bridge` both now properly provisioned, no more manual per-container patching needed for
  either.
- No `domain_bridge` process currently running (the step-7 test one was killed; nothing permanent set
  up yet — see next steps).
- `/tmp/test_bridge_config.yaml` no longer exists (its container was removed in step 14); it was never
  meant to persist.

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
4. `docker run`/`docker exec` env vars and mounts (`DISPLAY`, `/tmp/.X11-unix`, `ROS_DOMAIN_ID`,
   etc.) can't be added to an already-running container — needing GUI support after a container was
   started headless means recreating it, not reconfiguring it in place.
5. Anything installed live into a running container (`apt-get install ...`) is lost the moment that
   container is removed — it has to be added to `Dockerfile.server` **and the image rebuilt**
   (`docker build -f Dockerfile.server -t auro-server:latest .`) to actually persist across container
   recreations. Editing the Dockerfile alone isn't enough without the rebuild step.
6. `pkill -f "pattern1|pattern2|..."` did not reliably match all target processes (missed
   `ros2_control_node`/`robot_state_publisher` while catching `move_group`/`rviz2`/
   `static_transform_publisher` in the same call) — if a `pkill -f` doesn't fully clean up, verify
   with `ps aux` and kill remaining PIDs explicitly rather than assuming the pattern covered everything.
7. Long-running `ros2_control_node`/driver processes on a laptop can silently break at the DDS
   transport level if the host's network state changes underneath them (WiFi reconnect, sleep/wake,
   VPN, DHCP renewal) — symptoms are misleading (`ros2 param set` → `Node not found`, MoveIt2 reports
   `Execute` as `Failed` even though the arm actually moved). Check the driver's own log for repeated
   `ddsi_udp_conn_write ... failed` lines; the fix is restarting the affected launch, not debugging the
   higher-level symptom. Not yet mitigated long-term — the project's own `cyclonedds_local.xml`
   generation (used by `run_arm_llm_tmux.sh`/`run_perception_control_tmux.sh` to pin specific physical
   interfaces) was not used for this manual test session and might reduce how often this recurs; worth
   adopting for longer dual-arm test sessions.
8. `goal_time: 0.0` in `ros2_controllers.yaml` (zero tolerance for finishing a trajectory even slightly
   late) must be raised after *every* MoveIt2 restart, per arm — already documented in
   `kinova_pick_planner/README.md`'s Troubleshooting section, re-confirmed here for the dual-arm case:
   `ros2 param set /joint_trajectory_controller constraints.goal_time 30.0`.
9. RViz's `MotionPlanning` → `Planned Path` → `Loop Animation` (on by default in the generated
   `moveit.rviz`) continuously replays the last planned trajectory as a semi-transparent ghost,
   independent of live robot state — easy to mistake for the real robot not stopping. Check
   `/joint_states` for actual changes (not just visual impression) before assuming an execution issue;
   uncheck `Loop Animation` to remove the ghost replay entirely.
10. Running 2 full RViz + MoveIt2 stacks concurrently on this machine is genuinely CPU-heavy (~670%
    with one RViz window per arm; was over 1000% with two windows per arm) — avoid launching
    `kortex_bringup` with `launch_rviz:=true` *and* MoveIt2's own RViz for the same arm; one RViz
    window per arm (MoveIt2's) is enough and roughly halves the load.

## Not yet done (next steps)

- Write the *real* `domain_bridge` config for actual coordination topics (`arm_a_status`,
  `arm_b_status`, `arm_a_stop`, `arm_b_stop`, and any agent command topics), committed into the repo
  (e.g. `config/domain_bridge/`) rather than hand-typed into `/tmp` each session.
- Static per-arm keepout collision boxes in `kinova_pick_planner.py` (Day 3–5 of the project plan).
- Duplicate `arm_controller.py` into two namespaced instances, one per domain.
