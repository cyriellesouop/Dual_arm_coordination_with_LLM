# MuJoCo Installation + Validation Sequence

Status as of this writing: **MuJoCo physics engine and `mujoco_ros_pkgs` (the ROS2/`ros2_control`
bridge) installed, built, and fully validated end-to-end — first with the package's own pendulum
example, then with our actual Gen3 Lite robot, which now renders and runs correctly in MuJoCo's
native viewer.** Part 1 (below) covers the engine/package install and the pendulum validation. Part
2 covers getting our own robot loading and rendering.

## Context / why

Day 3–5's static "other-arm keepout" collision box (see `dual_arm_static_keepout_sequence.md`) is a
crude placeholder — a static box approximating where the other arm might be, not real geometry, not
live collision detection. To get an actual shared scene (table + both arms + manipulable objects,
with genuine physics-based collision between the two arms) requires a real physics simulator.

Compared against Gazebo, Isaac Sim, and PyBullet, MuJoCo was chosen because: it can plug into our
existing `ros2_control` stack as a drop-in hardware-interface backend (meaning `arm_controller.py`,
`kinova_pick_planner.py`, and our MoveIt2 configs need no changes — only the hardware backend
changes from `mock_components/GenericSystem` to a MuJoCo-backed one); it's lightweight enough for
this laptop; Gazebo's own Kinova support in `ros2_kortex` turned out to exist only for plain Gen3, not
Gen3 Lite (the same "wrong variant" trap hit repeatedly during the Day 1–3 dual-arm bringup
investigation); and Isaac Sim needs a capable NVIDIA GPU this machine may not have to spare, given how
CPU-constrained it already is running two RViz+MoveIt2 stacks.

This decision also means committing to a **namespace-separated** (not domain-isolated) architecture
for the eventual dual-arm MuJoCo scene — see the "not yet done" section for why, and the
per_arm_bringup_sequence.md discussion for how this differs from the Day 1–3 architecture.

## Which package: `mujoco_ros_pkgs` (ubi-agni / Bielefeld University)

Repo: https://github.com/ubi-agni/mujoco_ros_pkgs (branch `hybrid-devel` — supports both ROS1 and
ROS2 Humble from the same codebase). An alternative, `ros-controls/mujoco_ros2_control`, was
identified during initial research but not used — `mujoco_ros_pkgs` is what was actually installed
and tested. Citable source (useful for the eventual paper's methods/tools section):

```bibtex
@inproceedings{leinsMuJoCoROSIntegrating2025,
  author={Leins, David P. and Haschke, Robert and Ritter, Helge},
  title={MuJoCo ROS: Integrating ROS with the MuJoCo Engine for Accurate and Scalable Robotic Simulation},
  booktitle={2025 IEEE International Conference on Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR)},
  year={2025},
  doi={10.1109/SIMPAR62925.2025.10979045}
}
```

Key package for our purposes: `mujoco_ros_control` — the `ros2_control` hardware-interface plugin
that bridges MuJoCo's physics to the same `controller_manager`/`ros2_control` stack our Kinova arms
already use.

## Prerequisite check (before installing anything)

Verified compatibility empirically rather than assuming, given this session's track record of
version-mismatch surprises:

| Component | Installed in `auro-laptop` | MuJoCo's requirement |
|---|---|---|
| Python | 3.10.12 | `>= 3.10` (for pip bindings) |
| numpy | 1.26.4 | No known issue |
| cmake | 3.22.1 | No unusual minimum documented |
| g++ | 11.4.0 (Ubuntu 22.04 default) | Modern C++17/20-capable |
| OS | Ubuntu 22.04.5 LTS | Matches DeepMind's Linux tarball build target |
| OpenGL (Mesa/GLEW/GLU) | Present | Needed for rendering |
| GLFW | **Not present** | Needed only for the interactive `simulate` GUI viewer |

The missing GLFW means the interactive viewer isn't available yet (confirmed later: `GLFW3 not
found. GUI will not be available.` — benign, doesn't block the physics engine or `ros2_control`
integration). Fixable later with `apt-get install libglfw3-dev` if the GUI viewer is wanted.

## Part 1: Engine + package install, validated with the pendulum example

### 1. Install MuJoCo itself — inside the container, not the host

Same reasoning as every other dependency this session: `ros2_control`, ROS2 Humble, and the whole
robot stack live inside `auro-laptop`, not on the host laptop. MuJoCo has to be built against and
loaded by that same environment to be usable at all.

Downloaded (by hand, inside an interactive `docker exec -it auro-laptop bash` shell) from the
official prebuilt-binaries release page — MuJoCo's own README recommends this over building from
source. Initially considered the latest release (3.11.0), but see step 2 for why the final version
used was different.

### 2. Version chosen: 3.3.5 (not the latest 3.11.0, and not 3.9.0 either — see below)

First attempt used **3.9.0** ("should be more stable" — a reasonable general instinct, though not
actually the deciding factor here). Downloaded, extracted to `/root/.mujoco/mujoco-3.9.0/`, verified
directory structure (`bin/`, `include/`, `lib/`, `model/`, `sample/`, `simulate/` all present, matching
a standard release).

Environment variables set in `/root/.bashrc` (persists across interactive shells in this container):
```bash
export MUJOCO_DIR=$HOME/.mujoco/mujoco-3.9.0
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib
export LIBRARY_PATH=$LIBRARY_PATH:$MUJOCO_DIR/lib
```
**First gotcha hit here**: copy-pasting these from a `docker exec ... bash -c "..."`-formatted
command (which needs `\$` escaping for the *outer* shell) directly into an *already-interactive*
container shell wrote literal backslashes into `.bashrc`, breaking variable expansion. Fixed with
`sed -i '/MUJOCO/d' /root/.bashrc` followed by re-adding the three lines with correct (unescaped)
quoting. Verified with `ls $MUJOCO_DIR/lib/ | grep -i mujoco` → `libmujoco.so`, `libmujoco.so.3.9.0`.

### 3. Clone and build `mujoco_ros_pkgs` — first two failures were ordinary missing apt deps

```bash
cd /kortex_ws/src
git clone -b hybrid-devel https://github.com/ubi-agni/mujoco_ros_pkgs.git
cd /kortex_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_ros mujoco_ros_msgs mujoco_ros_testing_utils mujoco_ros_sensors mujoco_ros_laser mujoco_ros_mocap mujoco_ros_control
```
Placed in `/kortex_ws/src`, matching the existing pattern for vendor source (`ros2_kortex`,
`pymoveit2`) already in this Dockerfile.

**Failure 1**: `py_binding_tools was not found`. Checked `apt-cache policy
ros-humble-py-binding-tools` — came back empty. Confirmed via web search that this package *was*
officially bloom-released for Humble; the empty result was a **stale local apt index**, not genuine
unavailability. `apt-get update` then showed it as installable; installed cleanly.

**Failure 2**: `camera_info_manager was not found` (a CMake `find_package` failure this time, not a
missing binary at link time). Checked with a fresh index from the start this time —
`ros-humble-camera-info-manager` was available immediately, installed cleanly.

### 4. Failure 3 — a genuine MuJoCo API version mismatch, not a missing dependency

```
error: 'mjENBL_ISLAND' was not declared in this scope; did you mean 'mjDSBL_ISLAND'?
error: 'MJDATA_POINTERS_PREAMBLE' was not declared in this scope
error: 'mjOption' has no member named 'apirate'
error: 'mjWARN_VGEOMFULL' was not declared in this scope; did you mean 'mjWARN_CNSTRFULL'?
error: 'mjDSBL_PASSIVE' was not declared in this scope
error: 'mjENBL_MULTICCD' was not declared in this scope; did you mean 'mjDSBL_MULTICCD'?
```
The pattern (multiple renamed/removed enum constants and struct fields, each with a similarly-named
alternative) is the signature of a MuJoCo API surface change between versions — `mujoco_ros_pkgs`'
`hybrid-devel` branch was written against a different MuJoCo release than 3.9.0. Corroborating
evidence: the package's own README documented its install example using
`$HOME/.mujoco/mujoco-3.3.5` specifically, not an arbitrary placeholder.

**Decision: switch to MuJoCo 3.3.5.** Not a compatibility statement about 3.9 vs 3.11 in general
(both satisfy MuJoCo's own stated requirements) — specific to matching *this ROS wrapper's* tested
API version.

### 5. Install 3.3.5 alongside 3.9.0 (no uninstall needed — plain extracted directories, no package
manager involved)

Downloaded and extracted the same way, to `/root/.mujoco/mujoco-3.3.5/`. Updated the three env vars
in `.bashrc` — **hit two more mishaps here**, both self-corrected:

1. Appending the new `mujoco-3.3.5` lines *without first removing* the old (correct) `mujoco-3.9.0`
   lines meant `LD_LIBRARY_PATH`/`LIBRARY_PATH` accumulated *both* paths (each line appends to its own
   previous value, so `MUJOCO_DIR` itself correctly ended up as `3.3.5`, but the path variables kept
   the stale `3.9.0/lib` entry too — a real risk, since the dynamic linker could still resolve
   `libmujoco.so` from the wrong version at runtime). Fixed with another `sed -i '/MUJOCO/d'` +
   clean re-add.
2. Then ran the *same* `sed -i '/MUJOCO/d'` a second time, *after* re-adding the correct lines —
   deleting them again by accident. Caught via `tail -5 /root/.bashrc` showing no MUJOCO lines at
   all; fixed by re-adding one final time, verified with `ls $MUJOCO_DIR/lib/` → `libmujoco.so`,
   `libmujoco.so.3.3.5`.

### 6. Rebuild — hit a stale CMake cache from the earlier failed 3.9.0-era build

```
gmake[2]: *** No rule to make target '/root/.mujoco/mujoco-3.9.0/lib/libmujoco.so', ...
```
Even with `MUJOCO_DIR` now correctly pointing at 3.3.5, the Makefiles generated during the *first*
(failed) CMake configure pass — back when `MUJOCO_DIR` still said 3.9.0 — were cached in
`/kortex_ws/build/mujoco_ros/` and don't auto-refresh just because an environment variable changed
later. Fixed by clearing the stale build/install artifacts for just that one package and rebuilding:
```bash
rm -rf /kortex_ws/build/mujoco_ros /kortex_ws/install/mujoco_ros
colcon build --packages-select mujoco_ros mujoco_ros_msgs mujoco_ros_testing_utils mujoco_ros_sensors mujoco_ros_laser mujoco_ros_mocap mujoco_ros_control
```
(`mujoco_ros_msgs`/`mujoco_ros_testing_utils` didn't need clearing — they'd already built successfully
and don't depend on MuJoCo at all.)

**Result: all 7 packages built cleanly** — only benign warnings (`ccache` not found → slower
incremental rebuilds, not an error; `GLFW3 not found` → confirms the earlier prerequisite-check
finding, no GUI viewer yet).

### 7. `colcon test-result` — 6 failures, unrelated to the build itself

Ran separately, surfaced 6 pre-existing test failures, all the identical root cause:
```
ModuleNotFoundError: No module named 'mujoco'
```
This is the **Python** package (`pip install mujoco`) — distinct from the C++ library
(`libmujoco.so`) already installed. Not a blocker for the actual goal (the C++ `ros2_control`
plugin), but installed anyway (`pip install mujoco`) since it may be useful later for scripting the
scene (placing the table/objects procedurally) rather than hand-authoring MJCF/XML.

### 8. Validation: the package's own pendulum example — first attempt, mixed results

```bash
source /kortex_ws/install/setup.bash
ros2 launch mujoco_ros_control mujoco_ros_control.launch.py
```
This example loads **four** pendulums simultaneously (`fallback`/`motor`/`velocity`/`position`
mechanisms, each demonstrating a different `ros2_control` command interface), configured via
`ros2_control_plugins_example.yaml`:
```yaml
joint_state_broadcaster: {type: joint_state_broadcaster/JointStateBroadcaster}
fallback_effort_controller: {type: forward_command_controller/ForwardCommandController}
motor_effort_controller:    {type: forward_command_controller/ForwardCommandController}
velocity_controller:        {type: forward_command_controller/ForwardCommandController}
position_controller:        {type: forward_command_controller/ForwardCommandController}
```
First run: a `mock_components/GenericSystem` plugin-class error appeared (twice), then all 5
controller spawners failed with `FATAL: Failed loading controller`, plus a confusing contradiction —
`joint_state_broadcaster` reported "already loaded, skipping" immediately followed by "no controller
with this name exists."

Confirmed `forward_command_controller` (the controller type actually used) was already installed —
ruled out a missing-package explanation. A second run (after `pip install mujoco`) showed the
`MujocoRosSystem` hardware interface itself now initializing/configuring/activating successfully —
real progress — but the same controller-loading contradiction persisted identically.

### 9. Root cause: `ROS_DOMAIN_ID` was unset, colliding with Arm A on domain 0

`echo $ROS_DOMAIN_ID` returned empty → defaults to `0`, the same domain Arm A has been running on
this entire session (see `per_arm_bringup_sequence.md`). Every `controller_manager` process —
Arm A's driver, and this pendulum's `mujoco_node` — creates a ROS node with the identical name
`/controller_manager`. With both reachable on domain 0, DDS service calls
(`/controller_manager/load_controller`, `/configure_controller`) had no guaranteed routing to "the
right" instance:
- **"Failed loading controller X"**: request served by *Arm A's* controller_manager, which has no
  idea what a pendulum's `position_controller` is.
- **"already loaded, skipping"** for `joint_state_broadcaster`: Arm A's controller_manager genuinely
  *does* have its own `joint_state_broadcaster` (for the real Kinova arm) — a false positive from
  querying the wrong instance.
- **"no controller with this name exists"**: the very next request happened to land on the *correct*
  (pendulum's) controller_manager, which had never actually received the load request.

Which instance served which request wasn't consistent — DDS doesn't guarantee ordering/routing when
multiple servers advertise an identical service name on one domain, which is why the symptoms looked
like random contradictory chaos rather than one clean, explainable error.

### 10. Fix and final validation

```bash
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch mujoco_ros_control mujoco_ros_control.launch.py
```
`ROS_DOMAIN_ID=2` (a domain neither arm uses) is what actually fixed it — giving the pendulum's
entire ROS graph a domain with nothing else on it, eliminating the `/controller_manager` name
collision entirely. `RMW_IMPLEMENTATION` is consistency hygiene (matches the rest of the project's
DDS implementation choice), not itself the fix.

**Result: complete, clean success.** All 5 controllers (`joint_state_broadcaster`,
`position_controller`, `motor_effort_controller`, `fallback_effort_controller`,
`velocity_controller`) loaded → configured → activated, in sequence, with no errors. All 5 spawner
processes exited cleanly.

## Part 2: Getting our own Gen3 Lite arm loading and rendering

### 11. Checked our URDF's `<ros2_control>` block — two things missing

Inspected `kortex_description/arms/gen3_lite/6dof/urdf/kortex.ros2_control.xacro` (the vendor file
that defines the arm's `ros2_control` hardware/joint interfaces, used for the real driver and fake
hardware all session). Two gaps, both expected from the pendulum-test C++ source review:
- No `kp`/`kv` anywhere — every joint only declares a `position` command interface, no fallback gain.
  `MujocoRosSystem::initSim()` requires a positive `kp` for any position-controlled joint without a
  matching named MuJoCo actuator (`{joint}_act_pos`), which is our case (no actuators authored).
- No `sim_mujoco` branch in the `<hardware>` plugin selection — it already conditionally branches on
  `sim_gazebo`/`sim_ignition`/`sim_isaac`/`use_fake_hardware`/real-hardware, but nothing for MuJoCo.

### 12. Mapped the full vendor xacro chain needing the same treatment

`kortex_ros2_control` (the macro with the actual plugin/joint definitions) is included and its
params forwarded through **three** more vendor files, each with their own `sim_gazebo`/`sim_ignition`/
`sim_isaac` params and forwarding calls that needed a matching `sim_mujoco` addition:
```
kinova.urdf.xacro  →  kortex_robot.xacro  →  gen3_lite_macro.xacro  →  kortex.ros2_control.xacro
(top-level, what      (load_robot macro)     (load_arm macro)          (already identified above)
 xacro is invoked on)
```
Checked via `grep -n "sim_gazebo\|sim_ignition\|sim_isaac\|sim_mujoco"` on each file before writing
anything, to map the full depth in one pass rather than discovering a third level after already
patching two.

### 13. Wrote four maintained override files, same "never edit vendor" principle as before

All four written to `docker/kortex_overrides/` (mirroring the vendor filenames exactly), each with a
header comment documenting the precise diff from the original — same pattern as the parked,
unused prefixed-controller-yaml from the earlier shared-URDF investigation, except these are
actively wired in:
- `gen3_lite_kortex.ros2_control.xacro` — the `sim_mujoco` plugin branch + `kp` params (see step 15
  for a correction made to this file after the first test run).
- `gen3_lite_macro.xacro` — `sim_mujoco:=false` param + forwarding + a `ros2_control_name` label
  branch mirroring the existing `use_fake_hardware` one.
- `kortex_robot.xacro` — `sim_mujoco:=false` param + forwarding to `load_arm`.
- `kinova.urdf.xacro` — `<xacro:arg name="sim_mujoco" default="false" />` + forwarding to
  `load_robot`. No new `<gazebo>`-style plugin block added here (unlike the `sim_gazebo`/
  `sim_ignition` branches) — MuJoCo integration is entirely `<ros2_control>`-based, no Gazebo-style
  plugin tags involved.

All safety-relevant content (joint min/max limits, masses, inertias, mesh references) copied
verbatim in every file — never modified, never re-derived.

### 14. Installed live and tested the full compile chain

```bash
docker cp gen3_lite_kortex.ros2_control.xacro auro-laptop:/kortex_ws/install/kortex_description/share/kortex_description/arms/gen3_lite/6dof/urdf/kortex.ros2_control.xacro
docker cp kortex_robot.xacro auro-laptop:/kortex_ws/install/kortex_description/share/kortex_description/robots/kortex_robot.xacro
docker cp kinova.urdf.xacro auro-laptop:/kortex_ws/install/kortex_description/share/kortex_description/robots/kinova.urdf.xacro
```
**First attempt failed**: `error: Invalid parameter "sim_mujoco" when instantiating macro: load_arm`
— `gen3_lite_macro.xacro` had been written locally but never actually `docker cp`'d into the
container (only the other three were copied). Copied the missed file, retried:
```bash
docker cp gen3_lite_macro.xacro auro-laptop:/kortex_ws/install/kortex_description/share/kortex_description/arms/gen3_lite/6dof/urdf/gen3_lite_macro.xacro

xacro /kortex_ws/install/kortex_description/share/kortex_description/robots/kinova.urdf.xacro \
  name:=gen3_lite arm:=gen3_lite dof:=6 robot_ip:=192.168.1.10 \
  gripper:=gen3_lite_2f gripper_joint_name:=right_finger_bottom_joint \
  use_internal_bus_gripper_comm:=false \
  sim_mujoco:=true \
  -o /tmp/test_robot_description.urdf
```
**Result: exit code 0.** Verified the output directly:
```bash
grep -n "MujocoRosSystem\|<param name=\"kp\"" /tmp/test_robot_description.urdf
```
showed `<plugin>mujoco_ros_control/MujocoRosSystem</plugin>` and 7 `kp` entries — the full 4-file
override chain compiles correctly end-to-end.

### 15. Tested whether MuJoCo can load the URDF directly — yes, with one path fix

```python
import mujoco
model = mujoco.MjModel.from_xml_path('/tmp/test_robot_description.urdf')
```
**First attempt failed**: `Error opening file 'file:///kortex_ws/.../forearm_link.STL'` — MuJoCo's
mesh loader doesn't understand the `file://` URI scheme our xacro uses, it expects a plain
filesystem path. Confirmed the file genuinely exists at that path (just without the prefix) before
concluding it was a format issue, not a missing file. Fixed by stripping the prefix:
```python
with open('/tmp/test_robot_description.urdf') as f:
    urdf = f.read()
with open('/tmp/test_robot_description_fixed.urdf', 'w') as f:
    f.write(urdf.replace('file://', ''))
```
**Result: complete success.** `nbody: 11`, `njnt: 10` — `joint_1`–`joint_6` plus all four gripper
joints, matching the Gen3 Lite's real kinematic structure exactly, with **no separate MJCF authoring
needed at all** — MuJoCo imports the existing, already-tested URDF directly.

### 16. Built the actual launch flow — hit a second, different mesh-path issue

`mujoco_node`'s `modelfile` parameter loads a physics model from a file path; `ros2_control`'s
`robot_description` is fetched live as a parameter from `robot_state_publisher`. These are two
different consumption paths needing two prepared files:
```bash
mkdir -p /tmp/mujoco_gen3_lite
xacro .../kinova.urdf.xacro ... sim_mujoco:=true -o /tmp/mujoco_gen3_lite/robot_description.urdf
python3 -c "
with open('/tmp/mujoco_gen3_lite/robot_description.urdf') as f: urdf = f.read()
with open('/tmp/mujoco_gen3_lite/robot_description_mujoco.urdf', 'w') as f:
    f.write(urdf.replace('file://', ''))
"
```
Minimal plugin config written by hand (`/tmp/mujoco_gen3_lite/plugin_config.yaml`) — deliberately
started with **only** `joint_state_broadcaster`, no motion controller yet, to isolate "does the robot
load and the hardware interface activate" from "does motion control work," same incremental
de-risking used throughout this project:
```yaml
/mujoco_server:
  ros__parameters:
    MujocoPlugins.names: [mujoco_ros_control]
    MujocoPlugins.mujoco_ros_control.type: mujoco_ros_control/MujocoRosControlPlugin
/mujoco_server/mujoco_ros_control:
  ros__parameters:
    namespace: ""
    robot_description_node: robot_state_publisher
    robot_description: robot_description
/controller_manager:
  ros__parameters:
    update_rate: 50
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```
```bash
# on a fresh domain, distinct from the pendulum test's domain 2 and both arms' 0/1
docker exec -e ROS_DOMAIN_ID=3 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e DISPLAY="$DISPLAY" auro-laptop bash -c "
ros2 launch mujoco_ros launch_server.launch.xml \
  use_sim_time:=true \
  modelfile:=/tmp/mujoco_gen3_lite/robot_description_mujoco.urdf \
  verbose:=true \
  mujoco_plugin_config:=/tmp/mujoco_gen3_lite/plugin_config.yaml
"
```
**Failed**: `Error opening file '/tmp/mujoco_gen3_lite/base_link.STL': No such file or directory` — a
*different* mesh-path problem than step 15's. Confirmed the mesh filenames in the file still had
correct full absolute paths (ruled out a regeneration mistake), which revealed MuJoCo's actual
behavior here: it resolves mesh filenames by **basename only, relative to the loaded XML's own
directory**, discarding whatever directory the `filename` attribute specifies, absolute or not. Fixed
by copying the actual mesh files alongside the URDF:
```bash
cp kortex_description/.../gen3_lite/6dof/meshes/*.STL /tmp/mujoco_gen3_lite/
cp kortex_description/.../gen3_lite_2f/meshes/*.STL /tmp/mujoco_gen3_lite/
```
(11 files total — 6 arm links + 5 gripper links.)

### 17. Real bug found: `kp` was in the wrong XML location

Relaunched with meshes in place — model itself loaded correctly this time (`Model loaded in
0.257784 seconds`), but the `mujoco_ros_control` plugin then failed:
```
Joint joint_1 needs a positive kp parameter for position fallback control.
FATAL: Could not initialize robot simulation interface
```
Even though `kp` genuinely was present in the URDF. Root cause, traced back to
`mujoco_ros_system.cpp`'s `initSim()`: it reads `joint_info.parameters.find("kp")` — the **joint-level**
parameter map — but the override had nested `<param name="kp">` *inside* `<command_interface
name="position">`, which populates the interface-level parameter map instead (correct for `min`/`max`,
wrong for `kp`). Fixed in `gen3_lite_kortex.ros2_control.xacro` by moving `kp` to be a direct child of
`<joint>`, for all 7 joints:
```xml
<!-- before (wrong) -->
<joint name="joint_1">
  <command_interface name="position">
    <param name="min">-2.69</param>
    <param name="kp">50.0</param>  <!-- wrong scope -->
  </command_interface>
</joint>
<!-- after (correct) -->
<joint name="joint_1">
  <param name="kp">50.0</param>
  <command_interface name="position">
    <param name="min">-2.69</param>
  </command_interface>
</joint>
```
Re-copied the fixed file, regenerated both URDFs, killed and restarted `robot_state_publisher` (it
was serving the pre-fix content from a static file, not something that re-reads changes) and
`mujoco_node`. **Result: complete success** — `MujocoRosSystem` hardware interface fully
initialized → configured → activated, all 7 joints matched by name, no errors.

### 18. Viewer instability — unrelated to the integration itself

Two separate GUI issues surfaced *after* the ROS-level integration had already succeeded, neither of
which reflected a problem with the actual result:
- First launch attempt: `XIO: fatal IO error 11 (Resource temporarily unavailable) on X server`
  killed the whole `mujoco_node` process, immediately after the plugin load had already failed on
  the step-17 `kp` bug (may have been coincidental/downstream of that failure, not investigated
  further since the real bug was fixed anyway).
- Second launch (post-fix): GUI showed a desktop "not responding" dialog. Checked `docker stats` and
  the process's own CPU (167%, actively busy, not stuck at 0%) before deciding whether to Force Quit
  — consistent with the same X11-forwarded-OpenGL vsync/CPU-load pattern already documented for RViz
  (gotcha 10, `per_arm_bringup_sequence.md`), not a deadlock. Window was closed (not Force Quit) —
  the underlying `mujoco_node` process survived, just with the viewer disconnected.
- Checked for MuJoCo's offscreen-camera topics as an alternative visual (worked reliably in the
  pendulum test) — none present, since the pendulum's 2 cameras were defined *inside its own
  purpose-built MJCF*, not something added automatically for any loaded model. Our bare URDF has no
  camera definitions at all.

### 19. Clean relaunch — stable, robot rendered correctly

Killed the unresponsive `mujoco_node`, relaunched fresh once system load had settled:
```bash
docker exec auro-laptop kill -9 <mujoco_node_pid>
# same launch command as step 16
```
This time, `robot_state_publisher` was already up and ready, so the plugin load took 0.013 seconds
(vs. ~19–45 seconds waiting on earlier attempts). **Result: stable, responsive viewer, 28 FPS,
correctly rendered Gen3 Lite arm** (gray body, blue gripper fingers, matching the real robot's
geometry). Scene background is black/empty — expected, not a bug: the pendulum's floor/lighting/
skybox came from its own hand-authored MJCF scene file; our model is the bare URDF with none of that
environment geometry defined. Resolves naturally once the real scene (table, floor, lighting, both
arms) is built.

## Current state after this sequence

- MuJoCo 3.3.5 installed at `/root/.mujoco/mujoco-3.3.5/` inside `auro-laptop`, env vars set in
  `/root/.bashrc` **and now also in `Dockerfile.server`** — validated via a full `docker build` from
  scratch.
- MuJoCo 3.9.0 also still present at `/root/.mujoco/mujoco-3.9.0/` (never removed — harmless to leave,
  not referenced by any current env var).
- `mujoco_ros_pkgs` (`hybrid-devel` branch) cloned into `/kortex_ws/src/mujoco_ros_pkgs/`, all 7
  packages built successfully against MuJoCo 3.3.5.
- Python `mujoco` package installed via `pip install mujoco` (fixes the binding tests; not otherwise
  required yet).
- The pendulum example fully validated on `ROS_DOMAIN_ID=2` — confirms `mujoco_ros_control`
  genuinely bridges `ros2_control` to real MuJoCo physics.
- **Our own Gen3 Lite robot loads, renders, and can be commanded to move correctly in MuJoCo**,
  validated on `ROS_DOMAIN_ID=3`: `MujocoRosSystem` hardware interface fully activated, all 7 joints
  matched by name, `joint_state_broadcaster` + `joint_trajectory_controller` +
  `gen3_lite_2f_gripper_controller` all active, kp/damping/effort_limit tuned from measured
  step-response data. Full detail in `mujoco_motion_controller_tuning.md`.
- Four maintained xacro overrides live in `docker/kortex_overrides/` (`kinova.urdf.xacro`,
  `kortex_robot.xacro`, `gen3_lite_macro.xacro`, `gen3_lite_kortex.ros2_control.xacro`), installed
  live over the vendor paths in the running container **and now also `COPY`'d in `Dockerfile.server`**,
  validated via a full rebuild.
- Generated artifacts live in `/tmp/mujoco_gen3_lite/` inside the container (both URDF variants, the
  11 copied mesh STLs, `plugin_config.yaml`) — ephemeral, not saved anywhere durable yet.
- A real single-arm scene exists: `mujoco_scenes/scene_single_arm.xml` (table, floor, lighting, two
  placeholder objects, an `overview` camera) including `mujoco_scenes/gen3_lite.xml` (native MJCF,
  repositionable via its `gen3_lite_base` wrapper body) — background is no longer black/empty.

## Known gotchas from this pass

1. `apt-cache policy` returning empty for a package doesn't mean it's unavailable — could just be a
   stale local index. Run `apt-get update` before concluding a package doesn't exist (bit us twice
   this session now: `domain_bridge` earlier, `py_binding_tools` here).
2. Copy-pasting `docker exec ... bash -c "..."`-formatted commands (with `\$`-escaped variables)
   directly into an *already-interactive* container shell breaks the escaping — write `.bashrc`
   entries with plain, unescaped `$` when typing directly into an interactive shell.
3. Env vars that *append* to their own previous value (`export X=$X:...`) will silently accumulate
   stale entries if old lines aren't removed before adding new ones — check `tail -5 /root/.bashrc`
   before and after any change, don't assume.
4. Changing `MUJOCO_DIR` (or similar build-time env vars) does **not** invalidate an already-cached
   CMake configuration — `colcon build` will keep using stale paths from the first configure pass
   until the package's `build/`/`install/` directories are removed and it's reconfigured from
   scratch.
5. A MuJoCo API version mismatch shows up as multiple *similarly-named* missing symbols (e.g.
   `mjENBL_X` missing but `mjDSBL_X` present) — that pattern, not any single error, is the signal to
   check which MuJoCo version the wrapper package was actually written against (check its own README
   examples) rather than assuming the newest release is safest.
6. Multiple `ros2_control` processes with default names (`/controller_manager`) landing on the same
   `ROS_DOMAIN_ID` (including by accident, via an unset env var defaulting to `0`) produces
   confusing, seemingly-contradictory errors (a controller reported "already loaded" immediately
   followed by "doesn't exist") rather than one obvious collision error — always check
   `echo $ROS_DOMAIN_ID` explicitly before debugging `ros2_control` failures further.
7. A vendor xacro macro's `sim_*` params can be threaded through **multiple levels** of include/forward
   chains — check the *entire* chain (`grep` each candidate file) before writing overrides, not just
   the file with the actual `<plugin>` selection, or a fix will fail with `Invalid parameter` at
   whichever level got missed.
8. Easy to write an override file locally and forget to actually `docker cp` it into the running
   container — got bitten by this directly (`gen3_lite_macro.xacro` written but not copied on the
   first attempt). Re-verify with `docker exec ... cat`/`grep` after every copy, don't assume.
9. In `ros2_control` URDF, `<param>` tags nested inside `<command_interface>`/`<state_interface>`
   populate that *interface's* parameter map; `<param>` tags as a direct child of `<joint>` populate
   the *joint's* parameter map — these are different maps read by different code paths.
   `mujoco_ros_control`'s `kp`/`kv` fallback specifically reads the joint-level map. Getting this
   placement wrong doesn't error at compile time (`xacro`/`colcon` both accept it silently) — it only
   surfaces as a runtime error when the hardware plugin actually tries to read the parameter.
10. MuJoCo's URDF mesh loading resolves `<mesh filename="...">` by **basename only, relative to the
    loaded XML file's own directory** — it discards any directory component in the filename attribute
    entirely, absolute path or not (confirmed: a `file://`-stripped *absolute* path still failed,
    looking for the mesh in the URDF's own directory instead). Any URDF loaded directly into MuJoCo
    needs its referenced mesh files physically copied alongside it.
11. GUI "not responding" dialogs on an X11-forwarded OpenGL viewer aren't necessarily a hang — check
    `docker stats`/process CPU% first (a process actively burning CPU, not stuck at 0%, is usually
    just slow under load, not deadlocked) before force-quitting and losing a working session.
    Consistent with the same rendering-load pattern already seen with RViz (gotcha 10 in
    `per_arm_bringup_sequence.md`).

## Not yet done (next steps)

- ~~Wire up an actual motion controller (`joint_trajectory_controller` + a gripper controller...)~~ —
  **done**, along with tuning `kp`/`effort_limit`/joint damping based on measured step-response data.
  Full narrative in `mujoco_motion_controller_tuning.md`; only `joint_2`'s `kp` (50→250) actually
  needed changing, everything else verified fine at its original value.
- ~~Build the actual shared scene (table + arm + objects, floor, lighting)~~ — **done** for the
  single-arm case: `mujoco_scenes/scene_single_arm.xml` + `mujoco_scenes/gen3_lite.xml` (native MJCF,
  converted from the URDF via `mj_saveLastXML`, with a `gen3_lite_base` wrapper `<body>` added for
  repositioning — edit its `pos`/`quat` to move the robot). Not yet done for the dual-arm case (see
  below).
- Add a camera to the scene (for offscreen-rendering-based visual confirmation via `rqt_image_view`,
  as an alternative to the native viewer) — both `scene_single_arm.xml` and `scene_dual_arm.xml`
  already have one `overview` camera each; revisit once useful for perception.
- ~~Commit to the namespace-separated architecture for the dual-arm scene~~ — **done**: flat
  name-prefixed controllers (`armA_`/`armB_`) sharing one `controller_manager`, one `ROS_DOMAIN_ID`
  (domain 4). Both arms load, render, and are independently commandable, correctly positioned 1.10m
  apart facing each other (`OTHER_ARM_BASE_DISTANCE_X`, matching the real-hardware keepout convention).
  Full narrative, including a real `ros2_control` hardware-name-collision bug found and fixed along
  the way, in `mujoco_dual_arm_scene.md`.
- ~~Add MuJoCo + `mujoco_ros_pkgs` **and** the four xacro overrides to `Dockerfile.server`~~ — **done
  and validated**: a full `docker build` from scratch exercising this exact sequence completed
  successfully (exit code 0).
- Add `mujoco_scenes/*` (single-arm **and** dual-arm scene/config files) to `Dockerfile.server` —
  **still deliberately deferred**. Worth revisiting now that both scenes exist and are stable.
- Move the generated artifacts (`/tmp/mujoco_gen3_lite/*`, `/tmp/mujoco_dual_arm/*`) somewhere
  durable/repo-tracked once the launch flow is turned into an actual launch file, rather than
  hand-run commands against `/tmp`.
- Actual coordination/collision-avoidance behavior between the two arms — this milestone only proves
  independent control of both arms in one shared scene; the ICRA deliverable of one arm
  reacting to/avoiding the other (in physics, not just a static keepout box) is still ahead. **Next
  task to start.**
