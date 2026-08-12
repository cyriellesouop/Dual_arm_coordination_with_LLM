# MuJoCo Installation + Validation Sequence

Status as of this writing: **MuJoCo physics engine and `mujoco_ros_pkgs` (the ROS2/`ros2_control`
bridge) installed and built successfully inside `auro-laptop`, and fully validated end-to-end with
the package's own multi-controller pendulum example.** Our own Gen3 Lite robot has not been pointed
at MuJoCo yet — that's the next step, not covered here.

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

## Step-by-step sequence

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

## Current state after this sequence

- MuJoCo 3.3.5 installed at `/root/.mujoco/mujoco-3.3.5/` inside `auro-laptop`, env vars set in
  `/root/.bashrc` (live in the container only — **not yet added to `Dockerfile.server`**, so this
  will be lost if the container is recreated; same durability gap as everything before it gets made
  permanent).
- MuJoCo 3.9.0 also still present at `/root/.mujoco/mujoco-3.9.0/` (never removed — harmless to leave,
  not referenced by any current env var).
- `mujoco_ros_pkgs` (`hybrid-devel` branch) cloned into `/kortex_ws/src/mujoco_ros_pkgs/`, all 7
  packages built successfully against MuJoCo 3.3.5.
- Python `mujoco` package installed via `pip install mujoco` (fixes the binding tests; not otherwise
  required yet).
- The pendulum example fully validated on `ROS_DOMAIN_ID=2` — confirms `mujoco_ros_control`
  genuinely bridges `ros2_control` to real MuJoCo physics.
- **Our own Gen3 Lite robot has not been loaded into MuJoCo yet.**

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

## Not yet done (next steps)

- Point `mujoco_ros_control` at our own Gen3 Lite URDF (`robot_description` from `kortex_description`,
  same one used for MoveIt2 all session) instead of the pendulum example.
- Add `kp`/`kv` fallback parameters to our URDF's `<ros2_control>` joint tags — needed since MuJoCo's
  actuator-name-matching convention (`{joint}_act_pos` etc.) almost certainly isn't already present
  in our existing URDF, and the `kp`/`kv` PD-fallback path is the lower-effort alternative to
  authoring named actuators.
- Build the actual shared scene (table + both arms at the 1.10 m facing-each-other layout + cup/straw
  objects) — this is genuinely new authoring work, not something MuJoCo generates automatically.
- Commit to the **namespace-separated** (not domain-isolated) architecture for the eventual dual-arm
  scene, since `rclcpp::init()` happens once per MuJoCo process — both robots in one shared scene
  will necessarily share one `ROS_DOMAIN_ID`, differentiated by namespace instead. This pendulum test
  used a single robot on its own dedicated domain (2); the real dual-arm scene will need its own
  planning for which domain hosts both namespaced robots together.
- Add MuJoCo + `mujoco_ros_pkgs` installation to `Dockerfile.server` for durability, once the Gen3
  Lite integration is also validated (same "validate before persisting" pattern used for
  `domain_bridge` and the `setuptools`/`packaging` fix).
- Install `libglfw3-dev` if/when the interactive `simulate` GUI viewer is wanted.
