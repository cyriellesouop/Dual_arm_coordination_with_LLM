# MuJoCo Dual-Arm Scene

Follow-on to `mujoco_installation_sequence.md` (install + single arm rendering) and
`mujoco_motion_controller_tuning.md` (motion control + kp/damping tuning, single arm).
This document covers committing to a dual-arm architecture and building the actual
two-robot scene. `mujoco_installation_sequence.md` stays the running tracking list;
this file is the detailed narrative for this milestone.

All existing single-arm files (`gen3_lite.xml`, `scene_single_arm.xml`,
`plugin_config_single_arm.yaml`) were intentionally left untouched — everything below
is new, parallel tooling, per explicit instruction.

## The architecture question

The real dual-arm hardware setup uses domain isolation: Arm A and Arm B each run as a
fully separate process on its own `ROS_DOMAIN_ID` (0 and 1), bridged selectively via
`domain_bridge`. That doesn't carry over to MuJoCo: `mujoco_ros_control` hosts one
`controller_manager` per process, and `rclcpp::init()` happens once per process — so
both robots sharing one physics world (required for them to actually collide-check
against each other, not just respect a static keepout box) necessarily means one
process, one `ROS_DOMAIN_ID`.

Decision made (see conversation, "Sim topic layout" question): **flat name-prefixed
controllers** (`armA_joint_trajectory_controller`, `armB_joint_trajectory_controller`,
...) sharing one `controller_manager`, not wrapped in ROS namespaces. Each arm gets its
own independent controller set (separate action servers) rather than one shared
trajectory controller — a single shared controller would force every goal to specify
both arms' targets together, awkward for two independently-acting agents (this
project's core premise). Flat prefixing was chosen over real ROS namespaces to avoid
relying on unverified `mujoco_ros_control` namespace plugin behavior; the cost is that
sim topic names (`/armA_joint_trajectory_controller/...`) don't look identical to the
real hardware's per-domain topic names (`/joint_trajectory_controller/...`) — agent
code pointing at sim vs. real will need a small config layer either way, so this
doesn't add much extra cost.

Physical layout: reused `OTHER_ARM_BASE_DISTANCE_X = 1.10` from
`kinova_pick_planner.py` (already established for the real dual-arm keepout-box math)
— armA at `x=0`, armB at `x=1.10`, rotated 180° about Z so they face each other.

## New files

- **`mujoco_scenes/kinova_dual_arm.urdf.xacro`** — combines two `load_robot` macro
  calls (the same macro `kinova.urdf.xacro` normally calls once) with different
  `prefix` (`armA_`/`armB_`) and `origin` args, both attached to one shared `world`
  link. Not an override of any single vendor file — a genuinely new top-level file,
  since `kinova.urdf.xacro` itself declares `<link name="world"/>` once and can't be
  `<xacro:include>`d twice without duplicating it. Reuses the `prefix` param already
  threaded through the whole macro chain (added for the real dual-arm work), so no new
  xacro plumbing was needed for joint/link name-collision avoidance — just calling
  existing macros twice with different prefixes. Hardcodes reasonable literal values
  (robot IP, ports, etc.) directly as macro params rather than exposing them as
  `xacro:arg`s like `kinova.urdf.xacro` does, since this file is only ever invoked
  directly for the MuJoCo scene, not included by anything else.
- **`mujoco_scenes/dual_arm.xml`** — native MJCF, converted from the combined URDF via
  `mj_saveLastXML` (same process as the single-arm `gen3_lite.xml`). Both arms'
  positions came out correctly baked in by the conversion (`armB_shoulder_link` at
  `pos="1.1 0 0.12825" quat="0 0 0 1"`), then manually re-wrapped in named
  `armA_base`/`armB_base` bodies (same technique as `gen3_lite_base`) so either arm can
  be repositioned later by editing one `pos`/`quat` instead of re-running the whole
  xacro→URDF→MJCF pipeline.
- **`mujoco_scenes/scene_dual_arm.xml`** — table (unchanged geometry from
  `scene_single_arm.xml`'s `TABLE_POSITION`-derived box — already spans both arm bases
  without modification, x=[-0.575, 1.225]), floor, three lights (wider spread to cover
  both arms), two placeholder objects between the arms, one `overview` camera pulled
  back further than the single-arm version to frame the wider workspace.
- **`mujoco_scenes/plugin_config_dual_arm.yaml`** — 6 controllers total: `armA_`/`armB_`
  × (`joint_state_broadcaster`, `joint_trajectory_controller`,
  `gen3_lite_2f_gripper_controller`), one shared `controller_manager` at 500 Hz (same
  reasoning as the single-arm config).

## A real bug found and fixed: `ros2_control` hardware name collision

First controller-spawn attempt: `armA_joint_trajectory_controller` activated fine,
`armB_joint_trajectory_controller` failed with `Not available` for every
`armB_joint_*/position` command interface — even though both `MujocoRosSystem` hardware
components had been successfully **imported** (confirmed via two separate `initSim`
log entries).

Root cause, traced into `mujoco_ros_control.cpp`: after importing all hardware
components from the URDF, it activates each one with
`resource_manager_->set_component_state(i.name, state)` — keyed by the `<ros2_control
name="...">` attribute. `gen3_lite_macro.xacro`'s `sim_mujoco` branch (added earlier
this project, in `mujoco_installation_sequence.md` Part 2) set that name to the bare
literal `"MujocoRosSystem"`, **not prefixed** — unlike the real-hardware branch a few
lines below it, which correctly used `${prefix}KortexMultiInterfaceHardware`. With one
arm this was invisible (nothing to collide with); with two arms sharing one URDF, both
got the identical `<ros2_control name="MujocoRosSystem">`, and the second
`set_component_state` call (armB's) silently failed to actually activate it — the
component existed in the resource manager's inventory but never left the unconfigured
state, hence "not available."

Fixed in `docker/kortex_overrides/gen3_lite_macro.xacro`: changed
`value="MujocoRosSystem"` to `value="${prefix}MujocoRosSystem"`, matching the pattern
already used one branch down. Copied to both the `src/` and `install/` paths in the
container (the split-state gotcha from the motion-controller-tuning work — `src/` is
for editing, `install/` is what `xacro`'s `$(find kortex_description)` actually
resolves through). This is a durable fix to a shared override file — it'll also apply
automatically the next time `Dockerfile.server` is rebuilt, once `mujoco_scenes/*` is
added there (still deliberately deferred).

## Verification

Domain 4 (fresh — distinct from real arms on 0/1 and the single-arm sim on 3). Spawned
all 6 controllers **before** unpausing (the ordering gotcha learned during single-arm
tuning — avoids an uncontrolled free-fall window before any controller claims the
position interfaces).

Extended `step_response_test.py` into `step_response_test_dual.py` — same step-response
methodology, parameterized by arm prefix, and additionally records the **other** arm's
same joint throughout the test to check for cross-talk.

| Test | Result |
|---|---|
| Command `armA_joint_2` to `-0.6` rad | settled at `-0.652` (0.052 rad error, similar to single-arm tuning), 1 target-band crossing |
| → `armB_joint_2` during that test | max drift `0.014` rad (<1°) — negligible |
| Command `armB_joint_1` to `-0.6` rad | settled at `-0.601` (0.0009 rad error), 0 crossings |
| → `armA_joint_1` during that test | max drift `0.045` rad (~2.6°) — small, likely shared-table/physics coupling, not real cross-control |

Both arms confirmed independently commandable through their own action servers, with
the single-arm-tuned `kp`/`damping`/`effort_limit` values (inherited automatically via
the `${prefix}` macro parameter, no per-arm re-tuning needed) carrying over correctly.

One naming nuance, not a bug: `joint_state_broadcaster` publishes to the **absolute**
topic `/joint_states` regardless of controller name (doesn't use `~/`-relative naming
the way the trajectory controller's action/topic names do) — so both arms' broadcasters
publish to the same topic as separate partial messages (each containing only that arm's
7 joints) rather than one merged 14-joint message. A subscriber that accumulates state
by joint name across messages (as `step_response_test_dual.py`'s
`current_positions` dict does) sees both arms fine; a subscriber expecting one message
with all 14 joints present would not.

## Current state

- Both arms load, render, and are independently commandable in one shared MuJoCo
  physics scene, correctly positioned 1.10m apart facing each other.
- `docker/kortex_overrides/gen3_lite_macro.xacro` has a real bug fix (hardware name
  prefixing) that benefits any future multi-robot MuJoCo work, not just this scene.
- Not yet done: adding `mujoco_scenes/*` to `Dockerfile.server` (deliberately deferred,
  now that the dual-arm scene exists this is a more natural point to revisit); a real
  launch file (everything is still hand-run commands against `/tmp/mujoco_dual_arm/`,
  same as the single-arm setup); any actual coordination/collision-avoidance behavior
  between the two arms (this milestone only proves independent control — the ICRA
  deliverable of one arm reacting to/avoiding the other is still ahead).
