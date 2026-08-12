# MuJoCo Motion Controller Wiring + kp/damping Tuning

Follow-on to `mujoco_installation_sequence.md` (which covers install + getting the arm
rendering). This document covers the next milestone: making the MuJoCo-simulated Gen3
Lite arm actually *move* on command, and tuning it to do so without gravity droop or
oscillation. `mujoco_installation_sequence.md` stays the running tracking list (current
state / not-yet-done); this file is the detailed narrative of how this particular chunk
of work was done, kept separate per-milestone rather than growing one file indefinitely.

## Goal

Prior state: the arm loaded and rendered correctly in MuJoCo, but only
`joint_state_broadcaster` was active — nothing could command it to move. The goal here
was the last gate before MuJoCo is useful for the actual project (not just a static
render): given a target joint position, the arm reliably reaches it, without oscillating
or permanently sagging under gravity. This also had to happen *before* committing to the
dual-arm namespace-separated scene, so that whatever tuning pattern is found here can just
be replicated onto the second arm instead of debugging control response and multi-robot
integration at the same time.

## 1. Wired up `joint_trajectory_controller` + gripper controller

Added to a new repo-tracked `mujoco_scenes/plugin_config_single_arm.yaml` (previously this
only existed as a hand-authored file in `/tmp/mujoco_gen3_lite/plugin_config.yaml`, not
saved anywhere durable):

```yaml
/controller_manager:
  ros__parameters:
    update_rate: 500
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    gen3_lite_2f_gripper_controller:
      type: position_controllers/GripperActionController

joint_trajectory_controller:
  ros__parameters:
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    ...

gen3_lite_2f_gripper_controller:
  ros__parameters:
    default: true
    joint: right_finger_bottom_joint
    allow_stalling: true
```

The `joint_trajectory_controller`/gripper blocks are copied verbatim (joint names,
interfaces, gains) from the vendor's own
`ros2_kortex/kortex_description/arms/gen3_lite/6dof/config/ros2_controllers.yaml` — the
same controllers already used for the real/fake-hardware Gen3 Lite stack all session, just
hosted under this MuJoCo-embedded `controller_manager` instead of a separately-launched
one. `twist_controller`/`fault_controller` intentionally omitted (MoveIt-servo specific,
not needed for direct joint-trajectory testing).

Controllers aren't auto-activated just by being declared in this yaml — they still need to
be explicitly spawned:
```bash
ros2 run controller_manager spawner joint_state_broadcaster joint_trajectory_controller gen3_lite_2f_gripper_controller
```

### `update_rate`: 50 → 500 Hz

The original hand-authored config used `update_rate: 50`. Traced through
`mujoco_ros_control.cpp`: `PassiveCallback`/`ControlCallback` (which call
`controller_manager_->read/update/write`) fire on every MuJoCo physics step, but
`ControllerManager::update()` internally throttles itself to `update_rate`. MuJoCo's
default physics timestep is 0.002s (500 Hz, no `<option timestep>` override in
`gen3_lite.xml`/`scene_single_arm.xml`), so at 50 Hz the fallback controller was only
correcting once every 10 physics steps — a 20 ms discretization delay. Changed to 500 Hz to
match the physics step rate.

## 2. Added joint damping — the fallback controller has no D term

Code-reviewed `mujoco_ros_system.cpp`'s actual control law (not assumed from the "PD
fallback" naming used elsewhere in earlier docs — that naming turned out to be
inaccurate):

```cpp
// position command interface fallback, mujoco_ros_system.cpp:
effort = Clamp(joint.kp * (joint.joint_position_cmd - joint.joint_position), joint.effort_limit);
```

Pure-P. `kv` exists in the same source file but is only ever read for a *separate*
velocity-command-interface fallback — never combined with `kp` for position control. So
there is no damping term available from the controller at all; any energy dissipation has
to come from the physics model itself. Added a `damping="X"` attribute to every actuated
`<joint>` in `mujoco_scenes/gen3_lite.xml` (previously all joints had no damping/friction
at all — confirmed by checking the vendor URDF too, zero everywhere):

- `joint_1`/`joint_2`/`joint_3` (shoulder/elbow, heaviest loaded): `damping="2.0"`
- `joint_4`/`joint_5`/`joint_6` (wrist): `damping="0.5"`
- `right_finger_bottom_joint` (gripper): `damping="0.2"`

This measurably worked: every step-response test below shows 0-1 target-band crossings
after the commanded step (no real oscillation) once damping was in place.

## 3. Added `effort_limit` before touching any `kp` value

Before raising any `kp`, checked whether the fallback's torque output is bounded at all.
It wasn't: `effort_limit` is a real, source-supported param
(`joint_info.parameters.find("effort_limit")`, read exactly like `kp`) but nothing had ever
set it, so it defaulted to `numeric_limits<double>::max()` — the fallback's
`kp * error` torque was completely unclamped. Pulled the real per-joint torque limits
straight from the vendor URDF's own `<limit effort="...">` tags (the same ones the actual
hardware driver respects):

| Joint | effort_limit (Nm) | source |
|---|---|---|
| joint_1 | 10 | `gen3_lite_macro.xacro` |
| joint_2 | 14 | `gen3_lite_macro.xacro` |
| joint_3 | 10 | `gen3_lite_macro.xacro` |
| joint_4 | 7 | `gen3_lite_macro.xacro` |
| joint_5 | 7 | `gen3_lite_macro.xacro` |
| joint_6 | 7 | `gen3_lite_macro.xacro` |
| right_finger_bottom_joint | 50 | `gen3_lite_2f_macro.xacro` (0–0.85 range variant, matching our model) |

These exactly match the `actuatorfrcrange` values already present (inertly — MuJoCo only
applies that attribute to `<actuator>` elements, which this model doesn't define) in the
converted `gen3_lite.xml`, which is a good consistency check: both were independently
derived from the same vendor URDF limits.

Added as a `<param name="effort_limit">` direct child of each `<joint>` in
`docker/kortex_overrides/gen3_lite_kortex.ros2_control.xacro`, same joint-level placement
as `kp` (both are read from the same `joint_info.parameters` map — not the
`<command_interface>`-level one used by `min`/`max`).

## 4. Discovered: the simulation starts paused by default

First step-response test (joint_2, target `-0.6` rad) showed **zero movement at all** —
position stayed exactly `0.0` for the entire recording, despite the trajectory goal being
accepted and the controller computing the right commands. Root cause: `mujoco_node`'s
`launch_server.launch.xml` has an `unpause` arg that defaults to `false` — same as any
physics-sim GUI starting with a "paused"/"play" state (the pendulum example's first
screenshot literally showed a "PAUSE" badge, mentioned but not connected to this earlier).
Confirmed via `/clock` being frozen at `sec: 0, nanosec: 0` — sim time wasn't advancing at
all, so nothing could move regardless of what commands were being written.

Fixed by calling the service `mujoco_ros` provides for exactly this:
```bash
ros2 service call /mujoco_server/set_pause mujoco_ros_msgs/srv/SetPause '{paused: false, admin_hash: ""}'
```

**Gotcha for next time**: unpause *before* spawning controllers, not after. In one test
cycle, the sim was unpaused first and controllers were spawned ~15-20s later (tool-call
overhead) — during that gap nothing had claimed the position command interface yet, so
zero effort was applied and the arm free-fell under gravity (opposed only by the new
`damping`), landing joint_2 at `-2.593` rad, right against its `-2.61` joint limit. The
subsequent step test then had to travel +1.99 rad to reach its `-0.6` target from that
fallen start, which showed up as a spurious "332% overshoot" until traced back to the
actual cause. Not a control bug — just a sequencing mistake. Fixed going forward by
spawning controllers first (safe while paused), unpausing last.

## 5. Step-response test methodology

Wrote `step_response_test.py` (rclpy script, not saved in the repo — lives in the
working scratchpad, re-creatable from this description if needed): sends a
`FollowJointTrajectory` goal for one joint via `joint_trajectory_controller`, records
`/joint_states` for that joint throughout, and reports:
- **steady_state_error**: target minus the average of the last 20 samples (post-settle)
- **overshoot**: excursion past target *in the direction of travel*, measured only in the
  settle window (`t > duration`) so it isn't confused with still-approaching-target motion
  — this had to be fixed after the pause-related false positive above, since the original
  version assumed every test starts near position `0`
- **target_band_crossings_after_step**: how many times position crosses a ±0.01 rad band
  around target after the commanded step finishes — 0-1 means well damped, 2+ means
  genuine oscillation

Non-target joints are held at their *current* actual position (read from `/joint_states`
before sending the goal), not forced to `0.0` — otherwise testing one joint would yank
every other joint back to zero as a side effect, including ones already correctly tuned.

The gripper (`gen3_lite_2f_gripper_controller`, a `GripperActionController`) uses a
different action interface (`control_msgs/action/GripperCommand`, not
`FollowJointTrajectory`), tested separately via CLI:
```bash
ros2 action send_goal /gen3_lite_2f_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand '{command: {position: 0.5, max_effort: 50.0}}'
```

## 6. Results

| Joint | kp tested | steady-state error | overshoot | crossings | verdict |
|---|---|---|---|---|---|
| joint_1 | 50 | 0.000 rad | 3.4% | 1 | fine as-is |
| joint_2 | 50 | 0.204 rad (11.7°) | — | 0 | **needed a bump** |
| joint_2 | 250 | 0.035 rad (2°) | — | 0 | tuned value |
| joint_3 | 50 | 0.027 rad (1.5°) | 0% | 0 | fine as-is |
| joint_5 | 30 | 0.010 rad (0.6°) | 2.5% | 0 | fine as-is |
| gripper | 10 | ~0.009 rad (0.5°) | — | not stalled, reached_goal | fine as-is |

**Only `joint_2`'s `kp` was changed (50 → 250)**, in
`docker/kortex_overrides/gen3_lite_kortex.ros2_control.xacro`. `joint_4`/`joint_6` weren't
individually tested but are the same class of light wrist joint as `joint_5` (small mass,
short lever arm) — left unchanged by the same reasoning that held for `joint_5`.

This makes physical sense in hindsight: `joint_2` is the only joint holding up the entire
forearm+wrist+gripper assembly against gravity through a long lever arm. `joint_1` rotates
about a near-vertical axis (negligible gravity torque); `joint_3` carries less mass through
a shorter effective arm; `joint_4-6` carry only the gripper. A single blanket kp multiplier
across all joints (originally the plan) would have been wrong in both directions — it
would have left `joint_2` still sagging while making the already-fine wrist joints
needlessly stiff.

## 7. A real xacro gotcha hit while regenerating the URDF

While regenerating `robot_description.urdf` to pick up the new `kp`/`effort_limit`
values, running `xacro` directly on the **src** copy of
`kortex_description/robots/kinova.urdf.xacro`
(`/kortex_ws/src/ros2_kortex/kortex_description/robots/kinova.urdf.xacro`) silently
produced a URDF with the **wrong** hardware plugin
(`kortex_driver/KortexMultiInterfaceHardware` instead of
`mujoco_ros_control/MujocoRosSystem`), even though `sim_mujoco:=true` was passed and the
override chain was confirmed correct in both `src/` and `install/` copies of all 4
override files.

Root cause: this container's `src/` tree apparently never received the override for this
particular file (only `install/` had been live-patched, from earlier work) — and since
`xacro`'s `<xacro:include filename="$(find kortex_description)/...">` always resolves
through the **installed** share directory regardless of which copy you invoke `xacro` on
directly, invoking on the stale `src/` top-level file used a stale top-level arg
declaration while everything it *included* was already correct, an inconsistent partial
state that failed silently (no error — just a wrong plugin selection). Symptom:
`joint_state_broadcaster` failed to activate with `None of requested interfaces exist` —
the `MujocoRosSystem` hardware component was never imported because plugin loading for the
wrong class name failed early.

**Fix / lesson**: always invoke `xacro` on the **installed** path
(`/kortex_ws/install/kortex_description/share/kortex_description/robots/kinova.urdf.xacro`),
matching what real launch files actually use — never the `src/` copy, even though `src/` is
the "source of truth" for editing. `src/` is for editing; `install/` (after copying an
override there, or a real `colcon build`) is for actually generating anything from.

## Current state after this work

- `joint_trajectory_controller` + `gen3_lite_2f_gripper_controller` wired up and verified
  working — the arm can be commanded to a target joint position and gripper position, not
  just observed.
- Damping added to `mujoco_scenes/gen3_lite.xml` for all actuated joints.
- `effort_limit` added to `docker/kortex_overrides/gen3_lite_kortex.ros2_control.xacro`
  for all actuated joints, matching real vendor torque limits.
- `joint_2`'s `kp` tuned from 50 → 250 based on measured step-response data; every other
  joint verified fine at its original value.
- New repo-tracked `mujoco_scenes/plugin_config_single_arm.yaml` (`update_rate: 500`)
  replaces the previous hand-authored, non-durable `/tmp` version.
- Confirmed launch ordering: spawn controllers *before* calling `/mujoco_server/set_pause`
  with `paused: false`, not after — avoids an uncontrolled free-fall window.
- Not yet added to `Dockerfile.server`: `mujoco_scenes/*` files, deliberately deferred
  (per explicit decision) until after the dual-arm scene work is further along, so it's
  done once instead of twice.
