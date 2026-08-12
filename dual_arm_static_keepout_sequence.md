# Dual-Arm Static Keepout + Per-Arm Controller Sequence

Status as of this writing: **implemented, built, and verified — both pieces of Day 3–5 of the
project plan are done.** Builds directly on `per_arm_bringup_sequence.md` (Day 1–3) — read that
first if you haven't; this document assumes both arms are already up and isolated.

## Context / why this task

Day 3–5's goal, per the project plan: give each arm a static notion of where the *other* arm's
physical body sits (since the domain-isolated architecture means neither arm's MoveIt2 instance has
any live awareness of the other — confirmed in `per_arm_bringup_sequence.md` step 15, each one's
`world`→`base_link` transform is just identity), and get two independent instances of the arm
controller running side by side, one per arm.

## Physical layout used

Both arms are mounted **facing each other across a shared table, base-to-base distance 1.10 m**.
Each arm's own `+X` axis (forward) points toward the other arm's base by construction — that's what
"facing each other" means geometrically — so the same offset is correct in **both** arms' local
`base_link` frames without any per-arm code difference.

## Step-by-step sequence

### 1. Add the static keepout box

New constants in `src/kinova_pick_planner/kinova_pick_planner/kinova_pick_planner.py`, right after
the existing `TABLE_POSITION`:
```python
OTHER_ARM_BASE_DISTANCE_X = 1.10  # meters, base-to-base — physical rig measurement
OTHER_ARM_KEEPOUT = {
    "depth":  0.60,  # placeholder
    "width":  0.60,  # placeholder
    "height": 0.90,  # placeholder
}
```
The box dimensions are a **conservative placeholder** for the Gen3 Lite's body+reach envelope, not
a measured value — flagged in-code for tightening once the physical rig exists.

New method `add_other_arm_keepout()`, added right after the existing `add_table()` (same file,
mirrors its pattern exactly — one `add_collision_box` call, tracked in `_collision_ids`).

### 2. Wire it into all three controller entry points

Added `self.planner.add_other_arm_keepout()` immediately after every existing
`self.planner.add_table()` call, in:
- `src/kinova_pick_planner/kinova_pick_planner/arm_controller.py`
- `src/kinova_pick_planner/kinova_pick_planner/arm_controller_DUMMY.py`
- `src/kinova_pick_planner/kinova_pick_planner/arm_controller_ros.py`

No changes needed to `kinova_pick_planner.py`'s own `standalone_test()` entry point (lower priority,
solo dev-test path).

### 3. Rebuild — hit an unrelated, real build failure

```bash
docker exec auro-laptop bash -c "
source /opt/ros/humble/setup.bash && source /kortex_ws/install/setup.bash
cd /auro-final-project && colcon build --packages-select kinova_pick_planner
"
```
Failed with:
```
TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'
```
**Root cause**: `Dockerfile.server`'s `torch`/`ultralytics` install (after `kortex_ws`'s own build,
so that one was unaffected) silently pulls in a newer `setuptools` whose `_core_metadata.py` calls
`packaging.utils.canonicalize_version(..., strip_trailing_zero=False)` — a keyword the older
`packaging==21.3` pinned by the ROS Humble base image doesn't support. This breaks **every** later
`colcon build` run inside the container, including the live builds `run_arm_llm_tmux.sh` /
`run_perception_control_tmux.sh` do on every startup — not specific to our change.

**Fix, applied twice** (same durability pattern as `domain_bridge` earlier):
```bash
# live, immediate:
docker exec auro-laptop pip install "setuptools<80" "packaging>=23.2" --quiet
# durable, in Dockerfile.server, right after the torch/ultralytics install line:
RUN pip install "setuptools<80" "packaging>=23.2" --quiet
```
Rebuild succeeded after this.

### 4. Clean restart before testing

Both arms' driver/MoveIt2 processes had been running **5 days** (since the Day 1–3 session) — far
beyond the window where the DDS staleness issue was already diagnosed (`per_arm_bringup_sequence.md`
gotcha 7). Rather than risk re-diagnosing the same problem, did a proactive clean restart:
kill all `ros2_control_node`/`robot_state_publisher`/`move_group`/`rviz2`/`static_transform_publisher`
processes by explicit PID (same `pkill -f` pattern-matching gap as before — some processes needed
killing individually), then relaunched `kortex_bringup` (headless) + MoveIt2 (headless) fresh on both
domains, then re-set `goal_time` on both (`ros2 param set /joint_trajectory_controller
constraints.goal_time 30.0`, gotcha 8 — needed after every restart). All clean, no DDS errors.

### 5. Verify the new collision code loads correctly

```bash
ros2 run kinova_pick_planner arm_controller_demo
```
Run once per domain. Both logged, in order:
```
Table added to planning scene
Other-arm keepout zone added to planning scene
Arm status: ready
```

### 6. Visual confirmation

Started a standalone `rviz2` pointed at each domain's already-running `move_group`:
```bash
rviz2 -d /kortex_ws/install/kinova_gen3_lite_moveit_config/share/kinova_gen3_lite_moveit_config/config/moveit.rviz \
  --ros-args -r __node:=rviz2_moveit
```
(No need to relaunch MoveIt2 itself — RViz is a separate, independent process that just subscribes
to what's already running.) Confirmed visually: table, `table_keepout`, and the new
`other_arm_keepout` box all present in the Scene Objects panel, with the keepout box rendered at the
table's far end, in front of the arm, matching the "facing each other, 1.10 m apart" layout.

**Noted from the visual check**: at the current placeholder size, the box blocks roughly the last
~40 cm of the table's 1.8 m length, across its full width — conservative by design, but this
directly affects Day 5–7's planning: the actual handoff/placement zone for the cup+straw task needs
to sit *inside* this boundary, not encroach on it, or the box will need reshaping once real
measurements exist.

### 7. Confirm `arm_controller.py` needs no duplication at all

Ran the **exact same unmodified** `arm_controller_demo` on both domains simultaneously. Verified:
```bash
ros2 node list   # on each domain
```
Both domains show identical node names (`/arm_controller`, `/kinova_pick_planner`) running at the
same time, with zero collision — proven by the same domain-isolation property validated in
`per_arm_bringup_sequence.md`, now confirmed for the application layer, not just the driver layer.

**Conclusion**: the Day 3–5 checklist item "duplicate `arm_controller.py` into two namespaced
instances" needed **zero code changes**. Running the same file twice, once per `ROS_DOMAIN_ID`, *is*
the duplication — a direct payoff of the Day 1 architecture decision.

## Current state after this sequence

- Both arms: driver + MoveIt2 up fresh, `goal_time` set, `table`/`table_keepout`/
  `other_arm_keepout` all present in each arm's own planning scene.
- `arm_controller_demo` running on both domains simultaneously, unmodified, no conflicts.
- Two separate RViz windows open (one per domain) — **not** a combined single-scene view; that would
  require bridging all TF/robot-description/joint-state topics across domains purely for
  visualization, which hasn't been done (see Not yet done).
- `Dockerfile.server` updated with the `setuptools`/`packaging` pin — durable across rebuilds.

## Known gotchas added this pass

1. `torch`/`ultralytics` (Dockerfile.server) silently break `colcon build` workspace-wide via a
   `setuptools`/`packaging` version mismatch — now fixed, but worth knowing the mechanism if a
   similar break appears after adding some other pip dependency later (check
   `python3 -c 'import setuptools, packaging; print(setuptools.__version__, packaging.__version__)'`
   against `setuptools<80`, `packaging>=23.2`).
2. The 5-day DDS staleness recurrence confirms gotcha 7 from `per_arm_bringup_sequence.md` isn't a
   one-off — treat any arm/MoveIt2 process older than a few hours as suspect before trusting it,
   especially before a test session, not just after an observed failure.

## Not yet done (next steps)

- A combined single-RViz-window view showing both arms in one scene (would need a broader,
  visualization-only `domain_bridge` config — separate from, and larger than, the coordination-topic
  bridge planned for Day 5–7).
- Real physical measurement of `OTHER_ARM_BASE_DISTANCE_X` and the `OTHER_ARM_KEEPOUT` envelope once
  the physical rig exists — both are currently placeholders.
- Day 5–7: `dual_arm_coordinator` — publish to each arm's `/selected_object` in sequence, gated on
  the other arm's `/arm_status` via a real (committed, not throwaway) `domain_bridge` config.
