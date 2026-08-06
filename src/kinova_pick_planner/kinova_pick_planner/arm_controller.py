#!/usr/bin/env python3
"""
Kinova Gen3 Lite - Arm Controller (pymoveit2)
===============================================
Bridges LLM voice interface with the pymoveit2-based pick planner.

Subscribes to /detected_objects from the controller node (latched).
Subscribes to /selected_object from the LLM node.
Publishes /arm_status for the LLM node.

Scene update policy:
  /detected_objects is accepted both while WAITING_FOR_OBJECTS (haven't
  seen anything yet) and while OBJECTS_PUBLISHED (showing the LLM the
  current scene). This lets the world refresh in place if the user moves
  an object between pick and select. We only lock the scene during active
  motion (PICKING, PLACING, RETURNING_HOME).

  After a pick completes we briefly wait for a *fresh* /detected_objects
  message before publishing arm_status="ready". This closes a race where
  the LLM would otherwise fire /selected_object against stale data.

Motion strategy:
  - Free-space moves (home→approach, lift→place): joint-space planning
    (planner.move_to_pose), unchanged.
  - Precision moves (grasp/lift/place/retreat), both top and side grasps:
    single-shot joint-space planning (planner.move_joint) — no Cartesian,
    no waypoint hopping. Two failure modes ruled this out in practice:
      - Cartesian (compute_cartesian_path) repeatedly failed outright,
        especially when the straight-line interpolation had no room to
        maneuver around an awkward starting joint configuration (e.g. lift,
        right after a side grasp).
      - Waypoint hopping (chaining many ~3cm short moves, Cartesian or
        joint-space) caused a stop-plan-execute stutter that tripped the
        arm's hard-stop safety limit — OMPL in particular isn't well suited
        to repeatedly re-planned micro-moves.
    move_joint() is a single continuous MoveIt trajectory per step, still
    collision-checked against the full scene, just without either failure
    mode. Cartesian and waypoint-hop variants (move_cartesian,
    move_cartesian_waypoints, move_joint_waypoints) are kept in
    kinova_pick_planner.py for reference/rollback but aren't called here.
"""
from __future__ import annotations  # required for Python 3.8 (ROS Foxy)

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

import json
import time
import threading
from enum import Enum

from kinova_pick_planner.kinova_pick_planner import (
    KinovaPickPlanner,
    TargetPose,
    OBJECT_TYPES,
    GRASP_STRATEGIES,
    PLACE_STRATEGIES,
    compute_place_pose,
    TABLE_SURFACE_Z,
    TABLE_POSITION,
    ARM_JOINTS,
    table_z_to_base_z,
    compute_grasp_and_approach,
)


# =============================================================================
# CONFIGURATION
# =============================================================================

HOME_JOINT_POSITIONS = {
    "joint_1": 0.0,
    "joint_2": -0.2814,
    "joint_3": 1.3161,
    "joint_4": -0.0027,
    "joint_5": -1.0479,
    "joint_6": 0.0,
}

PLACE_POSITION = {
    "x": 0.30,
    "y": -0.15,
    "z_above_table": 0.01,
}

LIFT_HEIGHT = 0.20

# How long to wait after a pick completes for a fresh /detected_objects
# message before publishing "ready" anyway. Worst case is roughly
# controller_node.debounce_min_sec, so 3s is a comfortable margin.
POST_PICK_REFRESH_TIMEOUT = 3.0

# Fallback map for upstream labels that don't already arrive with a clean
# planner-compatible "type" field. controller_node.py does most of this
# normalization, but we keep this safety net in case:
#   - the upstream label set drifts (new YOLO classes, label renames)
#   - this node runs against a different upstream that doesn't normalize
#   - controller_node's LABEL_TO_OBJECT_TYPE has a gap
_NAME_TO_TYPE = {
    "orange": "orange",
    "apple": "apple",
    "red ball": "foam_ball",
    "ball": "foam_ball",
    "foam_ball": "foam_ball",
    "coffee cup": "coffee_cup",
    "coffee_cup": "coffee_cup",
    "cup": "coffee_cup",
    "mug": "coffee_cup",
    "water bottle": "water_bottle",
    "water_bottle": "water_bottle",
    "bottle": "water_bottle",
    "banana": "small_box",
    "small_box": "small_box",
    "box": "small_box",
}


class ControllerState(Enum):
    IDLE = "idle"
    WAITING_FOR_OBJECTS = "waiting_for_objects"
    OBJECTS_PUBLISHED = "objects_published"
    PICKING = "picking"
    PLACING = "placing"
    RETURNING_HOME = "returning_home"
    STOPPED = "stopped"


# States in which we accept /detected_objects updates. Anything outside this
# set means the arm is actively moving and the scene must be frozen.
_SCENE_UPDATE_STATES = (
    ControllerState.WAITING_FOR_OBJECTS,
    ControllerState.OBJECTS_PUBLISHED,
)


# =============================================================================
# ARM CONTROLLER
# =============================================================================

class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller")
        self.get_logger().info("Initializing Arm Controller...")

        self.cb_group = ReentrantCallbackGroup()

        self.state = ControllerState.IDLE
        self.stop_requested = False
        self.current_objects = []
        self.selected_object = None
        self._state_lock = threading.Lock()

        # Used to block at the end of a pick until a fresh scene arrives,
        # so "ready" isn't published against stale /detected_objects.
        self._fresh_scene_event = threading.Event()
        self._awaiting_fresh_scene = False

        # Publishers
        self.arm_status_pub = self.create_publisher(
            String, "/arm_status", 10
        )

        # QoS to match controller_node's TRANSIENT_LOCAL publisher on
        # /detected_objects, so a late-starting arm_controller still picks
        # up the most recent scene without waiting for a new camera frame.
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.detected_objects_sub = self.create_subscription(
            String, "/detected_objects", self._on_detected_objects,
            qos_latched, callback_group=self.cb_group,
        )
        self.selected_object_sub = self.create_subscription(
            String, "/selected_object", self._on_selected_object,
            10, callback_group=self.cb_group,
        )
        self.arm_stop_sub = self.create_subscription(
            Bool, "/arm_stop", self._on_arm_stop,
            10, callback_group=self.cb_group,
        )

        # Planner
        self.get_logger().info("Initializing pick planner...")
        self.planner = KinovaPickPlanner()
        self.planner.add_table()

        # Record home
        self._record_home_position()
        self.planner._home_joints = HOME_JOINT_POSITIONS.copy()

        self.state = ControllerState.WAITING_FOR_OBJECTS
        self.get_logger().info("Arm Controller ready!")
        self._publish_status("ready")

    def _record_home_position(self):
        joint_event = threading.Event()
        home = {}

        def cb(msg):
            for n, p in zip(msg.name, msg.position):
                home[n] = p
            joint_event.set()

        sub = self.create_subscription(JointState, "/joint_states", cb, 10)
        joint_event.wait(timeout=3.0)
        self.destroy_subscription(sub)

        if home:
            for j in ARM_JOINTS:
                if j in home:
                    HOME_JOINT_POSITIONS[j] = home[j]
            self.get_logger().info("Home position recorded")

    # -------------------------------------------------------------------------
    # Type resolution
    # -------------------------------------------------------------------------

    def _resolve_object_type(self, obj: dict) -> str:
        """
        Decide which planner OBJECT_TYPES key applies to this detection.

        Preference order:
          1. obj["type"] if it's already a valid OBJECT_TYPES key.
          2. _NAME_TO_TYPE lookup on the cleaned name.
          3. Cleaned name itself if it's an OBJECT_TYPES key.
          4. "small_box" fallback with a warning.
        """
        existing = obj.get("type")
        if existing and existing in OBJECT_TYPES:
            return existing

        name = obj.get("name", "")
        clean = name.strip().lower().replace("_", " ")

        if clean in _NAME_TO_TYPE:
            return _NAME_TO_TYPE[clean]

        # also try the underscored version (e.g. "coffee_cup")
        clean_us = name.strip().lower()
        if clean_us in _NAME_TO_TYPE:
            return _NAME_TO_TYPE[clean_us]

        if clean in OBJECT_TYPES:
            return clean
        if clean_us in OBJECT_TYPES:
            return clean_us

        self.get_logger().warn(
            f"No type mapping for '{name}' (incoming type={existing!r}), "
            f"defaulting to small_box"
        )
        return "small_box"

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _on_detected_objects(self, msg: String):
        """Receive detected objects from controller_node.

        Accepted in WAITING_FOR_OBJECTS and OBJECTS_PUBLISHED so the scene
        can refresh in place if the world changes between pick cycles.
        Dropped in PICKING/PLACING/RETURNING_HOME so the active pick uses
        a stable target set.
        """
        with self._state_lock:
            if self.state not in _SCENE_UPDATE_STATES:
                return

        try:
            objects = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Bad JSON from controller_node: {msg.data}")
            return

        if not objects:
            return

        self.current_objects = objects

        with self._state_lock:
            transitioned = False
            if self.state == ControllerState.WAITING_FOR_OBJECTS:
                self.state = ControllerState.OBJECTS_PUBLISHED
                transitioned = True

        names = [o.get("name", "?") for o in objects]
        if transitioned:
            self.get_logger().info(
                f"Received {len(objects)} object(s) from controller_node: {names}"
            )
        else:
            self.get_logger().info(
                f"Scene refreshed in place with {len(objects)} object(s): {names}"
            )

        # If a pick just finished and we were holding back "ready", release.
        if self._awaiting_fresh_scene:
            self._awaiting_fresh_scene = False
            self._fresh_scene_event.set()

    def _on_selected_object(self, msg: String):
        with self._state_lock:
            if self.state != ControllerState.OBJECTS_PUBLISHED:
                self.get_logger().warn(f"Selection in wrong state: {self.state.value}")
                return
            self.state = ControllerState.PICKING

        try:
            selected = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Bad JSON: {msg.data}")
            with self._state_lock:
                self.state = ControllerState.OBJECTS_PUBLISHED
            return

        self.selected_object = selected
        self.get_logger().info(
            f"Selected: '{selected.get('name')}' "
            f"at ({selected.get('x')}, {selected.get('y')}, {selected.get('z')})"
        )

        threading.Thread(
            target=self._execute_pick_place_sequence,
            args=(selected,), daemon=True,
        ).start()

    def _on_arm_stop(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("STOP REQUESTED!")
            self.stop_requested = True
            with self._state_lock:
                self.state = ControllerState.STOPPED

    # -------------------------------------------------------------------------
    # Pick-Place-Home Sequence
    # -------------------------------------------------------------------------

    def _execute_pick_place_sequence(self, selected: dict):
        self.stop_requested = False

        obj_name = selected.get("name", "unknown")
        selected_name = selected.get("name", "")

        # Resolve the planner type, preferring whatever controller_node
        # normalized for us but falling back to _NAME_TO_TYPE / OBJECT_TYPES.
        obj_type = self._resolve_object_type(selected)
        selected["type"] = obj_type  # ensure downstream pose helpers see it

        # Get strategy and compute poses
        strategy = GRASP_STRATEGIES.get(obj_type, GRASP_STRATEGIES["default"])
        approach_pose, grasp_pose, strategy = compute_grasp_and_approach(selected)

        self.get_logger().info(
            f"Pick '{obj_name}' (type={obj_type}, approach={strategy.approach}): "
            f"grasp=({grasp_pose.x:.3f}, {grasp_pose.y:.3f}, {grasp_pose.z:.3f}) "
            f"approach=({approach_pose.x:.3f}, {approach_pose.y:.3f}, {approach_pose.z:.3f})"
        )

        # Add non-target objects as obstacles
        target_clean = selected_name.strip().lower().replace("_", " ")
        for obj in self.current_objects:
            obj_clean = obj.get("name", "").strip().lower().replace("_", " ")
            if obj_clean == target_clean:
                continue
            otype = self._resolve_object_type(obj)
            self.planner.add_collision_object(
                obj_id=f"obstacle_{obj.get('name', 'obj')}",
                obj_type_name=otype,
                x=obj["x"], y=obj["y"],
                z_above_table=obj.get("z", 0.0),
            )

        # Add target as collision for safe approach
        target_collision_id = f"target_{selected_name}"
        target_otype = obj_type if obj_type in OBJECT_TYPES else "small_box"
        self.planner.add_collision_object(
            obj_id=target_collision_id,
            obj_type_name=target_otype,
            x=selected["x"], y=selected["y"],
            z_above_table=selected.get("z", 0.0),
        )

        self._publish_status("moving")
        self.get_logger().info(f"Starting pick for '{obj_name}'...")

        success = True

        # Step 1: Open gripper
        if not self._check_stop():
            self.get_logger().info("Step 1: Opening gripper...")
            self.planner.open_gripper()
            time.sleep(0.3)

        # Step 1.5: Pre-close gripper before the free-space transit swing.
        # Reduces the lateral finger spread (~6 cm when fully open) so the
        # open fingers are less likely to clip nearby objects or the table
        # edge while the joint-space planner swings the arm to approach.
        if not self._check_stop():
            self.get_logger().info("Step 1.5: Pre-closing gripper for transit...")
            self.planner.partial_close_gripper(fraction=0.25)
            time.sleep(0.3)

        # Step 2: Move to approach (free-space planning)
        if success and not self._check_stop():
            self.get_logger().info(
                f"Step 2: Approach ({strategy.approach})..."
            )
            success = self.planner.move_to_pose(approach_pose)

        # Remove target collision so gripper can reach it
        self.planner.remove_collision_object(target_collision_id)
        time.sleep(0.3)

        # Step 2.5: Set gripper to the correct pre-grasp width.
        #   Side: stay narrowed (fraction=0.25) to slip alongside the object.
        #   Top:  re-open fully so fingers can wrap around from above.
        if success and not self._check_stop():
            if strategy.approach == "side":
                self.get_logger().info("Step 2.5: Narrowing gripper for side grasp...")
                self.planner.partial_close_gripper(fraction=0.25)
                time.sleep(0.3)
            else:
                self.get_logger().info("Step 2.5: Re-opening gripper for top grasp...")
                self.planner.open_gripper()
                time.sleep(0.3)

        # TOP_GRASP_MIN_Z_ABOVE_TABLE (0.045) sits below the keepout slab's
        # top (0.05 above table), so any top grasp on a low object needs to
        # descend into keepout territory too, not just side grasps — remove
        # it for both before the gripper ever closes.
        if success and not self._check_stop():
            self.planner.remove_keepout()
            time.sleep(0.3)

        # Step 3: Move to grasp (locked orientation).
        #
        # Top grasps: single-shot joint-space (move_joint) — this has been
        # reliable, keep it as-is.
        #
        # Side grasps: the target's own collision box is already removed
        # (line ~423) so the gripper can reach it, which means a fully free
        # move_joint() plan has nothing stopping it from swinging in from an
        # arbitrary direction instead of reaching straight in along the
        # approach->grasp line — observed in practice sweeping the object
        # aside before the gripper ever closed. Try Cartesian first (exact
        # straight line when it works — this is the behavior we actually
        # want), then fall back to joint-space waypoints (short ~3cm hops,
        # so still hugs the line closely, but each hop is an independent
        # joint-space solve and so isn't blocked by the IK-branch-jump /
        # compute_cartesian_path issues that made plain Cartesian unreliable
        # here). Never fall through to a fully free move_joint() for this
        # step — that's the exact behavior that swept the object aside.
        if success and not self._check_stop():
            if strategy.approach == "side":
                self.get_logger().info("Step 3: Cartesian waypoints to grasp pose (side)...")
                success = self.planner.move_cartesian_waypoints(grasp_pose)
                if not success:
                    self.get_logger().warn(
                        "Cartesian to grasp failed, falling back to "
                        "joint-space waypoints (still line-hugging, short hops)..."
                    )
                    success = self.planner.move_joint_waypoints(grasp_pose)
            else:
                self.get_logger().info("Step 3: Joint-space move to grasp pose...")
                success = self.planner.move_joint(grasp_pose)

            if not success:
                # No fully free single-shot fallback here — see rationale
                # above. Abort the pick instead of risking a sweep through
                # the object on an arbitrary path.
                self.get_logger().error(
                    "Approach to grasp failed; aborting pick rather than "
                    "free-planning into the object."
                )

        # Step 4: Close gripper
        if success and not self._check_stop():
            self.get_logger().info("Step 4: Closing gripper...")
            self.planner.close_gripper(position=strategy.gripper_close_pos)
            time.sleep(0.5)

        # Remove keepout for lift
        self.planner.remove_keepout()
        time.sleep(0.3)

        # Step 5: Lift — single-shot joint-space plan (see Step 3 rationale).
        # Cartesian lift has failed here specifically when starting from an
        # awkward post-grasp joint configuration (straight-line IK has no
        # room to maneuver); joint-space planning can route around that.
        if success and not self._check_stop():
            self.get_logger().info("Step 5: Joint-space lift...")
            lift_pose = TargetPose(
                x=grasp_pose.x, y=grasp_pose.y,
                z=grasp_pose.z + LIFT_HEIGHT,
                qx=grasp_pose.qx, qy=grasp_pose.qy,
                qz=grasp_pose.qz, qw=grasp_pose.qw,
            )
            success = self.planner.move_joint(lift_pose)

            if not success:
                # A single free-space plan here has no guarantee it lifts
                # straight up rather than swinging the (now-grasped) object
                # sideways into the table or keepout — but it's still
                # collision-checked against both, unlike the old unconstrained
                # fallback this replaced.
                self.get_logger().error(
                    "Joint-space lift failed; aborting rather than "
                    "continuing while holding the object."
                )

        # Restore keepout. Side grasps' steeply tilted orientation means links
        # other than the tool_frame control point may still sit low right
        # after lift, so re-adding the slab here risks an in-collision start
        # state for Step 6's free-space plan. Defer the restore for side
        # grasps until the final restore after Step 9 (home) below.
        if strategy.approach != "side":
            self.planner.restore_keepout()
            time.sleep(0.3)

        # Step 6: Move to pre-place position
        if success and not self._check_stop():
            with self._state_lock:
                self.state = ControllerState.PLACING
            self.get_logger().info("Step 6: Moving to pre-place...")
            pre_place, place_pose, _retreat_pose = compute_place_pose(
                selected,
                PLACE_POSITION["x"],
                PLACE_POSITION["y"],
                PLACE_POSITION["z_above_table"],
            )
            success = self.planner.move_to_pose(pre_place)

        # Remove keepout for placing
        self.planner.remove_keepout()
        time.sleep(0.3)

        # Step 7: Lower to place position — single-shot joint-space plan.
        # Keepout was just removed for placing, so the collision-checked
        # joint-space plan still has to route around the table itself (still
        # in the scene); it just isn't held to a straight-line path anymore.
        if success and not self._check_stop():
            self.get_logger().info("Step 7: Joint-space move to place pose...")
            success = self.planner.move_joint(place_pose)
            if not success:
                self.get_logger().error(
                    "Joint-space place failed; aborting rather than "
                    "continuing near the table."
                )

        # Step 8: Release
        if success and not self._check_stop():
            self.get_logger().info("Step 8: Releasing...")
            self.planner.open_gripper()
            time.sleep(0.5)

        # No separate retreat waypoint: Step 9's home move is itself a
        # collision-checked joint-space plan starting from wherever release
        # left the arm, so a dedicated intermediate "go up to this exact
        # point" move was redundant — it was producing a double-motion
        # (wander up-and-over to the retreat coordinates, then a second,
        # separate free plan to home) instead of one direct move home.
        #
        # Keepout restore timing mirrors the lift-step logic above: a top
        # grasp's more compact post-release wrist pose clears keepout
        # immediately, but a side grasp's tilted wrist may still sit low
        # right after release, risking an in-collision start state for
        # Step 9's plan — so side grasps defer the restore until after
        # arriving home.
        if strategy.approach != "side":
            self.planner.restore_keepout()
            time.sleep(0.3)

        # Step 9: Home (joint-space)
        if not self._check_stop():
            with self._state_lock:
                self.state = ControllerState.RETURNING_HOME
            self.get_logger().info("Step 9: Returning home...")
            self.planner.move_to_joints(HOME_JOINT_POSITIONS)

        if strategy.approach == "side":
            self.planner.restore_keepout()
            time.sleep(0.3)

        # Clean up collision objects
        for obj in self.current_objects:
            obj_clean = obj.get("name", "").strip().lower().replace("_", " ")
            if obj_clean != target_clean:
                try:
                    self.planner.remove_collision_object(
                        f"obstacle_{obj.get('name', 'obj')}"
                    )
                except Exception:
                    pass

        # Drop the stale snapshot from before the pick. The next
        # /detected_objects callback will repopulate it.
        self.current_objects = []

        # Arm motion done. Before telling the LLM we're ready, wait briefly
        # for a fresh /detected_objects so the LLM never picks against the
        # stale pre-pick scene. controller_node's debounce republishes within
        # ~1 second, so POST_PICK_REFRESH_TIMEOUT=3s is a comfortable margin.
        self._fresh_scene_event.clear()
        self._awaiting_fresh_scene = True

        with self._state_lock:
            self.state = ControllerState.WAITING_FOR_OBJECTS
            self.stop_requested = False
            self.selected_object = None

        got_fresh = self._fresh_scene_event.wait(timeout=POST_PICK_REFRESH_TIMEOUT)
        self._awaiting_fresh_scene = False

        if got_fresh:
            self.get_logger().info("Fresh scene received post-pick, signaling ready.")
        else:
            self.get_logger().warn(
                f"No fresh /detected_objects within {POST_PICK_REFRESH_TIMEOUT:.1f}s "
                "post-pick; signaling ready anyway (LLM may pick against stale data)."
            )

        self._publish_status("ready")

        if success:
            self.get_logger().info(f"Pick-place complete for '{obj_name}'!")
        else:
            self.get_logger().error(f"Pick-place had errors for '{obj_name}'")

    def _check_stop(self) -> bool:
        if self.stop_requested:
            self.get_logger().warn("Stop flag set, aborting")
            return True
        return False

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.arm_status_pub.publish(msg)
        self.get_logger().info(f"Arm status: {status}")


# =============================================================================
# ENTRY POINTS
# =============================================================================

def main():
    """Main entry point — waits for controller_node to publish /detected_objects."""
    rclpy.init()
    controller = ArmController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    executor.add_node(controller.planner)

    def spin_with_recovery():
        while True:
            try:
                executor.spin()
                break
            except Exception as e:
                controller.get_logger().warn(f"Executor error (recovering): {e}")
                time.sleep(0.5)

    spin_thread = threading.Thread(target=spin_with_recovery, daemon=True)
    spin_thread.start()
    time.sleep(2.0)

    controller.get_logger().info(
        "Arm controller running. Waiting for /detected_objects..."
    )

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down...")
    finally:
        executor.shutdown()
        controller.destroy_node()
        controller.planner.destroy_node()
        rclpy.shutdown()


def standalone_test():
    """Bench test without /detected_objects — fakes one selection and runs."""
    rclpy.init()
    controller = ArmController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    executor.add_node(controller.planner)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(2.0)

    selected = {"name": "red ball", "x": 0.40, "y": 0.05, "z": 0.0, "type": "foam_ball"}

    controller.current_objects = [selected]
    controller.state = ControllerState.PICKING
    controller._execute_pick_place_sequence(selected)

    executor.shutdown()
    controller.destroy_node()
    controller.planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    import sys
    if "--standalone" in sys.argv:
        standalone_test()
    else:
        main()
