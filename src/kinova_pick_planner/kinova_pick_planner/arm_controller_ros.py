#!/usr/bin/env python3
"""
arm_controller_ros.py — Integrated arm controller for the full AURO pipeline.
==============================================================================

Differences from arm_controller.py (standalone / fallback):
  - Subscribes to /detected_objects (published by controller_node) instead of
    requiring objects to be injected via publish_detected_objects().
  - Applies ArUco-frame → arm base_link coordinate transform before planning.
  - main() starts a clean ROS spin loop (no simulated-camera injection).

ArUco frame → base_link transform (measured from robot placement):
  - Arm base is at ArUco position x = -0.36 m, y = +0.05 m
  - Arm +x axis is aligned with ArUco +y axis
  - Arm +y axis is aligned with ArUco +x axis
  - Z is height above table surface in both frames (unchanged)

  arm_x = aruco_y - 0.05
  arm_y = aruco_x + 0.36
"""
from __future__ import annotations  # required for Python 3.8 (ROS Foxy)

import json
import threading
import time
from enum import Enum

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from kinova_pick_planner.kinova_pick_planner import (
    ARM_JOINTS,
    GRASP_STRATEGIES,
    OBJECT_TYPES,
    KinovaPickPlanner,
    TargetPose,
    compute_grasp_and_approach,
    compute_place_pose,
    table_z_to_base_z,
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

# Place location in base_link frame — derived from ArUco measurement (-0.10, 0.20):
#   arm_x = aruco_y - 0.05 = 0.20 - 0.05 = 0.15
#   arm_y = aruco_x + 0.36 = -0.10 + 0.36 = 0.26
PLACE_POSITION = {
    "x": 0.15,
    "y": 0.26,
    "z_above_table": 0.05,
}

LIFT_HEIGHT = 0.20

# ArUco-frame position of the robot base.
_ARUCO_BASE_X = -0.36
_ARUCO_BASE_Y = 0.05


# =============================================================================
# COORDINATE TRANSFORM
# =============================================================================

def aruco_to_base_link(obj: dict) -> dict:
    """Return a copy of obj with x/y transformed from ArUco frame to arm base_link."""
    return {
        **obj,
        "x": obj["y"] - _ARUCO_BASE_Y,   # arm_x = aruco_y - 0.05
        "y": obj["x"] - _ARUCO_BASE_X,   # arm_y = aruco_x + 0.36
        "z": obj["z"],
    }


# =============================================================================
# STATE
# =============================================================================

class ControllerState(Enum):
    WAITING_FOR_OBJECTS = "waiting_for_objects"
    OBJECTS_PUBLISHED   = "objects_published"
    PICKING             = "picking"
    PLACING             = "placing"
    RETURNING_HOME      = "returning_home"
    STOPPED             = "stopped"


# =============================================================================
# ARM CONTROLLER NODE
# =============================================================================

class ArmControllerROS(Node):
    def __init__(self):
        super().__init__("arm_controller")
        self.get_logger().info("Initializing Arm Controller (ROS pipeline mode)...")

        self.cb_group = ReentrantCallbackGroup()

        self.state = ControllerState.WAITING_FOR_OBJECTS
        self.stop_requested = False
        self.current_objects: list = []   # stored in base_link frame
        self.selected_object = None
        self._state_lock = threading.Lock()

        # Publishers
        self._status_pub = self.create_publisher(String, "/arm_status", 10)

        # Latched QoS must match controller_node's /detected_objects publisher.
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.create_subscription(
            String, "/detected_objects",
            self._on_detected_objects,
            qos_latched,
        )
        self.create_subscription(
            String, "/selected_object",
            self._on_selected_object,
            10, callback_group=self.cb_group,
        )
        self.create_subscription(
            Bool, "/arm_stop",
            self._on_arm_stop,
            10, callback_group=self.cb_group,
        )

        self.get_logger().info("Connecting to MoveIt2 (must be running externally)...")
        self.planner = KinovaPickPlanner()
        self.planner.add_table()

        self._record_home_position()
        self.planner._home_joints = HOME_JOINT_POSITIONS.copy()

        self._publish_status("ready")
        self.get_logger().info("Arm Controller ready — waiting for /detected_objects.")

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
            self.get_logger().info("Home position recorded from /joint_states")

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _on_detected_objects(self, msg: String):
        """Receive filtered object list from controller_node (ArUco frame coords)."""
        with self._state_lock:
            if self.state != ControllerState.WAITING_FOR_OBJECTS:
                return  # mid-pick; ignore until sequence completes

        try:
            objects = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Bad JSON on /detected_objects: {exc}")
            return

        if not isinstance(objects, list) or not objects:
            return

        # Transform ArUco → base_link so all downstream code uses arm coords.
        transformed = [aruco_to_base_link(o) for o in objects]

        with self._state_lock:
            self.current_objects = transformed
            self.state = ControllerState.OBJECTS_PUBLISHED

        names = [o.get("name", "?") for o in transformed]
        self.get_logger().info(
            f"Objects received ({len(transformed)}): {names} — waiting for /selected_object"
        )

    def _on_selected_object(self, msg: String):
        with self._state_lock:
            if self.state != ControllerState.OBJECTS_PUBLISHED:
                self.get_logger().warn(
                    f"Selection received in state {self.state.value} — ignoring."
                )
                return
            self.state = ControllerState.PICKING

        try:
            selected_aruco = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Bad JSON in /selected_object: {msg.data}")
            with self._state_lock:
                self.state = ControllerState.OBJECTS_PUBLISHED
            return

        # LLM round-trips the ArUco coords it received — transform here too.
        selected = aruco_to_base_link(selected_aruco)

        self.get_logger().info(
            f"Selected '{selected.get('name')}' — "
            f"ArUco ({selected_aruco.get('x')}, {selected_aruco.get('y')}) → "
            f"base_link ({selected['x']:.3f}, {selected['y']:.3f}, {selected['z']:.3f})"
        )

        threading.Thread(
            target=self._execute_pick_place_sequence,
            args=(selected,),
            daemon=True,
        ).start()

    def _on_arm_stop(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("EMERGENCY STOP received!")
            self.stop_requested = True
            with self._state_lock:
                self.state = ControllerState.STOPPED

    # -------------------------------------------------------------------------
    # Pick-Place-Home Sequence
    # -------------------------------------------------------------------------

    def _execute_pick_place_sequence(self, selected: dict):
        """Execute full pick-place-home sequence. selected is in base_link frame."""
        self.stop_requested = False

        obj_name = selected.get("name", "unknown")
        selected_name = selected.get("name", "")
        obj_type = selected.get("type", "default")

        # If LLM didn't include type, look it up from current_objects (base_link frame)
        if obj_type == "default":
            selected_clean = selected_name.strip().lower().replace("_", " ")
            for obj in self.current_objects:
                obj_clean = obj.get("name", "").strip().lower().replace("_", " ")
                if obj_clean == selected_clean:
                    obj_type = obj.get("type", "default")
                    selected["type"] = obj_type
                    self.get_logger().info(f"Matched type '{obj_type}' for '{selected_name}'")
                    break

        if obj_type == "default":
            self.get_logger().warn(f"No type match for '{selected_name}', using default grasp")

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
            otype = obj.get("type", "small_box")
            if otype not in OBJECT_TYPES:
                otype = "small_box"
            self.planner.add_collision_object(
                obj_id=f"obstacle_{obj.get('name', 'obj')}",
                obj_type_name=otype,
                x=obj["x"], y=obj["y"],
                z_above_table=obj.get("z", 0.0),
            )

        # Add target as collision for safe approach planning
        target_collision_id = f"target_{selected_name}"
        target_otype = obj_type if obj_type in OBJECT_TYPES else "small_box"
        self.planner.add_collision_object(
            obj_id=target_collision_id,
            obj_type_name=target_otype,
            x=selected["x"], y=selected["y"],
            z_above_table=selected.get("z", 0.0),
        )

        self._publish_status("moving")
        success = True

        # Step 1: Open gripper
        if not self._check_stop():
            self.get_logger().info("Step 1: Opening gripper...")
            self.planner.open_gripper()
            time.sleep(0.3)

        # Step 2: Free-space move to approach position
        if success and not self._check_stop():
            self.get_logger().info(f"Step 2: Approach ({strategy.approach})...")
            success = self.planner.move_to_pose(approach_pose)

        # Remove target collision so gripper can enter the object's space
        self.planner.remove_collision_object(target_collision_id)
        time.sleep(0.3)

        # Step 2.5: Narrow gripper for side approach
        if success and not self._check_stop() and strategy.approach == "side":
            self.get_logger().info("Step 2.5: Narrowing gripper for side approach...")
            self.planner.partial_close_gripper(fraction=0.3)
            time.sleep(0.3)

        # Step 3: Cartesian straight-line move to grasp
        if success and not self._check_stop():
            self.get_logger().info("Step 3: Cartesian to grasp pose...")
            success = self.planner.move_cartesian(grasp_pose)
            if not success:
                self.get_logger().warn("Cartesian failed — falling back to free-space...")
                success = self.planner.move_to_pose(grasp_pose)

        # Step 4: Close gripper
        if success and not self._check_stop():
            self.get_logger().info("Step 4: Closing gripper...")
            self.planner.close_gripper(position=strategy.gripper_close_pos)
            time.sleep(0.5)

        # Remove keepout so lift doesn't collide with the zone above table surface
        self.planner.remove_keepout()
        time.sleep(0.3)

        # Step 5: Cartesian lift straight up
        if success and not self._check_stop():
            self.get_logger().info("Step 5: Cartesian lift...")
            lift_pose = TargetPose(
                x=grasp_pose.x, y=grasp_pose.y,
                z=grasp_pose.z + LIFT_HEIGHT,
                qx=grasp_pose.qx, qy=grasp_pose.qy,
                qz=grasp_pose.qz, qw=grasp_pose.qw,
            )
            success = self.planner.move_cartesian(lift_pose)
            if not success:
                self.get_logger().warn("Cartesian lift failed — falling back to free-space...")
                success = self.planner.move_to_pose(lift_pose)

        # Restore keepout now that we are above the table
        self.planner.restore_keepout()
        time.sleep(0.3)

        # Step 6: Free-space move to pre-place position
        if success and not self._check_stop():
            with self._state_lock:
                self.state = ControllerState.PLACING
            self.get_logger().info("Step 6: Moving to pre-place...")
            pre_place, place_pose, retreat_pose = compute_place_pose(
                selected,
                PLACE_POSITION["x"],
                PLACE_POSITION["y"],
                PLACE_POSITION["z_above_table"],
            )
            success = self.planner.move_to_pose(pre_place)

        # Remove keepout for the descending place move
        self.planner.remove_keepout()
        time.sleep(0.3)

        # Step 7: Cartesian lower to place position
        if success and not self._check_stop():
            self.get_logger().info("Step 7: Cartesian to place pose...")
            success = self.planner.move_cartesian(place_pose)
            if not success:
                self.get_logger().warn("Cartesian place failed — falling back to free-space...")
                success = self.planner.move_to_pose(place_pose)

        # Step 8: Release
        if success and not self._check_stop():
            self.get_logger().info("Step 8: Releasing...")
            self.planner.open_gripper()
            time.sleep(0.5)

        # Step 8.5: Cartesian retreat
        if success and not self._check_stop():
            self.get_logger().info("Step 8.5: Retreating...")
            self.planner.move_cartesian(retreat_pose)

        # Restore keepout after place
        self.planner.restore_keepout()
        time.sleep(0.3)

        # Step 9: Return home (joint-space)
        if not self._check_stop():
            with self._state_lock:
                self.state = ControllerState.RETURNING_HOME
            self.get_logger().info("Step 9: Returning home...")
            self.planner.move_to_joints(HOME_JOINT_POSITIONS)

        # Clean up obstacle collision objects
        for obj in self.current_objects:
            obj_clean = obj.get("name", "").strip().lower().replace("_", " ")
            if obj_clean != target_clean:
                try:
                    self.planner.remove_collision_object(f"obstacle_{obj.get('name', 'obj')}")
                except Exception:
                    pass

        # Reset — controller_node will re-detect and republish the updated scene
        with self._state_lock:
            self.state = ControllerState.WAITING_FOR_OBJECTS
            self.stop_requested = False
            self.selected_object = None
            self.current_objects = []

        self._publish_status("ready")

        if success:
            self.get_logger().info(f"Pick-place complete for '{obj_name}'!")
        else:
            self.get_logger().error(f"Pick-place had errors for '{obj_name}'")

    def _check_stop(self) -> bool:
        if self.stop_requested:
            self.get_logger().warn("Stop flag set — aborting sequence")
            return True
        return False

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f"Arm status: {status}")


# =============================================================================
# ENTRY POINT
# =============================================================================

def main(args=None):
    rclpy.init(args=args)

    controller = ArmControllerROS()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    executor.add_node(controller.planner)

    def _spin():
        while rclpy.ok():
            try:
                executor.spin()
                break
            except Exception as exc:
                controller.get_logger().warn(f"Executor error (recovering): {exc}")
                time.sleep(0.5)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    try:
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        controller.destroy_node()
        controller.planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
