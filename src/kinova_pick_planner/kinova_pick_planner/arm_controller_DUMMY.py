#!/usr/bin/env python3
"""
Kinova Gen3 Lite - Arm Controller (pymoveit2)
===============================================
Bridges LLM voice interface with the pymoveit2-based pick planner.

Motion strategy:
  - Free-space moves (home→approach, lift→place): joint-space planning
  - Precision moves (approach→grasp, grasp→lift): Cartesian straight-line
"""

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
    "z_above_table": 0.05,
}

LIFT_HEIGHT = 0.20


class ControllerState(Enum):
    IDLE = "idle"
    WAITING_FOR_OBJECTS = "waiting_for_objects"
    OBJECTS_PUBLISHED = "objects_published"
    PICKING = "picking"
    PLACING = "placing"
    RETURNING_HOME = "returning_home"
    STOPPED = "stopped"


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

        # Latched QoS — matches the LLM node's /detected_objects subscription
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.detected_objects_pub = self.create_publisher(
            String, "/detected_objects", qos_latched
        )
        self.arm_status_pub = self.create_publisher(
            String, "/arm_status", 10
        )

        # Subscribers
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
    # Public API
    # -------------------------------------------------------------------------

    def publish_detected_objects(self, objects: list[dict]):
        with self._state_lock:
            if self.state != ControllerState.WAITING_FOR_OBJECTS:
                self.get_logger().warn(f"Cannot publish in state {self.state.value}")
                return
            self.current_objects = objects
            self.state = ControllerState.OBJECTS_PUBLISHED

        msg = String()
        msg.data = json.dumps(objects)
        self.detected_objects_pub.publish(msg)
        names = [o.get("name", "?") for o in objects]
        self.get_logger().info(f"Published {len(objects)} objects: {names}")

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

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
        obj_type = selected.get("type", "default")

        # If LLM didn't include type, look it up
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
            self.get_logger().warn(f"No type match for '{selected_name}'")

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
            otype = obj.get("type", "small_box")
            if otype not in OBJECT_TYPES:
                otype = "small_box"
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
        # Side: stay narrowed at fraction=0.3 to slip alongside the object.
        # Top: re-open fully so fingers can wrap around from above.
        if success and not self._check_stop():
            if strategy.approach == "side":
                self.get_logger().info("Step 2.5: Narrowing gripper for side grasp...")
                self.planner.partial_close_gripper(fraction=0.3)
                time.sleep(0.3)
            else:
                self.get_logger().info("Step 2.5: Re-opening gripper for top grasp...")
                self.planner.open_gripper()
                time.sleep(0.3)

        # Step 3: Cartesian move to grasp (straight line, locked orientation)
        if success and not self._check_stop():
            self.get_logger().info("Step 3: Cartesian to grasp pose...")
            success = self.planner.move_cartesian(grasp_pose)

            if not success:
                # Fallback: try free-space if Cartesian fails
                self.get_logger().warn("Cartesian failed, trying free-space...")
                # success = self.planner.move_to_pose(grasp_pose)

        # Step 4: Close gripper
        if success and not self._check_stop():
            self.get_logger().info("Step 4: Closing gripper...")
            self.planner.close_gripper(position=strategy.gripper_close_pos)
            time.sleep(0.5)

        # Remove keepout for lift
        self.planner.remove_keepout()
        time.sleep(0.3)

        # Step 5: Cartesian lift (straight up)
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
                self.get_logger().warn("Cartesian lift failed, trying free-space...")
                success = self.planner.move_to_pose(lift_pose)

        # Restore keepout
        self.planner.restore_keepout()
        time.sleep(0.3)

        # # Step 6: Move to place (free-space)
        # if success and not self._check_stop():
        #     with self._state_lock:
        #         self.state = ControllerState.PLACING
        #     self.get_logger().info("Step 6: Moving to place...")
        #     place_z = table_z_to_base_z(PLACE_POSITION["z_above_table"]) + 0.08
        #     place_pose = TargetPose(
        #         x=PLACE_POSITION["x"], y=PLACE_POSITION["y"],
        #         z=place_z,
        #         qx=1.0, qy=0.0, qz=0.0, qw=0.0,
        #     )
        #     success = self.planner.move_to_pose(place_pose)

        # # Step 7: Release
        # if success and not self._check_stop():
        #     self.get_logger().info("Step 7: Releasing...")
        #     self.planner.open_gripper()
        #     time.sleep(0.5)

        # # Step 8: Lift away (Cartesian)
        # if success and not self._check_stop():
        #     self.get_logger().info("Step 8: Lifting away...")
        #     retreat_z = table_z_to_base_z(PLACE_POSITION["z_above_table"]) + LIFT_HEIGHT
        #     self.planner.move_to_pose(TargetPose(
        #         x=PLACE_POSITION["x"], y=PLACE_POSITION["y"],
        #         z=retreat_z,
        #         qx=1.0, qy=0.0, qz=0.0, qw=0.0,
        #     ))

        # Step 6: Move to pre-place position
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

        # Remove keepout for placing
        self.planner.remove_keepout()
        time.sleep(0.3)

        # Step 7: Cartesian lower to place position
        if success and not self._check_stop():
            self.get_logger().info("Step 7: Cartesian to place pose...")
            success = self.planner.move_cartesian(place_pose)
            if not success:
                self.get_logger().warn("Cartesian place failed, trying free-space...")
                success = self.planner.move_to_pose(place_pose)

        # Step 8: Release
        if success and not self._check_stop():
            self.get_logger().info("Step 8: Releasing...")
            self.planner.open_gripper()
            time.sleep(0.5)

        # Step 8.5: Retreat
        if success and not self._check_stop():
            self.get_logger().info("Step 8.5: Retreating...")
            self.planner.move_cartesian(retreat_pose)

        # Restore keepout
        self.planner.restore_keepout()
        time.sleep(0.3)

        # Step 9: Home (joint-space)
        if not self._check_stop():
            with self._state_lock:
                self.state = ControllerState.RETURNING_HOME
            self.get_logger().info("Step 9: Returning home...")
            self.planner.move_to_joints(HOME_JOINT_POSITIONS)

        # Clean up
        for obj in self.current_objects:
            obj_clean = obj.get("name", "").strip().lower().replace("_", " ")
            if obj_clean != target_clean:
                try:
                    self.planner.remove_collision_object(
                        f"obstacle_{obj.get('name', 'obj')}"
                    )
                except Exception:
                    pass

        # Reset state
        with self._state_lock:
            self.state = ControllerState.WAITING_FOR_OBJECTS
            self.stop_requested = False
            self.selected_object = None

        self._publish_status("ready")

        # Re-publish remaining
        remaining = [o for o in self.current_objects
                     if o.get("name", "").strip().lower().replace("_", " ")
                     != target_clean]

        if success:
            self.get_logger().info(f"Pick-place complete for '{obj_name}'!")
        else:
            self.get_logger().error(f"Pick-place had errors for '{obj_name}'")

        if remaining:
            time.sleep(1.0)
            self.current_objects = remaining
            with self._state_lock:
                self.state = ControllerState.WAITING_FOR_OBJECTS
            self.publish_detected_objects(remaining)

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

def demo_with_simulated_camera():
    rclpy.init()
    controller = ArmController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    executor.add_node(controller.planner)

    # spin_thread = threading.Thread(target=executor.spin, daemon=True)

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

    detected_objects = [
        {"name": "red ball", "x": 0.40, "y": 0.05, "z": 0.0, "type": "foam_ball"},
        {"name": "coffee cup", "x": 0.35, "y": 0.15, "z": 0.0, "type": "coffee_cup"},
        {"name": "water bottle", "x": 0.45, "y": -0.10, "z": 0.0, "type": "water_bottle"},
    ]

    controller.publish_detected_objects(detected_objects)
    controller.get_logger().info("Objects published! Waiting for /selected_object...")

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
        demo_with_simulated_camera()