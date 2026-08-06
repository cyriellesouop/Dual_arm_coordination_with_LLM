#!/usr/bin/env python3
"""
Kinova Gen3 Lite - Arm Controller
===================================
Bridges LLM voice interface with pick planner.
Uses object-specific grasp strategies and corrected Z coordinates.

Object Z coordinates from camera = height above table surface.
The planner converts these to base_link frame internally.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

import json
import time
import threading
from enum import Enum

from kinova_pick_planner.kinova_pick_planner import (
    KinovaPickPlanner,
    TargetPose,
    SceneObject,
    OBJECT_TYPES,
    GRASP_STRATEGIES,
    TABLE_SURFACE_Z,
    TABLE_POSITION,
    table_z_to_base_z,
    compute_grasp_pose,
    compute_oriented_grasp,
    compute_approach_pose,
    wait_for_future,
)


# =============================================================================
# CONFIGURATION
# =============================================================================

# Home position — from actual robot home pose
HOME_JOINT_POSITIONS = {
    "joint_1": 0.0,
    "joint_2": -0.28,
    "joint_3": 1.31,
    "joint_4": 0.0,
    "joint_5": -1.05,
    "joint_6": 0.0,
}

# Place location — where to drop objects (base_link frame)
PLACE_POSITION = {
    "x": 0.30,
    "y": -0.15,
    "z_above_table": 0.05,  # 5cm above table surface
}

LIFT_HEIGHT = 0.20       # how high above grab point to lift


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

        # Publishers
        self.detected_objects_pub = self.create_publisher(
            String, "/detected_objects", 10
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
            for j in ["joint_1", "joint_2", "joint_3",
                       "joint_4", "joint_5", "joint_6"]:
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

        # obj_name = selected.get("name", "unknown")
        # obj_type = selected.get("type", "default")
        # selected_name = selected.get("name", "")

        # # If LLM didn't include type, look it up from our object list
        # if obj_type == "default":
        #     for obj in self.current_objects:
        #         if obj.get("name") == selected_name:
        #             obj_type = obj.get("type", "default")
        #             selected["type"] = obj_type
        #             break

        obj_name = selected.get("name", "unknown")
        selected_name = selected.get("name", "")
        obj_type = selected.get("type", "default")

        # If LLM didn't include type, look it up from our object list
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
            self.get_logger().warn(f"No type match found for '{selected_name}', using default")

        # Get grasp strategy
        strategy = GRASP_STRATEGIES.get(obj_type, GRASP_STRATEGIES["default"])

        # Compute grasp pose
        grasp_pose = compute_grasp_pose(selected)
        self.get_logger().info(
            f"Grasp pose for '{obj_name}' (type={obj_type}, approach={strategy.approach}): "
            f"({grasp_pose.x:.3f}, {grasp_pose.y:.3f}, {grasp_pose.z:.3f})"
        )

        # Add non-target objects as obstacles
        target_name_clean = selected_name.strip().lower().replace("_", " ")
        for obj in self.current_objects:
            obj_name_clean = obj.get("name", "").strip().lower().replace("_", " ")
            if obj_name_clean == target_name_clean:
                continue
            otype = obj.get("type", "small_box")
            if otype not in OBJECT_TYPES:
                otype = "small_box"
            self.planner.add_collision_object(SceneObject(
                object_id=f"obstacle_{obj.get('name', 'obj')}",
                object_type=otype,
                x=obj["x"], y=obj["y"], z=obj["z"],
            ))

        # Add target as collision mesh for safe approach
        target_collision_id = f"target_{selected_name}"
        target_otype = obj_type if obj_type in OBJECT_TYPES else "small_box"
        self.planner.add_collision_object(SceneObject(
            object_id=target_collision_id,
            object_type=target_otype,
            x=selected["x"], y=selected["y"], z=selected["z"],
        ))

        self._publish_status("moving")
        self.get_logger().info(f"Starting pick for '{obj_name}'...")

        success = True

        # Step 1: Open gripper
        if not self._check_stop():
            self.get_logger().info("Step 1: Opening gripper...")
            self.planner.open_gripper()
            time.sleep(0.3)

        # Step 2: Move to approach pose
        # if success and not self._check_stop():
        #     approach_pose = compute_approach_pose(grasp_pose, selected)
        #     self.get_logger().info(
        #         f"Step 2: Approach ({strategy.approach}) at "
        #         f"({approach_pose.x:.3f}, {approach_pose.y:.3f}, {approach_pose.z:.3f})..."
        #     )
        #     success = self.planner.move_to_pose(approach_pose)

        # Step 2: Move to approach pose (with tighter Z to prevent wild rotation)
        if success and not self._check_stop():

            # approach_pose = compute_approach_pose(grasp_pose, selected)
            # self.get_logger().info(
            #     f"Step 2: Approach ({strategy.approach}) at "
            #     f"({approach_pose.x:.3f}, {approach_pose.y:.3f}, {approach_pose.z:.3f})..."
            # )
            # Compute oriented grasp and approach poses

            approach_pose, grasp_pose = compute_oriented_grasp(selected)
            self.get_logger().info(
                f"Grasp for '{obj_name}' (type={obj_type}, approach={strategy.approach}): "
                f"grasp=({grasp_pose.x:.3f}, {grasp_pose.y:.3f}, {grasp_pose.z:.3f}) "
                f"approach=({approach_pose.x:.3f}, {approach_pose.y:.3f}, {approach_pose.z:.3f})"
            )
            import kinova_pick_planner.kinova_pick_planner as kpp
            old_z_tol = kpp.ORIENT_Z_TOLERANCE
            kpp.ORIENT_Z_TOLERANCE = 0.3  # allow some Z rotation but not full spin
            success = self.planner.move_to_pose(approach_pose)
            kpp.ORIENT_Z_TOLERANCE = old_z_tol

        # Remove target mesh so gripper can reach it
        self.planner.remove_collision_object(target_collision_id)
        time.sleep(0.3)

        # Step 2.5: Partially close gripper to reduce width before final approach
        if success and not self._check_stop():
            strategy_obj = GRASP_STRATEGIES.get(obj_type, GRASP_STRATEGIES["default"])
            if strategy_obj.approach == "side":
                self.get_logger().info("Step 2.5: Narrowing gripper for approach...")
                self.planner.close_gripper(
                    position=strategy_obj.gripper_close_pos * 0.3,  # partially close
                    effort=10.0,  # light effort
                )
                time.sleep(0.3)

        # # Step 3: Move to grasp pose
        # if success and not self._check_stop():
        #     self.get_logger().info("Step 3: Moving to grasp pose...")
        #     import kinova_pick_planner.kinova_pick_planner as kpp
        #     old_z_tol = kpp.ORIENT_Z_TOLERANCE
        #     kpp.ORIENT_Z_TOLERANCE = 0.3
        #     success = self.planner.move_to_pose(grasp_pose)
        #     kpp.ORIENT_Z_TOLERANCE = old_z_tol

        # # Step 3: Move to grasp pose (tight tolerance, no wiggle recovery)
        # if success and not self._check_stop():
        #     self.get_logger().info("Step 3: Moving to grasp pose...")
        #     import kinova_pick_planner.kinova_pick_planner as kpp
        #     old_z_tol = kpp.ORIENT_Z_TOLERANCE
        #     old_retries = kpp.MAX_MOTION_RETRIES
        #     kpp.ORIENT_Z_TOLERANCE = 0.2
        #     kpp.MAX_MOTION_RETRIES = 1  # don't wiggle, just try once
            
        #     grasp_success = self.planner.move_to_pose(grasp_pose)
            
        #     kpp.ORIENT_Z_TOLERANCE = old_z_tol
        #     kpp.MAX_MOTION_RETRIES = old_retries
            
        #     if not grasp_success:
        #         # Failed with tight constraints — retry from approach with looser tolerance
        #         self.get_logger().warn("Tight grasp failed, retrying with looser tolerance...")
        #         kpp.ORIENT_Z_TOLERANCE = 0.5
        #         grasp_success = self.planner.move_to_pose(grasp_pose)
        #         kpp.ORIENT_Z_TOLERANCE = old_z_tol
            
        #     success = grasp_success

        # # Step 4: Close gripper with object-specific settings
        # if success and not self._check_stop():
        #     self.get_logger().info("Step 4: Closing gripper...")
        #     self.planner.close_gripper(
        #         position=strategy.gripper_close_pos,
        #         effort=strategy.gripper_effort,
        #     )
        #     time.sleep(0.5)

        # # Remove table keepout for lift (arm may be close to table after grasping)
        # if "table_keepout" in self.planner.collision_objects:
        #     self.planner.remove_collision_object("table_keepout")
        #     time.sleep(0.3)

        # # Step 5: Lift
        # if success and not self._check_stop():
        #     self.get_logger().info("Step 5: Lifting...")
        #     lift_pose = TargetPose(
        #         x=grasp_pose.x, y=grasp_pose.y,
        #         z=grasp_pose.z + LIFT_HEIGHT,
        #         qx=grasp_pose.qx, qy=grasp_pose.qy,
        #         qz=grasp_pose.qz, qw=grasp_pose.qw,
        #     )
        #     success = self.planner.move_to_pose(lift_pose)

        # Step 3: Move to grasp pose (Cartesian straight line — maintains orientation)
        if success and not self._check_stop():
            self.get_logger().info("Step 3: Cartesian move to grasp pose...")
            success = self.planner.move_cartesian(grasp_pose)

        # Step 4: Close gripper
        if success and not self._check_stop():
            self.get_logger().info("Step 4: Closing gripper...")
            self.planner.close_gripper(
                position=strategy.gripper_close_pos,
                effort=strategy.gripper_effort,
            )
            time.sleep(0.5)

        # Remove table keepout for lift
        if "table_keepout" in self.planner.collision_objects:
            self.planner.remove_collision_object("table_keepout")
            time.sleep(0.3)

        # Step 5: Lift (Cartesian straight line — goes straight up)
        if success and not self._check_stop():
            self.get_logger().info("Step 5: Cartesian lift...")
            lift_pose = TargetPose(
                x=grasp_pose.x, y=grasp_pose.y,
                z=grasp_pose.z + LIFT_HEIGHT,
                qx=grasp_pose.qx, qy=grasp_pose.qy,
                qz=grasp_pose.qz, qw=grasp_pose.qw,
            )
            success = self.planner.move_cartesian(lift_pose)

        # Re-add table keepout for safe travel
        if success and not self._check_stop():
            self.planner.add_collision_object(SceneObject(
                object_id="table_keepout",
                object_type="table_keepout",
                x=TABLE_POSITION["x"], y=TABLE_POSITION["y"],
                z=0.0, padding=0.0,
            ))
            time.sleep(0.3)

        # Step 6: Move to place location
        if success and not self._check_stop():
            with self._state_lock:
                self.state = ControllerState.PLACING
            self.get_logger().info("Step 6: Moving to place...")
            place_z = table_z_to_base_z(PLACE_POSITION["z_above_table"]) + 0.08
            place_pose = TargetPose(
                x=PLACE_POSITION["x"], y=PLACE_POSITION["y"],
                z=place_z,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            )
            success = self.planner.move_to_pose(place_pose)

        # Step 7: Release
        if success and not self._check_stop():
            self.get_logger().info("Step 7: Releasing...")
            self.planner.open_gripper()
            time.sleep(0.5)

        # Step 8: Lift away
        if success and not self._check_stop():
            self.get_logger().info("Step 8: Lifting away...")
            retreat_z = table_z_to_base_z(PLACE_POSITION["z_above_table"]) + LIFT_HEIGHT
            self.planner.move_to_pose(TargetPose(
                x=PLACE_POSITION["x"], y=PLACE_POSITION["y"],
                z=retreat_z,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            ))

        # Step 9: Home
        if not self._check_stop():
            with self._state_lock:
                self.state = ControllerState.RETURNING_HOME
            self.get_logger().info("Step 9: Returning home...")
            self.planner.move_to_joints(HOME_JOINT_POSITIONS)

        # Clean up obstacles
        for obj in self.current_objects:
            if obj.get("name") != selected_name:
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

        # Re-publish remaining objects for next cycle (always, even on failure)
        remaining = [o for o in self.current_objects
                     if o.get("name", "").strip().lower().replace("_", " ") 
                     != selected_name.strip().lower().replace("_", " ")]

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

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    # Object coordinates: x, y relative to base_link; z = height above table
    # z=0 means the object base sits on the table surface
    detected_objects = [
        {
            "name": "red ball",
            "x": 0.40, "y": 0.05, "z": 0.0,
            "type": "foam_ball",
        },
        {
            "name": "coffee cup",
            "x": 0.35, "y": 0.15, "z": 0.00,
            "type": "coffee_cup",
        },
        {
            "name": "water bottle",
            "x": 0.45, "y": -0.10, "z": 0.0,
            "type": "water_bottle",
        },
    ]

    controller.publish_detected_objects(detected_objects)
    controller.get_logger().info(
        "Objects published! Waiting for /selected_object from LLM..."
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
    rclpy.init()
    controller = ArmController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    executor.add_node(controller.planner)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    selected = {
        "name": "green ball",
        "x": 0.40, "y": 0.05, "z": 0.0,
        "type": "foam_ball",
    }

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