#!/usr/bin/env python3
"""
Kinova Gen3 Lite - Arm Controller (Reliable)
==============================================
Uses /move_action for all motions (same as RViz).
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
    TABLE_POSITION,
    wait_for_future,
)


# =============================================================================
# CONFIGURATION
# =============================================================================

# Home position — from actual robot home pose
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
    "z": 0.08,
}

LIFT_HEIGHT = 0.20
APPROACH_HEIGHT = 0.15
PLACE_RELEASE_HEIGHT = 0.08

GRIPPER_PROFILES = {
    "foam_ball": {"position": 0.5, "effort": 20.0},
    "coffee_cup": {"position": 0.6, "effort": 35.0},
    "water_bottle": {"position": 0.5, "effort": 40.0},
    "small_box": {"position": 0.6, "effort": 35.0},
    "default": {"position": 0.6, "effort": 30.0},
}


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

        # Record home from current position
        self._record_home_position()

        self.state = ControllerState.WAITING_FOR_OBJECTS
        self.get_logger().info("Arm Controller ready!")
        self._publish_status("ready")

    def _record_home_position(self):
        joint_state_received = threading.Event()
        home_joints = {}

        def joint_cb(msg):
            for name, pos in zip(msg.name, msg.position):
                home_joints[name] = pos
            joint_state_received.set()

        sub = self.create_subscription(JointState, "/joint_states", joint_cb, 10)
        joint_state_received.wait(timeout=3.0)
        self.destroy_subscription(sub)

        if home_joints:
            for j in ["joint_1", "joint_2", "joint_3",
                       "joint_4", "joint_5", "joint_6"]:
                if j in home_joints:
                    HOME_JOINT_POSITIONS[j] = home_joints[j]
            self.get_logger().info("Home position recorded")

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def publish_detected_objects(self, objects: list[dict]):
        with self._state_lock:
            if self.state != ControllerState.WAITING_FOR_OBJECTS:
                self.get_logger().warn(
                    f"Cannot publish in state {self.state.value}"
                )
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
                self.get_logger().warn(
                    f"Selection in wrong state: {self.state.value}"
                )
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
    # Pick-Place-Home
    # -------------------------------------------------------------------------

    def _execute_pick_place_sequence(self, selected: dict):
        self.stop_requested = False

        target_x = selected["x"]
        target_y = selected["y"]
        target_z = selected["z"]
        obj_name = selected.get("name", "unknown")
        obj_type = selected.get("type", "default")
        selected_name = selected.get("name", "")

        grip = GRIPPER_PROFILES.get(obj_type, GRIPPER_PROFILES["default"])

        # Add obstacles (skip the target)
        for obj in self.current_objects:
            if obj.get("name") == selected_name:
                continue
            otype = obj.get("type", "small_box")
            if otype not in OBJECT_TYPES:
                otype = "small_box"
            self.planner.add_collision_object(SceneObject(
                object_id=f"obstacle_{obj.get('name', 'obj')}",
                object_type=otype,
                x=obj["x"], y=obj["y"], z=obj["z"],
            ))

        self._publish_status("moving")
        self.get_logger().info(f"Starting pick for '{obj_name}'...")

        success = True

        # Step 1: Open gripper
        if not self._check_stop():
            self.get_logger().info("Step 1: Opening gripper...")
            self.planner.open_gripper()
            time.sleep(0.3)

        # Step 2: Approach
        # if success and not self._check_stop():
        #     self.get_logger().info("Step 2: Approach...")
        #     success = self.planner.move_to_pose(TargetPose(
        #         x=target_x, y=target_y,
        #         z=target_z + APPROACH_HEIGHT,
        #         qx=1.0, qy=0.0, qz=0.0, qw=0.0,
        #     ))

        # Step 2: Move directly to pick pose (planner finds its own path)
        if success and not self._check_stop():
            self.get_logger().info("Step 2: Moving to pick location...")
            success = self.planner.move_to_pose(TargetPose(
                x=target_x, y=target_y, z=target_z,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            ))

        # Step 3: Descend
        # if success and not self._check_stop():
        #     self.get_logger().info("Step 3: Descend...")
        #     success = self.planner.move_to_pose(TargetPose(
        #         x=target_x, y=target_y, z=target_z,
        #         qx=1.0, qy=0.0, qz=0.0, qw=0.0,
        #     ))

        # Step 4: Close gripper
        if success and not self._check_stop():
            self.get_logger().info("Step 4: Closing gripper...")
            self.planner.close_gripper(effort=grip["effort"])
            time.sleep(0.5)

        # Step 5: Lift
        if success and not self._check_stop():
            self.get_logger().info("Step 5: Lifting...")
            success = self.planner.move_to_pose(TargetPose(
                x=target_x, y=target_y,
                z=target_z + LIFT_HEIGHT,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            ))

        # Step 6: Move to place
        if success and not self._check_stop():
            with self._state_lock:
                self.state = ControllerState.PLACING
            self.get_logger().info("Step 6: Moving to place...")
            success = self.planner.move_to_pose(TargetPose(
                x=PLACE_POSITION["x"], y=PLACE_POSITION["y"],
                z=PLACE_POSITION["z"] + PLACE_RELEASE_HEIGHT,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            ))

        # Step 7: Release
        if success and not self._check_stop():
            self.get_logger().info("Step 7: Releasing...")
            self.planner.open_gripper()
            time.sleep(0.5)

        # Step 8: Lift away
        if success and not self._check_stop():
            self.get_logger().info("Step 8: Lifting away...")
            self.planner.move_to_pose(TargetPose(
                x=PLACE_POSITION["x"], y=PLACE_POSITION["y"],
                z=PLACE_POSITION["z"] + LIFT_HEIGHT,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            ))

        # Step 9: Home (joint-space, most reliable)
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

        # Re-publish remaining objects for next cycle
        if success:
            self.get_logger().info(f"Pick-place complete for '{obj_name}'!")
            remaining = [o for o in self.current_objects
                         if o.get("name") != selected_name]
            if remaining:
                time.sleep(1.0)
                self.current_objects = remaining
                with self._state_lock:
                    self.state = ControllerState.WAITING_FOR_OBJECTS
                self.publish_detected_objects(remaining)
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

def demo_with_simulated_camera():
    rclpy.init()
    controller = ArmController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    executor.add_node(controller.planner)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    detected_objects = [
        {
            "name": "red ball",
            "x": 0.40, "y": 0.05, "z": 0.03,
            "type": "foam_ball",
        },
        {
            "name": "coffee cup",
            "x": 0.35, "y": 0.15, "z": 0.06,
            "type": "coffee_cup",
        },
        {
            "name": "water bottle",
            "x": 0.45, "y": -0.10, "z": 0.11,
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
        "name": "red ball",
        "x": 0.40, "y": 0.05, "z": 0.04,
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