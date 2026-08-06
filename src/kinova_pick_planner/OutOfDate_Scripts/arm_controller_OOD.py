#!/usr/bin/env python3
"""
Kinova Gen3 Lite - Arm Controller (Thread-Safe)
=================================================
Bridges the LLM voice interface with the pick planner.

Uses MultiThreadedExecutor and event-based future waiting so that
service/action calls work correctly from background threads.

Usage:
    ros2 run kinova_pick_planner arm_controller
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
from dataclasses import dataclass

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

HOME_JOINT_POSITIONS = {
    "joint_1": 0.0,
    "joint_2": -0.2814,
    "joint_3": 1.3161,
    "joint_4": -0.0027,
    "joint_5": -1.0479,
    "joint_6": 0.0,
}

PLACE_POSITION = {
    "x": 0.25,
    "y": -0.05,
    "z": 0.05,
}

LIFT_HEIGHT = 0.25
APPROACH_HEIGHT = 0.15
PLACE_RELEASE_HEIGHT = 0.08

GRIPPER_PROFILES = {
    "foam_ball": {"position": 0.5, "effort": 20.0},
    "coffee_cup": {"position": 0.6, "effort": 35.0},
    "water_bottle": {"position": 0.5, "effort": 40.0},
    "small_box": {"position": 0.6, "effort": 35.0},
    "default": {"position": 0.6, "effort": 30.0},
}


# =============================================================================
# CONTROLLER STATES
# =============================================================================

class ControllerState(Enum):
    IDLE = "idle"
    WAITING_FOR_OBJECTS = "waiting_for_objects"
    OBJECTS_PUBLISHED = "objects_published"
    PICKING = "picking"
    PLACING = "placing"
    RETURNING_HOME = "returning_home"
    STOPPED = "stopped"


# =============================================================================
# ARM CONTROLLER NODE
# =============================================================================

class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller")
        self.get_logger().info("Initializing Arm Controller...")

        self.cb_group = ReentrantCallbackGroup()

        # State
        self.state = ControllerState.IDLE
        self.stop_requested = False
        self.current_objects = []
        self.selected_object = None
        self.planner = None
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

        # Initialize planner
        self.get_logger().info("Initializing pick planner...")
        self.planner = KinovaPickPlanner()
        self.planner.add_table()

        # Record home position
        self._record_home_position()

        self.state = ControllerState.WAITING_FOR_OBJECTS
        self.get_logger().info("Arm Controller ready! State: WAITING_FOR_OBJECTS")
        self._publish_status("ready")

    def _record_home_position(self):
        """Record current joint positions as home."""
        joint_state_received = threading.Event()
        home_joints = {}

        def joint_cb(msg):
            for name, pos in zip(msg.name, msg.position):
                home_joints[name] = pos
            joint_state_received.set()

        sub = self.create_subscription(
            JointState, "/joint_states", joint_cb, 10
        )

        joint_state_received.wait(timeout=3.0)
        self.destroy_subscription(sub)

        if home_joints:
            arm_joints = ["joint_1", "joint_2", "joint_3",
                          "joint_4", "joint_5", "joint_6"]
            for j in arm_joints:
                if j in home_joints:
                    HOME_JOINT_POSITIONS[j] = home_joints[j]
            self.get_logger().info("Home position recorded from current state")
        else:
            self.get_logger().warn("Could not read joints, using default home")

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def publish_detected_objects(self, objects: list[dict]):
        with self._state_lock:
            if self.state != ControllerState.WAITING_FOR_OBJECTS:
                self.get_logger().warn(
                    f"Cannot publish objects in state {self.state.value}"
                )
                return
            self.current_objects = objects
            self.state = ControllerState.OBJECTS_PUBLISHED

        msg = String()
        msg.data = json.dumps(objects)
        self.detected_objects_pub.publish(msg)

        obj_names = [o.get("name", "unknown") for o in objects]
        self.get_logger().info(
            f"Published {len(objects)} detected objects: {obj_names}"
        )

    # -------------------------------------------------------------------------
    # Subscriber callbacks
    # -------------------------------------------------------------------------

    def _on_selected_object(self, msg: String):
        with self._state_lock:
            if self.state != ControllerState.OBJECTS_PUBLISHED:
                self.get_logger().warn(
                    f"Received selection in wrong state: {self.state.value}"
                )
                return
            self.state = ControllerState.PICKING

        try:
            selected = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in /selected_object: {msg.data}")
            with self._state_lock:
                self.state = ControllerState.OBJECTS_PUBLISHED
            return

        self.selected_object = selected
        self.get_logger().info(
            f"LLM selected: '{selected.get('name', 'unknown')}' "
            f"at ({selected.get('x')}, {selected.get('y')}, {selected.get('z')})"
        )

        thread = threading.Thread(
            target=self._execute_pick_place_sequence,
            args=(selected,),
            daemon=True,
        )
        thread.start()

    def _on_arm_stop(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("STOP REQUESTED!")
            self.stop_requested = True
            with self._state_lock:
                self.state = ControllerState.STOPPED

    # -------------------------------------------------------------------------
    # Pick-Place-Home sequence
    # -------------------------------------------------------------------------

    def _execute_pick_place_sequence(self, selected: dict):
        self.stop_requested = False

        target_x = selected["x"]
        target_y = selected["y"]
        target_z = selected["z"]
        obj_name = selected.get("name", "unknown")
        obj_type = selected.get("type", "default")

        grip = GRIPPER_PROFILES.get(obj_type, GRIPPER_PROFILES["default"])

        selected_name = selected.get("name", "")
        
        for obj in self.current_objects:
            if obj.get("name") == selected_name:
                continue
            otype = obj.get("type", "small_box")
            if otype not in OBJECT_TYPES:
                otype = "small_box"
            scene_obj = SceneObject(
                object_id=f"obstacle_{obj.get('name', 'obj')}",
                object_type=otype,
                x=obj["x"],
                y=obj["y"],
                z=obj["z"],
            )
            self.planner.add_collision_object(scene_obj)


        # Add non-target objects as collision obstacles
        # selected_name = selected.get("name", "")
        # selected_x = selected.get("x")
        # selected_y = selected.get("y")

        # for obj in self.current_objects:
        #     # Skip the selected target
        #     if (obj.get("name") == selected_name and
        #             obj.get("x") == selected_x and
        #             obj.get("y") == selected_y):
        #         continue
        #     otype = obj.get("type", "small_box")
        #     if otype not in OBJECT_TYPES:
        #         otype = "small_box"
        #     scene_obj = SceneObject(
        #         object_id=f"obstacle_{obj.get('name', 'obj')}",
        #         object_type=otype,
        #         x=obj["x"],
        #         y=obj["y"],
        #         z=obj["z"],
        #     )
        #     self.planner.add_collision_object(scene_obj)

        # Publish "moving" status
        self._publish_status("moving")
        self.get_logger().info(f"Starting pick sequence for '{obj_name}'...")

        success = True

        # Step 1: Open gripper
        if not self._check_stop():
            self.get_logger().info("Step 1: Opening gripper...")
            self.planner.open_gripper()
            time.sleep(0.3)

        # Step 2: Approach
        if success and not self._check_stop():
            self.get_logger().info("Step 2: Moving to approach pose...")
            approach_pose = TargetPose(
                x=target_x, y=target_y,
                z=target_z + APPROACH_HEIGHT,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            )
            success = self.planner.plan_and_execute(approach_pose, execute=True)

        # Step 3: Descend
        if success and not self._check_stop():
            self.get_logger().info("Step 3: Descending to target...")
            pick_pose = TargetPose(
                x=target_x, y=target_y, z=target_z,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            )
            success = self.planner.plan_and_execute(pick_pose, execute=True)

        # Step 4: Close gripper
        if success and not self._check_stop():
            self.get_logger().info("Step 4: Closing gripper...")
            self.planner.close_gripper(effort=grip["effort"])
            time.sleep(0.5)

        # Step 5: Lift
        if success and not self._check_stop():
            self.get_logger().info("Step 5: Lifting object...")
            lift_pose = TargetPose(
                x=target_x, y=target_y,
                z=target_z + LIFT_HEIGHT,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            )
            success = self.planner.plan_and_execute(lift_pose, execute=True)

        # Step 6: Move to place location
        if success and not self._check_stop():
            with self._state_lock:
                self.state = ControllerState.PLACING
            self.get_logger().info("Step 6: Moving to place location...")
            place_approach = TargetPose(
                x=PLACE_POSITION["x"], y=PLACE_POSITION["y"],
                z=PLACE_POSITION["z"] + PLACE_RELEASE_HEIGHT,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            )
            success = self.planner.plan_and_execute(place_approach, execute=True)

        # Step 7: Release
        if success and not self._check_stop():
            self.get_logger().info("Step 7: Releasing object...")
            self.planner.open_gripper()
            time.sleep(0.5)

        # Step 8: Lift away
        if success and not self._check_stop():
            self.get_logger().info("Step 8: Lifting away...")
            retreat_pose = TargetPose(
                x=PLACE_POSITION["x"], y=PLACE_POSITION["y"],
                z=PLACE_POSITION["z"] + LIFT_HEIGHT,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            )
            self.planner.plan_and_execute(retreat_pose, execute=True)

        # Step 9: Return home
        if not self._check_stop():
            with self._state_lock:
                self.state = ControllerState.RETURNING_HOME
            self.get_logger().info("Step 9: Returning to home position...")
            self._go_home()

        # Clean up obstacles
        for obj in self.current_objects:
            if obj != selected:
                obj_id = f"obstacle_{obj.get('name', 'obj')}"
                try:
                    self.planner.remove_collision_object(obj_id)
                except Exception:
                    pass

        # Reset state
        with self._state_lock:
            self.state = ControllerState.WAITING_FOR_OBJECTS
            self.stop_requested = False
            self.selected_object = None

        self._publish_status("ready")

        # Re-publish remaining objects so LLM can start another conversation
        remaining = [o for o in self.current_objects
                     if not (o.get("name") == selected.get("name") and
                             o.get("x") == selected.get("x") and
                             o.get("y") == selected.get("y"))]
        if remaining:
            time.sleep(1.0)  # brief pause before next round
            self.current_objects = remaining
            with self._state_lock:
                self.state = ControllerState.WAITING_FOR_OBJECTS
            self.publish_detected_objects(remaining)

        if success:
            self.get_logger().info(
                f"Pick-place sequence complete for '{obj_name}'!"
            )
        else:
            self.get_logger().error(
                f"Pick-place sequence had errors for '{obj_name}'"
            )

    def _go_home(self):
        """Plan and execute motion back to home joint positions."""
        from moveit_msgs.msg import Constraints, JointConstraint, RobotState
        from moveit_msgs.srv import GetMotionPlan

        request = GetMotionPlan.Request()
        mp = request.motion_plan_request
        mp.group_name = "arm"
        mp.num_planning_attempts = 10
        mp.allowed_planning_time = 10.0
        mp.max_velocity_scaling_factor = 0.3
        mp.max_acceleration_scaling_factor = 0.3
        mp.pipeline_id = "ompl"
        mp.start_state.is_diff = True

        mp.workspace_parameters.header.frame_id = "base_link"
        mp.workspace_parameters.min_corner.x = -1.0
        mp.workspace_parameters.min_corner.y = -1.0
        mp.workspace_parameters.min_corner.z = -1.0
        mp.workspace_parameters.max_corner.x = 1.0
        mp.workspace_parameters.max_corner.y = 1.0
        mp.workspace_parameters.max_corner.z = 1.0

        constraints = Constraints()
        for jname, jpos in HOME_JOINT_POSITIONS.items():
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = jpos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        mp.goal_constraints.append(constraints)

        # Thread-safe service call
        future = self.planner.plan_client.call_async(request)
        response = wait_for_future(future, timeout_sec=15.0)

        if response is None:
            self.get_logger().error("Home planning service call failed")
            return

        error_code = response.motion_plan_response.error_code.val

        if error_code == 1:
            self.get_logger().info("Home plan succeeded, executing...")
            self.planner.execute_trajectory(
                response.motion_plan_response.trajectory
            )
        else:
            self.get_logger().error(f"Home planning failed with code: {error_code}")

    def _check_stop(self) -> bool:
        if self.stop_requested:
            self.get_logger().warn("Stop flag is set, aborting sequence")
            return True
        return False

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.arm_status_pub.publish(msg)
        self.get_logger().info(f"Arm status: {status}")


# =============================================================================
# DEMO: Simulate camera detection
# =============================================================================

def demo_with_simulated_camera():
    rclpy.init()

    controller = ArmController()

    # Use MultiThreadedExecutor so callbacks + service calls work together
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    executor.add_node(controller.planner)

    # Spin in background so callbacks are processed
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Give executor time to start
    time.sleep(1.0)

    # Simulate camera detecting objects
    detected_objects = [
        {
            "name": "red ball",
            "x": 0.40, "y": 0.05,
            "z": 0.04,
            "type": "foam_ball",
        },
        {
            "name": "coffee cup",
            "x": 0.35, "y": 0.15,
            "z": 0.07,
            "type": "coffee_cup",
        },
        {
            "name": "water bottle",
            "x": 0.45, "y": -0.10,
            "z": 0.11,
            "type": "water_bottle",
        },
    ]

    controller.publish_detected_objects(detected_objects)

    controller.get_logger().info(
        "Objects published! Waiting for /selected_object from LLM...\n"
        "To test manually, run:\n"
        '  ros2 topic pub --once /selected_object std_msgs/String '
        '\'{"data": "{\\"name\\": \\"red ball\\", \\"x\\": 0.40, '
        '\\"y\\": 0.05, \\"z\\": 0.025, \\"type\\": \\"foam_ball\\"}"}\''
    )

    try:
        # Keep main thread alive
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
        "x": 0.40,
        "y": 0.05,
        "z": 0.04,
        "type": "foam_ball",
    }

    controller.current_objects = [selected]
    controller.state = ControllerState.PICKING

    controller.get_logger().info("Running standalone pick-place test...")
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