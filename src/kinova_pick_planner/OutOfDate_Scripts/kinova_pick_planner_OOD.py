#!/usr/bin/env python3
"""
Kinova Gen3 Lite - MoveIt2 Pick Planner (Thread-Safe)
======================================================
ROS2 Jazzy + MoveIt2 script for planning arm motion to target coordinates
while avoiding known obstacles in the planning scene.

This version is thread-safe — all service/action calls use event-based
waiting instead of rclpy.spin_until_future_complete, so it works correctly
when called from background threads (e.g., from the arm controller).

Usage:
    ros2 run kinova_pick_planner pick_planner
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    CollisionObject,
    PlanningScene,
    MotionPlanRequest,
    WorkspaceParameters,
    RobotState,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
from control_msgs.action import GripperCommand
from std_msgs.msg import Header

import math
import time
import threading
from dataclasses import dataclass
from typing import Optional


# =============================================================================
# CONFIGURATION
# =============================================================================

PLANNING_GROUP = "arm"
EE_LINK = "tool_frame"
BASE_FRAME = "base_link"
GRIPPER_TIP_OFFSET_Z = 0.0

PLANNING_TIME_SEC = 10.0
MAX_VELOCITY_SCALING = 0.15
MAX_ACCELERATION_SCALING = 0.15
NUM_PLANNING_ATTEMPTS = 10

GRIPPER_ACTION = "/gen3_lite_2f_gripper_controller/gripper_cmd"
GRIPPER_OPEN_POSITION = 0.0
GRIPPER_CLOSE_POSITION = 0.6
GRIPPER_MAX_EFFORT = 30.0


# =============================================================================
# PREDEFINED OBJECT TYPES
# =============================================================================

@dataclass
class ObjectType:
    name: str
    shape: int
    dimensions: list
    z_offset: float = 0.0

OBJECT_TYPES = {
    "foam_ball": ObjectType(
        name="foam_ball",
        shape=SolidPrimitive.SPHERE,
        dimensions=[0.04],
        z_offset=0.04,
    ),
    "coffee_cup": ObjectType(
        name="coffee_cup",
        shape=SolidPrimitive.CYLINDER,
        dimensions=[0.12, 0.04],
        z_offset=0.06,
    ),
    "water_bottle": ObjectType(
        name="water_bottle",
        shape=SolidPrimitive.CYLINDER,
        dimensions=[0.22, 0.035],
        z_offset=0.11,
    ),
    "small_box": ObjectType(
        name="small_box",
        shape=SolidPrimitive.BOX,
        dimensions=[0.06, 0.06, 0.06],
        z_offset=0.03,
    ),
    "table": ObjectType(
        name="table",
        shape=SolidPrimitive.BOX,
        dimensions=[0.80, 0.60, 0.02],
        z_offset=-0.01,
    ),
}


@dataclass
class SceneObject:
    object_id: str
    object_type: str
    x: float
    y: float
    z: float
    padding: float = 0.02


@dataclass
class TargetPose:
    x: float
    y: float
    z: float
    qx: float = 1.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 0.0
    approach_height: float = 0.10


TABLE_POSITION = {
    "x": 0.40,
    "y": 0.00,
    "z": -0.025,
    "length": 0.80,
    "width": 0.60,
    "thickness": 0.02,
}


# =============================================================================
# THREAD-SAFE FUTURE WAITING
# =============================================================================

def wait_for_future(future, timeout_sec=10.0):
    """
    Wait for a future to complete without calling spin.
    Works from any thread, even when an executor is already spinning.
    """
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    if event.wait(timeout=timeout_sec):
        return future.result()
    return None


# =============================================================================
# MAIN PLANNER NODE
# =============================================================================

class KinovaPickPlanner(Node):
    def __init__(self):
        super().__init__("kinova_pick_planner")
        self.get_logger().info("Initializing Kinova Pick Planner...")

        self.cb_group = ReentrantCallbackGroup()

        # Planning scene publisher (topic-based, no service needed)
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )

        # ApplyPlanningScene service
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
            callback_group=self.cb_group,
        )

        # GetMotionPlan service
        self.plan_client = self.create_client(
            GetMotionPlan,
            "/plan_kinematic_path",
            callback_group=self.cb_group,
        )

        # ExecuteTrajectory action
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory",
            callback_group=self.cb_group,
        )

        # Gripper action client
        self.gripper_client = ActionClient(
            self, GripperCommand, GRIPPER_ACTION,
            callback_group=self.cb_group,
        )

        # Track collision objects
        self.collision_objects: dict[str, CollisionObject] = {}

        # Wait for services
        self._wait_for_services()

        self.get_logger().info("Kinova Pick Planner ready!")

    def _wait_for_services(self):
        self.get_logger().info("Waiting for /plan_kinematic_path service...")
        if not self.plan_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(
                "/plan_kinematic_path not available! Is MoveIt2 running?"
            )
            raise RuntimeError("Planning service not available")

        self.get_logger().info("Waiting for /execute_trajectory action...")
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/execute_trajectory not available!")
            raise RuntimeError("Execute action not available")

        self.get_logger().info("Waiting for ApplyPlanningScene service...")
        if not self.apply_scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                "ApplyPlanningScene not available, using topic instead."
            )
            self._use_scene_service = False
        else:
            self._use_scene_service = True

        self.get_logger().info("Waiting for gripper action server...")
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                "Gripper action not available. Gripper commands will be skipped."
            )
            self._gripper_available = False
        else:
            self._gripper_available = True
            self.get_logger().info("Gripper action server connected!")

    # -------------------------------------------------------------------------
    # Planning Scene Management
    # -------------------------------------------------------------------------

    def add_table(self):
        OBJECT_TYPES["table"].dimensions = [
            TABLE_POSITION["length"],
            TABLE_POSITION["width"],
            TABLE_POSITION["thickness"],
        ]
        table_obj = SceneObject(
            object_id="table",
            object_type="table",
            x=TABLE_POSITION["x"],
            y=TABLE_POSITION["y"],
            z=TABLE_POSITION["z"],
            padding=0.0,
        )
        self.add_collision_object(table_obj)
        self.get_logger().info(
            f"Added table at ({table_obj.x}, {table_obj.y}, {table_obj.z})"
        )

    def add_collision_object(self, scene_obj: SceneObject):
        obj_type = OBJECT_TYPES.get(scene_obj.object_type)
        if obj_type is None:
            self.get_logger().error(
                f"Unknown object type: {scene_obj.object_type}. "
                f"Available: {list(OBJECT_TYPES.keys())}"
            )
            return

        collision_obj = CollisionObject()
        collision_obj.header.frame_id = BASE_FRAME
        collision_obj.header.stamp = self.get_clock().now().to_msg()
        collision_obj.id = scene_obj.object_id

        primitive = SolidPrimitive()
        primitive.type = obj_type.shape
        padded_dims = [d + scene_obj.padding for d in obj_type.dimensions]
        primitive.dimensions = padded_dims

        obj_pose = Pose()
        obj_pose.position.x = scene_obj.x
        obj_pose.position.y = scene_obj.y
        obj_pose.position.z = scene_obj.z + obj_type.z_offset
        obj_pose.orientation.w = 1.0

        collision_obj.primitives.append(primitive)
        collision_obj.primitive_poses.append(obj_pose)
        collision_obj.operation = CollisionObject.ADD

        self._apply_collision_object(collision_obj)
        self.collision_objects[scene_obj.object_id] = collision_obj

        self.get_logger().info(
            f"Added '{scene_obj.object_id}' ({scene_obj.object_type}) "
            f"at ({scene_obj.x:.3f}, {scene_obj.y:.3f}, {scene_obj.z:.3f})"
        )

    def remove_collision_object(self, object_id: str):
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = BASE_FRAME
        collision_obj.header.stamp = self.get_clock().now().to_msg()
        collision_obj.id = object_id
        collision_obj.operation = CollisionObject.REMOVE

        self._apply_collision_object(collision_obj)
        self.collision_objects.pop(object_id, None)
        self.get_logger().info(f"Removed '{object_id}'")

    def clear_all_objects(self):
        for obj_id in list(self.collision_objects.keys()):
            self.remove_collision_object(obj_id)

    def _apply_collision_object(self, collision_obj: CollisionObject):
        """Apply collision object — always use topic for thread safety."""
        scene_msg = PlanningScene()
        scene_msg.world.collision_objects.append(collision_obj)
        scene_msg.is_diff = True

        if self._use_scene_service:
            request = ApplyPlanningScene.Request()
            request.scene = scene_msg
            future = self.apply_scene_client.call_async(request)
            result = wait_for_future(future, timeout_sec=5.0)
            if result is not None:
                if not result.success:
                    self.get_logger().warn("ApplyPlanningScene returned failure")
            else:
                # Fallback to topic
                self.get_logger().warn(
                    "ApplyPlanningScene timed out, using topic fallback"
                )
                self.planning_scene_pub.publish(scene_msg)
                time.sleep(0.5)
        else:
            self.planning_scene_pub.publish(scene_msg)
            time.sleep(0.5)

    # -------------------------------------------------------------------------
    # Gripper Control
    # -------------------------------------------------------------------------

    def open_gripper(self) -> bool:
        return self._send_gripper_command(GRIPPER_OPEN_POSITION, GRIPPER_MAX_EFFORT)

    def close_gripper(self, effort: float = None) -> bool:
        if effort is None:
            effort = GRIPPER_MAX_EFFORT
        return self._send_gripper_command(GRIPPER_CLOSE_POSITION, effort)

    def _send_gripper_command(self, position: float, effort: float) -> bool:
        if not self._gripper_available:
            self.get_logger().warn("Gripper not available, skipping command")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort

        self.get_logger().info(
            f"Gripper command: position={position:.2f}, effort={effort:.1f}"
        )

        # Send goal (thread-safe)
        future = self.gripper_client.send_goal_async(goal)
        goal_handle = wait_for_future(future, timeout_sec=5.0)

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        result = wait_for_future(result_future, timeout_sec=10.0)

        if result is not None:
            self.get_logger().info("Gripper command completed")
            return True
        else:
            self.get_logger().error("Gripper command timed out")
            return False

    # -------------------------------------------------------------------------
    # Motion Planning & Execution
    # -------------------------------------------------------------------------

    def plan_to_pose(self, target: TargetPose):
        """Plan a trajectory to the target pose. Thread-safe."""
        request = GetMotionPlan.Request()
        mp = request.motion_plan_request

        mp.group_name = PLANNING_GROUP
        mp.num_planning_attempts = NUM_PLANNING_ATTEMPTS
        mp.allowed_planning_time = PLANNING_TIME_SEC
        mp.max_velocity_scaling_factor = MAX_VELOCITY_SCALING
        mp.max_acceleration_scaling_factor = MAX_ACCELERATION_SCALING
        mp.pipeline_id = "ompl"

        mp.workspace_parameters.header.frame_id = BASE_FRAME
        mp.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        mp.workspace_parameters.min_corner.x = -1.0
        mp.workspace_parameters.min_corner.y = -1.0
        mp.workspace_parameters.min_corner.z = -1.0
        mp.workspace_parameters.max_corner.x = 1.0
        mp.workspace_parameters.max_corner.y = 1.0
        mp.workspace_parameters.max_corner.z = 1.0

        ee_z = target.z + GRIPPER_TIP_OFFSET_Z

        constraints = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = BASE_FRAME
        pos_constraint.header.stamp = self.get_clock().now().to_msg()
        pos_constraint.link_name = EE_LINK
        pos_constraint.weight = 1.0

        bounding_volume = BoundingVolume()
        tolerance_sphere = SolidPrimitive()
        tolerance_sphere.type = SolidPrimitive.SPHERE
        tolerance_sphere.dimensions = [0.01]

        sphere_pose = Pose()
        sphere_pose.position.x = target.x
        sphere_pose.position.y = target.y
        sphere_pose.position.z = ee_z
        sphere_pose.orientation.w = 1.0

        bounding_volume.primitives.append(tolerance_sphere)
        bounding_volume.primitive_poses.append(sphere_pose)
        pos_constraint.constraint_region = bounding_volume

        constraints.position_constraints.append(pos_constraint)

        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = BASE_FRAME
        orient_constraint.header.stamp = self.get_clock().now().to_msg()
        orient_constraint.link_name = EE_LINK
        orient_constraint.orientation.x = target.qx
        orient_constraint.orientation.y = target.qy
        orient_constraint.orientation.z = target.qz
        orient_constraint.orientation.w = target.qw
        orient_constraint.absolute_x_axis_tolerance = 0.5
        orient_constraint.absolute_y_axis_tolerance = 0.5
        orient_constraint.absolute_z_axis_tolerance = 3.14159
        orient_constraint.weight = 1.0

        constraints.orientation_constraints.append(orient_constraint)

        mp.goal_constraints.append(constraints)

        mp.start_state = RobotState()
        mp.start_state.is_diff = True

        self.get_logger().info(
            f"Planning to: ({target.x:.3f}, {target.y:.3f}, {target.z:.3f}) "
            f"[EE target Z: {ee_z:.3f}]"
        )

        # Thread-safe service call
        future = self.plan_client.call_async(request)
        response = wait_for_future(future, timeout_sec=PLANNING_TIME_SEC + 5.0)

        if response is None:
            self.get_logger().error("Planning service call failed/timed out")
            return None

        error_code = response.motion_plan_response.error_code.val

        if error_code == 1:
            self.get_logger().info("Planning succeeded!")
            return response.motion_plan_response.trajectory
        else:
            error_names = {
                -1: "FAILURE",
                -2: "PLANNING_FAILED",
                -3: "INVALID_MOTION_PLAN",
                -4: "INVALID_GOAL_CONSTRAINTS",
                -5: "INVALID_ROBOT_STATE",
                -6: "INVALID_LINK_NAME",
                -7: "INVALID_OBJECT_NAME",
                -10: "FRAME_TRANSFORM_FAILURE",
                -12: "NO_IK_SOLUTION",
                -31: "TIMED_OUT",
                99999: "FAILURE (masked by Jazzy bug)",
            }
            error_name = error_names.get(error_code, f"UNKNOWN({error_code})")
            self.get_logger().error(f"Planning failed: {error_name}")
            return None

    def execute_trajectory(self, trajectory) -> bool:
        """Execute a planned trajectory. Thread-safe."""
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info("Executing trajectory...")

        # Send goal
        future = self.execute_client.send_goal_async(goal)
        goal_handle = wait_for_future(future, timeout_sec=5.0)

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Execute goal rejected")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        result = wait_for_future(result_future, timeout_sec=60.0)

        if result is None:
            self.get_logger().error("Execution timed out")
            return False

        error_code = result.result.error_code.val
        if error_code == 1:
            self.get_logger().info("Execution succeeded!")
            time.sleep(1.0) #controller settle
            return True
        else:
            self.get_logger().error(f"Execution failed with code: {error_code}")
            return False

    def plan_and_execute(self, target: TargetPose, execute: bool = True) -> bool:
        """Plan and optionally execute motion to target."""
        trajectory = self.plan_to_pose(target)
        if trajectory is None:
            return False

        if not execute:
            self.get_logger().info("Plan-only mode — not executing.")
            return True

        return self.execute_trajectory(trajectory)

    def move_to_approach(self, target: TargetPose, execute: bool = True) -> bool:
        """Two-step approach: move above target, then descend."""
        approach = TargetPose(
            x=target.x,
            y=target.y,
            z=target.z + target.approach_height,
            qx=target.qx, qy=target.qy, qz=target.qz, qw=target.qw,
        )

        self.get_logger().info(
            f"Step 1: Approach ({target.approach_height*100:.0f}cm above)..."
        )
        if not self.plan_and_execute(approach, execute):
            self.get_logger().error("Failed to reach approach pose")
            return False

        time.sleep(1.0)

        self.get_logger().info("Step 2: Descending to target...")
        if not self.plan_and_execute(target, execute):
            self.get_logger().error("Failed to descend to target")
            return False

        self.get_logger().info("Approach + descent complete!")
        return True


# =============================================================================
# HELPER: Build scene from camera data
# =============================================================================

def build_scene_from_camera_data(
    node: KinovaPickPlanner,
    target_coord: tuple[float, float, float],
    obstacle_coords: list[dict],
) -> TargetPose:
    for obj_id in list(node.collision_objects.keys()):
        if obj_id != "table":
            node.remove_collision_object(obj_id)

    if "table" not in node.collision_objects:
        node.add_table()

    for i, obs in enumerate(obstacle_coords):
        obj_id = obs.get("id", f"obstacle_{i}")
        scene_obj = SceneObject(
            object_id=obj_id,
            object_type=obs["type"],
            x=obs["x"],
            y=obs["y"],
            z=obs["z"],
        )
        node.add_collision_object(scene_obj)

    return TargetPose(
        x=target_coord[0],
        y=target_coord[1],
        z=target_coord[2],
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
    )


# =============================================================================
# DEMO SCENARIO (standalone usage)
# =============================================================================

def demo_scenario():
    rclpy.init()
    node = KinovaPickPlanner()

    # Use a MultiThreadedExecutor so callbacks work during waits
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.get_logger().info("=" * 50)
        node.get_logger().info("SETTING UP PLANNING SCENE")
        node.get_logger().info("=" * 50)

        node.add_table()
        time.sleep(0.5)

        target_obj = SceneObject(
            object_id="target_ball",
            object_type="foam_ball",
            x=0.40, y=0.05,
            z=TABLE_POSITION["z"] + TABLE_POSITION["thickness"] / 2,
        )
        node.add_collision_object(target_obj)
        time.sleep(1.0)
        node.remove_collision_object("target_ball")

        node.get_logger().info("Opening gripper...")
        node.open_gripper()
        time.sleep(0.5)

        node.get_logger().info("=" * 50)
        node.get_logger().info("PLANNING MOTION TO TARGET")
        node.get_logger().info("=" * 50)

        target_pose = TargetPose(
            x=target_obj.x,
            y=target_obj.y,
            z=target_obj.z + 0.04,
            qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            approach_height=0.10,
        )

        # *** CHANGE TO True WHEN READY TO MOVE ***
        success = node.move_to_approach(target_pose, execute=False)

        if success:
            node.get_logger().info("Closing gripper...")
            node.close_gripper()
            time.sleep(1.0)

            lift_pose = TargetPose(
                x=target_obj.x,
                y=target_obj.y,
                z=target_obj.z + 0.15,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            )
            node.plan_and_execute(lift_pose, execute=False)
        else:
            node.get_logger().error("Motion planning failed!")

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    demo_scenario()