#!/usr/bin/env python3
"""
Kinova Gen3 Lite - MoveIt2 Pick Planner (Reliable)
====================================================
Uses /move_action (MoveGroup action) with plan+execute and replan,
matching how RViz operates for maximum reliability.

Thread-safe — all calls use event-based waiting.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    CollisionObject,
    PlanningScene,
    RobotState,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    BoundingVolume,
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene
from control_msgs.action import GripperCommand

import time
import threading
from dataclasses import dataclass


# =============================================================================
# CONFIGURATION
# =============================================================================

PLANNING_GROUP = "arm"
EE_LINK = "tool_frame"
BASE_FRAME = "base_link"
GRIPPER_TIP_OFFSET_Z = 0.0

PLANNING_TIME_SEC = 10.0
MAX_VELOCITY_SCALING = 0.3
MAX_ACCELERATION_SCALING = 0.3
NUM_PLANNING_ATTEMPTS = 10

GRIPPER_ACTION = "/gen3_lite_2f_gripper_controller/gripper_cmd"
GRIPPER_OPEN_POSITION = 0.0
GRIPPER_CLOSE_POSITION = 0.6
GRIPPER_MAX_EFFORT = 30.0

# Orientation tolerance (radians)
# Tighter = gripper more precisely downward, but harder to plan
# Looser = easier to plan, gripper may approach at slight angle
ORIENT_XY_TOLERANCE = 0.4   # ~23 degrees from vertical
ORIENT_Z_TOLERANCE = 3.14159  # free rotation around Z

# Position tolerance (meters)
POSITION_TOLERANCE = 0.015  # 1.5cm sphere


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
    "z": -0.05,
    "length": 0.80,
    "width": 0.60,
    "thickness": 0.02,
}


# =============================================================================
# THREAD-SAFE FUTURE WAITING
# =============================================================================

def wait_for_future(future, timeout_sec=10.0):
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

        # Planning scene
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
            callback_group=self.cb_group,
        )

        # MoveGroup action — same interface RViz uses
        self.move_group_client = ActionClient(
            self, MoveGroup, "/move_action",
            callback_group=self.cb_group,
        )

        # Gripper
        self.gripper_client = ActionClient(
            self, GripperCommand, GRIPPER_ACTION,
            callback_group=self.cb_group,
        )

        self.collision_objects: dict[str, CollisionObject] = {}
        self._wait_for_services()
        self.get_logger().info("Kinova Pick Planner ready!")

    def _wait_for_services(self):
        self.get_logger().info("Waiting for /move_action...")
        if not self.move_group_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("/move_action not available!")
            raise RuntimeError("MoveGroup not available")

        self.get_logger().info("Waiting for ApplyPlanningScene...")
        if not self.apply_scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("ApplyPlanningScene not available, using topic.")
            self._use_scene_service = False
        else:
            self._use_scene_service = True

        self.get_logger().info("Waiting for gripper...")
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Gripper not available.")
            self._gripper_available = False
        else:
            self._gripper_available = True
            self.get_logger().info("Gripper connected!")

    # -------------------------------------------------------------------------
    # Planning Scene
    # -------------------------------------------------------------------------

    def add_table(self):
        OBJECT_TYPES["table"].dimensions = [
            TABLE_POSITION["length"],
            TABLE_POSITION["width"],
            TABLE_POSITION["thickness"],
        ]
        self.add_collision_object(SceneObject(
            object_id="table", object_type="table",
            x=TABLE_POSITION["x"], y=TABLE_POSITION["y"],
            z=TABLE_POSITION["z"], padding=0.0,
        ))
        self.get_logger().info("Table added to planning scene")

    def add_collision_object(self, scene_obj: SceneObject):
        obj_type = OBJECT_TYPES.get(scene_obj.object_type)
        if obj_type is None:
            self.get_logger().error(f"Unknown type: {scene_obj.object_type}")
            return

        co = CollisionObject()
        co.header.frame_id = BASE_FRAME
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = scene_obj.object_id

        prim = SolidPrimitive()
        prim.type = obj_type.shape
        prim.dimensions = [d + scene_obj.padding for d in obj_type.dimensions]

        pose = Pose()
        pose.position.x = scene_obj.x
        pose.position.y = scene_obj.y
        pose.position.z = scene_obj.z + obj_type.z_offset
        pose.orientation.w = 1.0

        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        self._apply_collision_object(co)
        self.collision_objects[scene_obj.object_id] = co
        self.get_logger().info(
            f"Added '{scene_obj.object_id}' at "
            f"({scene_obj.x:.3f}, {scene_obj.y:.3f}, {scene_obj.z:.3f})"
        )

    def remove_collision_object(self, object_id: str):
        co = CollisionObject()
        co.header.frame_id = BASE_FRAME
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = object_id
        co.operation = CollisionObject.REMOVE
        self._apply_collision_object(co)
        self.collision_objects.pop(object_id, None)

    def clear_all_objects(self):
        for oid in list(self.collision_objects.keys()):
            self.remove_collision_object(oid)

    def _apply_collision_object(self, co: CollisionObject):
        scene_msg = PlanningScene()
        scene_msg.world.collision_objects.append(co)
        scene_msg.is_diff = True

        if self._use_scene_service:
            req = ApplyPlanningScene.Request()
            req.scene = scene_msg
            future = self.apply_scene_client.call_async(req)
            result = wait_for_future(future, timeout_sec=5.0)
            if result is None or not result.success:
                self.planning_scene_pub.publish(scene_msg)
                time.sleep(0.3)
        else:
            self.planning_scene_pub.publish(scene_msg)
            time.sleep(0.3)

    # -------------------------------------------------------------------------
    # Gripper
    # -------------------------------------------------------------------------

    def open_gripper(self) -> bool:
        return self._send_gripper_command(GRIPPER_OPEN_POSITION, GRIPPER_MAX_EFFORT)

    def close_gripper(self, effort: float = None) -> bool:
        return self._send_gripper_command(
            GRIPPER_CLOSE_POSITION, effort or GRIPPER_MAX_EFFORT
        )

    def _send_gripper_command(self, position: float, effort: float) -> bool:
        if not self._gripper_available:
            self.get_logger().warn("Gripper not available")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort

        self.get_logger().info(f"Gripper: pos={position:.2f}, effort={effort:.1f}")

        future = self.gripper_client.send_goal_async(goal)
        handle = wait_for_future(future, timeout_sec=5.0)
        if handle is None or not handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        result = wait_for_future(handle.get_result_async(), timeout_sec=10.0)
        if result is not None:
            self.get_logger().info("Gripper done")
            return True
        self.get_logger().error("Gripper timed out")
        return False

    # -------------------------------------------------------------------------
    # Motion: Pose Goal (via /move_action, like RViz)
    # -------------------------------------------------------------------------

    def move_to_pose(self, target: TargetPose, plan_only: bool = False) -> bool:
        """
        Plan and execute to a Cartesian pose using /move_action.
        Matches RViz behavior: plan+execute together, with replanning.
        """
        goal = MoveGroup.Goal()
        mp = goal.request

        mp.group_name = PLANNING_GROUP
        mp.num_planning_attempts = NUM_PLANNING_ATTEMPTS
        mp.allowed_planning_time = PLANNING_TIME_SEC
        mp.max_velocity_scaling_factor = MAX_VELOCITY_SCALING
        mp.max_acceleration_scaling_factor = MAX_ACCELERATION_SCALING
        mp.pipeline_id = "ompl"
        mp.planner_id = ""  # let OMPL choose (same as RViz default)

        mp.start_state.is_diff = True

        # Workspace
        mp.workspace_parameters.header.frame_id = BASE_FRAME
        mp.workspace_parameters.min_corner.x = -1.0
        mp.workspace_parameters.min_corner.y = -1.0
        mp.workspace_parameters.min_corner.z = -1.0
        mp.workspace_parameters.max_corner.x = 1.0
        mp.workspace_parameters.max_corner.y = 1.0
        mp.workspace_parameters.max_corner.z = 1.0

        ee_z = target.z + GRIPPER_TIP_OFFSET_Z

        # --- Position constraint ---
        constraints = Constraints()

        pos_c = PositionConstraint()
        pos_c.header.frame_id = BASE_FRAME
        pos_c.link_name = EE_LINK
        pos_c.weight = 1.0

        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [POSITION_TOLERANCE]

        spose = Pose()
        spose.position.x = target.x
        spose.position.y = target.y
        spose.position.z = ee_z
        spose.orientation.w = 1.0

        bv.primitives.append(sphere)
        bv.primitive_poses.append(spose)
        pos_c.constraint_region = bv
        constraints.position_constraints.append(pos_c)

        # --- Orientation constraint (loose) ---
        orient_c = OrientationConstraint()
        orient_c.header.frame_id = BASE_FRAME
        orient_c.link_name = EE_LINK
        orient_c.orientation.x = target.qx
        orient_c.orientation.y = target.qy
        orient_c.orientation.z = target.qz
        orient_c.orientation.w = target.qw
        orient_c.absolute_x_axis_tolerance = ORIENT_XY_TOLERANCE
        orient_c.absolute_y_axis_tolerance = ORIENT_XY_TOLERANCE
        orient_c.absolute_z_axis_tolerance = ORIENT_Z_TOLERANCE
        orient_c.weight = 1.0
        constraints.orientation_constraints.append(orient_c)

        mp.goal_constraints.append(constraints)

        # Planning options — plan+execute with replanning (like RViz)
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        max_retries = 3
        for attempt in range(max_retries):
            if attempt > 0:
                self.get_logger().info(f"Retry {attempt}/{max_retries-1}...")
                time.sleep(2.0)  # let TF and controller settle

            self.get_logger().info(
                f"{'Planning' if plan_only else 'Moving'} to: "
                f"({target.x:.3f}, {target.y:.3f}, {target.z:.3f}) "
                f"[EE Z: {ee_z:.3f}]"
            )

            future = self.move_group_client.send_goal_async(goal)
            handle = wait_for_future(future, timeout_sec=5.0)

            if handle is None:
                self.get_logger().error("Failed to send goal")
                continue
            if not handle.accepted:
                self.get_logger().error("Goal rejected")
                continue

            self.get_logger().info("Goal accepted, waiting for result...")
            result = wait_for_future(
                handle.get_result_async(),
                timeout_sec=PLANNING_TIME_SEC + 30.0
            )

            if result is None:
                self.get_logger().error("Timed out")
                continue

            code = result.result.error_code.val
            if code == 1:
                self.get_logger().info("Motion succeeded!")
                time.sleep(0.5)
                return True
            else:
                error_names = {
                    -1: "FAILURE", -2: "PLANNING_FAILED",
                    -4: "INVALID_GOAL", -10: "FRAME_TRANSFORM_FAILURE",
                    -12: "NO_IK_SOLUTION", -31: "TIMED_OUT",
                    99999: "FAILURE (Jazzy bug)",
                }
                name = error_names.get(code, f"UNKNOWN({code})")
                self.get_logger().error(f"Attempt {attempt+1}: {name}")

        self.get_logger().warn("All retries exhausted, trying wiggle-and-retry...")
        return self._wiggle_and_retry(target, plan_only)

    # -------------------------------------------------------------------------
    # Motion: Joint Goal (for home position)
    # -------------------------------------------------------------------------

    def move_to_joints(self, joint_positions: dict, plan_only: bool = False) -> bool:
        """Move to a joint-space goal using /move_action."""
        goal = MoveGroup.Goal()
        mp = goal.request

        mp.group_name = PLANNING_GROUP
        mp.num_planning_attempts = NUM_PLANNING_ATTEMPTS
        mp.allowed_planning_time = PLANNING_TIME_SEC
        mp.max_velocity_scaling_factor = MAX_VELOCITY_SCALING
        mp.max_acceleration_scaling_factor = MAX_ACCELERATION_SCALING
        mp.pipeline_id = "ompl"
        mp.planner_id = ""

        mp.start_state.is_diff = True

        mp.workspace_parameters.header.frame_id = BASE_FRAME
        mp.workspace_parameters.min_corner.x = -1.0
        mp.workspace_parameters.min_corner.y = -1.0
        mp.workspace_parameters.min_corner.z = -1.0
        mp.workspace_parameters.max_corner.x = 1.0
        mp.workspace_parameters.max_corner.y = 1.0
        mp.workspace_parameters.max_corner.z = 1.0

        constraints = Constraints()
        for jname, jpos in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = jpos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        mp.goal_constraints.append(constraints)

        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        max_retries = 3
        for attempt in range(max_retries):
            if attempt > 0:
                self.get_logger().info(f"Joint retry {attempt}/{max_retries-1}...")
                time.sleep(2.0)

            self.get_logger().info("Moving to joint target...")

            future = self.move_group_client.send_goal_async(goal)
            handle = wait_for_future(future, timeout_sec=5.0)

            if handle is None or not handle.accepted:
                self.get_logger().error("Joint goal rejected")
                continue

            result = wait_for_future(
                handle.get_result_async(),
                timeout_sec=PLANNING_TIME_SEC + 30.0
            )

            if result is None:
                self.get_logger().error("Joint motion timed out")
                continue

            code = result.result.error_code.val
            if code == 1:
                self.get_logger().info("Joint motion succeeded!")
                time.sleep(0.5)
                return True
            else:
                self.get_logger().error(f"Joint attempt {attempt+1} failed: {code}")

        self.get_logger().error("All joint retries exhausted")
        return False
    # -------------------------------------------------------------------------
    # Convenience methods
    # -------------------------------------------------------------------------

    def _wiggle_and_retry(self, target: TargetPose, plan_only: bool = False) -> bool:
        """Nudge multiple joints then retry."""
        import random
        from sensor_msgs.msg import JointState

        self.get_logger().info("Attempting wiggle-and-retry...")

        joint_event = threading.Event()
        current = {}

        def _cb(msg):
            for n, p in zip(msg.name, msg.position):
                current[n] = p
            joint_event.set()

        sub = self.create_subscription(JointState, "/joint_states", _cb, 10)
        joint_event.wait(timeout=3.0)
        self.destroy_subscription(sub)

        if not current:
            self.get_logger().error("Could not read joints for wiggle")
            return False

        # Try up to 3 different wiggle patterns
        for wiggle_attempt in range(3):
            nudged = {}
            for j in ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]:
                if j in current:
                    # Wiggle all joints slightly, more on 1 and 4
                    if j in ("joint_1", "joint_4"):
                        offset = random.uniform(-0.15, 0.15)
                    else:
                        offset = random.uniform(-0.08, 0.08)
                    nudged[j] = current[j] + offset

            self.get_logger().info(f"Wiggle attempt {wiggle_attempt + 1}/3...")
            if self.move_to_joints(nudged):
                time.sleep(0.5)
                self.get_logger().info("Retrying target after wiggle...")
                if self.move_to_pose(target, plan_only=plan_only):
                    return True
                # Update current positions for next wiggle
                joint_event.clear()
                sub2 = self.create_subscription(JointState, "/joint_states", _cb, 10)
                joint_event.wait(timeout=3.0)
                self.destroy_subscription(sub2)

        self.get_logger().error("All wiggle attempts exhausted")
        return False


    def plan_and_execute(self, target: TargetPose, execute: bool = True) -> bool:
        return self.move_to_pose(target, plan_only=not execute)

    def move_to_approach(self, target: TargetPose, execute: bool = True) -> bool:
        approach = TargetPose(
            x=target.x, y=target.y,
            z=target.z + target.approach_height,
            qx=target.qx, qy=target.qy, qz=target.qz, qw=target.qw,
        )

        self.get_logger().info(
            f"Step 1: Approach ({target.approach_height*100:.0f}cm above)..."
        )
        if not self.move_to_pose(approach, plan_only=not execute):
            self.get_logger().error("Failed to reach approach pose")
            return False

        self.get_logger().info("Step 2: Descending...")
        if not self.move_to_pose(target, plan_only=not execute):
            self.get_logger().error("Failed to descend")
            return False

        self.get_logger().info("Approach + descent complete!")
        return True


# =============================================================================
# HELPER
# =============================================================================

def build_scene_from_camera_data(node, target_coord, obstacle_coords):
    for oid in list(node.collision_objects.keys()):
        if oid != "table":
            node.remove_collision_object(oid)
    if "table" not in node.collision_objects:
        node.add_table()
    for i, obs in enumerate(obstacle_coords):
        node.add_collision_object(SceneObject(
            object_id=obs.get("id", f"obstacle_{i}"),
            object_type=obs.get("type", "small_box"),
            x=obs["x"], y=obs["y"], z=obs["z"],
        ))
    return TargetPose(
        x=target_coord[0], y=target_coord[1], z=target_coord[2],
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
    )


# =============================================================================
# DEMO
# =============================================================================

def demo_scenario():
    rclpy.init()
    node = KinovaPickPlanner()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.add_table()
        time.sleep(0.5)

        node.open_gripper()
        time.sleep(0.5)

        target_pose = TargetPose(
            x=0.40, y=0.05, z=0.04,
            qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            approach_height=0.15,
        )

        success = node.move_to_approach(target_pose, execute=True)

        if success:
            node.close_gripper()
            time.sleep(0.5)

            lift = TargetPose(x=0.40, y=0.05, z=0.20,
                              qx=1.0, qy=0.0, qz=0.0, qw=0.0)
            node.plan_and_execute(lift, execute=True)

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    demo_scenario()