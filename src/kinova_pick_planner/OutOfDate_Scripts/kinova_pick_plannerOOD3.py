#!/usr/bin/env python3
"""
Kinova Gen3 Lite - MoveIt2 Pick Planner
========================================
Uses /move_action (same as RViz) with retry + wiggle recovery.
Thread-safe for use from arm_controller background threads.

Object dimensions and table offset calibrated to actual hardware:
  - Robot base sits on 5cm mounting plate → table surface at z = -0.05
  - All object z coordinates are heights above the table surface
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
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
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene
from control_msgs.action import GripperCommand

import time
import random
import threading
from dataclasses import dataclass, field


# =============================================================================
# ROBOT & WORKSPACE CONFIGURATION
# =============================================================================

PLANNING_GROUP = "arm"
EE_LINK = "tool_frame"
BASE_FRAME = "base_link"

# Robot base sits on a 5cm plate on the table
# Table surface is 5cm below base_link origin
BASE_HEIGHT_ABOVE_TABLE = 0.05
TABLE_SURFACE_Z = -BASE_HEIGHT_ABOVE_TABLE  # -0.05 in base_link frame

PLANNING_TIME_SEC = 10.0
MAX_VELOCITY_SCALING = 0.3
MAX_ACCELERATION_SCALING = 0.3
NUM_PLANNING_ATTEMPTS = 15

# Retry configuration
MAX_MOTION_RETRIES = 3
RETRY_DELAY_SEC = 2.0

# Gripper
GRIPPER_ACTION = "/gen3_lite_2f_gripper_controller/gripper_cmd"
GRIPPER_OPEN_POSITION = 0.1

# Orientation tolerance (radians)
ORIENT_XY_TOLERANCE = 0.25    # ~23 degrees (0.4) (0.5) is about 29 degrees
ORIENT_Z_TOLERANCE = 3.14159  # free rotation around Z
POSITION_TOLERANCE = 0.01    # 1.5cm - 1cm

# Safety clearances above table surface
TOP_GRASP_MIN_Z_ABOVE_TABLE = 0.040   # 4 cm minimum for top grasps
SIDE_GRASP_MIN_Z_ABOVE_TABLE = 0.055  # 5.5 cm minimum for side grasps
APPROACH_MIN_Z_ABOVE_TABLE = 0.070    # 7 cm minimum during approach


# =============================================================================
# TABLE CONFIGURATION
# =============================================================================

# Table dimensions and position relative to base_link
# Robot is 5cm into the 75cm width, 30cm from table edge along 180cm length
TABLE_POSITION = {
    "x": 0.325,           # table center in front of robot (75/2 - 5 = 32.5cm)
    "y": 0.00,            # centered left-right
    "z": TABLE_SURFACE_Z, # table surface
    "length": 1.80,        # 180cm (y direction)
    "width": 0.75,         # 75cm (x direction, extends in front of robot)
    "thickness": 0.02,
}


# =============================================================================
# OBJECT TYPES — actual measured dimensions
# =============================================================================

@dataclass
class ObjectType:
    """Physical object definition for collision meshes."""
    name: str
    shape: int              # SolidPrimitive type
    dimensions: list        # shape-specific dims
    height: float           # actual object height (meters)
    z_offset: float = 0.01   # center offset above table surface

    # Collision mesh dimensions:
    #   SPHERE:   [radius]
    #   CYLINDER: [height, radius]
    #   BOX:      [x, y, z]

OBJECT_TYPES = {
    "foam_ball": ObjectType(
        name="foam_ball",
        shape=SolidPrimitive.SPHERE,
        dimensions=[0.03],          # 3cm radius (6cm diameter)
        height=0.06,
        z_offset=0.032,              # center at half height
    ),
    "coffee_cup": ObjectType(
        name="coffee_cup",
        shape=SolidPrimitive.CYLINDER,
        dimensions=[0.135, 0.045],  # 13.5cm tall, 4.5cm radius (9cm diameter)
        height=0.135,
        z_offset=0.0677,            # center at half height
    ),
    "water_bottle": ObjectType(
        name="water_bottle",
        shape=SolidPrimitive.CYLINDER,
        dimensions=[0.21, 0.035],   # 21cm tall, 3.5cm radius (7cm diameter)
        height=0.21,
        z_offset=0.107,             # center at half height
    ),
    "small_box": ObjectType(
        name="small_box",
        shape=SolidPrimitive.BOX,
        dimensions=[0.06, 0.06, 0.06],
        height=0.06,
        z_offset=0.032,
    ),
    "table": ObjectType(
        name="table",
        shape=SolidPrimitive.BOX,
        # dimensions=[0.75, 1.80, 0.02],  # width x length x thickness
        dimensions=[0.75, 1.80, 0.06],  # width x length x thickness
        # height=0.02,
        height=0.06,
        z_offset=-0.01,                  # top surface at TABLE_SURFACE_Z
    ),
    "table_keepout": ObjectType(
        name="table_keepout",
        shape=SolidPrimitive.BOX,
        dimensions=[0.75, 1.80, 0.02],  # 3 cm keepout thickness
        height=0.03,
        z_offset=0.015,                 # centered 1.5 cm above table surface
    ),
}


# =============================================================================
# GRASP STRATEGIES — how to grab each object type
# =============================================================================

@dataclass
class GraspStrategy:
    """Defines how the gripper approaches and grabs an object type."""
    approach: str              # "top" or "side"
    grab_z_ratio: float        # fraction of object height for grab point (0=bottom, 1=top)
    grab_z_offset: float       # additional z offset from computed grab point
    gripper_close_pos: float   # gripper close position
    gripper_effort: float      # gripper effort
    approach_offset: float = 0.12 # how far to offset approach
    # Orientation quaternion for the gripper at grab point
    qx: float = 1.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 0.0

GRASP_STRATEGIES = {
    "foam_ball": GraspStrategy(
        approach="top",
        grab_z_ratio=0.8,       # grab near top of ball
        grab_z_offset=0.01,     # 1cm above computed point for clearance
        gripper_close_pos=0.55,
        gripper_effort=20.0,
        approach_offset=0.14,
        # Gripper pointing straight down
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
    ),
    "coffee_cup": GraspStrategy(
        approach="side",
        grab_z_ratio=0.6,       # grab at 70% height (upper portion)
        grab_z_offset=0.0,
        gripper_close_pos=0.5,  # don't crush the cup
        gripper_effort=25.0,
        approach_offset=0.18,
        qx=0.197, qy=0.689, qz=0.677, qw=0.169,
    ),
    "water_bottle": GraspStrategy(
        approach="side",
        grab_z_ratio=0.5,       # grab at midpoint
        grab_z_offset=0.0,
        gripper_close_pos=0.6,
        gripper_effort=35.0,
        approach_offset=0.18,
        qx=0.197, qy=0.680, qz=0.677, qw=0.169,
    ),
    "small_box": GraspStrategy(
        approach="top",
        grab_z_ratio=0.8,
        grab_z_offset=0.01,
        gripper_close_pos=0.6,
        gripper_effort=30.0,
        approach_offset=0.12,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
    ),
    "default": GraspStrategy(
        approach="top",
        grab_z_ratio=0.7,
        grab_z_offset=0.01,
        gripper_close_pos=0.6,
        gripper_effort=30.0,
        approach_offset=0.12,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
    ),
}


# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class SceneObject:
    object_id: str
    object_type: str
    x: float
    y: float
    z: float            # height above table surface (passed in from camera)
    padding: float = 0.005


@dataclass
class TargetPose:
    x: float
    y: float
    z: float            # in base_link frame (already converted)
    qx: float = 1.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 0.0


# =============================================================================
# HELPER: Convert table-relative Z to base_link Z
# =============================================================================

def table_z_to_base_z(z_above_table: float) -> float:
    """Convert a height above table surface to base_link frame Z."""
    return TABLE_SURFACE_Z + z_above_table


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
# COMPUTE GRASP POSE FROM OBJECT
# =============================================================================

# def compute_grasp_pose(obj_data: dict) -> TargetPose:
#     """
#     Compute the gripper target pose for grasping an object.

#     Args:
#         obj_data: dict with keys "x", "y", "z" (height above table), "type"

#     Returns:
#         TargetPose in base_link frame with correct grab height and orientation.
#     """
#     obj_type_name = obj_data.get("type", "default")
#     strategy = GRASP_STRATEGIES.get(obj_type_name, GRASP_STRATEGIES["default"])
#     obj_type = OBJECT_TYPES.get(obj_type_name)

#     obj_x = obj_data["x"]
#     obj_y = obj_data["y"]
#     obj_z_above_table = obj_data["z"]  # height of object base above table

#     if obj_type:
#         # Compute grab height: base + (height * ratio) + offset
#         grab_height_above_table = (
#             obj_z_above_table
#             + obj_type.height * strategy.grab_z_ratio
#             + strategy.grab_z_offset
#         )
#     else:
#         # Unknown object — grab at the provided z + small offset
#         grab_height_above_table = obj_z_above_table + strategy.grab_z_offset

#     # Convert to base_link frame
#     grab_z_base = table_z_to_base_z(grab_height_above_table)

#     return TargetPose(
#         x=obj_x,
#         y=obj_y,
#         z=grab_z_base,
#         qx=strategy.qx,
#         qy=strategy.qy,
#         qz=strategy.qz,
#         qw=strategy.qw,
#     )

def compute_grasp_pose(obj_data: dict) -> TargetPose:
    obj_type_name = obj_data.get("type", "default")
    strategy = GRASP_STRATEGIES.get(obj_type_name, GRASP_STRATEGIES["default"])
    obj_type = OBJECT_TYPES.get(obj_type_name)

    obj_x = obj_data["x"]
    obj_y = obj_data["y"]
    obj_z_above_table = obj_data["z"]

    if obj_type:
        grab_height_above_table = (
            obj_z_above_table
            + obj_type.height * strategy.grab_z_ratio
            + strategy.grab_z_offset
        )
    else:
        grab_height_above_table = obj_z_above_table + strategy.grab_z_offset

    min_allowed = (
        TOP_GRASP_MIN_Z_ABOVE_TABLE
        if strategy.approach == "top"
        else SIDE_GRASP_MIN_Z_ABOVE_TABLE
    )
    grab_height_above_table = max(grab_height_above_table, min_allowed)

    grab_z_base = table_z_to_base_z(grab_height_above_table)

    return TargetPose(
        x=obj_x,
        y=obj_y,
        z=grab_z_base,
        qx=strategy.qx,
        qy=strategy.qy,
        qz=strategy.qz,
        qw=strategy.qw,
    )

def compute_approach_pose(grasp_pose: TargetPose, obj_data: dict) -> TargetPose:
    """
    Compute an approach pose offset from the grasp pose.
    Top approach: directly above. Side approach: pulled back toward robot.
    """
    obj_type_name = obj_data.get("type", "default")
    strategy = GRASP_STRATEGIES.get(obj_type_name, GRASP_STRATEGIES["default"])

    if strategy.approach == "top":
        # Approach from above — same XY, higher Z
        return TargetPose(
            x=grasp_pose.x,
            y=grasp_pose.y,
            z=grasp_pose.z + strategy.approach_offset,
            qx=grasp_pose.qx, qy=grasp_pose.qy,
            qz=grasp_pose.qz, qw=grasp_pose.qw,
        )
    else:
        # Side approach — pull back toward robot (reduce X)
        # The robot base is at x=0, objects are at positive x
        # So pulling back means reducing x
        return TargetPose(
            x=grasp_pose.x - strategy.approach_offset,
            y=grasp_pose.y,
            z=grasp_pose.z,
            qx=grasp_pose.qx, qy=grasp_pose.qy,
            qz=grasp_pose.qz, qw=grasp_pose.qw,
        )

def compute_oriented_grasp(obj_data: dict) -> tuple:
    """
    Compute grasp and approach poses with the gripper Z-rotation aligned
    so the fingers straddle the object, not push into it.
    
    Returns (approach_pose, grasp_pose) tuple.
    """
    import math

    obj_type_name = obj_data.get("type", "default")
    strategy = GRASP_STRATEGIES.get(obj_type_name, GRASP_STRATEGIES["default"])
    
    # Get the base grasp pose (position + default orientation)
    grasp_pose = compute_grasp_pose(obj_data)
    
    if strategy.approach == "top":
        # For top grasps, compute Z rotation so fingers align with the 
        # line from robot base to object
        angle = math.atan2(obj_data["y"], obj_data["x"])
        
        # Rotate the downward-pointing quaternion around Z
        # Base orientation: gripper down (qx=1, qy=0, qz=0, qw=0)
        # Apply Z rotation: multiply by quaternion (0, 0, sin(a/2), cos(a/2))
        half_a = angle / 2.0
        # Quaternion multiplication: down_quat * z_rotation_quat
        # down = (1, 0, 0, 0), z_rot = (0, 0, sin, cos)
        # Result: (cos, -sin, sin*0+cos*0, ...)
        # Simplified for (1,0,0,0) * (0,0,sz,cz):
        sz = math.sin(half_a)
        cz = math.cos(half_a)
        grasp_pose.qx = cz    # 1*cz - 0*sz
        grasp_pose.qy = -sz   # 1*(-sz) + 0*cz  
        grasp_pose.qz = sz    # 0*cz + 0*sz ... actually let me just use proper math
        grasp_pose.qw = cz    # This needs proper quaternion multiply
        
        # Proper quaternion multiplication: q1 * q2
        # q1 = (1, 0, 0, 0) = gripper down
        # q2 = (0, 0, sin(a/2), cos(a/2)) = rotate around Z
        # Result = (q1w*q2x + q1x*q2w + q1y*q2z - q1z*q2y,
        #           q1w*q2y - q1x*q2z + q1y*q2w + q1z*q2x,
        #           q1w*q2z + q1x*q2y - q1y*q2x + q1z*q2w,
        #           q1w*q2w - q1x*q2x - q1y*q2y - q1z*q2z)
        q1x, q1y, q1z, q1w = 1.0, 0.0, 0.0, 0.0
        q2x, q2y, q2z, q2w = 0.0, 0.0, sz, cz
        
        grasp_pose.qx = q1w*q2x + q1x*q2w + q1y*q2z - q1z*q2y
        grasp_pose.qy = q1w*q2y - q1x*q2z + q1y*q2w + q1z*q2x
        grasp_pose.qz = q1w*q2z + q1x*q2y - q1y*q2x + q1z*q2w
        grasp_pose.qw = q1w*q2w - q1x*q2x - q1y*q2y - q1z*q2z
        
        # Approach from above
        # approach_pose = TargetPose(
        #     x=grasp_pose.x,
        #     y=grasp_pose.y,
        #     z=grasp_pose.z + strategy.approach_offset,
        #     qx=grasp_pose.qx, qy=grasp_pose.qy,
        #     qz=grasp_pose.qz, qw=grasp_pose.qw,
        # )

        approach_z = max(
            grasp_pose.z + 0.03,
            table_z_to_base_z(APPROACH_MIN_Z_ABOVE_TABLE)
        )

        approach_pose = TargetPose(
            x=grasp_pose.x - strategy.approach_offset * math.cos(angle),
            y=grasp_pose.y - strategy.approach_offset * math.sin(angle),
            z=approach_z,
            qx=grasp_pose.qx, qy=grasp_pose.qy,
            qz=grasp_pose.qz, qw=grasp_pose.qw,
        )
        
    else:
        # Side grasp — approach along the line from robot to object
        angle = math.atan2(obj_data["y"], obj_data["x"])
        #angle = math.atan2(obj_data["y"], obj_data["x"]) + math.pi / 2.0

        # The side grasp base quaternion from your arm test
        bqx, bqy, bqz, bqw = strategy.qx, strategy.qy, strategy.qz, strategy.qw
        
        # Apply Z rotation to the base side-grasp quaternion
        half_a = angle / 2.0
        sz = math.sin(half_a)
        cz = math.cos(half_a)
        
        grasp_pose.qx = bqw*0 + bqx*cz + bqy*sz - bqz*0
        grasp_pose.qy = bqw*0 - bqx*sz + bqy*cz + bqz*0  
        grasp_pose.qz = bqw*sz + bqx*0 - bqy*0 + bqz*cz
        grasp_pose.qw = bqw*cz - bqx*0 - bqy*0 - bqz*sz
        
        # Approach pulled back along the line from robot to object
        approach_pose = TargetPose(
            x=grasp_pose.x - strategy.approach_offset * math.cos(angle),
            y=grasp_pose.y - strategy.approach_offset * math.sin(angle),
            z=grasp_pose.z,
            qx=grasp_pose.qx, qy=grasp_pose.qy,
            qz=grasp_pose.qz, qw=grasp_pose.qw,
        )
    
    return approach_pose, grasp_pose

# =============================================================================
# MAIN PLANNER NODE
# =============================================================================

class KinovaPickPlanner(Node):
    def __init__(self):
        super().__init__("kinova_pick_planner")
        self.get_logger().info("Initializing Kinova Pick Planner...")

        self.cb_group = ReentrantCallbackGroup()

        self.planning_scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene",
            callback_group=self.cb_group,
        )
        self.move_group_client = ActionClient(
            self, MoveGroup, "/move_action",
            callback_group=self.cb_group,
        )
        self.gripper_client = ActionClient(
            self, GripperCommand, GRIPPER_ACTION,
            callback_group=self.cb_group,
        )

        # Cartesian path service
        from moveit_msgs.srv import GetCartesianPath
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path",
            callback_group=self.cb_group,
        )

        # Execute trajectory action (for Cartesian paths)
        from moveit_msgs.action import ExecuteTrajectory
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory",
            callback_group=self.cb_group,
        )

        self.collision_objects: dict[str, CollisionObject] = {}
        self._wait_for_services()
        self.get_logger().info("Kinova Pick Planner ready!")
        self._home_joints = {}

    def _wait_for_services(self):
        self.get_logger().info("Waiting for /move_action...")
        if not self.move_group_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("MoveGroup not available")

        self.get_logger().info("Waiting for ApplyPlanningScene...")
        if not self.apply_scene_client.wait_for_service(timeout_sec=10.0):
            self._use_scene_service = False
        else:
            self._use_scene_service = True

        self.get_logger().info("Waiting for gripper...")
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self._gripper_available = False
        else:
            self._gripper_available = True
            self.get_logger().info("Gripper connected!")

        self.get_logger().info("Waiting for /compute_cartesian_path...")
        if not self.cartesian_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Cartesian path service not available")
            self._cartesian_available = False
        else:
            self._cartesian_available = True

        self.get_logger().info("Waiting for /execute_trajectory...")
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("Execute trajectory not available")
        else:
            self.get_logger().info("Execute trajectory connected!")

    # -------------------------------------------------------------------------
    # Planning Scene
    # -------------------------------------------------------------------------

    def add_table(self):
        OBJECT_TYPES["table"].dimensions = [
            TABLE_POSITION["width"],
            TABLE_POSITION["length"],
            TABLE_POSITION["thickness"],
        ]
        OBJECT_TYPES["table"].height = TABLE_POSITION["thickness"]
        OBJECT_TYPES["table"].z_offset = -TABLE_POSITION["thickness"] / 2.0

        self.add_collision_object(SceneObject(
            object_id="table", object_type="table",
            x=TABLE_POSITION["x"], y=TABLE_POSITION["y"],
            z=0.0,  # table surface is at z=0 above table
            padding=0.0,
            
        ))

        self.add_collision_object(SceneObject(
            object_id="table_keepout",
            object_type="table_keepout",
            x=TABLE_POSITION["x"], y=TABLE_POSITION["y"],
            z=0.0,
            padding=0.0,
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
        # Object z is height above table → convert to base_link frame
        # Then add z_offset to place center of collision mesh correctly
        pose.position.z = table_z_to_base_z(scene_obj.z) + obj_type.z_offset
        pose.orientation.w = 1.0

        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        self._apply_collision_object(co)
        self.collision_objects[scene_obj.object_id] = co
        self.get_logger().info(
            f"Added '{scene_obj.object_id}' at "
            f"({scene_obj.x:.3f}, {scene_obj.y:.3f}, "
            f"table+{scene_obj.z:.3f}m → base_z={pose.position.z:.3f})"
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
        return self._send_gripper_command(GRIPPER_OPEN_POSITION, 30.0)

    def close_gripper(self, position: float = 0.6, effort: float = 30.0) -> bool:
        return self._send_gripper_command(position, effort)

    def _send_gripper_command(self, position: float, effort: float) -> bool:
        if not self._gripper_available:
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
    # Motion: Pose Goal with retry + wiggle
    # -------------------------------------------------------------------------

    def move_to_pose(self, target: TargetPose, plan_only: bool = False) -> bool:
        """Move to Cartesian pose via /move_action. Retries on failure."""
        goal = self._build_pose_goal(target, plan_only)

        for attempt in range(MAX_MOTION_RETRIES):
            if attempt > 0:
                self.get_logger().info(f"Retry {attempt}/{MAX_MOTION_RETRIES-1}...")
                time.sleep(RETRY_DELAY_SEC)

            self.get_logger().info(
                f"{'Planning' if plan_only else 'Moving'} to: "
                f"({target.x:.3f}, {target.y:.3f}, {target.z:.3f})"
            )

            future = self.move_group_client.send_goal_async(goal)
            handle = wait_for_future(future, timeout_sec=5.0)

            if handle is None or not handle.accepted:
                self.get_logger().error("Goal rejected")
                continue

            self.get_logger().info("Goal accepted, waiting...")
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
                names = {
                    -1: "FAILURE", -2: "PLANNING_FAILED",
                    -4: "INVALID_GOAL", -10: "FRAME_TRANSFORM_FAILURE",
                    -12: "NO_IK_SOLUTION", -31: "TIMED_OUT",
                    99999: "FAILURE (Jazzy bug)",
                }
                self.get_logger().error(
                    f"Attempt {attempt+1}: {names.get(code, f'UNKNOWN({code})')}"
                )

        # All retries failed — try wiggle recovery
        self.get_logger().warn("Retries exhausted, trying wiggle recovery...")
        return self._wiggle_and_retry(target, plan_only)

    def _build_pose_goal(self, target: TargetPose, plan_only: bool) -> MoveGroup.Goal:
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

        # Position
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
        spose.position.z = target.z
        spose.orientation.w = 1.0

        bv.primitives.append(sphere)
        bv.primitive_poses.append(spose)
        pos_c.constraint_region = bv
        constraints.position_constraints.append(pos_c)

        # Orientation
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

        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        return goal

    # -------------------------------------------------------------------------
    # Wiggle Recovery
    # -------------------------------------------------------------------------
    def _wiggle_and_retry(self, target: TargetPose, plan_only: bool = False) -> bool:
        """Nudge joints, then try home reset if wiggles fail."""
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

        arm_joints = ["joint_1", "joint_2", "joint_3",
                       "joint_4", "joint_5", "joint_6"]

        # Phase 1: Try small wiggles
        for wiggle_attempt in range(2):
            nudged = {}
            for j in arm_joints:
                if j in current:
                    if j in ("joint_1", "joint_4"):
                        offset = random.uniform(-0.15, 0.15)
                    else:
                        offset = random.uniform(-0.08, 0.08)
                    nudged[j] = current[j] + offset

            self.get_logger().info(f"Wiggle attempt {wiggle_attempt + 1}/2...")
            if self.move_to_joints(nudged):
                time.sleep(0.5)
                goal = self._build_pose_goal(target, plan_only)
                future = self.move_group_client.send_goal_async(goal)
                handle = wait_for_future(future, timeout_sec=5.0)
                if handle and handle.accepted:
                    result = wait_for_future(
                        handle.get_result_async(),
                        timeout_sec=PLANNING_TIME_SEC + 30.0
                    )
                    if result and result.result.error_code.val == 1:
                        self.get_logger().info("Wiggle recovery succeeded!")
                        time.sleep(0.5)
                        return True

                # Update current joints
                joint_event.clear()
                sub2 = self.create_subscription(JointState, "/joint_states", _cb, 10)
                joint_event.wait(timeout=3.0)
                self.destroy_subscription(sub2)

        # Phase 2: Go home and retry from clean state
        self.get_logger().warn("Wiggles failed — resetting to home and retrying...")
        if self.move_to_joints(self._home_joints):
            time.sleep(1.0)
            self.get_logger().info("Retrying from home position...")
            goal = self._build_pose_goal(target, plan_only)
            future = self.move_group_client.send_goal_async(goal)
            handle = wait_for_future(future, timeout_sec=5.0)
            if handle and handle.accepted:
                result = wait_for_future(
                    handle.get_result_async(),
                    timeout_sec=PLANNING_TIME_SEC + 30.0
                )
                if result and result.result.error_code.val == 1:
                    self.get_logger().info("Home-reset recovery succeeded!")
                    time.sleep(0.5)
                    return True

        self.get_logger().error("All recovery attempts exhausted")
        return False

    # -------------------------------------------------------------------------
    # Motion: Joint Goal with retry
    # -------------------------------------------------------------------------

    def move_to_joints(self, joint_positions: dict, plan_only: bool = False) -> bool:
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

        for attempt in range(MAX_MOTION_RETRIES):
            if attempt > 0:
                self.get_logger().info(f"Joint retry {attempt}...")
                time.sleep(RETRY_DELAY_SEC)

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

    def move_cartesian(self, target: TargetPose, max_step: float = 0.005) -> bool:
        """
        Move in a straight Cartesian line from current pose to target.
        Maintains orientation throughout the motion (SLERP interpolation).
        Use for short, critical moves like approach→grasp and grasp→lift.
        
        Args:
            target: Destination pose
            max_step: Interpolation resolution in meters (smaller = smoother, default 5mm)
        
        Returns:
            True if the full path was achieved and executed.
        """
        from moveit_msgs.srv import GetCartesianPath as GCP
        from moveit_msgs.action import ExecuteTrajectory as ET

        request = GCP.Request()
        request.header.frame_id = BASE_FRAME
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = PLANNING_GROUP
        request.link_name = EE_LINK
        
        # Start from current state
        request.start_state.is_diff = True
        
        # Single waypoint = straight line to target
        target_pose = Pose()
        target_pose.position.x = target.x
        target_pose.position.y = target.y
        target_pose.position.z = target.z
        target_pose.orientation.x = target.qx
        target_pose.orientation.y = target.qy
        target_pose.orientation.z = target.qz
        target_pose.orientation.w = target.qw
        request.waypoints.append(target_pose)
        
        request.max_step = max_step
        request.jump_threshold = 0.0  # disable jump detection
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = MAX_VELOCITY_SCALING
        request.max_acceleration_scaling_factor = MAX_ACCELERATION_SCALING

        self.get_logger().info(
            f"Cartesian move to: ({target.x:.3f}, {target.y:.3f}, {target.z:.3f})"
        )

        for attempt in range(MAX_MOTION_RETRIES):
            if attempt > 0:
                self.get_logger().info(f"Cartesian retry {attempt}...")
                time.sleep(RETRY_DELAY_SEC)

            future = self.cartesian_client.call_async(request)
            result = wait_for_future(future, timeout_sec=10.0)

            if result is None:
                self.get_logger().error("Cartesian path service call failed")
                continue

            fraction = result.fraction
            self.get_logger().info(f"Cartesian path fraction: {fraction:.2f}")

            if fraction < 0.95:
                self.get_logger().warn(
                    f"Only {fraction*100:.0f}% of Cartesian path achievable"
                )
                continue

            # Execute the trajectory
            exec_goal = ET.Goal()
            exec_goal.trajectory = result.solution

            exec_future = self.execute_client.send_goal_async(exec_goal)
            exec_handle = wait_for_future(exec_future, timeout_sec=5.0)

            if exec_handle is None or not exec_handle.accepted:
                self.get_logger().error("Cartesian execution rejected")
                continue

            exec_result = wait_for_future(
                exec_handle.get_result_async(),
                timeout_sec=30.0
            )

            if exec_result is None:
                self.get_logger().error("Cartesian execution timed out")
                continue

            code = exec_result.result.error_code.val
            if code == 1:
                self.get_logger().info("Cartesian move succeeded!")
                time.sleep(0.5)
                return True
            else:
                self.get_logger().error(f"Cartesian execution failed: {code}")

        self.get_logger().error("Cartesian move failed after retries")
        return False

    # -------------------------------------------------------------------------
    # Convenience
    # -------------------------------------------------------------------------

    def plan_and_execute(self, target: TargetPose, execute: bool = True) -> bool:
        return self.move_to_pose(target, plan_only=not execute)


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

        # Test with a ball sitting on the table
        # z=0.0 means the ball base is on the table surface
        ball_data = {"x": 0.40, "y": 0.05, "z": 0.0, "type": "foam_ball"}
        grasp_pose = compute_grasp_pose(ball_data)

        node.get_logger().info(
            f"Ball grasp pose: ({grasp_pose.x:.3f}, {grasp_pose.y:.3f}, "
            f"{grasp_pose.z:.3f})"
        )

        node.open_gripper()
        time.sleep(0.5)

        success = node.move_to_pose(grasp_pose)
        if success:
            node.close_gripper(position=0.5, effort=20.0)
            time.sleep(0.5)

            lift = TargetPose(
                x=grasp_pose.x, y=grasp_pose.y,
                z=grasp_pose.z + 0.20,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            )
            node.move_to_pose(lift)

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    demo_scenario()