#!/usr/bin/env python3
"""
Kinova Gen3 Lite - MoveIt2 Pick Planner (pymoveit2)
=====================================================
Built on pymoveit2 for reliable Cartesian path planning,
proper orientation control, and clean collision management.

Robot base sits on 5cm plate → table surface at z = -0.05 in base_link frame.
Object z coordinates = height above table surface.
"""
from __future__ import annotations  # required for Python 3.8 (ROS Foxy)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2, GripperInterface
from control_msgs.action import GripperCommand

import math
import time
import random
import threading
from dataclasses import dataclass


# =============================================================================
# ROBOT CONFIGURATION
# =============================================================================

ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]
GRIPPER_JOINTS = ["right_finger_bottom_joint"]
BASE_LINK = "base_link"
END_EFFECTOR = "tool_frame"
ARM_GROUP = "arm"
GRIPPER_GROUP = "gripper"

GRIPPER_ACTION_NAME = "/gen3_lite_2f_gripper_controller/gripper_cmd"
GRIPPER_DEFAULT_EFFORT = 30.0
GRIPPER_SETTLE_SEC = 1.5  # time to let the gripper physically move

# Robot base sits on 5cm plate
BASE_HEIGHT_ABOVE_TABLE = 0.05
TABLE_SURFACE_Z = -BASE_HEIGHT_ABOVE_TABLE  # -0.05

# Motion parameters
MAX_VELOCITY = 0.3
MAX_ACCELERATION = 0.3
PLANNING_ATTEMPTS = 15
PLANNING_TIME = 5.0

# Retry
MAX_RETRIES = 3
RETRY_DELAY = 2.0

# Default hop length for move_cartesian_waypoints(). Breaking a long
# Cartesian pull into short hops re-solves IK from the arm's actual current
# joint state after each one, which helps keep the arm from drifting far
# from a sane configuration between hops. It does NOT by itself prevent a
# single hop's internal IK solve from landing in a different configuration
# branch than the previous hop — see CARTESIAN_REVOLUTE_JUMP_THRESHOLD.
CARTESIAN_WAYPOINT_STEP = 0.03  # meters

# pymoveit2's compute_cartesian_path wrapper defaults this to 0.0, meaning a
# 0%-fraction plan (IK failure on the very first internal step) is still
# handed back as an "executable" trajectory and reported as succeeded.
# Setting a real threshold makes pymoveit2 itself reject and log the
# achieved fraction, instead of us discovering the truncation after the
# fact via _verify_reached()'s FK check.
CARTESIAN_FRACTION_THRESHOLD = 0.95

# moveit_msgs/GetCartesianPath's revolute_jump_threshold (and the legacy
# combined jump_threshold) are also zero-inited with no default, and are
# each documented as "only enforced if set > 0" — so out of the box,
# compute_cartesian_path has ZERO check that consecutive interpolated
# waypoints stay in the same IK configuration branch. Since this 6-DOF arm's
# IK is non-unique, the same request can non-deterministically resolve into
# a completely different elbow/wrist configuration between retries, and if
# that configuration also happens to pass collision/fraction checks, MoveIt
# will happily execute the resulting large joint-space swing as a
# "successful" Cartesian move. This is a starting value, not a tuned one —
# raise it if legitimate hops through natural wrist motion start getting
# rejected, lower it if wide swings still get through.
CARTESIAN_REVOLUTE_JUMP_THRESHOLD = 1.57  # radians (~90 degrees) per hop

# Safety clearances above table surface
TOP_GRASP_MIN_Z_ABOVE_TABLE = 0.045
SIDE_GRASP_MIN_Z_ABOVE_TABLE = 0.060
APPROACH_MIN_Z_ABOVE_TABLE = 0.080

# wait_until_executed() only reports whether MoveIt's execution action
# completed without error, not whether the commanded pose was reached — a
# short/truncated Cartesian plan (e.g. cut off by a collision constraint)
# reports the same success. Verify the achieved pose against the target
# before trusting it.
CARTESIAN_POSITION_TOLERANCE = 0.02  # meters


# =============================================================================
# TABLE CONFIGURATION
# =============================================================================

TABLE_POSITION = {
    "x": 0.325,
    "y": 0.00,
    "z": TABLE_SURFACE_Z,
    "length": 1.80,
    "width": 0.75,
    "thickness": 0.02,
}


# =============================================================================
# DUAL-ARM STATIC KEEPOUT (domain-isolated architecture — see
# per_arm_bringup_sequence.md)
# =============================================================================
# Both arms are physically mounted facing each other across the shared
# table, base-to-base distance 1.10 m. Each arm's own +X axis points toward
# the other arm's base by construction (that's what "facing each other"
# means), so the same offset is correct in both arms' local base_link
# frames without modification — this code does not need to differ per arm.
#
# The two arms do not share TF/planning scene (confirmed in
# per_arm_bringup_sequence.md step 15: each arm's MoveIt2 instance has an
# identity world->base_link transform and has no notion the other arm
# exists). This keepout is therefore a static approximation, not a live
# collision check — it does not update if the other arm moves outside this
# conservative envelope.
OTHER_ARM_BASE_DISTANCE_X = 1.10  # meters, base-to-base — physical rig measurement

# Box size is a conservative PLACEHOLDER for the Gen3 Lite's body+reach
# envelope, not a measured value. Tighten once the physical rig exists and
# the other arm's actual reach/resting posture is known.
OTHER_ARM_KEEPOUT = {
    "depth":  0.60,  # along the approach axis (X), placeholder
    "width":  0.60,  # across (Y), placeholder
    "height": 0.90,  # from table surface up, placeholder
}


# =============================================================================
# DUAL-ARM CUP HANDOFF + BALL TASK (ICRA evaluation harness — MuJoCo only,
# see mujoco_dual_arm_scene.md / the evaluation-harness plan). Arm A picks the
# cup from its own side, carries it to HANDOFF_POSE (roughly the table
# midpoint, within both arms' reach), Arm B takes over and carries it to
# CUP_FINAL_PLACE_POSITION on its own side, then Arm A picks a ping-pong ball
# and places it within BALL_PLACE_TOLERANCE_M of the cup's final resting pose
# (originally a straw for this second object -- see the ping_pong_ball
# ObjectType entry below for why that changed).
# Positions below assume the dual-arm MuJoCo layout built this session:
# armA_base at x=0, armB_base at x=OTHER_ARM_BASE_DISTANCE_X (1.10), table
# spanning x=[-0.575, 1.225] (TABLE_POSITION below) — NOT valid for the
# single, domain-isolated real-arm setup, where each arm's planning frame
# has no notion of the other arm's position at all.
# =============================================================================

# NEAR_BASE_* variants kept here (not deleted) — the original pickup/place
# positions, ~21-25cm from the owning arm's base. Diagnosed during live
# testing as too close: a near-base reach puts the arm in a folded joint
# configuration prone to swinging sideways under joint-space interpolation
# (measured ~7cm of object displacement mid-descent). Not currently used by
# the task, but earmarked as a deliberately-harder configuration variant for
# the paper (comparing task success across reach distance / joint-config
# difficulty), so keep these exact numbers rather than losing them.
NEAR_BASE_CUP_PICKUP_POSITION = {"x": 0.15, "y": 0.15, "z_above_table": 0.0}
NEAR_BASE_BALL_PICKUP_POSITION = {"x": 0.15, "y": -0.15, "z_above_table": 0.0}
NEAR_BASE_CUP_FINAL_PLACE_POSITION = {"x": 0.90, "y": 0.15, "z_above_table": 0.0}

# Current default: pulled out to a comfortable ~34cm mid-range reach from each
# arm's own base (symmetric on both sides) instead of the cramped near-base
# positions above.
CUP_PICKUP_POSITION = {"x": 0.30, "y": 0.15, "z_above_table": 0.0}   # Arm A's side
BALL_PICKUP_POSITION = {"x": 0.30, "y": -0.15, "z_above_table": 0.0}  # Arm A's side, offset in y from the cup so the two pickups don't overlap

# Roughly the midpoint between armA_base (x=0) and armB_base (x=1.10) —
# reachable by both arms without either needing to lean into the other's
# keepout envelope (OTHER_ARM_KEEPOUT above). z_above_table=0.0 (resting ON
# the table): the dual-arm coordinator uses a place-then-pick handoff, not a
# true simultaneous mid-air handoff (both grippers converging on the cup at
# once caused a physically unstable contact blow-up — see
# mujoco_dual_arm_scene.md/dual_arm_coordinator.py) — this used to be 0.15
# (a mid-air hold height) for that abandoned design; left at 0.0 now to match
# CUP_FINAL_PLACE_POSITION's convention.
HANDOFF_POSE = {"x": 0.55, "y": 0.0, "z_above_table": 0.0}

CUP_FINAL_PLACE_POSITION = {"x": 0.80, "y": 0.15, "z_above_table": 0.0}  # Arm B's side, ~34cm from armB_base

# "Success" for the ball sub-task is the ball resting within this radius of
# the cup's final position — not literal insertion into the cup opening,
# which is a much harder precision problem and out of scope (see the
# evaluation-harness plan's Task Design section). Originally a straw for this
# spot; swapped for a ping-pong ball after finding the gripper's mechanical
# minimum closed gap (~30.4mm, measured directly from the MJCF geometry) makes
# an 8mm-diameter straw physically impossible to pinch-grip at any close
# position -- not a tuning problem, a geometric one. A standard ping-pong ball
# (40mm diameter) is well inside the gripper's graspable range and reuses the
# existing sphere/top-grasp pattern already used for foam_ball.
BALL_PLACE_TOLERANCE_M = 0.05


# =============================================================================
# OBJECT TYPES
# =============================================================================

@dataclass
class ObjectType:
    name: str
    shape: str              # "sphere", "cylinder", "box"
    dimensions: tuple       # shape-specific: sphere(r), cylinder(h,r), box(x,y,z)
    height: float
    z_offset: float         # center offset above table surface


OBJECT_TYPES = {
    "foam_ball": ObjectType(
        name="foam_ball", shape="sphere",
        dimensions=(0.03,),         # 3cm radius
        height=0.06, z_offset=0.035,
    ),
    "apple": ObjectType(
        name="apple", shape="sphere",
        dimensions=(0.03,),         # 3cm radius
        height=0.06, z_offset=0.035,
    ),
    "orange": ObjectType(
        name="orange", shape="sphere",
        dimensions=(0.035,),        # 3.5cm radius (~7cm diameter, navel orange)
        height=0.06, z_offset=0.035,
    ),
    "coffee_cup": ObjectType(
        name="coffee_cup", shape="cylinder",
        dimensions=(0.135, 0.045),  # 13.5cm tall, 4.5cm radius
        height=0.135, z_offset=0.075,
    ),
    "water_bottle": ObjectType(
        name="water_bottle", shape="cylinder",
        dimensions=(0.21, 0.035),   # 21cm tall, 3.5cm radius
        height=0.21, z_offset=0.115,
    ),
    "small_box": ObjectType(
        name="small_box", shape="box",
        dimensions=(0.06, 0.06, 0.06),
        height=0.06, z_offset=0.035,
    ),
    # mujoco_coffee_cup: 3.5cm radius / 70mm diameter -- narrower than the
    # real-hardware "coffee_cup"'s 4.5cm/90mm. At 90mm this gripper's
    # finger_prox_link only has ~7.6mm of clearance even fully open (for the
    # arm pose a top-down grasp of this object requires), which produced
    # inconsistent outcomes (clean sometimes, slipped free on lift other
    # times, occasionally a violent launch) even after softening contact
    # solver params. Separately, a real joint-runaway bug (right_finger_
    # bottom_joint's actuator torque was uncapped enough to tunnel through
    # its own 0-0.85 rad range limit under load, measured at -328 rad after
    # one stuck gripper action) was muddying the 90mm-vs-70mm comparison --
    # now fixed (actuatorfrcrange capped, solreflimit/solimplimit stiffened
    # on the bottom joints, see dual_arm.xml). At 3.5cm radius, clearance is
    # ~18mm (53mm prox-link-to-object-center distance, roughly fixed by the
    # arm's required pose regardless of object size, minus this radius) --
    # more than double the 90mm case's margin.
    "mujoco_coffee_cup": ObjectType(
        name="mujoco_coffee_cup", shape="cylinder",
        dimensions=(0.135, 0.035), height=0.135, z_offset=0.075,
    ),
    # Second MuJoCo-harness pick-place object. Originally a thin "straw"
    # (4mm radius cylinder) -- replaced after measuring the gripper's
    # mechanical minimum closed gap directly from the MJCF geometry
    # (~30.4mm, fingertip-to-fingertip at max closure) and finding an 8mm-
    # diameter object can never be pinch-gripped at any close position, not a
    # tuning problem but a geometric one. Standard ping-pong ball (40mm
    # diameter, ITTF spec) is comfortably inside the graspable range and
    # reuses the same sphere/top-grasp pattern as foam_ball, just correctly
    # sized rather than reusing foam_ball's own (slightly different) numbers.
    "ping_pong_ball": ObjectType(
        name="ping_pong_ball", shape="sphere",
        dimensions=(0.02,),   # 2cm radius (4cm / 40mm diameter, standard ping-pong ball)
        height=0.04, z_offset=0.025,
    ),
}


# =============================================================================
# GRASP STRATEGIES
# =============================================================================

@dataclass
class GraspStrategy:
    approach: str           # "top" or "side"
    grab_z_ratio: float     # fraction of height for grab point
    grab_z_offset: float    # additional z offset
    gripper_close_pos: float
    gripper_effort: float
    approach_offset: float  # distance for approach
    # Base orientation for this grasp type
    qx: float = 1.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 0.0


GRASP_STRATEGIES = {
    "foam_ball": GraspStrategy(
        approach="top", grab_z_ratio=0.8, grab_z_offset=0.01,
        gripper_close_pos=0.5, gripper_effort=20.0,
        approach_offset=0.12,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
    ),
    "apple": GraspStrategy(
        approach="top", grab_z_ratio=0.8, grab_z_offset=0.01,
        gripper_close_pos=0.5, gripper_effort=20.0,
        approach_offset=0.12,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
    ),
    "orange": GraspStrategy(
        approach="top", grab_z_ratio=0.8, grab_z_offset=0.01,
        gripper_close_pos=0.5, gripper_effort=20.0,
        approach_offset=0.12,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
    ),
    # side_grasp_quaternion()'s analytic construction was tried here but
    # measured ~36 degrees off from this hand-tuned quaternion's actual
    # approach-axis bearing even at the reference angle it was captured at
    # (see side_grasp_quaternion's docstring) — reverted to the hand-tuned
    # base quaternion + bearing-delta rotation until that's resolved.
    "coffee_cup": GraspStrategy(
        approach="side", grab_z_ratio=0.5, grab_z_offset=0.0,
        gripper_close_pos=0.55, gripper_effort=25.0,
        approach_offset=0.18,
        qx=0.197, qy=0.689, qz=0.677, qw=0.169,
    ),
    "water_bottle": GraspStrategy(
        approach="side", grab_z_ratio=0.4, grab_z_offset=0.0,
        gripper_close_pos=0.5, gripper_effort=35.0,
        approach_offset=0.18,
        qx=0.197, qy=0.689, qz=0.677, qw=0.169,
    ),
    "small_box": GraspStrategy(
        approach="top", grab_z_ratio=0.8, grab_z_offset=0.01,
        gripper_close_pos=0.6, gripper_effort=30.0,
        approach_offset=0.12,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
    ),
    "default": GraspStrategy(
        approach="top", grab_z_ratio=0.7, grab_z_offset=0.01,
        gripper_close_pos=0.6, gripper_effort=30.0,
        approach_offset=0.12,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
    ),
    # mujoco_coffee_cup: top-approach variant, used ONLY by the MuJoCo-direct
    # path (mujoco_arm_executor.py) -- NOT a replacement for the side-grasp
    # "coffee_cup" entry above, which stays untouched (side grasp is the
    # physically sensible real-world choice for picking up an actual cup, and
    # that quaternion is still the real-hardware-tuned one via MoveIt2). This
    # is a MuJoCo-specific workaround: extensive debugging found the 6D pose
    # IK needed to hit the side-grasp orientation reliably doesn't converge
    # (see mujoco_ik.py's docstring -- settles at ~pi orientation error, root
    # cause not isolated), and position-only IK combined with a SIDE approach
    # was found live to cause the gripper to broadside the object (uncontrolled
    # orientation + a horizontal approach = collision, not a clean slide-in).
    # A top approach is far more tolerant of imperfect orientation (gripper
    # descends roughly along Z regardless of exact wrist roll), so it works
    # with position-only IK where the side approach didn't.
    #
    # gripper_close_pos=0.27, NOT the 0.55 an earlier version of this entry
    # used -- measured directly from the MJCF geometry (see
    # dual_arm_coordinator.py / conversation) that this gripper's fingertip
    # gap at close_pos=0.55 is only 59.9mm, LESS than the cup's own 90mm
    # diameter -- i.e. that value commanded the gripper to close 30mm THROUGH
    # the cup, which a plain position controller with no effort limit resolves
    # by generating an ever-increasing squeeze force against the object once
    # blocked by contact, a real and independent contributor to the violent
    # launches seen this session (the same failure pattern documented in
    # github.com/moveit/mujoco_ros2_control/issues/40: "pure position control
    # without effort limits ... will squeeze the boxes until they jump around/
    # away"). 0.27 gives an ~87.5mm gap -- a firm ~2.5mm compliant squeeze
    # against the actual 90mm object, not an impossible over-closure.
    # qx=0.0, qy=1.0, qz=0.0, qw=0.0, NOT (1,0,0,0) -- both represent the same
    # physical "point straight down" family (differ only by a 180-degree roll
    # about the vertical approach axis, which a round cup doesn't care about),
    # but (1,0,0,0) turned out to be a badly-conditioned choice for this arm's
    # kinematics: the 6D solve stalled at an orientation residual near pi
    # (traced to a real bug, since fixed -- see mujoco_ik.py's solve_position_ik,
    # mju_subQuat's local-frame vector was being combined with mj_jac's
    # world-frame Jacobian), and even after that fix, converged only to ~0.1-0.2
    # rad residual with armA_joint_5 pinned at its +-2.53 rad limit. A roll
    # sweep (0-330 degrees in 30-degree steps) against the corrected solver
    # found roll=150-270 degrees all converge cleanly with joint_5 well clear
    # of its limit; 180 degrees (this value) sits in the middle of that safe
    # range. Verified for both approach_pose and grasp_pose at
    # CUP_PICKUP_POSITION with the corrected solver before being adopted here.
    # gripper_close_pos=0.6 (was 0.5) -- with the incremental-close +
    # per-step settle fix (mujoco_arm_executor.py's pick()) and the
    # cup-vs-table contact softness bug fixed (scene_dual_arm.xml), the
    # descend+close sequence now disturbs the cup by only ~3mm total, down
    # from multi-cm/violent-launch territory -- there's real headroom to
    # squeeze harder without risk. Measured (with the finger equality
    # coupling correctly applied): close_pos=0.6 gives ~54.9mm gap against
    # this 70mm cup, ~7.5mm of compliant squeeze each side (was ~2.6mm at
    # 0.5) -- picked because 0.5's grip wasn't generating enough normal
    # force/friction to hold the cup's weight through the lift (arm lifted
    # clean, cup stayed on the table, Z unchanged).
    # grab_z_ratio=0.45 (was 0.7, near the rim) -- gripping close to the rim
    # leaves nothing below the pinch to stop the fingers sliding UP and off
    # the cup during any vertical lift force; the cup was never actually
    # secured through many other fixes (contact softness, incremental
    # close, velocity settling), so trying a lower grip point with cylinder
    # wall both above and below the pinch for a more mechanically stable
    # vertical hold.
    "mujoco_coffee_cup": GraspStrategy(
        approach="top", grab_z_ratio=0.45, grab_z_offset=0.01,
        gripper_close_pos=0.6, gripper_effort=25.0,
        approach_offset=0.12,
        qx=0.0, qy=1.0, qz=0.0, qw=0.0,
    ),
    # ping_pong_ball: same top-grasp pattern as foam_ball/apple/orange, but
    # with gripper_close_pos computed the same measured way as
    # mujoco_coffee_cup above rather than copied from a similarly-sized
    # entry -- close_pos=0.77 gives an ~38mm fingertip gap against the ball's
    # actual 40mm diameter (a firm ~2mm squeeze), comfortably inside this
    # gripper's ~30.4mm mechanical minimum closed gap.
    "ping_pong_ball": GraspStrategy(
        approach="top", grab_z_ratio=0.8, grab_z_offset=0.01,
        gripper_close_pos=0.77, gripper_effort=15.0,
        approach_offset=0.12,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
    ),
}

# =============================================================================
# PLACE STRATEGIES — how to set down each object type
# =============================================================================

@dataclass
class PlaceStrategy:
    """Defines how to place an object down."""
    place_z_ratio: float       # fraction of object height for release point
    place_z_offset: float      # additional z offset above surface
    release_retreat: float     # how high to retreat after releasing

PLACE_STRATEGIES = {
    "foam_ball": PlaceStrategy(
        place_z_ratio=0.8,      # lower to near-surface
        place_z_offset=0.01,    # tiny clearance
        release_retreat=0.10,
    ),
    "apple": PlaceStrategy(
        place_z_ratio=0.8,
        place_z_offset=0.01,
        release_retreat=0.10,
    ),
    "orange": PlaceStrategy(
        place_z_ratio=0.8,
        place_z_offset=0.01,
        release_retreat=0.10,
    ),
    "coffee_cup": PlaceStrategy(
        place_z_ratio=0.1,      # set down near bottom
        place_z_offset=0.01,
        release_retreat=0.15,
    ),
    "water_bottle": PlaceStrategy(
        place_z_ratio=0.1,
        place_z_offset=0.01,
        release_retreat=0.15,
    ),
    "small_box": PlaceStrategy(
        place_z_ratio=0.5,
        place_z_offset=0.01,
        release_retreat=0.10,
    ),
    "default": PlaceStrategy(
        place_z_ratio=0.5,
        place_z_offset=0.02,
        release_retreat=0.12,
    ),
    # mujoco_coffee_cup / ping_pong_ball: place_z_ratio matches these types'
    # GRASP_STRATEGIES grab_z_ratio (0.7 / 0.8) rather than coffee_cup's 0.1 --
    # for a TOP grasp the gripper holds near the object's upper portion the
    # whole time it's carried, so releasing at that same relative height means
    # ~zero drop distance when set down (natural, matches how it's actually
    # being held), instead of assuming a near-the-base release point that only
    # made sense for the side-grasp's different hold geometry.
    "mujoco_coffee_cup": PlaceStrategy(
        place_z_ratio=0.7, place_z_offset=0.01, release_retreat=0.12,
    ),
    "ping_pong_ball": PlaceStrategy(
        place_z_ratio=0.8, place_z_offset=0.01, release_retreat=0.10,
    ),
}


def compute_place_pose(obj_data: dict, place_x: float, place_y: float,
                        place_z_above_table: float = 0.0) -> tuple:
    """
    Compute place and pre-place poses that mirror the grasp orientation.
    Returns (pre_place_pose, place_pose, retreat_pose).
    """
    obj_type_name = obj_data.get("type", "default")
    grasp_strategy = GRASP_STRATEGIES.get(obj_type_name, GRASP_STRATEGIES["default"])
    place_strategy = PLACE_STRATEGIES.get(obj_type_name, PLACE_STRATEGIES["default"])
    obj_type = OBJECT_TYPES.get(obj_type_name)

    # Compute the orientation for placing (same convention as the grasp)
    angle = math.atan2(place_y, place_x)
    base_quat = (grasp_strategy.qx, grasp_strategy.qy,
                 grasp_strategy.qz, grasp_strategy.qw)
    base_angle = math.atan2(0.191, 0.423)
    z_rot = quat_from_z_rotation(angle - base_angle)
    oriented_quat = quat_multiply(z_rot, base_quat)

    # Compute place height
    if obj_type:
        place_height = (
            place_z_above_table
            + obj_type.height * place_strategy.place_z_ratio
            + place_strategy.place_z_offset
        )
    else:
        place_height = place_z_above_table + place_strategy.place_z_offset

    # Apply minimum safety
    min_z = (TOP_GRASP_MIN_Z_ABOVE_TABLE if grasp_strategy.approach == "top"
             else SIDE_GRASP_MIN_Z_ABOVE_TABLE)
    place_height = max(place_height, min_z)
    place_z = table_z_to_base_z(place_height)

    # Place pose — where to release
    place_pose = TargetPose(
        x=place_x, y=place_y, z=place_z,
        qx=oriented_quat[0], qy=oriented_quat[1],
        qz=oriented_quat[2], qw=oriented_quat[3],
    )

    # Pre-place — above the place location (approach from above for top, pulled back for side)
    if grasp_strategy.approach == "top":
        pre_place_pose = TargetPose(
            x=place_x, y=place_y,
            z=place_z + 0.12,
            qx=place_pose.qx, qy=place_pose.qy,
            qz=place_pose.qz, qw=place_pose.qw,
        )
    else:
        pre_place_pose = TargetPose(
            x=place_x - 0.12 * math.cos(angle),
            y=place_y - 0.12 * math.sin(angle),
            z=max(place_z + 0.03, table_z_to_base_z(APPROACH_MIN_Z_ABOVE_TABLE)),
            qx=place_pose.qx, qy=place_pose.qy,
            qz=place_pose.qz, qw=place_pose.qw,
        )

    # Retreat — straight up/back after releasing
    retreat_pose = TargetPose(
        x=place_x, y=place_y,
        z=place_z + place_strategy.release_retreat + 0.05,
        qx=place_pose.qx, qy=place_pose.qy,
        qz=place_pose.qz, qw=place_pose.qw,
    )

    return pre_place_pose, place_pose, retreat_pose


# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class TargetPose:
    x: float
    y: float
    z: float
    qx: float = 1.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 0.0


# =============================================================================
# HELPERS
# =============================================================================

def table_z_to_base_z(z_above_table: float) -> float:
    return TABLE_SURFACE_Z + z_above_table


def quat_multiply(q1, q2):
    """Multiply two quaternions (x,y,z,w format)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def quat_from_z_rotation(angle):
    """Create quaternion for rotation around Z axis."""
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def _matrix_to_quat(m00, m01, m02, m10, m11, m12, m20, m21, m22):
    """Rotation matrix (row-major) -> quaternion (x, y, z, w).

    Numerically stable (Shepperd's method): picks whichever of the trace
    and the three diagonal entries is largest as the basis for the sqrt,
    avoiding division by a near-zero term at any orientation.
    """
    trace = m00 + m11 + m22
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return ((m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s, 0.25 / s)
    if m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)


def side_grasp_quaternion(angle: float):
    """
    NOT CURRENTLY USED — see the comment on GRASP_STRATEGIES["coffee_cup"].

    Orientation for a horizontal side-grasp aimed at world bearing `angle`
    (radians, atan2(obj_y, obj_x)), built directly from the required
    geometry instead of rotating a single hand-measured sample pose around
    world Z. That approach baked in whatever roll/tilt error was present in
    the one captured configuration, and got worse the further the bearing
    angle was rotated from wherever that sample happened to be taken.

    tool_frame's local +Z is assumed to be the approach axis (from the
    working top-grasp quaternion (1,0,0,0), which points local Z straight
    down) and local +Y the finger "up/spread" axis. This constructs a
    right-handed frame:
      - local +Z (approach) -> horizontal, pointing at `angle`
      - local +Y (up)       -> world +Z, so the gripper never rolls
      - local +X            -> completes the frame (finger-closing axis)

    Decomposing the old hand-tuned coffee_cup quaternion confirmed the
    local-Y-to-world-Z assumption (99.9% aligned), but its local-Z bearing
    was ~36 degrees off from the object bearing it was captured at — so the
    approach-axis-to-bearing mapping here is wrong in some way not yet
    identified (only one hardware sample to check against). Needs a second
    real capture at a different bearing angle before this is trustworthy.
    """
    c, s = math.cos(angle), math.sin(angle)
    return _matrix_to_quat(
        -s, 0.0, c,
        c, 0.0, s,
        0.0, 1.0, 0.0,
    )


def compute_grasp_and_approach(obj_data: dict):
    """
    Compute grasp and approach poses with proper orientation.
    Returns (approach_pose, grasp_pose, strategy).
    """
    obj_type_name = obj_data.get("type", "default")
    strategy = GRASP_STRATEGIES.get(obj_type_name, GRASP_STRATEGIES["default"])
    obj_type = OBJECT_TYPES.get(obj_type_name)

    obj_x = obj_data["x"]
    obj_y = obj_data["y"]
    obj_z_above_table = obj_data.get("z", 0.0)

    # Compute grab height.
    # obj_z_above_table from overhead cameras is approximately the visible
    # top surface of the object. Subtract the full object height to estimate
    # the base (clamped at 0), then apply the ratio up from the base.
    if obj_type:
        estimated_base = max(0.0, obj_z_above_table - obj_type.height)
        grab_height = (
            estimated_base
            + obj_type.height * strategy.grab_z_ratio
            + strategy.grab_z_offset
        )
    else:
        grab_height = obj_z_above_table + strategy.grab_z_offset

    # Apply minimum safety heights
    min_z = (TOP_GRASP_MIN_Z_ABOVE_TABLE if strategy.approach == "top"
             else SIDE_GRASP_MIN_Z_ABOVE_TABLE)
    grab_height = max(grab_height, min_z)
    grab_z = table_z_to_base_z(grab_height)

    # Orient gripper relative to where the object sits in the workspace.
    # base_quat was hand-tuned at angle = atan2(0.191, 0.423), so we apply
    # the delta rotation between the current angle and that reference.
    base_quat = (strategy.qx, strategy.qy, strategy.qz, strategy.qw)
    angle = math.atan2(obj_y, obj_x)
    base_angle = math.atan2(0.191, 0.423)  # ~0.424 rad

    z_rot = quat_from_z_rotation(angle - base_angle)
    oriented_quat = quat_multiply(z_rot, base_quat)

    grasp_pose = TargetPose(
        x=obj_x, y=obj_y, z=grab_z,
        qx=oriented_quat[0], qy=oriented_quat[1],
        qz=oriented_quat[2], qw=oriented_quat[3],
    )

    # Compute approach pose
    if strategy.approach == "top":
        approach_z = grab_z + strategy.approach_offset
        approach_pose = TargetPose(
            x=obj_x, y=obj_y, z=approach_z,
            qx=grasp_pose.qx, qy=grasp_pose.qy,
            qz=grasp_pose.qz, qw=grasp_pose.qw,
        )
    else:
        # Side approach: pull back along line from robot to object
        approach_z = max(
            grab_z + 0.03,
            table_z_to_base_z(APPROACH_MIN_Z_ABOVE_TABLE),
        )
        approach_pose = TargetPose(
            x=obj_x - strategy.approach_offset * math.cos(angle),
            y=obj_y - strategy.approach_offset * math.sin(angle),
            z=approach_z,
            qx=grasp_pose.qx, qy=grasp_pose.qy,
            qz=grasp_pose.qz, qw=grasp_pose.qw,
        )

    return approach_pose, grasp_pose, strategy


# =============================================================================
# PLANNER NODE
# =============================================================================

class KinovaPickPlanner(Node):
    def __init__(self):
        super().__init__("kinova_pick_planner")
        self.get_logger().info("Initializing Kinova Pick Planner (pymoveit2)...")

        self.cb_group = ReentrantCallbackGroup()

        # MoveIt2 arm interface
        self.arm = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=ARM_GROUP,
            callback_group=self.cb_group,
        )

        # Gripper interface (kept around for compatibility; we drive the
        # gripper through the direct action client below because pymoveit2's
        # GripperInterface.move_to_position() does not reliably complete on
        # small moves — wait_until_executed() can hang on the partial-close
        # commands we use during Step 1.5 / Step 2.5).
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=GRIPPER_JOINTS,
            open_gripper_joint_positions=[0.0],
            closed_gripper_joint_positions=[0.8],
            gripper_group_name=GRIPPER_GROUP,
            callback_group=self.cb_group,
            gripper_command_action_name=GRIPPER_ACTION_NAME,
        )

        # Direct gripper action client — primary control path
        self.gripper_action = ActionClient(
            self, GripperCommand,
            GRIPPER_ACTION_NAME,
            callback_group=self.cb_group,
        )

        # Configure motion parameters
        self.arm.max_velocity = MAX_VELOCITY
        self.arm.max_acceleration = MAX_ACCELERATION
        self.arm.num_planning_attempts = PLANNING_ATTEMPTS
        self.arm.allowed_planning_time = PLANNING_TIME

        # moveit_msgs/GetCartesianPath's avoid_collisions field has no
        # default, so it zero-inits to False, and pymoveit2's Cartesian
        # planning path never sets it — only jump_threshold (a joint-space
        # discontinuity check, not a collision check) can truncate a
        # Cartesian plan unless this is explicitly enabled. Without this,
        # compute_cartesian_path has been free to plan straight through the
        # table/collision objects.
        self.arm.cartesian_avoid_collisions = True
        self.arm.cartesian_revolute_jump_threshold = CARTESIAN_REVOLUTE_JUMP_THRESHOLD

        # Track collision objects
        self._collision_ids = set()

        # Home joints (set by arm_controller)
        self._home_joints = {}

        self.get_logger().info("Kinova Pick Planner ready!")

    # -------------------------------------------------------------------------
    # Collision Objects
    # -------------------------------------------------------------------------

    def add_table(self):
        """Add table and keepout zone to planning scene."""
        # Main table
        self.arm.add_collision_box(
            id="table",
            size=(TABLE_POSITION["width"], TABLE_POSITION["length"],
                  TABLE_POSITION["thickness"]),
            position=(TABLE_POSITION["x"], TABLE_POSITION["y"],
                      TABLE_SURFACE_Z - TABLE_POSITION["thickness"] / 2),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        self._collision_ids.add("table")

        # Keepout zone above table — 5 cm slab so open finger tips can't
        # clip the table surface or edge during joint-space transit moves.
        self.arm.add_collision_box(
            id="table_keepout",
            size=(TABLE_POSITION["width"], TABLE_POSITION["length"], 0.05),
            position=(TABLE_POSITION["x"], TABLE_POSITION["y"],
                      TABLE_SURFACE_Z + 0.025),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        self._collision_ids.add("table_keepout")
        self.get_logger().info("Table added to planning scene")

    def add_other_arm_keepout(self):
        """Add a static collision box for the other arm's approximate footprint.

        See OTHER_ARM_BASE_DISTANCE_X / OTHER_ARM_KEEPOUT above — placeholder
        dimensions until the physical rig exists to measure the real envelope.
        """
        self.arm.add_collision_box(
            id="other_arm_keepout",
            size=(OTHER_ARM_KEEPOUT["depth"], OTHER_ARM_KEEPOUT["width"],
                  OTHER_ARM_KEEPOUT["height"]),
            position=(OTHER_ARM_BASE_DISTANCE_X, 0.0,
                      TABLE_SURFACE_Z + OTHER_ARM_KEEPOUT["height"] / 2),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        self._collision_ids.add("other_arm_keepout")
        self.get_logger().info("Other-arm keepout zone added to planning scene")

    def add_collision_object(self, obj_id: str, obj_type_name: str,
                              x: float, y: float, z_above_table: float,
                              padding: float = 0.015):
        """Add an object collision mesh to the planning scene."""
        obj_type = OBJECT_TYPES.get(obj_type_name)
        if obj_type is None:
            self.get_logger().warn(f"Unknown type '{obj_type_name}', using small_box")
            obj_type = OBJECT_TYPES["small_box"]

        center_z = table_z_to_base_z(z_above_table) + obj_type.z_offset

        if obj_type.shape == "sphere":
            r = obj_type.dimensions[0] + padding
            # pymoveit2 doesn't have add_collision_sphere, use a small box approximation
            d = r * 2
            self.arm.add_collision_box(
                id=obj_id,
                size=(d, d, d),
                position=(x, y, center_z),
                quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            )
        elif obj_type.shape == "cylinder":
            h = obj_type.dimensions[0] + padding
            r = obj_type.dimensions[1] + padding
            self.arm.add_collision_cylinder(
                id=obj_id,
                height=h, radius=r,
                position=(x, y, center_z),
                quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            )
        elif obj_type.shape == "box":
            dims = tuple(d + padding for d in obj_type.dimensions)
            self.arm.add_collision_box(
                id=obj_id,
                size=dims,
                position=(x, y, center_z),
                quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            )

        self._collision_ids.add(obj_id)
        self.get_logger().info(
            f"Added '{obj_id}' at ({x:.3f}, {y:.3f}, base_z={center_z:.3f})"
        )

    def remove_collision_object(self, obj_id: str):
        """Remove a collision object."""
        try:
            self.arm.remove_collision_object(obj_id)
            self._collision_ids.discard(obj_id)
        except Exception:
            pass

    def remove_keepout(self):
        """Temporarily remove table keepout for low moves."""
        self.remove_collision_object("table_keepout")

    def restore_keepout(self):
        """Re-add table keepout after lift."""
        self.arm.add_collision_box(
            id="table_keepout",
            size=(TABLE_POSITION["width"], TABLE_POSITION["length"], 0.05),
            position=(TABLE_POSITION["x"], TABLE_POSITION["y"],
                      TABLE_SURFACE_Z + 0.025),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        self._collision_ids.add("table_keepout")

    # -------------------------------------------------------------------------
    # Gripper Control (direct action client)
    # -------------------------------------------------------------------------

    def _send_gripper_direct(self, position: float,
                              max_effort: float = GRIPPER_DEFAULT_EFFORT) -> bool:
        """
        Send a GripperCommand goal directly to the action server.

        Bypasses pymoveit2's GripperInterface, which can hang on
        wait_until_executed() for small position changes.
        """
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        self.get_logger().info(
            f"Gripper direct: pos={position:.2f} effort={max_effort:.1f}"
        )
        # Fire-and-forget: we don't await the future. The settle sleep below
        # gives the gripper time to physically execute the command.
        self.gripper_action.send_goal_async(goal)
        time.sleep(GRIPPER_SETTLE_SEC)
        return True

    def open_gripper(self) -> bool:
        self.get_logger().info("Opening gripper")
        return self._send_gripper_direct(0.0)

    def close_gripper(self, position: float = 0.6,
                       effort: float = GRIPPER_DEFAULT_EFFORT) -> bool:
        self.get_logger().info(f"Closing gripper to {position:.2f}")
        return self._send_gripper_direct(position, max_effort=effort)

    def partial_close_gripper(self, fraction: float = 0.3) -> bool:
        """Partially close for narrow approach. fraction in [0.0, 1.0]."""
        pos = 0.8 * fraction
        self.get_logger().info(f"Narrowing gripper to {pos:.2f}")
        return self._send_gripper_direct(pos)

    # -------------------------------------------------------------------------
    # Motion: Free space (joint-space planning)
    # -------------------------------------------------------------------------

    def _move_to_pose_plain(self, target: TargetPose) -> bool:
        """Joint-space planning to `target`, with retries but WITHOUT the
        wiggle/home-reset recovery in move_to_pose(). Shared by move_to_pose()
        and by move_joint_waypoints()'s per-hop calls, where a home-reset
        mid-sequence would defeat the purpose of hopping (e.g. mid-grasp or
        while holding a grasped object)."""
        for attempt in range(MAX_RETRIES):
            if attempt > 0:
                self.get_logger().info(f"Retry {attempt}...")
                time.sleep(RETRY_DELAY)

            self.get_logger().info(
                f"Moving to: ({target.x:.3f}, {target.y:.3f}, {target.z:.3f})"
            )

            self.arm.move_to_pose(
                position=(target.x, target.y, target.z),
                quat_xyzw=(target.qx, target.qy, target.qz, target.qw),
                cartesian=False,
            )

            # Joint-space planning reports failure directly (no known silent-
            # truncation case like move_cartesian's compute_cartesian_path),
            # so we trust wait_until_executed() here rather than re-verifying
            # via FK.
            if self.arm.wait_until_executed():
                self.get_logger().info("Move succeeded!")
                time.sleep(0.3)
                return True
            else:
                self.get_logger().error(f"Move failed (attempt {attempt + 1})")

        return False

    def move_to_pose(self, target: TargetPose) -> bool:
        """Move to pose using joint-space planning (free path), with
        wiggle/home-reset recovery if the plain attempts all fail."""
        if self._move_to_pose_plain(target):
            return True

        # Wiggle recovery
        return self._wiggle_and_retry(target)

    @staticmethod
    def _interpolate_hops(start: tuple, target: "TargetPose", step_size: float):
        """Break a straight pull from `start` to `target` into hops of
        roughly `step_size` length, holding target's orientation fixed
        throughout. Shared by move_cartesian_waypoints() and
        move_joint_waypoints() so both hop through identical geometry —
        only how each hop is planned (Cartesian vs. joint-space) differs."""
        total_dist = math.dist(start, (target.x, target.y, target.z))
        n_segments = max(1, math.ceil(total_dist / step_size))
        hops = []
        for i in range(1, n_segments + 1):
            frac = i / n_segments
            hops.append(TargetPose(
                x=start[0] + frac * (target.x - start[0]),
                y=start[1] + frac * (target.y - start[1]),
                z=start[2] + frac * (target.z - start[2]),
                qx=target.qx, qy=target.qy, qz=target.qz, qw=target.qw,
            ))
        return hops

    # -------------------------------------------------------------------------
    # LEGACY: Cartesian motion (kept for reference / re-enabling later).
    #
    # Not currently called anywhere in arm_controller.py — side grasps were
    # unreliable here (compute_cartesian_path failing outright, plus the
    # non-zero CARTESIAN_REVOLUTE_JUMP_THRESHOLD rejecting hops that land in
    # a different IK configuration branch than the previous hop). Replaced
    # by move_joint_waypoints() below, which hops through the same waypoint
    # geometry but plans each hop in joint space instead.
    # -------------------------------------------------------------------------

    def move_cartesian(self, target: TargetPose) -> bool:
        """Move in straight Cartesian line. Maintains orientation."""
        for attempt in range(MAX_RETRIES):
            if attempt > 0:
                self.get_logger().info(f"Cartesian retry {attempt}...")
                time.sleep(RETRY_DELAY)

            self.get_logger().info(
                f"Cartesian to: ({target.x:.3f}, {target.y:.3f}, {target.z:.3f})"
            )
            self._log_joint_angles("Cartesian start config")

            self.arm.move_to_pose(
                position=(target.x, target.y, target.z),
                quat_xyzw=(target.qx, target.qy, target.qz, target.qw),
                cartesian=True,
                cartesian_max_step=0.005,
                cartesian_fraction_threshold=CARTESIAN_FRACTION_THRESHOLD,
            )

            if self.arm.wait_until_executed() and self._verify_reached(target, "Cartesian move"):
                self.get_logger().info("Cartesian move succeeded!")
                time.sleep(0.3)
                return True
            else:
                self.get_logger().error(f"Cartesian move failed (attempt {attempt + 1})")

        self.get_logger().error("Cartesian move failed after retries")
        return False

    def move_cartesian_waypoints(self, target: TargetPose,
                                  step_size: float = CARTESIAN_WAYPOINT_STEP) -> bool:
        """Straight-line Cartesian move, broken into short hops.

        Same fixed-orientation guarantee as move_cartesian(), but chains
        several short segments instead of one long compute_cartesian_path()
        call. Each hop re-seeds IK from the arm's actual achieved pose, so a
        singularity/joint-limit that would abort one 15-20cm call near its
        first waypoint is far less likely to block a 3cm one.
        """
        start = self._achieved_position()
        if start is None:
            self.get_logger().warn(
                "move_cartesian_waypoints: FK unavailable, "
                "falling back to single-shot cartesian"
            )
            return self.move_cartesian(target)

        hops = self._interpolate_hops(start, target, step_size)
        for i, waypoint in enumerate(hops, start=1):
            self.get_logger().info(
                f"Cartesian waypoint {i}/{len(hops)} -> "
                f"({waypoint.x:.3f}, {waypoint.y:.3f}, {waypoint.z:.3f})"
            )
            if not self.move_cartesian(waypoint):
                self.get_logger().error(
                    f"Cartesian waypoint {i}/{len(hops)} failed, "
                    "aborting waypoint sequence"
                )
                return False

        return True

    # -------------------------------------------------------------------------
    # Motion: Joint-space waypoints (Cartesian replacement)
    #
    # Currently the active precision-move path in arm_controller.py, used for
    # approach->grasp, grasp->lift, place, and retreat. Hops through the same
    # waypoint geometry as move_cartesian_waypoints() above, but each hop is
    # solved with ordinary joint-space (OMPL) planning instead of
    # compute_cartesian_path.
    #
    # Joint-space planning is collision-checked against every collision
    # object currently in the scene (table, keepout slab, other objects) as a
    # fundamental part of the planner, not an opt-in flag like Cartesian's
    # avoid_collisions/fraction-threshold/jump-threshold — so it can't
    # silently truncate or land in a rejected configuration branch the way
    # compute_cartesian_path did for side grasps. Chaining short hops (rather
    # than one long free-space plan straight to the final pose) keeps the
    # path close to a straight line and keeps each hop's plan short/local
    # instead of being a totally unconstrained reroute.
    # -------------------------------------------------------------------------

    def move_joint(self, target: TargetPose) -> bool:
        """Single-shot joint-space plan straight to `target` — no Cartesian,
        no waypoint hopping. One continuous MoveIt trajectory (retried up to
        MAX_RETRIES on failure, no wiggle/home-reset — see move_to_pose() for
        that).

        Use this over move_joint_waypoints() when the hop-by-hop version's
        stop-plan-execute cycle per ~3cm segment produces jerky, high-jerk
        motion: OMPL's sampling-based planners aren't built for repeatedly
        re-planned micro-moves the way Cartesian's linear interpolation is,
        so chaining many short joint-space hops can trade one problem
        (Cartesian planning failures) for another (a stutter of hard stops
        that can trip the arm's motion-controller safety limits).
        """
        return self._move_to_pose_plain(target)

    def move_joint_waypoints(self, target: TargetPose,
                              step_size: float = CARTESIAN_WAYPOINT_STEP) -> bool:
        """Straight-line-ish move to `target` via joint-space planning,
        broken into the same short hops as move_cartesian_waypoints().

        No wiggle/home-reset recovery per hop (see _move_to_pose_plain) —
        a failed hop aborts the whole sequence rather than rerouting through
        an unconstrained recovery move.

        Prefer move_joint() (single-shot, no hopping) unless you specifically
        need the path kept close to a straight line — see move_joint()'s
        docstring for why hopping can produce jerkier motion than Cartesian
        hopping did.
        """
        start = self._achieved_position()
        if start is None:
            self.get_logger().warn(
                "move_joint_waypoints: FK unavailable, "
                "falling back to single-shot joint-space move"
            )
            return self._move_to_pose_plain(target)

        hops = self._interpolate_hops(start, target, step_size)
        for i, waypoint in enumerate(hops, start=1):
            self.get_logger().info(
                f"Joint waypoint {i}/{len(hops)} -> "
                f"({waypoint.x:.3f}, {waypoint.y:.3f}, {waypoint.z:.3f})"
            )
            if not self._move_to_pose_plain(waypoint):
                self.get_logger().error(
                    f"Joint waypoint {i}/{len(hops)} failed, "
                    "aborting waypoint sequence"
                )
                return False

        return True

    def move_to_joints(self, positions: dict) -> bool:
        """Move to joint-space target."""
        joint_list = [positions.get(j, 0.0) for j in ARM_JOINTS]

        for attempt in range(MAX_RETRIES):
            if attempt > 0:
                time.sleep(RETRY_DELAY)

            self.get_logger().info("Moving to joint target...")
            self.arm.move_to_configuration(joint_positions=joint_list)

            if self.arm.wait_until_executed():
                self.get_logger().info("Joint move succeeded!")
                time.sleep(0.3)
                return True
            else:
                self.get_logger().error(f"Joint move failed (attempt {attempt + 1})")

        return False

    def move_to_named_pose(self, pose_name: str) -> bool:
        """Move to a predefined named pose."""
        poses = {
            "home": [0.0, -0.2814, 1.3161, -0.0027, -1.0479, 0.0],
            "vertical": [0.0, 0.0, 3.14, 0.0, 0.0, 0.0],
        }
        if pose_name not in poses:
            self.get_logger().error(f"Unknown pose: {pose_name}")
            return False

        self.arm.move_to_configuration(joint_positions=poses[pose_name])
        return self.arm.wait_until_executed()

    # -------------------------------------------------------------------------
    # Wiggle Recovery
    # -------------------------------------------------------------------------

    def _wiggle_and_retry(self, target: TargetPose) -> bool:
        """Nudge joints then retry. Falls back to home reset."""
        joint_state = self.arm.joint_state
        if joint_state is None:
            return False

        current = {}
        for name, pos in zip(joint_state.name, joint_state.position):
            if name in ARM_JOINTS:
                current[name] = pos

        # Phase 1: Small wiggles
        for i in range(2):
            nudged = {}
            for j in ARM_JOINTS:
                if j in current:
                    offset = random.uniform(-0.15, 0.15) if j in ("joint_1", "joint_4") else random.uniform(-0.08, 0.08)
                    nudged[j] = current[j] + offset

            self.get_logger().info(f"Wiggle {i + 1}/2...")
            if self.move_to_joints(nudged):
                time.sleep(0.5)
                self.arm.move_to_pose(
                    position=(target.x, target.y, target.z),
                    quat_xyzw=(target.qx, target.qy, target.qz, target.qw),
                    cartesian=False,
                )
                if self.arm.wait_until_executed():
                    self.get_logger().info("Wiggle recovery succeeded!")
                    return True

        # Phase 2: Home reset
        if self._home_joints:
            self.get_logger().warn("Wiggles failed, going home...")
            if self.move_to_joints(self._home_joints):
                time.sleep(1.0)
                self.arm.move_to_pose(
                    position=(target.x, target.y, target.z),
                    quat_xyzw=(target.qx, target.qy, target.qz, target.qw),
                    cartesian=False,
                )
                if self.arm.wait_until_executed():
                    self.get_logger().info("Home reset recovery succeeded!")
                    return True

        self.get_logger().error("All recovery attempts failed")
        return False

    # -------------------------------------------------------------------------
    # Get current pose
    # -------------------------------------------------------------------------

    def get_current_pose(self):
        """Get current end effector pose."""
        return self.arm.compute_fk()

    def _log_joint_angles(self, label: str):
        """Log current arm joint angles. Diagnostic for Cartesian failures:
        lets us see whether a hop that fails at ~0% fraction starts from a
        joint configuration sitting near a limit/singularity for the
        commanded orientation, vs. one that fails partway through a
        genuinely long pull."""
        joint_state = self.arm.joint_state
        if joint_state is None:
            return
        angles = {}
        for name, pos in zip(joint_state.name, joint_state.position):
            if name in ARM_JOINTS:
                angles[name] = pos
        ordered = ", ".join(f"{j}={angles[j]:.3f}" for j in ARM_JOINTS if j in angles)
        self.get_logger().info(f"{label}: {ordered}")

    def _achieved_position(self):
        """Defensively extract (x, y, z) from compute_fk(); None if unavailable."""
        fk = self.get_current_pose()
        if fk is None:
            return None
        if isinstance(fk, (list, tuple)):
            fk = fk[-1] if fk else None
            if fk is None:
                return None
        pose = getattr(fk, "pose", fk)  # PoseStamped -> Pose, or already Pose
        pos = getattr(pose, "position", None)
        if pos is None:
            return None
        return (pos.x, pos.y, pos.z)

    def _verify_reached(self, target: TargetPose, label: str) -> bool:
        """Confirm the arm actually reached target, not just that MoveIt's
        execution action reported success (which can be true for a
        truncated/partial trajectory)."""
        actual = self._achieved_position()
        if actual is None:
            self.get_logger().warn(
                f"{label}: could not verify pose (FK unavailable), trusting executor"
            )
            return True  # fail-open: don't regress behavior if FK is flaky

        err = math.dist((target.x, target.y, target.z), actual)
        if err > CARTESIAN_POSITION_TOLERANCE:
            self.get_logger().error(
                f"{label} under-shot target: commanded "
                f"({target.x:.3f}, {target.y:.3f}, {target.z:.3f}), "
                f"actual ({actual[0]:.3f}, {actual[1]:.3f}, {actual[2]:.3f}), "
                f"error={err:.3f}m"
            )
            return False
        return True


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
    time.sleep(2.0)

    try:
        node.add_table()
        time.sleep(0.5)

        node.open_gripper()
        time.sleep(0.5)

        ball = {"x": 0.40, "y": 0.05, "z": 0.0, "type": "foam_ball"}
        approach, grasp, strategy = compute_grasp_and_approach(ball)

        node.get_logger().info(
            f"Approach: ({approach.x:.3f}, {approach.y:.3f}, {approach.z:.3f})"
        )
        node.get_logger().info(
            f"Grasp: ({grasp.x:.3f}, {grasp.y:.3f}, {grasp.z:.3f})"
        )

        # Approach
        node.move_to_pose(approach)

        # Cartesian to grasp
        node.move_cartesian(grasp)

        # Grip
        node.close_gripper(position=strategy.gripper_close_pos)
        time.sleep(0.5)

        # Lift
        node.remove_keepout()
        lift = TargetPose(
            x=grasp.x, y=grasp.y, z=grasp.z + 0.20,
            qx=grasp.qx, qy=grasp.qy, qz=grasp.qz, qw=grasp.qw,
        )
        node.move_cartesian(lift)
        node.restore_keepout()

        # Home
        node.move_to_named_pose("home")

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    demo_scenario()
