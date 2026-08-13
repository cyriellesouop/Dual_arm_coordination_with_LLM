#!/usr/bin/env python3

#Multi-robot interaction and contact dynamics.
#Replaces simultaneous grasping with a decoupled place-then-pick sequence
"""
Dual-arm cup-handoff + ball-placement task coordinator, for the MuJoCo
evaluation harness (see mujoco_dual_arm_scene.md / the evaluation-harness plan).
Second object is a ping-pong ball, not a straw -- see kinova_pick_planner.py's
"ping_pong_ball" ObjectType entry for why (an 8mm-diameter straw turned out to
be physically impossible for this gripper to pinch-grip at all, a measured
geometric fact, not a tuning problem).

Owns two MujocoArmExecutor instances (armA_/armB_) in ONE node -- both arms
already share one ROS graph / one domain (built this session: flat
armA_/armB_-prefixed controllers, no domain_bridge), so no cross-process or
cross-domain coordination machinery is needed here at all; this node just
calls both executors' methods in sequence.

Task sequence (see kinova_pick_planner.py's "DUAL-ARM CUP HANDOFF + BALL TASK"
constants for the actual positions). Place-then-pick handoff, NOT a true
simultaneous mid-air handoff -- verified by testing that both grippers
converging on the same small cup at once causes a physically unstable
contact/interpenetration blow-up (MuJoCo itself logged "Nan, Inf or huge value
in QACC ... simulation is unstable" during the first live test of this task,
right as Arm B's gripper closed on the cup while Arm A's was still holding it):
  1. Arm A picks the cup from its own side.
  2. Arm A carries it to HANDOFF_POSE and places it down there, then fully
     retreats home.
  3. Arm B picks the cup up from HANDOFF_POSE (an ordinary pick(), same as any
     other pickup position) once Arm A is clear.
  4. Arm B carries the cup to CUP_FINAL_PLACE_POSITION and places it.
  5. Arm A picks the ping-pong ball and places it near the cup's final position.

Run SERIALLY end to end in v1, not with Arm B's carry (step 4) running
concurrently with Arm A's ball pick (step 5) -- true concurrent execution via
asyncio.gather-style scheduling on top of rclpy's executor is possible but
wasn't verified working under this session's time constraints, so it's
deferred rather than shipped unverified. This means the "arm parallelism"
metric mentioned in the evaluation-harness plan will honestly read ~0% in v1
-- a real number, not a placeholder, just not yet exercising true parallelism.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from kinova_pick_planner.mujoco_ik import load_model
from kinova_pick_planner.mujoco_arm_executor import MujocoArmExecutor
from kinova_pick_planner.kinova_pick_planner import (
    OBJECT_TYPES,
    CUP_PICKUP_POSITION,
    BALL_PICKUP_POSITION,
    HANDOFF_POSE,
    CUP_FINAL_PLACE_POSITION,
    BALL_PLACE_TOLERANCE_M,
)

# Matches how the rest of this session's hand-run MuJoCo work has been laid
# out -- see the "Not yet done" note (mujoco_installation_sequence.md) that
# these /tmp paths should eventually move into a real launch file/repo-tracked
# location. Overridable via the MUJOCO_DUAL_ARM_MJCF env var for now.
import os
DEFAULT_MJCF_PATH = '/tmp/mujoco_dual_arm/dual_arm.xml'

# Transforms coordinate parameters into the structured dictionary required by the motion planner (compute_grasp_and_approach).
def _obj_data(position_dict, object_type_name, name):
    """Builds the {"x","y","z","type","name"} dict compute_grasp_and_approach
    expects (kinova_pick_planner.py:545) from this project's other
    "z_above_table" position convention. "z" here is NOT z_above_table -- it's
    "height above table as an overhead camera would see it" (roughly the
    object's full height, for something resting flat on the table); the two
    conventions differ on purpose, see mujoco_arm_executor.py's pick()
    docstring and compute_grasp_and_approach's own body for the exact
    base-height estimation this feeds into."""
    return {
        'name': name,
        'type': object_type_name,
        'x': position_dict['x'],
        'y': position_dict['y'],
        'z': OBJECT_TYPES[object_type_name].height + position_dict.get('z_above_table', 0.0),
    }

#Sets up the single ROS 2 coordinator node and instantiates executor instances.
class DualArmCoordinator(Node):

    def __init__(self, mjcf_path: str = None):
        super().__init__('dual_arm_coordinator')
        mjcf_path = mjcf_path or os.environ.get('MUJOCO_DUAL_ARM_MJCF', DEFAULT_MJCF_PATH)
        self.model, self.data = load_model(mjcf_path) #Loads the unified dual-arm MuJoCo model
        # Instantiates two MujocoArmExecutor instances, prefix 'armA_' and 'armB_' to match the MuJoCo model's controller names.
        self.armA = MujocoArmExecutor(self, 'armA_', self.model, self.data)
        self.armB = MujocoArmExecutor(self, 'armB_', self.model, self.data)

    #The main asynchronous finite-state machine (FSM) orchestrating the complete multi-stage task sequence.
    # Preparation: Calculates task targets for cup and ball locations incorporating fault injection offsets (place_offset_xy).Step 1 (Homing): Arm A & Arm B execute waypointed movements to safe home poses.
    # Step 2 (Arm A Cup Pick): Arm A picks the coffee cup from CUP_PICKUP_POSITION.
    # Step 3 (Arm A Handoff Placement): Arm A places the cup at the intermediate HANDOFF_POSE.
    # Step 4 (Arm A Clear): Arm A moves back to home to clear the workspace.
    # Step 5 (Arm B Cup Pick): Arm B picks up the cup from HANDOFF_POSE.
    # Step 6 (Arm B Final Placement): Arm B carries the cup to CUP_FINAL_PLACE_POSITION.
    # Step 7 (Arm A Ball Manipulation): Arm A picks up the ping-pong ball and places it 6cm adjacent to the final cup location.
    #Step 8 (Final Homing): Arm A and Arm B return to home positions.
    async def run_handoff_task(self, place_offset_xy=(0.0, 0.0)) -> dict:
        """Runs the full sequence once. `place_offset_xy` is where fault
        injection hooks in for misplacement (see fault_injector.py) -- an
        (dx, dy) added to CUP_FINAL_PLACE_POSITION's world target, threaded
        straight through to Arm B's place() call, no separate interface
        needed (compute_place_pose-style parameterization, same idea as the
        arm_controller.py PLACE_POSITION discussion earlier this session).
        Returns a dict summarizing what happened, for run_trials.py to log.
        """
        t_start = time.time()
        # "mujoco_coffee_cup", NOT "coffee_cup" -- the top-approach grasp
        # strategy variant (see GRASP_STRATEGIES in kinova_pick_planner.py),
        # chosen because position-only IK combined with the real side-grasp
        # strategy was found live to broadside the object instead of sliding
        # in cleanly. Same physical dimensions either way (OBJECT_TYPES
        # entries are identical), just a different approach/orientation
        # strategy. "ping_pong_ball" has no such real-hardware/MuJoCo split --
        # it's a MuJoCo-harness-only object with only one entry.
        cup_obj = _obj_data(CUP_PICKUP_POSITION, 'mujoco_coffee_cup', 'cup')
        ball_obj = _obj_data(BALL_PICKUP_POSITION, 'ping_pong_ball', 'ball')
        cup_place_x = CUP_FINAL_PLACE_POSITION['x'] + place_offset_xy[0]
        cup_place_y = CUP_FINAL_PLACE_POSITION['y'] + place_offset_xy[1]
        # Ball's "placement" target: near the cup's actual (possibly
        # misplaced) final position, matching BALL_PLACE_TOLERANCE_M's
        # success criterion (see kinova_pick_planner.py) -- placed adjacent,
        # not on top of, the cup.
        ball_place_x, ball_place_y = cup_place_x + 0.06, cup_place_y

        steps = {}
        log = self.get_logger().info

        # Both arms start the MuJoCo sim at qpos=0 (all joints zero), NOT at
        # HOME_JOINT_ANGLES -- confirmed via /joint_states on a fresh launch.
        # The first real test of this task skipped straight to "approach"
        # from that zero pose, an uncontrolled joint-space swing with no
        # collision awareness that could (and did) sweep through the table/
        # cup area unpredictably depending on exactly where the cup sits --
        # explains why moving the cup farther from the base made the initial
        # disturbance WORSE, not better (different sweep path, different
        # collision point, not actually related to reach distance). Every
        # task run must start from the known HOME_JOINT_ANGLES pose, not
        # whatever qpos the sim happens to be in.
        log('Step: armA go home (from sim start pose)')
        steps['armA_go_home_initial'] = await self.armA.go_home(via_waypoints=4)
        log('Step: armB go home (from sim start pose)')
        steps['armB_go_home_initial'] = await self.armB.go_home(via_waypoints=4)
        if not (steps['armA_go_home_initial'] and steps['armB_go_home_initial']):
            return self._summarize(steps, t_start, aborted_at='initial_go_home')

        # Place-then-pick handoff, NOT a true simultaneous mid-air handoff --
        # verified by testing that both grippers converging on the same small
        # cup at once causes a physically unstable contact/interpenetration
        # blow-up (MuJoCo itself logged "Nan, Inf or huge value in QACC ...
        # simulation is unstable" during the first live test of this task, right
        # as Arm B's gripper closed on the cup while Arm A's was still holding
        # it). Arm A sets the cup down at the handoff spot and fully retreats
        # before Arm B ever approaches -- both arms just reuse the ordinary
        # pick()/place() primitives, no special choreography needed.
        log('Step: armA pick cup')
        steps['armA_pick_cup'] = await self.armA.pick(cup_obj)
        if not steps['armA_pick_cup']:
            return self._summarize(steps, t_start, aborted_at='armA_pick_cup')

        log('Step: armA place cup at handoff pose')
        steps['armA_place_at_handoff'] = await self.armA.place(
            cup_obj, HANDOFF_POSE['x'], HANDOFF_POSE['y'],
            HANDOFF_POSE['z_above_table'], tolerance_m=0.10)
        if not steps['armA_place_at_handoff']:
            return self._summarize(steps, t_start, aborted_at='armA_place_at_handoff')

        log('Step: armA clear home before armB approaches')
        steps['armA_go_home_1'] = await self.armA.go_home()
        if not steps['armA_go_home_1']:
            return self._summarize(steps, t_start, aborted_at='armA_go_home_1')

        log('Step: armB pick cup from handoff pose')
        cup_at_handoff = _obj_data(HANDOFF_POSE, 'mujoco_coffee_cup', 'cup')
        steps['armB_pick_cup'] = await self.armB.pick(cup_at_handoff)
        if not steps['armB_pick_cup']:
            return self._summarize(steps, t_start, aborted_at='armB_pick_cup')

        log('Step: armB carry cup to final placement')
        steps['armB_carry_cup'] = await self.armB.place(
            cup_obj, cup_place_x, cup_place_y, CUP_FINAL_PLACE_POSITION['z_above_table'],
            tolerance_m=0.08)  # cup: looser tolerance than the ball's own check

        log('Step: armA pick ball')
        steps['armA_pick_ball'] = await self.armA.pick(ball_obj)
        if steps['armA_pick_ball']:
            log('Step: armA place ball next to cup')
            steps['armA_place_ball'] = await self.armA.place(
                ball_obj, ball_place_x, ball_place_y, CUP_FINAL_PLACE_POSITION['z_above_table'],
                tolerance_m=BALL_PLACE_TOLERANCE_M)
            steps['armA_go_home_2'] = await self.armA.go_home()

        log('Step: armB go home')
        steps['armB_go_home'] = await self.armB.go_home()

        log('Task complete')
        return self._summarize(steps, t_start)
  #Constructs the benchmark metric report for performance logging.
    def _summarize(self, steps, t_start, aborted_at=None):
        return {
            'steps': steps,
            'aborted_at': aborted_at,
            'all_steps_succeeded': aborted_at is None and all(steps.values()),
            'duration_sec': time.time() - t_start,
        }


def main():
    rclpy.init()
    node = DualArmCoordinator()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    task = executor.create_task(node.run_handoff_task())
    executor.spin_until_future_complete(task)
    node.get_logger().info(f'Task result: {task.result()}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
