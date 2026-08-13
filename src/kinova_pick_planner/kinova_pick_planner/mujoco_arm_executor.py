#!/usr/bin/env python3
"""
Per-arm motion executor for the MuJoCo-direct dual-arm evaluation harness.
Replaces arm_controller.py's role for MuJoCo specifically (that file's sequence
is MoveIt2-backed end to end, and no MoveIt2 config exists for this robot --
see the harness plan / conversation for the "bypass MoveIt2" decision). Drives
armA_/armB_ joint_trajectory_controller + gripper action servers directly,
using mujoco_ik.solve_position_ik (live, seeded from the previous joint config
each call) instead of MoveIt2's planner+IK service.

NOT an rclpy.Node itself -- takes the owning node (dual_arm_coordinator) so
both arms' executors share one node/one executor/one process, matching the
"one shared MuJoCo process, flat-prefixed topics" architecture already built
this session (no domain_bridge, no per-arm process needed).

All motion/gripper methods are coroutines and must be awaited from a coroutine
callback on the owning node (same reasoning as failure_monitor.py: calling
send_goal_async + waiting for the result from a plain callback on a node
that's already being spun would deadlock; awaiting a Future directly is the
pattern rclpy actually supports for this).
"""
import json
import time

import numpy as np

from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String
from mujoco_ros_msgs.srv import GetBodyState

from kinova_pick_planner.mujoco_ik import solve_position_ik, IKConvergenceError
from kinova_pick_planner.kinova_pick_planner import compute_grasp_and_approach, compute_place_pose

JOINT_SUFFIXES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Handles smooth 3D orientation interpolation across trajectory waypoints.
def _slerp_xyzw(q0_xyzw, q1_xyzw, t):
    """Spherical linear interpolation between two quaternions (xyzw order),
    t in [0, 1]. Picks the short-arc hemisphere (negates q1 if the dot
    product is negative) -- same double-cover issue as mujoco_ik.py's
    hemisphere fix, needed here too or the interpolated path could swing the
    long way around."""
    q0 = np.asarray(q0_xyzw, dtype=float)
    q1 = np.asarray(q1_xyzw, dtype=float)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:  # nearly identical -- linear interpolation is numerically safer
        result = q0 + t * (q1 - q0)
        return tuple(result / np.linalg.norm(result))
    theta0 = np.arccos(dot)
    theta = theta0 * t
    q2 = q1 - q0 * dot
    q2 = q2 / np.linalg.norm(q2)
    result = q0 * np.cos(theta) + q2 * np.sin(theta)
    return tuple(result / np.linalg.norm(result))

# Same "ready" configuration as arm_controller.py's HOME_JOINT_POSITIONS
# (validated on real hardware) -- used to seed the very first IK solve of a
# trial, before any move has established a current joint config to seed from.
HOME_JOINT_ANGLES = {
    'joint_1': 0.0, 'joint_2': -0.2814, 'joint_3': 1.3161,
    'joint_4': -0.0027, 'joint_5': -1.0479, 'joint_6': 0.0,
}


class MujocoArmExecutor:

    def __init__(self, node, arm_prefix: str, model, data,
                 ee_body_name=None, action_timeout_sec: float = 15.0):
        self.node = node
        self.arm_prefix = arm_prefix
        self.model = model
        self.data = data  # shared MjData for offline IK -- see module docstring on why sharing is safe
        # Fingertip midpoint, NOT the wrist ("end_effector_link" alone) -- fixed
        # after the first live test: targeting the wrist put it exactly at the
        # cup's center, so the gripper (extending further out) plowed into the
        # cup and slid it away before the "grasp" position was reached. See
        # mujoco_ik.py's solve_position_ik docstring for the full reasoning.
        self.ee_body_name = ee_body_name or [
            f'{arm_prefix}right_finger_dist_link', f'{arm_prefix}left_finger_dist_link']
        self.action_timeout_sec = action_timeout_sec

        self.joint_names = [f'{arm_prefix}{s}' for s in JOINT_SUFFIXES]
        self.gripper_joint_name = f'{arm_prefix}right_finger_bottom_joint'

        self._traj_client = ActionClient(
            node, FollowJointTrajectory,
            f'/{arm_prefix}joint_trajectory_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(
            node, GripperCommand, f'/{arm_prefix}gen3_lite_2f_gripper_controller/gripper_cmd')
        self._status_pub = node.create_publisher(String, f'/{arm_prefix}arm_status', 10)
        self._failure_cmd_pub = node.create_publisher(String, '/failure_monitor/cmd', 10)
        self._gbs_client = node.create_client(GetBodyState, '/mujoco_server/get_body_state')

        # {joint_name (unprefixed): angle} -- seeds the next IK solve AND (via
        # _current_ee_xyz) the straight-line start point for waypointed moves,
        # so this has to reflect the arm's TRUE current pose, not just serve as
        # an IK initial guess. Both arms actually start the sim at qpos=0 (all
        # joints zero) -- confirmed via /joint_states on a fresh launch, NOT
        # HOME_JOINT_ANGLES. Getting this wrong was a real bug: using
        # HOME_JOINT_ANGLES here as a "close enough" guess meant the first
        # waypointed move of a trial (go_home's own first call) computed its
        # straight-line path from a fictional starting point instead of the
        # true zero pose, defeating the whole point of waypointing that move.
        self._current_qpos = {s: 0.0 for s in JOINT_SUFFIXES}

    def publish_status(self, status: str, **extra):
        payload = {'status': status, **extra}
        self._status_pub.publish(String(data=json.dumps(payload)))

    def _prefixed_seed(self):
        return {f'{self.arm_prefix}{k}': v for k, v in self._current_qpos.items()}

    async def _wait(self, duration_sec: float):
        """Non-blocking wait using only rclpy-native primitives -- same
        reasoning as _await_with_timeout: asyncio.sleep isn't safe to assume
        works under rclpy's executor (it steps coroutines with its own
        scheduling, not a real asyncio event loop), confirmed the hard way
        earlier this session. A one-shot timer resolving an rclpy.task.Future
        is the pattern already proven to work here."""
        from rclpy.task import Future as RclpyFuture
        gate = RclpyFuture()

        def on_timeout():
            if not gate.done():
                gate.set_result(None)
        timer = self.node.create_timer(duration_sec, on_timeout)
        await gate
        timer.cancel()

    #Monitors real-time object linear velocity via a ROS 2 service (GetBodyState).
    async def _wait_until_still(self, body_name: str, vel_threshold: float = 0.005,
                                 poll_interval: float = 0.05, max_wait: float = 1.5) -> bool:
        """Polls GetBodyState for `body_name`'s actual linear velocity and
        returns once it drops below vel_threshold (m/s), instead of a fixed
        guessed pause duration. Replaces a blind sleep with a real
        stability check -- a stepped diagnostic found the cup still visibly
        drifting during a fixed 0.5s post-close pause with no further
        commands issued, meaning contact forces hadn't actually reached
        equilibrium by then; polling the real velocity is a directly
        verifiable stopping condition instead of another guessed constant.

        Also stops at a LOCAL VELOCITY MINIMUM (two consecutive rising
        samples), not just a near-zero threshold -- a longer poll (5s) found
        the held cup's speed decreases to a minimum around ~8mm/s then
        RE-ACCELERATES (creep/stick-slip against the compliant contact, not
        simple settling), so waiting for a full stop that may never come is
        actively counterproductive: the best achievable hold is right at
        that minimum, and continuing to wait past it only lets more drift
        accumulate. Returns True if it reached the threshold or a genuine
        local minimum within max_wait, False if it timed out still
        decelerating (never got a chance to confirm a minimum) -- callers
        can log/react to that, same spirit as move_to_xyz returning False
        on failure rather than silently proceeding.
        """
        if not self._gbs_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn(
                f'[{self.arm_prefix}] GetBodyState service unavailable, falling back to fixed wait')
            await self._wait(max_wait)
            return False
        elapsed = 0.0
        prev_speed = None
        rising_streak = 0
        while elapsed < max_wait:
            req = GetBodyState.Request()
            req.name = body_name
            req.admin_hash = ''
            result = await self._gbs_client.call_async(req)
            v = result.state.twist.twist.linear
            speed = (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5
            if speed < vel_threshold:
                return True
            if prev_speed is not None and speed > prev_speed * 1.05:
                rising_streak += 1
                if rising_streak >= 2:
                    return True
            else:
                rising_streak = 0
            prev_speed = speed
            await self._wait(poll_interval)
            elapsed += poll_interval
        self.node.get_logger().warn(
            f'[{self.arm_prefix}] {body_name} still moving after {max_wait}s settle wait')
        return False

    async def _await_with_timeout(self, future, timeout_sec, what='action result'):
        """Bounds an rclpy Future's wait to timeout_sec. NOT implemented with
        asyncio.wait_for/asyncio.sleep -- confirmed the hard way this session
        that those aren't safe to assume here: rclpy's executor steps
        coroutine callbacks with its own scheduling, not a real asyncio event
        loop, so an awaited plain-asyncio primitive can just hang forever
        instead of timing out (a mujoco_node crash mid-action left a gripper
        result future pending indefinitely with no client-side error at all --
        exactly what a missing timeout looks like). Uses only rclpy-native
        primitives instead: a second rclpy.task.Future that whichever fires
        first -- the real future's own done-callback, or a one-shot timer --
        resolves, since awaiting a plain rclpy Future is proven to work
        (every successful action call earlier in this same session's testing
        did exactly that).
        """
        from rclpy.task import Future as RclpyFuture
        gate = RclpyFuture()

        def on_done(f):
            if not gate.done():
                gate.set_result(('done', f))
        future.add_done_callback(on_done)

        def on_timeout():
            if not gate.done():
                gate.set_result(('timeout', None))
        timer = self.node.create_timer(timeout_sec, on_timeout)

        outcome, resolved = await gate
        timer.cancel()
        if outcome == 'timeout':
            self.node.get_logger().error(
                f'[{self.arm_prefix}] TIMEOUT waiting for {what} after {timeout_sec}s')
            return None
        return resolved.result()

    #Executes Forward Kinematics FK on the cached 
    def _current_ee_xyz(self):
        """FK's self._current_qpos to get this arm's actual current fingertip-
        midpoint position, for computing a straight-line path in move_to_xyz."""
        import mujoco
        for suffix, angle in self._current_qpos.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f'{self.arm_prefix}{suffix}')
            self.data.qpos[self.model.jnt_qposadr[jid]] = angle
        mujoco.mj_fwdPosition(self.model, self.data)
        names = [self.ee_body_name] if isinstance(self.ee_body_name, str) else self.ee_body_name
        ee_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, n) for n in names]
        return np.mean([self.data.xpos[i] for i in ee_ids], axis=0)

    def _current_ee_quat_xyzw(self):
        """FK's self._current_qpos to get this arm's actual current wrist/
        fingertip orientation (first ee_body_name if a list, same convention
        as solve_position_ik's orient_ref_body default) -- used to SLERP
        orientation across waypoints in move_to_xyz, same reasoning as
        _current_ee_xyz for position."""
        import mujoco
        for suffix, angle in self._current_qpos.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f'{self.arm_prefix}{suffix}')
            self.data.qpos[self.model.jnt_qposadr[jid]] = angle
        mujoco.mj_fwdPosition(self.model, self.data)
        name = self.ee_body_name[0] if isinstance(self.ee_body_name, list) else self.ee_body_name
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        w, x, y, z = self.data.xquat[bid]
        return (x, y, z, w)


    # Generates Cartesian position and orientation waypoints.
    # Sequentially solves IK for all waypoints.
    # Formulates and dispatches a single ROS 2 trajectory action goal
    # Updates _current_qpos upon successful controller execution
    async def move_to_xyz(self, target_xyz, target_quat_xyzw=None, duration: float = 1.5,
                           via_waypoints: int = 1) -> bool:
        """IK-solve to target_xyz (world-frame meters) and execute as a joint-
        space trajectory. Returns False on IK failure or action failure/
        timeout -- callers should treat that as a step failure, same as a
        MoveIt2 planning failure would have been in arm_controller.py.

        target_quat_xyzw (ROS/geometry_msgs order, matching
        kinova_pick_planner.py's TargetPose.qx/qy/qz/qw) requests full 6D pose
        IK instead of position-only -- REQUIRED for grasp/place moves. Position-
        only IK leaves the wrist orientation uncontrolled (redundant 6-DOF arm,
        3D target), which was confirmed live to let one gripper finger contact
        an object before the other and knock it away instead of framing it
        symmetrically. Held constant across all waypoints if via_waypoints > 1
        (only position is interpolated) -- fine since these are short moves
        where the orientation shouldn't need to change mid-way.

        via_waypoints > 1 breaks the move into that many IK-solved points along
        a straight Cartesian line from the current position to target_xyz, sent
        as one multi-point trajectory (each point's own time_from_start spaced
        evenly across `duration`), instead of one direct joint-space jump.
        This matters because joint_trajectory_controller interpolates in JOINT
        space, not Cartesian space -- confirmed the hard way: a single-point
        "descend to grasp" move for two individually-fine endpoints (15cm
        above the cup -> at the cup's center) swung sideways through the cup
        mid-motion and displaced it ~7cm before the gripper ever closed, even
        with the fingertip-midpoint IK fix already in place. Small waypoint
        steps stay close to a true straight line without needing actual
        Cartesian path control -- the same reason arm_controller.py falls back
        to Cartesian-waypoint hopping for precision moves on real hardware
        (see that file's own module docstring), just implemented here via
        repeated IK solves instead of MoveIt2's compute_cartesian_path.
        """
        if via_waypoints > 1:
            start = self._current_ee_xyz()
            target = np.asarray(target_xyz, dtype=float)
            waypoints = [start + (target - start) * (i / via_waypoints) for i in range(1, via_waypoints + 1)]
        else:
            waypoints = [target_xyz]

        # SLERP orientation across the same waypoints when a target is given
        # and there's more than one -- NOT held constant from waypoint 1 like
        # an earlier version of this method did. Discovered live: a single-jump
        # "approach" move from go_home (fingertip orientation ~180 degrees
        # about world Z) to a grasp target (~180 degrees about world X) is a
        # LARGE reorientation, not the small in-place tweak the "orientation
        # doesn't need to change mid-way" assumption was written for -- with
        # orientation front-loaded onto waypoint 1, the controller's
        # joint-space interpolation from the actual current joints to that
        # almost-fully-rotated waypoint swept an unpredictable path that
        # knocked the cup ~15cm sideways before the gripper ever opened, even
        # though position alone was waypointed. Interpolating orientation too
        # keeps each waypoint's required joint change small and predictable,
        # the same fix already applied to position for the same reason.
        quat_waypoints = [target_quat_xyzw] * len(waypoints)
        if target_quat_xyzw is not None and via_waypoints > 1:
            start_quat = self._current_ee_quat_xyzw()
            quat_waypoints = [
                _slerp_xyzw(start_quat, target_quat_xyzw, i / via_waypoints)
                for i in range(1, via_waypoints + 1)]

        solutions = []
        seed = self._prefixed_seed()
        for wp, wp_quat in zip(waypoints, quat_waypoints):
            try:
                solution = solve_position_ik(
                    self.model, self.data, self.arm_prefix, self.ee_body_name,
                    wp, target_quat_xyzw=wp_quat, initial_qpos=seed)
            except IKConvergenceError as exc:
                self.node.get_logger().error(f'[{self.arm_prefix}] IK failed for waypoint {wp}: {exc}')
                return False
            solutions.append(solution)
            seed = solution  # chain: each waypoint solve seeds the next

        if not self._traj_client.wait_for_server(timeout_sec=self.action_timeout_sec):
            self.node.get_logger().error(f'[{self.arm_prefix}] joint_trajectory_controller action server unavailable')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        goal.trajectory.points = []
        for i, solution in enumerate(solutions):
            pt = JointTrajectoryPoint()
            pt.positions = [solution[jn] for jn in self.joint_names]
            t = duration * (i + 1) / len(solutions)
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal.trajectory.points.append(pt)
        solution = solutions[-1]  # final waypoint's solution is what _current_qpos becomes on success

        goal_handle = await self._await_with_timeout(
            self._traj_client.send_goal_async(goal), self.action_timeout_sec, 'trajectory goal acceptance')
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error(f'[{self.arm_prefix}] trajectory goal rejected or timed out')
            return False
        # Execution itself can legitimately take longer than action_timeout_sec
        # (it's a motion duration, not a round-trip) -- give it duration plus
        # generous margin, not the same short budget used for goal acceptance.
        result = await self._await_with_timeout(
            goal_handle.get_result_async(), duration + self.action_timeout_sec, 'trajectory execution result')
        if result is None:
            return False
        success = result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        if success:
            # unprefixed keys, to match _current_qpos's convention
            self._current_qpos = {jn[len(self.arm_prefix):]: solution[jn] for jn in self.joint_names}
        else:
            self.node.get_logger().error(
                f'[{self.arm_prefix}] trajectory execution failed: error_code={result.result.error_code}')
        return success

    #Dispatches an action goal to the gripper controller (GripperCommand)
    async def set_gripper(self, position: float, max_effort: float = 30.0) -> bool:
        if not self._gripper_client.wait_for_server(timeout_sec=self.action_timeout_sec):
            self.node.get_logger().error(f'[{self.arm_prefix}] gripper action server unavailable')
            return False
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        goal_handle = await self._await_with_timeout(
            self._gripper_client.send_goal_async(goal), self.action_timeout_sec, 'gripper goal acceptance')
        if goal_handle is None or not goal_handle.accepted:
            return False
        result = await self._await_with_timeout(
            goal_handle.get_result_async(), self.action_timeout_sec, 'gripper result')
        if result is None:
            return False
        return bool(result.result.reached_goal or not result.result.stalled)
    
    #Notifies the external /failure_monitor system to begin verifying whether the object stays attached to the end-effector
    def track_grasp(self, object_name: str):
        # failure_monitor.py's GetBodyState-based tracking expects a single
        # body name, not the fingertip-midpoint list used for IK targeting --
        # pass just one fingertip as a representative reference point. The few
        # cm of asymmetry versus the true midpoint doesn't matter here:
        # RIGID_TRACKING_TOLERANCE_M (3cm) already has slack for a "did we
        # drop it" check, unlike IK targeting where the wrist-vs-fingertip
        # error was the whole bug.
        ee_link = self.ee_body_name[0] if isinstance(self.ee_body_name, list) else self.ee_body_name
        self._failure_cmd_pub.publish(String(data=json.dumps(
            {'cmd': 'track_grasp', 'object': object_name, 'ee_link': ee_link})))

    # Commands /failure_monitor to stop tracking a released object.
    def release_tracking(self, object_name: str):
        self._failure_cmd_pub.publish(String(data=json.dumps(
            {'cmd': 'release', 'object': object_name})))

    # Requests an external placement validation check after releasing an object.
    def check_placement(self, object_name: str, target_xyz, tolerance_m: float, request_id: str):
        self._failure_cmd_pub.publish(String(data=json.dumps(
            {'cmd': 'check_placement', 'object': object_name, 'target': list(target_xyz),
             'tolerance': tolerance_m, 'request_id': request_id})))

    async def pick(self, obj_data: dict, gripper_open: float = 0.0) -> bool:
        """Approach -> open -> descend to grasp -> close -> confirm (via
        failure_monitor) -> lift. Mirrors arm_controller.py's step structure
        (Steps 1-5). Uses compute_grasp_and_approach(obj_data) directly for
        POSITIONS -- the same bearing-adjusted pose math arm_controller.py
        itself calls (kinova_pick_planner.py:545), reused rather than
        re-derived, a real improvement over computing raw XYZ myself.

        NOW using compute_grasp_and_approach's ORIENTATION output
        (grasp_pose.qx/qy/qz/qw) for the approach and descend-to-grasp moves --
        the frame-convention bug that caused the combined 6D solve to stall at
        an orientation error of ~pi is fixed (mju_subQuat's local-frame vector
        was being combined with mj_jac's world-frame Jacobian, see
        mujoco_ik.py's solve_position_ik). GRASP_STRATEGIES["mujoco_coffee_cup"]'s
        qx/qy/qz/qw was also changed from (1,0,0,0) to (0,1,0,0) -- both
        represent the same "point straight down" family (a 180-degree roll
        apart, and roll about the vertical approach axis doesn't matter for a
        round cup), but (1,0,0,0) turned out to be a badly-conditioned target
        for this arm's kinematics even after the frame fix (armA_joint_5
        pinned at its limit); a roll sweep against the corrected solver found
        (0,1,0,0) converges cleanly. This was the actual fix for the
        long-standing "cup gets nudged sideways during descent and never gets
        genuinely lifted" bug -- position-only IK left the wrist orientation
        uncontrolled, so the gripper wasn't square over the cup when it
        closed, confirmed via a stepped diagnostic showing ~3cm of lateral
        cup displacement during the descend-to-grasp move despite the
        fingertip midpoint itself tracking its target to ~1mm.

        obj_data: {"x": ..., "y": ..., "z": ..., "type": "mujoco_coffee_cup"/
        "ping_pong_ball", "name": <label used for failure_monitor tracking>}.
        "x"/"y" are
        world-frame meters (= armA_base frame, since armA_base sits at world
        origin by construction). "z" is the object's height above the table AS
        AN OVERHEAD CAMERA WOULD SEE IT (i.e. roughly the object's full height
        for something resting flat on the table) -- NOT this project's other
        "z_above_table" convention (center-offset), see
        compute_grasp_and_approach's own docstring/body for the exact
        base-height estimation. compute_grasp_and_approach applies
        TABLE_SURFACE_Z internally, so grasp_pose.z/approach_pose.z come back
        as absolute world z already -- move_to_xyz needs no further conversion.
        """
        approach_pose, grasp_pose, strategy = compute_grasp_and_approach(obj_data)
        object_name = obj_data.get('name') or obj_data.get('type')

        self.publish_status('moving')

        grasp_quat = (grasp_pose.qx, grasp_pose.qy, grasp_pose.qz, grasp_pose.qw)
        # via_waypoints=4: the move INTO approach_pose is where the wrist does
        # most of its reorientation (from wherever go_home left it, e.g.
        # ~180 degrees about world Z, to the ~180-about-X-ish grasp
        # orientation) -- a single-point move here front-loads that whole
        # reorientation onto one joint-space jump, confirmed live to sweep
        # the arm through the cup and knock it ~15cm sideways before the
        # gripper even opened. move_to_xyz SLERPs orientation across
        # waypoints together with position for exactly this case.
        if not await self.move_to_xyz(
                (approach_pose.x, approach_pose.y, approach_pose.z),
                target_quat_xyzw=grasp_quat, via_waypoints=4):
            return False
        if not await self.set_gripper(gripper_open, strategy.gripper_effort):
            return False
        # via_waypoints=4: this descent is the precision move that actually has
        # to arrive at the object without disturbing it -- see move_to_xyz's
        # docstring for why a single joint-space jump isn't safe here (measured
        # ~7cm of object displacement mid-descent without this).
        if not await self.move_to_xyz(
                (grasp_pose.x, grasp_pose.y, grasp_pose.z), target_quat_xyzw=grasp_quat, via_waypoints=4):
            return False

        # Incremental close (5 steps to strategy.gripper_close_pos), NOT a
        # single jump -- live testing found a single close-to-target command
        # itself could disturb the cup by several cm (measured up to ~8cm),
        # bigger than the descend step's own disturbance. A stepped diagnostic
        # closing 0.1 at a time showed each increment only added 1-3mm of
        # drift, confirming the single fast close (finger prox-link sweeping
        # quickly through its remaining range) was itself a meaningful part
        # of the problem, not just the initial descent graze.
        # 10 steps + a REAL settle (polled velocity, not a fixed guessed
        # pause) after EACH step -- a stepped diagnostic logging cup pose
        # after every close increment found the disturbance concentrated in
        # the LAST couple of increments (where the fingers first make real
        # contact) and STILL GROWING during a fixed 0.3-0.5s pause placed
        # only after the final step, meaning contact forces genuinely hadn't
        # reached equilibrium by a guessed duration -- confirming this
        # needs an actual stopping condition, not a bigger constant.
        # _wait_until_still polls the object's own GetBodyState velocity and
        # returns as soon as it's genuinely at rest (bounded by max_wait so
        # a never-settling case doesn't hang the whole task).
        # 20 steps (was 10) -- the cup drift starts specifically once the
        # fingers make real contact (observed around the last ~40% of
        # close_pos), and the lift never actually secured the cup at all
        # (Z stayed flat through "lift"), suggesting it can drift out of a
        # proper pinch position before full closure is even reached. Finer
        # steps mean less travel between settle checks in exactly that
        # critical contact range.
        close_steps = 20
        for i in range(1, close_steps + 1):
            step_pos = strategy.gripper_close_pos * i / close_steps
            if not await self.set_gripper(step_pos, strategy.gripper_effort):
                return False
            await self._wait_until_still(object_name, max_wait=0.3)

        # Final settle before lifting -- longer budget (was 1.0s) so the
        # local-velocity-minimum detection in _wait_until_still actually has
        # a chance to fire: a longer diagnostic poll found the held cup's
        # speed decreasing to a minimum around t~2.7s before re-accelerating
        # (creep/stick-slip, not simple settling), so 1.0s was cutting the
        # wait off mid-decay, well before the best achievable moment to lift.
        await self._wait_until_still(object_name, max_wait=4.0)

        # track_grasp just publishes -- failure_monitor processes it on its own
        # schedule independent of what we do next, so no sleep/wait is needed
        # here (an artificial sleep inside this coroutine would block the whole
        # node's executor, including the OTHER arm's concurrent operations
        # during the parallel phases of the task -- worth avoiding even if it
        # were otherwise harmless).
        self.track_grasp(object_name)

        # Also waypointed: lifting straight up right after a grasp is exactly
        # the kind of precision move where a joint-space swing could shear the
        # object out of the gripper before it's even clear of the table. Keeps
        # grasp_quat too -- letting orientation go uncontrolled again right
        # after closing on the object risks the wrist rolling and loosening
        # the grip before it's even clear of the table.
        #
        # via_waypoints=6, duration=3.0 (was 3 / default 1.5) -- slowed down
        # ~4x from the other moves' pace. Live testing found the grasp could
        # close cleanly and centered on the cup, then still explosively lose
        # it during a fast lift -- with the cup/gripper contact geoms softened
        # to survive the marginal ~mm-scale clearance graze from descent (see
        # kinova_pick_planner.py's GRASP_STRATEGIES comment and
        # scene_dual_arm.xml / dual_arm.xml), a fast upward acceleration
        # right after closing was enough dynamic load to break that marginal
        # grip loose violently. A slower, finer-grained lift gives the
        # compliant contact time to settle into a stable holding force
        # instead of being yanked.
        lift_xyz = (grasp_pose.x, grasp_pose.y, grasp_pose.z + 0.15)
        if not await self.move_to_xyz(
                lift_xyz, duration=3.0, via_waypoints=6):
            return False

        self.publish_status('secured', object=object_name)
        return True

    async def place(self, obj_data: dict, place_x: float, place_y: float,
                     place_z_above_table: float = 0.0, tolerance_m: float = 0.05,
                     gripper_open: float = 0.0, request_id: str = None) -> bool:
        """Pre-place -> lower -> release -> confirm placement -> retreat. Uses
        compute_place_pose(obj_data, place_x, place_y, place_z_above_table) for
        both position AND orientation now (see pick()'s docstring for the
        mujoco_ik.py frame-convention fix and the GRASP_STRATEGIES roll fix
        that made 6D IK reliable for this target family -- compute_place_pose
        derives its orientation from the same GRASP_STRATEGIES entry, so
        fixing that quaternion fixed place() too, no separate change needed
        here beyond passing it through to move_to_xyz). Note this
        place_z_above_table IS the "center offset above table" convention
        (matches kinova_pick_planner.py's PLACE_POSITION/CUP_FINAL_PLACE_POSITION
        style, unlike pick()'s obj_data["z"] which is a different convention --
        see compute_place_pose vs. compute_grasp_and_approach in
        kinova_pick_planner.py for the exact difference)."""
        pre_place, place_pose, retreat_pose = compute_place_pose(
            obj_data, place_x, place_y, place_z_above_table)
        object_name = obj_data.get('name') or obj_data.get('type')

        self.publish_status('moving')

        place_quat = (place_pose.qx, place_pose.qy, place_pose.qz, place_pose.qw)
        # via_waypoints=4: same reasoning as pick()'s move into approach_pose --
        # this move can involve a real reorientation (place_quat's bearing
        # term differs from whatever grasp_quat the object is currently held
        # at) while the object is actively held, so a sudden single-jump
        # reorientation risks shearing it out of the gripper, not just
        # disturbing an unheld object.
        if not await self.move_to_xyz(
                (pre_place.x, pre_place.y, pre_place.z), target_quat_xyzw=place_quat, via_waypoints=4):
            return False
        # Same reasoning as pick()'s descend-to-grasp: this is the precision
        # move that has to arrive at the actual place target without a
        # joint-space swing knocking the held object sideways first.
        if not await self.move_to_xyz(
                (place_pose.x, place_pose.y, place_pose.z), target_quat_xyzw=place_quat, via_waypoints=4):
            return False
        if not await self.set_gripper(gripper_open, 25.0):
            return False

        place_xyz = (place_pose.x, place_pose.y, place_pose.z)
        self.release_tracking(object_name)
        self.check_placement(object_name, place_xyz, tolerance_m,
                             request_id or f'{object_name}_{time.time()}')

        success = await self.move_to_xyz((retreat_pose.x, retreat_pose.y, retreat_pose.z))
        self.publish_status('ready')
        return success

    async def hold_at(self, xyz, duration: float = 1.5) -> bool:
        """Named alias for move_to_xyz, used at the handoff pose for clarity
        in dual_arm_coordinator.py -- semantically "get here and wait for the
        other arm," not just "move.\""""
        self.publish_status('moving')
        ok = await self.move_to_xyz(xyz, duration=duration)
        if ok:
            self.publish_status('holding')
        return ok

    async def go_home(self, duration: float = 2.0, via_waypoints: int = 1) -> bool:
        self.publish_status('moving')
        ok = await self.move_to_xyz(self._home_ee_xyz(), duration=duration, via_waypoints=via_waypoints)
        self.publish_status('ready')
        return ok

    def _home_ee_xyz(self):
        """Forward-kinematics the arm's HOME_JOINT_ANGLES to get a world xyz to
        move_to_xyz against, so go_home() can reuse the same IK/action path as
        every other move instead of a separate direct-qpos-command mechanism.
        self.ee_body_name may be a single body or a list (fingertip midpoint,
        see __init__) -- average over whichever it is, same as solve_position_ik."""
        import mujoco
        for suffix, angle in HOME_JOINT_ANGLES.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f'{self.arm_prefix}{suffix}')
            self.data.qpos[self.model.jnt_qposadr[jid]] = angle
        mujoco.mj_fwdPosition(self.model, self.data)
        names = [self.ee_body_name] if isinstance(self.ee_body_name, str) else self.ee_body_name
        ee_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, n) for n in names]
        return tuple(np.mean([self.data.xpos[i] for i in ee_ids], axis=0))
