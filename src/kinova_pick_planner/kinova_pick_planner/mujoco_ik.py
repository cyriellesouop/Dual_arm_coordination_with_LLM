#!/usr/bin/env python3
"""
Jacobian-based inverse kinematics for the MuJoCo-direct dual-arm evaluation
harness (see mujoco_arm_executor.py). No MoveIt2 dependency -- this project's
MoveIt2 setup (kinova_gen3_lite_moveit_config) has no configuration for the
MuJoCo dual-arm robot, and building one was explicitly decided against for this
harness in favor of driving joint_trajectory_controller directly (see
conversation / the evaluation-harness plan).

Computes joint angles for a target end-effector position using MuJoCo's own
Jacobian (mj_jac) and damped least squares, entirely offline against a static
copy of the scene model (mujoco.MjModel.from_xml_path) -- NOT the live running
sim. MuJoCo's Jacobian isn't exposed via any mujoco_ros_msgs service (checked
during the failure_monitor work: GetBodyState/SetBodyState/GetSimInfo, no
Jacobian/kinematics equivalent), so this has to run against a local model
instance, not through ROS.

Supports both position-only (3D) and full pose (position + orientation, 6D)
IK. Position-only is fine for free-space moves (approach, lift, home) where
any reasonable wrist orientation is acceptable -- with a redundant 6-DOF arm
solving only a 3D target, the wrist's final orientation is otherwise
uncontrolled (whatever the damped-least-squares null space happens to
produce). That's NOT fine for the actual grasp/place descent: confirmed via a
live screenshot that an uncontrolled approach orientation let one gripper
finger contact the cup before the other, knocking it away instead of framing
it symmetrically. Fix: solve full 6D pose IK for those moves, targeting the
grasp quaternions in kinova_pick_planner.py's GRASP_STRATEGIES instead of
leaving orientation to chance.

NOTE, corrected after checking rather than assuming: those quaternions are
NOT independently validated real-hardware data -- verified by investigation
that no test/log/data in this repo demonstrates a successful grasp with them,
and the bearing-correction math they rely on
(base_angle=atan2(0.191,0.423)) was kept because the analytic alternative
(side_grasp_quaternion) was found even worse (~36 degrees off), not because
this one was confirmed good. They're the best-grounded option available
(closer to first-principles-correct than an uncalibrated guess), reused
rather than re-derived, but still need empirical verification in THIS
context -- not to be cited as "already proven" in future comments.
"""
import numpy as np
import mujoco


class IKConvergenceError(RuntimeError):
    pass


def load_model(mjcf_path: str):
    """Loads a standalone MuJoCo model/data pair for offline IK use."""
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)
    return model, data


def solve_position_ik(model, data, arm_prefix: str, ee_body_name,
                       target_pos, target_quat_xyzw=None, orient_ref_body: str = None,
                       initial_qpos: dict = None,
                       max_iters: int = 300, tol: float = 1e-4, orient_tol: float = 2e-2,
                       damping: float = 0.05, max_step_rad: float = 0.15) -> dict:
    """
    Returns {joint_name: angle_rad} for the 6 arm joints under `arm_prefix`
    (e.g. "armA_") that bring the reference point to `target_pos` (3-vector,
    meters), and -- if `target_quat_xyzw` is given -- bring `orient_ref_body`'s
    orientation to that quaternion too (full 6D pose IK, not just position).
    `initial_qpos` (dict of joint_name -> angle) seeds the solve -- pass the
    nearest known-good waypoint (e.g. the calibrated "place" config) when
    doing a small-perturbation correction, or leave None (uses whatever
    `data.qpos` already holds, typically zero) for a fresh calibration solve.

    `ee_body_name` is either a single body name (its own world-frame origin is
    the reference point) or a list/tuple of body names (their position AND
    Jacobian are averaged -- e.g. the two fingertip bodies, giving the
    gripper's actual midpoint as the reference point). Use the midpoint form
    for grasp/place targets: solving against the wrist link alone (a single
    body, e.g. "armA_end_effector_link") targets the WRIST, not where the
    fingers actually are -- confirmed the hard way in this session's first
    live test, where the wrist arriving exactly at the cup's center meant the
    gripper (which extends further out) plowed into the cup and slid it away
    before the "grasp" position was even reached. Averaging the two fingertip
    bodies' positions/Jacobians is the correct fix, not a fudge-factor offset
    -- it's an exact linear combination (midpoint of two points has Jacobian
    = average of their Jacobians), not an approximation of gripper geometry.

    `target_quat_xyzw` uses ROS/geometry_msgs quaternion order (x, y, z, w),
    matching kinova_pick_planner.py's GRASP_STRATEGIES convention -- converted
    internally to MuJoCo's (w, x, y, z) order. `orient_ref_body` defaults to
    the first `ee_body_name` if not given (position-only IK with `ee_body_name`
    as a fingertip-midpoint list still needs a SINGLE body for orientation --
    averaging quaternions isn't a simple linear operation the way averaging
    positions is, and the two fingertips share essentially the same pointing
    direction as the wrist anyway, so one fingertip is a fine orientation
    reference).

    Without target_quat_xyzw, this is position-only (3D) -- fine for free-space
    moves (approach, lift, home) where any reasonable wrist orientation is
    acceptable, since a redundant 6-DOF arm solving only a 3D target otherwise
    leaves the final orientation uncontrolled. NOT fine for the actual
    grasp/place descent -- confirmed via a live screenshot that an
    uncontrolled approach let one gripper finger contact the cup before the
    other, knocking it away instead of framing it symmetrically. Pass
    target_quat_xyzw (e.g. GRASP_STRATEGIES["coffee_cup"]'s qx/qy/qz/qw --
    the best-grounded orientation available, NOT independently confirmed
    real-hardware data, see this module's own docstring) for those moves
    instead of leaving orientation to chance.

    Raises IKConvergenceError if the residual is still above tolerance after
    `max_iters` -- callers should treat this as "this target is unreachable /
    a bad initial guess," not silently proceed with a stale pose.
    """
    ee_body_names = [ee_body_name] if isinstance(ee_body_name, str) else list(ee_body_name)
    use_orientation = target_quat_xyzw is not None
    if use_orientation:
        orient_ref_body = orient_ref_body or ee_body_names[0]
        # xyzw (ROS convention, matches GRASP_STRATEGIES) -> wxyz (MuJoCo convention)
        x, y, z, w = target_quat_xyzw
        target_quat_wxyz = np.array([w, x, y, z], dtype=float)

    joint_names = [f'{arm_prefix}joint_{i}' for i in range(1, 7)]
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in joint_names]
    if any(jid < 0 for jid in joint_ids):
        missing = [j for j, jid in zip(joint_names, joint_ids) if jid < 0]
        raise ValueError(f'joints not found in model: {missing}')
    qpos_adrs = [model.jnt_qposadr[jid] for jid in joint_ids]
    dof_adrs = [model.jnt_dofadr[jid] for jid in joint_ids]
    # Real per-joint limits, already loaded from the MJCF's own range="..." attrs
    # (same numbers as the vendor URDF's <limit> tags -- see mujoco_ik.py's
    # sibling work this session on effort_limit). Plain damped least squares has
    # no joint-limit awareness on its own -- confirmed by testing: an
    # unclamped solve for a perfectly reachable target drove joint_2/joint_3 to
    # ~2.83/~3.99 rad, well past their actual +-2.61 rad hard stops. Clamping
    # after every step keeps every intermediate and final qpos physically valid.
    joint_ranges = [model.jnt_range[jid] for jid in joint_ids]

    ee_body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in ee_body_names]
    if any(bid < 0 for bid in ee_body_ids):
        missing = [n for n, bid in zip(ee_body_names, ee_body_ids) if bid < 0]
        raise ValueError(f'unknown body/bodies: {missing}')
    n_ref = len(ee_body_ids)
    orient_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, orient_ref_body) if use_orientation else None
    if use_orientation and orient_body_id < 0:
        raise ValueError(f'unknown orientation reference body: {orient_ref_body}')

    if initial_qpos:
        for name, val in initial_qpos.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                data.qpos[model.jnt_qposadr[jid]] = val

    target = np.asarray(target_pos, dtype=float)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    pos_err = np.zeros(3)
    orient_err = np.zeros(3)
    rotmat_flat = np.zeros(9)

    if use_orientation:
        # Quaternion double-cover fix: q and -q represent the IDENTICAL
        # rotation, but mju_subQuat doesn't know that -- if current_quat and
        # target_quat_wxyz are in opposite hemispheres (dot product < 0), it
        # computes the rotation vector for the LONG way around instead of the
        # short way. Chosen ONCE here, from the INITIAL orientation, not
        # recomputed every iteration -- found the hard way that recomputing it
        # every iteration lets the effective target flip between q and -q as
        # the arm's orientation crosses near the dot=0 boundary mid-solve,
        # which chases a moving target and produces a stable 2-iteration limit
        # cycle (measured: orient_err pinned at ~pi, 2.96-3.08 rad, never
        # decreasing, for 40+ iterations) rather than genuine non-convergence.
        # The initial hemisphere choice is stable to commit to since it's not
        # revisited as the arm moves during the solve.
        mujoco.mj_fwdPosition(model, data)
        initial_quat = data.xquat[orient_body_id].copy()
        target_quat_for_err = (target_quat_wxyz if np.dot(target_quat_wxyz, initial_quat) >= 0
                                else -target_quat_wxyz)

    for _ in range(max_iters):
        mujoco.mj_fwdPosition(model, data)
        positions = [data.xpos[bid].copy() for bid in ee_body_ids]
        current = np.mean(positions, axis=0)
        pos_err = target - current

        jac_sum = np.zeros((3, model.nv))
        for pos, bid in zip(positions, ee_body_ids):
            mujoco.mj_jac(model, data, jacp, None, pos, bid)
            jac_sum += jacp
        jac_avg = jac_sum / n_ref

        if use_orientation:
            current_quat = data.xquat[orient_body_id].copy()
            # mju_subQuat(res, qa, qb) is documented as "qb*quat(res) = qa" --
            # that's a POST-multiplication, meaning res is expressed in qb's
            # (current_quat's) own BODY-LOCAL frame, not the world frame. But
            # mj_jac's jacr maps joint velocities to the body's angular
            # velocity in the WORLD frame. Combining a local-frame error
            # vector with a world-frame Jacobian is a frame mismatch -- for
            # small corrections local ~= world (to first order) so this went
            # unnoticed in earlier small-perturbation checks, but it's why the
            # combined solve reliably froze at an orient residual near pi for
            # any target requiring a large (~180 degree class) reorientation
            # from the seed: confirmed by isolating pure-orientation DLS
            # offline, which stalled at a fixed ~3.05 rad error with a
            # near-zero step every iteration using the raw local-frame error,
            # then converged in 10 iterations once the error was rotated into
            # world frame below. Fix: rotate the local-frame vector into world
            # frame via R(current_quat) @ local_err before using it with jacr.
            mujoco.mju_subQuat(orient_err, target_quat_for_err, current_quat)
            mujoco.mju_quat2Mat(rotmat_flat, current_quat)
            orient_err_world = rotmat_flat.reshape(3, 3) @ orient_err
            mujoco.mj_jac(model, data, None, jacr, data.xpos[orient_body_id], orient_body_id)
            j_full = np.vstack([jac_avg, jacr])[:, dof_adrs]  # 6 x 6
            combined_err = np.concatenate([pos_err, orient_err_world])
            converged = np.linalg.norm(pos_err) < tol and np.linalg.norm(orient_err) < orient_tol
        else:
            j_full = jac_avg[:, dof_adrs]  # 3 x 6
            combined_err = pos_err
            converged = np.linalg.norm(pos_err) < tol

        if converged:
            return {name: float(data.qpos[adr]) for name, adr in zip(joint_names, qpos_adrs)}

        jjt = j_full @ j_full.T
        dq = j_full.T @ np.linalg.solve(jjt + (damping ** 2) * np.eye(j_full.shape[0]), combined_err)
        # Trust-region step clamp: found this the hard way -- combining
        # position (meters) and orientation (radians) errors into one 6-vector
        # for the DLS solve produced a poorly-conditioned combined Jacobian
        # for some target/seed combinations, causing dq to wildly overshoot
        # each iteration (measured: pos_err and orient_err oscillating between
        # 0.08-0.7m / 0.9-3.1rad for 30+ iterations with no convergence trend,
        # not settling or even monotonically improving). Bounding the max
        # per-joint step per iteration is a standard trust-region fix -- it
        # doesn't matter WHY the raw Jacobian solve produced a bad step, it
        # just prevents any single step from being physically implausible.
        dq = np.clip(dq, -max_step_rad, max_step_rad)
        for k, adr in enumerate(qpos_adrs):
            lo, hi = joint_ranges[k]
            data.qpos[adr] = float(np.clip(data.qpos[adr] + dq[k], lo, hi))

    raise IKConvergenceError(
        f'IK did not converge for {ee_body_names} -> {target_pos.tolist() if hasattr(target_pos, "tolist") else target_pos} '
        f'after {max_iters} iterations (pos residual {np.linalg.norm(pos_err):.4f} m, '
        f'orient residual {np.linalg.norm(orient_err):.4f} rad)'
    )
