"""Arm motion primitives used by teleoperation lifecycle sequences."""

import time

import numpy as np
import logging_mp

logger_mp = logging_mp.getLogger(__name__)

# Low-state quantization and servo settling can leave a small residual error;
# this is an acceptance threshold, not a command limit.
SAFETY_POSE_TOLERANCE = 0.12

def interruptible_wait(duration, check_cancel=None):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        if check_cancel is not None:
            check_cancel()
        time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))

def move_dual_arm_to_q_slow(
        arm_ctrl,
        q_target,
        tauff_target=None,
        velocity_limit=0.8,
        timeout=12.0,
        tolerance=SAFETY_POSE_TOLERANCE,
        restore_previous=True, check_cancel=None):
    previous_velocity_limit = getattr(arm_ctrl, "arm_velocity_limit", None)
    previous_speed_gradual_max = getattr(arm_ctrl, "_speed_gradual_max", None)
    if hasattr(arm_ctrl, "_speed_gradual_max"):
        arm_ctrl._speed_gradual_max = False
    if hasattr(arm_ctrl, "arm_velocity_limit"):
        arm_ctrl.arm_velocity_limit = velocity_limit

    if tauff_target is None:
        tauff_target = np.zeros_like(q_target)
    if check_cancel is not None:
        check_cancel()
    arm_ctrl.ctrl_dual_arm(q_target, tauff_target)

    reached = False
    deadline = time.time() + timeout
    last_report = 0.0
    while time.time() < deadline:
        if check_cancel is not None:
            check_cancel()
        current_q = arm_ctrl.get_current_dual_arm_q()
        if time.time() - last_report >= 5.0:
            error = float(np.max(np.abs(current_q - q_target)))
            logger_mp.info(
                "Safety pose progress: max joint error %.3f rad (target command is being held).",
                error,
            )
            last_report = time.time()
        if np.all(np.abs(current_q - q_target) < tolerance):
            reached = True
            break
        time.sleep(0.05)

    if restore_previous and previous_velocity_limit is not None:
        arm_ctrl.arm_velocity_limit = previous_velocity_limit
    if restore_previous and previous_speed_gradual_max is not None:
        arm_ctrl._speed_gradual_max = previous_speed_gradual_max
    return reached

def hold_current_arm_pose(arm_ctrl, hold_time=0.5, velocity_limit=0.2, label="current arm pose", restore_previous=True):
    previous_velocity_limit = getattr(arm_ctrl, "arm_velocity_limit", None)
    previous_speed_gradual_max = getattr(arm_ctrl, "_speed_gradual_max", None)
    if hasattr(arm_ctrl, "_speed_gradual_max"):
        arm_ctrl._speed_gradual_max = False
    if hasattr(arm_ctrl, "arm_velocity_limit"):
        arm_ctrl.arm_velocity_limit = velocity_limit

    current_q = arm_ctrl.get_current_dual_arm_q()
    tauff_target = np.zeros_like(current_q)
    logger_mp.info(f"Hold {label}.")
    deadline = time.time() + hold_time
    while time.time() < deadline:
        arm_ctrl.ctrl_dual_arm(current_q, tauff_target)
        time.sleep(0.02)

    if restore_previous and previous_velocity_limit is not None:
        arm_ctrl.arm_velocity_limit = previous_velocity_limit
    if restore_previous and previous_speed_gradual_max is not None:
        arm_ctrl._speed_gradual_max = previous_speed_gradual_max
    return current_q

def move_dual_arm_to_safety_pose(arm_ctrl, velocity_limit=0.8, timeout=30.0, hold_time=0.3, restore_previous=True, check_cancel=None):
    # Preserve the established initial safety pose from the working commit.
    q_target = np.zeros_like(arm_ctrl.get_current_dual_arm_q())
    reached = move_dual_arm_to_q_slow(
        arm_ctrl,
        q_target,
        np.zeros_like(q_target),
        velocity_limit=velocity_limit,
        timeout=timeout,
        tolerance=0.08,
        restore_previous=restore_previous,
        check_cancel=check_cancel,
    )
    if reached:
        logger_mp.info("Arm safety pose reached.")
        interruptible_wait(hold_time, check_cancel)
    else:
        error = float(np.max(np.abs(arm_ctrl.get_current_dual_arm_q() - q_target)))
        logger_mp.warning(
            "Arm safety pose timed out; max joint error %.3f rad. "
            "The robot did not converge to the commanded safety target.", error)
    return reached

def get_startup_dual_arm_q(arm_ctrl):
    startup_q = getattr(arm_ctrl, "startup_dual_arm_q", None)
    if startup_q is None:
        startup_q = getattr(arm_ctrl, "initial_dual_arm_q", None)
    if startup_q is None:
        return None
    startup_q = np.asarray(startup_q, dtype=float)
    current_q = arm_ctrl.get_current_dual_arm_q()
    if startup_q.shape != current_q.shape:
        logger_mp.warning("Startup arm pose shape does not match current arm pose; skip controlled lowering.")
        return None
    return startup_q.copy()

def move_dual_arm_to_startup_pose(arm_ctrl, velocity_limit=0.35, timeout=35.0, hold_time=0.5):
    startup_q = get_startup_dual_arm_q(arm_ctrl)
    if startup_q is None:
        logger_mp.warning("Controlled lowering skipped: startup arm pose is unavailable.")
        return False

    logger_mp.info("Exit safety sequence: slowly lower arms to startup standing pose under SDK control.")
    reached = move_dual_arm_to_q_slow(
        arm_ctrl,
        startup_q,
        np.zeros_like(startup_q),
        velocity_limit=velocity_limit,
        timeout=timeout,
        tolerance=0.10,
        restore_previous=False,
    )
    if reached:
        logger_mp.info("Startup standing arm pose reached.")
    else:
        logger_mp.warning("Startup standing arm pose timed out; holding current arm pose before release.")
    hold_current_arm_pose(
        arm_ctrl,
        hold_time=hold_time,
        velocity_limit=0.2,
        label="exit release pose",
        restore_previous=False,
    )
    return reached

def make_exit_retreat_waypoints(left_pose, right_pose, lateral_clearance=0.30, forward_clearance=0.25, min_height=0.12, lift=0.05):
    left_lateral = left_pose.copy()
    right_lateral = right_pose.copy()
    left_lateral[1, 3] = max(left_lateral[1, 3], lateral_clearance)
    right_lateral[1, 3] = min(right_lateral[1, 3], -lateral_clearance)
    left_lateral[2, 3] = max(left_lateral[2, 3] + lift, min_height)
    right_lateral[2, 3] = max(right_lateral[2, 3] + lift, min_height)

    left_forward = left_lateral.copy()
    right_forward = right_lateral.copy()
    left_forward[0, 3] = max(left_forward[0, 3], forward_clearance)
    right_forward[0, 3] = max(right_forward[0, 3], forward_clearance)

    return [(left_lateral, right_lateral), (left_forward, right_forward)]

def retreat_arms_before_motion_release(arm_ctrl, arm_ik, velocity_limit=0.8, waypoint_timeout=12.0):
    if not hasattr(arm_ik, "get_current_ee_poses"):
        logger_mp.warning("Exit retreat skipped: arm IK does not expose current EE poses.")
        return False

    logger_mp.info("Exit retreat: moving wrists outward before releasing arm SDK.")
    current_q = arm_ctrl.get_current_dual_arm_q()
    left_pose, right_pose = arm_ik.get_current_ee_poses(current_q)
    waypoints = make_exit_retreat_waypoints(left_pose, right_pose)

    completed_any = False
    for idx, (left_target, right_target) in enumerate(waypoints, start=1):
        current_q = arm_ctrl.get_current_dual_arm_q()
        current_dq = arm_ctrl.get_current_dual_arm_dq()
        sol_q, sol_tauff = arm_ik.solve_ik(left_target, right_target, current_q, current_dq)
        if sol_q is None or not np.all(np.isfinite(sol_q)):
            logger_mp.warning(f"Exit retreat waypoint {idx} skipped: IK target is invalid.")
            continue
        reached = move_dual_arm_to_q_slow(arm_ctrl, sol_q, sol_tauff, velocity_limit=velocity_limit, timeout=waypoint_timeout)
        completed_any = completed_any or reached
        if reached:
            logger_mp.info(f"Exit retreat waypoint {idx} reached.")
        else:
            logger_mp.warning(f"Exit retreat waypoint {idx} timed out; continuing with current arm pose.")
            break

    current_q = arm_ctrl.get_current_dual_arm_q()
    arm_ctrl.ctrl_dual_arm(current_q, np.zeros_like(current_q))
    time.sleep(0.3)
    return completed_any
