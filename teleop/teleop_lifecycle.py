"""Coordinate arm and hand startup and shutdown in their established order."""

import logging_mp

logger_mp = logging_mp.getLogger(__name__)

from teleop.robot_control.arm_motion import (
    hold_current_arm_pose, move_dual_arm_to_safety_pose,
    move_dual_arm_to_startup_pose,
)

def run_startup_sequence(args, arm_ctrl, end_effector=None, check_cancel=None):
    """Raise arms, start deferred hand control, then open hands."""
    if not args.disable_hand_safety_sequence:
        logger_mp.info("Pre-teleop safety sequence: raise arms to safety pose before opening hands.")
        move_dual_arm_to_safety_pose(
            arm_ctrl,
            velocity_limit=args.arm_safety_velocity,
            timeout=args.arm_safety_timeout,
            check_cancel=check_cancel,
        )
        if check_cancel is not None:
            check_cancel()
        if end_effector is not None:
            end_effector.start()
            end_effector.set_grasp(False, args.hand_safety_settle)


def run_shutdown_sequence(args, arm_ctrl=None, motion_switcher=None, end_effector=None, robot_initialized=False):
    """Run the existing exit motion and mode-release sequence."""
    from teleop.utils.motion_switcher import MotionSwitcher

    exit_lower_completed = False
    try:
        if arm_ctrl is not None:
            logger_mp.info("Exit safety sequence: hold current pose before any return motion.")
            hold_current_arm_pose(
                arm_ctrl,
                hold_time=args.exit_initial_hold,
                velocity_limit=0.2,
                label="current teleop arm pose",
                restore_previous=False,
            )
            logger_mp.info("Exit safety sequence: move arms to safety pose before closing hands.")
            move_dual_arm_to_safety_pose(
                arm_ctrl,
                velocity_limit=args.arm_safety_velocity,
                timeout=args.arm_safety_timeout,
                hold_time=args.exit_safety_hold,
                restore_previous=False,
            )
        if not args.disable_hand_safety_sequence:
            if end_effector is not None:
                end_effector.set_grasp(True, args.hand_safety_settle)
        if arm_ctrl is not None:
            if args.motion and not args.disable_exit_lower:
                exit_lower_completed = move_dual_arm_to_startup_pose(
                    arm_ctrl,
                    velocity_limit=args.exit_lower_velocity,
                    timeout=args.exit_lower_timeout,
                    hold_time=args.exit_hold_before_release,
                )
            else:
                hold_current_arm_pose(
                    arm_ctrl,
                    hold_time=args.exit_hold_before_release,
                    velocity_limit=0.2,
                    label="exit pose",
                    restore_previous=False,
                )
    except TypeError:
        if arm_ctrl is not None:
            arm_ctrl.ctrl_dual_arm_go_home()
    except Exception as e:
        logger_mp.error(f"Failed to run arm exit motion: {e}")

    try:
        if args.motion and arm_ctrl is not None and hasattr(arm_ctrl, "release_arm_sdk_mode"):
            if not args.disable_exit_lower and not exit_lower_completed:
                logger_mp.warning("Releasing arm SDK after controlled lowering did not fully complete; release will stay slow.")
            arm_ctrl.release_arm_sdk_mode(duration=args.exit_release_duration)
    except Exception as e:
        logger_mp.error(f"Failed to release arm sdk mode: {e}")

    try:
        if not args.sim and (robot_initialized or motion_switcher is not None):
            if motion_switcher is None:
                motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Exit_Debug_Mode()
            logger_mp.info(f"Switch to AI/remote mode: {'Success' if status == 0 else 'Failed'}")
    except Exception as e:
        logger_mp.error(f"Failed to switch to AI/remote mode: {e}")

