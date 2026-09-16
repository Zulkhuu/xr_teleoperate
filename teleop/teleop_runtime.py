"""Quest operator workflow; hardware is supplied by the entry point."""
import logging
import time

import numpy as np

from teleop.teleop_lifecycle import run_startup_sequence
from teleop.utils.pose_utils import align_wrist_pose_to_start
from teleop.utils.quest_control import (
    ACTIVATION_RAMP_SEC, ANCHOR_POSITION_TOL, ANCHOR_ROTATION_TOL,
    ANCHOR_SETTLE_SEC, ARM_TARGET_RATE, ROBOT_STALE_SEC,
    QuestControllerInput, TeleopState, TeleopStateMachine,
)
from teleop.utils.teleop_recording import build_recording_payload
from teleop.utils.xr_hud import draw_hud, prepare_xr_image

logger = logging.getLogger(__name__)


class PreparationCancelled(Exception):
    pass


class TeleopRuntime:
    def __init__(self, args, xr, arm, ik, hands, images, camera_config,
                 local_image, recorder=None, loco=None, damping=None,
                 keyboard=None, sim_state=None, prepared=None):
        self.args, self.xr, self.arm, self.ik, self.hands = args, xr, arm, ik, hands
        self.images, self.camera_config, self.local_image = images, camera_config, local_image
        self.recorder, self.loco, self.damping = recorder, loco, damping
        self.keyboard, self.sim_state = keyboard, sim_state
        self.machine = TeleopStateMachine()
        self.buttons = QuestControllerInput()
        self.xr_anchor = self.robot_anchor = None
        self.settle_pose = None
        self.settle_since = None
        self.record_started = 0.0
        self.started_at = 0.0
        self.last_target = None
        self.frozen_q = None
        self.frozen_tauff = None
        self.prepared = args.disable_hand_safety_sequence if prepared is None else bool(prepared)
        self.ever_activated = False
        self.damping_latched = False
        self.head = self.left_image = self.right_image = None

    def stop_recording(self):
        if self.machine.recording:
            self.machine.recording = False
            self.recorder.save_episode()
            logger.info('[Teleop] Recording stopped')

    def freeze(self):
        if not self.arm.commands_suspended:
            if self.frozen_q is None:
                self.frozen_q = self.arm.get_current_dual_arm_q().copy()
                previous_tau = getattr(self.arm, 'tauff_target', None)
                self.frozen_tauff = (np.asarray(previous_tau, dtype=float).copy()
                                     if previous_tau is not None
                                     and np.shape(previous_tau) == np.shape(self.frozen_q)
                                     else np.zeros_like(self.frozen_q))
            q = self.frozen_q
            if np.all(np.isfinite(q)):
                self.arm.ctrl_dual_arm(q.copy(), self.frozen_tauff.copy())
        if self.loco is not None and not self.damping_latched:
            self.loco.Move(0.0, 0.0, 0.0)

    def read_operator(self):
        data, tracking, input_fresh = self.xr.poll()
        now = time.monotonic()
        self.buttons.update(data, now, input_fresh and self.args.input_mode == 'controller')
        event = self.buttons.events()
        if self.keyboard is not None:
            while not self.keyboard.empty():
                key = self.keyboard.get_nowait()
                if key == 'b':
                    event.anchor = True
                elif key == 'r':
                    event.start = True
                elif key == 's':
                    event.record = True
                elif key == 'p':
                    event.pause = True
                elif key == 'q':
                    # Keyboard debugging equivalent: pause, then request exit.
                    self.machine.pause('Keyboard exit')
                    event.exit = True
        q = self.arm.get_current_dual_arm_q()
        dq = self.arm.get_current_dual_arm_dq()
        robot_ready = (q.shape == dq.shape and q.ndim == 1
                       and np.all(np.isfinite(q)) and np.all(np.isfinite(dq))
                       and now - self.arm.state_received_at <= ROBOT_STALE_SEC)
        tracking = tracking and all(
            np.shape(p) == (4, 4) and np.all(np.isfinite(p))
            for p in (data.left_wrist_pose, data.right_wrist_pose))
        previous = self.machine.state
        self.machine.update(event, tracking, robot_ready, prepared=self.prepared)
        if self.machine.state == TeleopState.DAMPED and not self.damping_latched:
            # Application damping, NOT a replacement for the physical G1 E-stop.
            self.arm.suspend_commands()
            self.damping_latched = True
            self.stop_recording()
            if self.damping is None:
                # Construct lazily: initializing LocoClient during startup can
                # contend with rt/arm_sdk even in --upper-body-only mode.
                try:
                    from teleop.utils.motion_switcher import LocoClientWrapper
                    self.damping = LocoClientWrapper()
                except Exception:
                    logger.exception('Could not initialize damping RPC')
            if self.damping is not None:
                try:
                    self.damping.Enter_Damp_Mode()
                except Exception:
                    logger.exception('Damp RPC failed; arm publisher remains inhibited')
            else:
                logger.error('No verified damping RPC for this robot; publisher inhibited')
        elif previous in (TeleopState.ACTIVE, TeleopState.PREPARING) and self.machine.state != previous:
            self.freeze()
        if self.machine.state != TeleopState.ACTIVE:
            self.stop_recording()
        if not self.machine.anchor_requested:
            self.settle_pose = self.settle_since = None
        return data, tracking, robot_ready, q, dq, event, now

    def capture_anchor(self, data, q, now):
        if not self.prepared:
            return
        poses = (data.left_wrist_pose, data.right_wrist_pose)
        stable = self.settle_pose is not None and all(
            np.linalg.norm(p[:3, 3] - old[:3, 3]) <= ANCHOR_POSITION_TOL
            and np.linalg.norm(p[:3, :3] - old[:3, :3]) <= ANCHOR_ROTATION_TOL
            for p, old in zip(poses, self.settle_pose))
        if not stable:
            self.settle_pose = tuple(p.copy() for p in poses)
            self.settle_since = now
            return
        if now - self.settle_since < ANCHOR_SETTLE_SEC:
            return
        robot_poses = None
        if self.args.input_mode == 'controller' and not self.args.absolute_wrist_pose:
            if hasattr(self.ik, 'get_current_ee_poses'):
                robot_poses = self.ik.get_current_ee_poses(q)
                if not all(np.shape(p) == (4, 4) and np.all(np.isfinite(p)) for p in robot_poses):
                    return
            else:
                logger.warning('FK unavailable: retaining existing absolute wrist conversion')
        self.xr_anchor = tuple(p.copy() for p in poses)
        self.robot_anchor = tuple(p.copy() for p in robot_poses) if robot_poses is not None else None
        self.machine.anchored()
        # X must begin a new deliberate hold after this anchor is committed.
        self.buttons.x.require_release()
        # Capturing an anchor never enables the background publisher.

    def solve(self, data, q, dq):
        poses = (data.left_wrist_pose, data.right_wrist_pose)
        if self.robot_anchor is not None:
            poses = tuple(align_wrist_pose_to_start(p, x, r)
                          for p, x, r in zip(poses, self.xr_anchor, self.robot_anchor))
        try:
            target, torque = self.ik.solve_ik(*poses, q, dq)
            valid = (self.ik.last_solve_valid and np.shape(target) == q.shape
                     and np.shape(torque) == q.shape and np.all(np.isfinite(target))
                     and np.all(np.isfinite(torque)))
            model = self.ik.reduced_robot.model
            valid = valid and np.all(target >= model.lowerPositionLimit) and np.all(target <= model.upperPositionLimit)
            return target, torque, bool(valid)
        except Exception:
            logger.exception('IK failed; activation/control denied')
            return q, np.zeros_like(q), False

    def prepare(self):
        """Existing safety motion is explicit, interruptible, and invalidates anchoring."""
        self.machine.anchor = self.machine.anchor_requested = False
        self.xr_anchor = self.robot_anchor = None
        self.machine.transition(TeleopState.PREPARING, 'A: interrupt preparation')
        def check_cancel():
            _, tracking, ready, _, _, event, _ = self.read_operator()
            self.render(tracking, 'Preparing arms; A pauses')
            if (not tracking or not ready or event.pause or event.pause_down
                    or self.machine.state != TeleopState.PREPARING):
                raise PreparationCancelled
        try:
            check_cancel()
            run_startup_sequence(self.args, self.arm, self.hands, check_cancel=check_cancel)
            check_cancel()
            self.prepared = True
        except PreparationCancelled:
            pass
        except RuntimeError:
            logger.exception('Arm preparation failed; hold X to retry')
        finally:
            if self.machine.state not in (TeleopState.DAMPED, TeleopState.EXITING):
                if self.prepared:
                    self.machine.transition(TeleopState.WAITING_FOR_ANCHOR,
                                            'Preparation finished; B: set anchor')
                else:
                    self.machine.pause('Preparation interrupted; hold X to retry')
            self.buttons.x.require_release()
            self.freeze()

    def render(self, tracking, reason=None):
        if self.camera_config['head_camera']['enable_zmq']:
            self.head = self.images.get_head_frame()
        if self.local_image and self.head is not None and self.head.bgr is not None:
            # EpisodeWriter continues to receive self.head, whose BGR stays raw.
            image = prepare_xr_image(
                self.head.bgr,
                binocular=self.camera_config['head_camera']['binocular'],
                scale=getattr(self.args, 'xr_image_scale', 1.0),
                offset_y=getattr(self.args, 'xr_image_offset_y', 0.0))
            draw_hud(image, self.machine.state, tracking,
                     recording=self.machine.recording,
                     elapsed=time.monotonic() - self.record_started,
                     episode=self.recorder.episode_id if self.recorder else 0,
                     binocular=self.camera_config['head_camera']['binocular'],
                     reason=self.machine.reason if reason is None else reason,
                     preparation_required=not self.prepared,
                     recording_enabled=self.recorder is not None)
            self.xr.render_to_xr(image)

    def record_frame(self, data, q, target, loco_action):
        if not self.machine.recording:
            return
        if self.camera_config['left_wrist_camera']['enable_zmq']:
            self.left_image = self.images.get_left_wrist_frame()
        if self.camera_config['right_wrist_camera']['enable_zmq']:
            self.right_image = self.images.get_right_wrist_frame()
        ee = self.hands.snapshot()
        if getattr(self.args, 'recording_backend', 'episode') == 'neuracore':
            from teleop.utils.neuracore_recorder import make_sample
            try:
                sample = make_sample(self.arm, target, ee,
                                     dict(head_camera=self.head, left_wrist_camera=self.left_image,
                                          right_wrist_camera=self.right_image), self.camera_config)
                self.recorder.add_sample(sample)
            except Exception:
                logger.exception('[Neuracore] Recording stopped because data or recorder failed')
                self.stop_recording()
            return
        body = self.arm.get_current_motor_q().tolist() if loco_action else []
        colors, depths, states, actions = build_recording_payload(
            self.camera_config, self.head, self.left_image, self.right_image,
            q[:7], q[-7:], target[:7], target[-7:], ee.left_state, ee.right_state,
            ee.left_action, ee.right_action, body, loco_action)
        extra = {'sim_state': self.sim_state.read_data()} if self.sim_state else {}
        self.recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, **extra)

    def step(self):
        data, tracking, ready, q, dq, event, now = self.read_operator()
        if (not self.prepared and event.start and tracking and ready
                and not event.pause_down and not event.pause and not event.anchor
                and self.machine.state in (TeleopState.WAITING_FOR_PREPARATION,
                                           TeleopState.PAUSED)):
            # The fixed safety trajectory precedes XR anchoring and needs no
            # controller-target IK. Tracking activation remains separately gated.
            self.arm.ctrl_dual_arm(q.copy(), np.zeros_like(q))
            self.arm.resume_commands()
            self.damping_latched = False
            self.ever_activated = True
            self.prepare()
            self.render(tracking)
            return
        capturing = self.machine.anchor_requested
        if capturing and tracking and ready and not event.pause_down:
            self.capture_anchor(data, q, now)
        target, torque = q, np.zeros_like(q)
        start = (event.start and not capturing and not event.pause_down and not event.anchor
                 and self.machine.state == TeleopState.ALIGNED and tracking and ready)
        if start or self.machine.state == TeleopState.ACTIVE:
            target, torque, valid = self.solve(data, q, dq)
            # Solving may be slow: consume pause/damping/tracking changes again
            # before permitting ANY new arm target or recording item.
            _, tracking, ready, q, dq, late_event, checked_at = self.read_operator()
            event.record = event.record or late_event.record
            valid = valid and tracking and ready
            if not valid:
                self.machine.pause('IK or tracking invalid; B: re-align')
                self.freeze()
                self.stop_recording()
            elif start and self.machine.state == TeleopState.ALIGNED:
                if self.prepared and self.machine.activate(valid):
                    self.last_target = q.copy()
                    self.frozen_q = self.frozen_tauff = None
                    self.started_at = checked_at
                    self.arm.ctrl_dual_arm(q.copy(), np.zeros_like(q))
                    self.arm.resume_commands()
                    self.damping_latched = False
                    self.ever_activated = True
        loco_action = []
        if self.machine.state == TeleopState.ACTIVE:
            blend = min(1., max(0., (time.monotonic() - self.started_at) / ACTIVATION_RAMP_SEC))
            delta = (target - self.last_target) * blend
            limit = ARM_TARGET_RATE / self.args.frequency
            delta /= max(1., np.max(np.abs(delta)) / limit)
            target = self.last_target + delta
            self.arm.ctrl_dual_arm(target, torque * blend)
            self.last_target = target.copy()
            self.hands.update(data)
            if self.args.input_mode == 'controller' and (self.loco is not None or self.args.ee == 'dex1'):
                loco_action = [-data.left_ctrl_thumbstickValue[1] * .3,
                               -data.left_ctrl_thumbstickValue[0] * .3,
                               -data.right_ctrl_thumbstickValue[0] * .3]
                if self.loco is not None:
                    self.loco.Move(*loco_action)
            if event.record and self.recorder is not None:
                if self.machine.recording:
                    self.stop_recording()
                elif self.recorder.is_ready() and self.recorder.create_episode():
                    self.machine.recording = True
                    self.record_started = now
                    logger.info('[Teleop] Recording started')
        else:
            # Keep the captured pause pose commanded every loop. This is
            # needed because the arm publisher runs independently of XR input.
            self.freeze()
        self.render(tracking)
        self.record_frame(data, q, target, loco_action)

    def run(self):
        try:
            while self.machine.state != TeleopState.EXITING:
                start = time.monotonic()
                self.step()
                time.sleep(max(0., 1 / self.args.frequency - (time.monotonic() - start)))
        finally:
            try:
                self.freeze()
            finally:
                self.stop_recording()
