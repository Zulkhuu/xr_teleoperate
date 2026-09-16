"""Quest controls and the real orchestration loop, without SDK/XR hardware."""
import queue
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from teleop.robot_control.command_gate import ArmCommandGate
from teleop.teleop_config import parse_args
from teleop.teleop_runtime import TeleopRuntime
from teleop.utils.quest_control import (
    Button, Events, QuestControllerInput, TeleopState as S, TeleopStateMachine,
)
from teleop.utils.xr_hud import draw_hud


class ButtonTests(unittest.TestCase):
    def test_edges_and_tap_once(self):
        b = Button()
        b.update(False, 0)
        b.update(True, .1)
        self.assertTrue(b.pressed)
        b.update(True, .2)
        self.assertFalse(b.pressed)
        self.assertFalse(b.tap)
        b.update(False, .3)
        self.assertTrue(b.released)
        self.assertTrue(b.tap)
        b.update(False, .4)
        self.assertFalse(b.released)
        self.assertFalse(b.tap)

    def test_long_press_once_short_and_overlong_tap(self):
        b = Button()
        b.update(False, 0)
        b.update(True, .1)
        b.update(True, .4)
        self.assertFalse(b.long_pressed(.5))
        b.update(False, .45)
        self.assertFalse(b.long_pressed(.5))
        b.update(True, 1)
        b.update(True, 1.5)
        self.assertTrue(b.long_pressed(.5))
        self.assertFalse(b.long_pressed(.5))
        b.update(False, 2)
        self.assertFalse(b.tap)

    def test_physical_mapping_and_disconnect_does_not_synthesize_tap(self):
        buttons = QuestControllerInput()
        data = SimpleNamespace()
        buttons.update(data, 0)
        data.left_ctrl_bButton = True
        buttons.update(data, .1)
        self.assertFalse(buttons.events().record)
        buttons.update(data, .2)
        self.assertFalse(buttons.events().record)
        data.left_ctrl_bButton = False
        buttons.update(data, .3)
        self.assertTrue(buttons.events().record)
        data.right_ctrl_bButton = True
        buttons.update(data, .4)
        buttons.update(data, .5, fresh=False)
        data.right_ctrl_bButton = False
        buttons.update(data, .6)
        self.assertFalse(buttons.events().anchor)
        data.right_ctrl_bButton = True
        buttons.update(data, .7)
        data.right_ctrl_bButton = False
        buttons.update(data, .8)
        self.assertTrue(buttons.events().anchor)

    def test_reconnection_with_held_x_requires_release(self):
        buttons = QuestControllerInput()
        data = SimpleNamespace(left_ctrl_aButton=True)
        buttons.update(data, 0)
        buttons.update(data, 1)
        self.assertFalse(buttons.events().start)
        data.left_ctrl_aButton = False
        buttons.update(data, 2)
        data.left_ctrl_aButton = True
        buttons.update(data, 3)
        buttons.update(data, 3.5)
        self.assertTrue(buttons.events().start)


class MachineTests(unittest.TestCase):
    def test_safety_precedes_activation_and_recording(self):
        m = TeleopStateMachine()
        m.update(Events(), True, True)
        self.assertEqual(m.state, S.WAITING_FOR_ANCHOR)
        m.update(Events(anchor=True), True, True)
        self.assertTrue(m.anchor_requested)
        m.anchored()
        self.assertFalse(m.activate(False))
        self.assertTrue(m.activate(True))
        m.update(Events(pause=True, start=True, record=True), True, True)
        self.assertEqual(m.state, S.PAUSED)
        self.assertFalse(m.record_requested)
        self.assertFalse(m.activate(True))
        m.update(Events(damp=True, anchor=True, start=True), True, True)
        self.assertEqual(m.state, S.DAMPED)
        self.assertFalse(m.activate(True))


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch('time.monotonic', return_value=10.)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.data = SimpleNamespace(left_wrist_pose=np.eye(4), right_wrist_pose=np.eye(4),
                                    left_ctrl_thumbstickValue=[0, 0], right_ctrl_thumbstickValue=[0, 0])
        self.tracking = self.fresh = True
        self.xr = Mock()
        self.xr.poll.side_effect = lambda: (self.data, self.tracking, self.fresh)
        self.arm = ArmCommandGate()
        self.arm._init_command_gate()
        self.arm.get_current_dual_arm_q = Mock(return_value=np.zeros(14))
        self.arm.get_current_dual_arm_dq = Mock(return_value=np.zeros(14))
        self.arm.get_current_motor_q = Mock(return_value=np.zeros(35))
        self.arm.ctrl_dual_arm = Mock()
        self.arm.lowcmd_publisher = Mock()
        self.arm.msg = object()
        self.ik = Mock()
        self.ik.last_solve_valid = True
        self.ik.reduced_robot.model.lowerPositionLimit = np.full(14, -2.)
        self.ik.reduced_robot.model.upperPositionLimit = np.full(14, 2.)
        self.ik.get_current_ee_poses.return_value = (np.eye(4), np.eye(4))
        self.ik.solve_ik.return_value = (np.full(14, .5), np.zeros(14))
        self.raw = np.zeros((240, 640, 3), dtype=np.uint8)
        self.images = Mock()
        self.images.get_head_frame.return_value = SimpleNamespace(bgr=self.raw)
        self.recorder = Mock(episode_id=3)
        self.recorder.is_ready.return_value = True
        self.recorder.create_episode.return_value = True
        self.hands = Mock()
        self.hands.snapshot.return_value = SimpleNamespace(left_state=[], right_state=[], left_action=[], right_action=[])
        self.config = dict(head_camera=dict(enable_zmq=True, binocular=True, image_shape=[240, 640]),
                           left_wrist_camera=dict(enable_zmq=False), right_wrist_camera=dict(enable_zmq=False))
        self.keys = queue.SimpleQueue()
        self.runtime = TeleopRuntime(
            parse_args(['--input-mode', 'controller', '--disable-hand-safety-sequence']),
            self.xr, self.arm, self.ik, self.hands, self.images, self.config, True,
            recorder=self.recorder, damping=Mock(), keyboard=self.keys)
        self.step(10)

    def step(self, now, **buttons):
        self.now.return_value = now
        self.arm.state_received_at = now
        for name, value in buttons.items():
            setattr(self.data, QuestControllerInput.FIELDS[name], value)
        self.runtime.step()

    def anchor(self, now=11):
        self.step(now, b=True)
        self.assertFalse(self.runtime.machine.anchor)
        self.step(now + .1, b=False)
        self.step(now + .3)
        self.assertEqual(self.runtime.machine.state, S.ALIGNED)
        self.step(now + .31)

    def activate(self, now=12):
        self.step(now, x=True)
        self.step(now + .51)
        self.assertEqual(self.runtime.machine.state, S.ACTIVE)
        self.step(now + .52, x=False)

    def record(self, now=13):
        self.step(now, y=True)
        self.step(now + .1, y=False)

    def test_anchor_start_record_pause_exit_and_raw_stereo(self):
        self.anchor()
        self.arm.ctrl_dual_arm.reset_mock()
        self.step(11.4, x=True)
        self.step(11.6, x=False)
        self.assertEqual(self.runtime.machine.state, S.ALIGNED)
        self.activate()
        self.step(13, y=True)
        self.assertFalse(self.runtime.machine.recording)
        self.step(13.2)
        self.assertFalse(self.runtime.machine.recording)
        self.step(13.3, y=False)
        self.assertTrue(self.runtime.machine.recording)
        self.step(13.4)
        self.recorder.create_episode.assert_called_once()
        payload = self.recorder.add_item.call_args.kwargs
        self.assertFalse(payload['colors']['color_0'].any())
        self.assertFalse(self.raw.any())
        overlay = self.xr.render_to_xr.call_args.args[0]
        np.testing.assert_array_equal(overlay[:, :320], overlay[:, 320:])
        self.assertTrue(overlay.any())
        self.step(14, a=True)
        self.assertEqual(self.runtime.machine.state, S.PAUSED)
        self.recorder.save_episode.assert_called_once()
        calls = self.arm.ctrl_dual_arm.call_count
        self.step(14.2, a=False)
        self.assertGreater(self.arm.ctrl_dual_arm.call_count, calls)
        self.assertEqual(self.runtime.machine.state, S.PAUSED)
        self.step(15, a=True)
        self.step(16.51)
        self.assertEqual(self.runtime.machine.state, S.EXITING)

    def test_record_toggle_only_active_and_once_per_release(self):
        self.record(10.1)
        self.recorder.create_episode.assert_not_called()
        self.anchor()
        self.activate()
        self.record()
        self.record(14)
        self.assertFalse(self.runtime.machine.recording)
        self.recorder.save_episode.assert_called_once()
        self.step(15, y=True)
        self.step(16, y=False)
        self.assertFalse(self.runtime.machine.recording)

    def test_damping_latch_blocks_targets_until_anchor_and_new_hold(self):
        self.anchor()
        self.activate()
        self.record()
        self.arm.ctrl_dual_arm.reset_mock()
        self.step(14, l3=True, r3=True)
        self.assertEqual(self.runtime.machine.state, S.DAMPED)
        self.assertTrue(self.arm.commands_suspended)
        self.recorder.save_episode.assert_called_once()
        self.arm.ctrl_dual_arm.assert_not_called()
        self.step(14.1, l3=False, r3=False, x=True)
        self.step(15)
        self.assertEqual(self.runtime.machine.state, S.DAMPED)
        self.arm.ctrl_dual_arm.assert_not_called()
        self.runtime.damping.Enter_Damp_Mode.assert_called_once()
        self.step(15.1, x=False)
        self.anchor(16)
        self.assertTrue(self.arm.commands_suspended)
        self.activate(17)
        self.assertFalse(self.arm.commands_suspended)

    def test_tracking_loss_and_robot_stale_require_new_anchor(self):
        self.anchor()
        self.activate()
        self.record()
        self.tracking = False
        self.step(14)
        self.assertEqual(self.runtime.machine.state, S.PAUSED)
        self.assertFalse(self.runtime.machine.recording)
        self.tracking = True
        self.step(15, x=True)
        self.step(16)
        self.assertEqual(self.runtime.machine.state, S.PAUSED)
        self.step(16.1, x=False)
        self.anchor(17)
        self.now.return_value = 18
        self.runtime.step()  # Deliberately do not refresh robot state timestamp.
        self.assertEqual(self.runtime.machine.state, S.PAUSED)

    def test_pause_arriving_during_ik_prevents_target(self):
        self.anchor()
        self.activate()
        self.record()
        def solve(*args):
            self.data.right_ctrl_aButton = True
            return np.ones(14), np.zeros(14)
        self.ik.solve_ik.side_effect = solve
        self.arm.ctrl_dual_arm.reset_mock()
        self.step(14)
        self.assertEqual(self.runtime.machine.state, S.PAUSED)
        self.assertFalse(self.runtime.machine.recording)
        for call in self.arm.ctrl_dual_arm.call_args_list:
            np.testing.assert_array_equal(call.args[0], np.zeros(14))

    def test_failed_ik_and_joint_limits_block_start(self):
        for kind in ('solver', 'limits', 'nan'):
            with self.subTest(kind=kind):
                self.anchor(20)
                self.ik.last_solve_valid = kind != 'solver'
                self.ik.solve_ik.return_value = (np.full(14, np.nan if kind == 'nan' else 3.), np.zeros(14))
                self.step(21, x=True)
                self.step(21.6)
                self.assertEqual(self.runtime.machine.state, S.PAUSED)
                self.assertFalse(self.runtime.ever_activated)
                self.step(21.7, x=False)

    def test_preparation_precedes_anchor_and_tracking(self):
        self.runtime.prepared = False
        self.step(11, b=True)
        self.step(11.1, b=False)
        self.step(11.3)
        self.assertEqual(self.runtime.machine.state, S.WAITING_FOR_PREPARATION)
        self.assertFalse(self.runtime.machine.anchor_requested)
        self.ik.get_current_ee_poses.assert_not_called()
        self.arm.ctrl_dual_arm.reset_mock()
        with patch('teleop.teleop_runtime.run_startup_sequence') as prepare:
            self.step(12, x=True)
            prepare.assert_not_called()
            self.step(12.6)
            prepare.assert_called_once()
        self.assertEqual(self.runtime.machine.state, S.WAITING_FOR_ANCHOR)
        self.assertFalse(self.runtime.machine.anchor)
        self.ik.solve_ik.assert_not_called()
        self.step(12.7, x=False)
        self.anchor(13)
        self.activate(14)

    def test_damping_during_ik_never_sends_another_arm_target(self):
        self.anchor()
        self.activate()
        def solve(*args):
            self.data.left_ctrl_thumbstick = self.data.right_ctrl_thumbstick = True
            return np.ones(14), np.zeros(14)
        self.ik.solve_ik.side_effect = solve
        self.arm.ctrl_dual_arm.reset_mock()
        self.step(14)
        self.assertEqual(self.runtime.machine.state, S.DAMPED)
        self.arm.ctrl_dual_arm.assert_not_called()
        self.assertTrue(self.arm.commands_suspended)

    def test_preparation_pause_cancels_before_hands_open(self):
        self.runtime.prepared = False
        self.step(11)
        def prepare(args, arm, hands, check_cancel):
            self.data.right_ctrl_aButton = True
            check_cancel()
            hands.set_grasp(False)  # Must never be reached.
        with patch('teleop.teleop_runtime.run_startup_sequence', side_effect=prepare):
            self.step(12, x=True)
            self.step(12.6)
        self.assertEqual(self.runtime.machine.state, S.PAUSED)
        self.assertFalse(self.runtime.prepared)
        self.hands.set_grasp.assert_not_called()

    def test_failed_preparation_blocks_anchor_and_can_retry_without_anchor(self):
        self.runtime.prepared = False
        with patch('teleop.teleop_runtime.run_startup_sequence', side_effect=RuntimeError('timeout')):
            self.step(11, x=True)
            self.step(11.6)
        self.assertFalse(self.runtime.prepared)
        self.step(12, x=False, b=True)
        self.step(12.1, b=False)
        self.assertFalse(self.runtime.machine.anchor_requested)
        with patch('teleop.teleop_runtime.run_startup_sequence') as prepare:
            self.step(13, x=True)
            self.step(13.6)
            prepare.assert_called_once()
        self.assertEqual(self.runtime.machine.state, S.WAITING_FOR_ANCHOR)

    def test_damped_preparation_requires_recovery_ack_before_retry(self):
        self.runtime.prepared = False
        def damp(args, arm, hands, check_cancel):
            self.data.left_ctrl_thumbstick = self.data.right_ctrl_thumbstick = True
            check_cancel()
        with patch('teleop.teleop_runtime.run_startup_sequence', side_effect=damp):
            self.step(11, x=True)
            self.step(11.6)
        self.assertEqual(self.runtime.machine.state, S.DAMPED)
        self.assertTrue(self.arm.commands_suspended)
        self.step(12, x=False, l3=False, r3=False)
        with patch('teleop.teleop_runtime.run_startup_sequence') as prepare:
            self.step(13, x=True)
            self.step(13.6)
            prepare.assert_not_called()
            self.step(14, x=False, b=True)
            self.step(14.1, b=False)
            self.assertEqual(self.runtime.machine.state, S.WAITING_FOR_PREPARATION)
            self.assertFalse(self.runtime.machine.anchor)
            self.assertTrue(self.arm.commands_suspended)
            self.step(15, x=True)
            self.step(15.6)
            prepare.assert_called_once()
        self.assertEqual(self.runtime.machine.state, S.WAITING_FOR_ANCHOR)

    def test_runtime_failure_finalizes_recording_without_return_motion(self):
        self.anchor()
        self.activate()
        self.record()
        self.xr.poll.side_effect = RuntimeError('server stopped')
        with self.assertRaisesRegex(RuntimeError, 'server stopped'):
            self.runtime.run()
        self.assertFalse(self.runtime.machine.recording)
        self.recorder.save_episode.assert_called_once()

    def test_anchor_waits_for_stability_and_keyboard_uses_same_guards(self):
        self.keys.put('r')
        self.step(11)
        self.assertEqual(self.runtime.machine.state, S.WAITING_FOR_ANCHOR)
        self.keys.put('b')
        self.step(12)
        self.data.left_wrist_pose[0, 3] += .1
        self.step(12.2)
        self.assertFalse(self.runtime.machine.anchor)
        self.step(12.4)
        self.assertTrue(self.runtime.machine.anchor)
        self.keys.put('r')
        self.step(13)
        self.assertEqual(self.runtime.machine.state, S.ACTIVE)
        self.keys.put('p')
        self.step(14)
        self.assertEqual(self.runtime.machine.state, S.PAUSED)


class GateAndHUDTests(unittest.TestCase):
    def test_every_robot_publisher_honors_gate(self):
        # Execute each actual publishing loop for one iteration with fake DDS.
        # This catches integration failures hidden by runtime-level arm mocks.
        import ast
        import threading
        from pathlib import Path
        tree = ast.parse(Path('teleop/robot_control/robot_arm.py').read_text())
        for cls in tree.body:
            if not isinstance(cls, ast.ClassDef) or not cls.name.endswith('_ArmController'):
                continue
            method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_ctrl_motor_state')
            namespace = {'np': np, 'time': Mock(), 'logger_mp': Mock()}
            namespace['time'].time.return_value = 1.
            namespace['time'].sleep.side_effect = RuntimeError('iteration done')
            for name in ('G1_29', 'G1_23', 'H1_2', 'H1', 'H2'):
                namespace[name + '_JointArmIndex'] = [0, 1]
                namespace[name + '_JointIndex'] = SimpleNamespace(kNotUsedJoint0=2)
            exec(compile(ast.Module(body=[method], type_ignores=[]), '<publisher>', 'exec'), namespace)
            for suspended in (False, True):
                with self.subTest(controller=cls.name, suspended=suspended):
                    arm = ArmCommandGate()
                    arm._init_command_gate()
                    arm.ctrl_lock = threading.Lock()
                    arm.q_target = arm.tauff_target = np.zeros(14)
                    arm._last_command_q = arm.q_target.copy()
                    arm._trajectory_epoch = arm._command_epoch
                    arm.get_current_dual_arm_q = Mock(return_value=arm.q_target.copy())
                    arm.arm_sdk_weight = 1.
                    arm.motion_mode = arm.simulation_mode = True
                    arm._speed_gradual_max = False
                    arm.control_dt = .004
                    arm.msg = SimpleNamespace(motor_cmd=[SimpleNamespace() for _ in range(3)])
                    arm.crc = Mock()
                    arm.lowcmd_publisher = Mock()
                    if suspended:
                        arm.suspend_commands()
                    with self.assertRaisesRegex(RuntimeError, 'iteration done'):
                        namespace['_ctrl_motor_state'](arm)
                    self.assertEqual(arm.lowcmd_publisher.Write.call_count, 0 if suspended else 1)

    def test_gate_rejects_inflight_commands_across_damp_and_resume(self):
        arm = ArmCommandGate()
        arm._init_command_gate()
        arm.lowcmd_publisher = Mock()
        arm.msg = object()
        epoch = arm._command_epoch
        arm._publish_command(epoch)
        arm.lowcmd_publisher.Write.assert_called_once()
        arm.lowcmd_publisher.reset_mock()
        arm.suspend_commands()
        arm._publish_command(arm._command_epoch)
        arm.resume_commands()
        arm._publish_command(epoch)
        arm.lowcmd_publisher.Write.assert_not_called()
        arm._publish_command(arm._command_epoch)
        arm.lowcmd_publisher.Write.assert_called_once()

    def test_all_states_mono_stereo(self):
        for state in S:
            for stereo in (True, False):
                frame = np.zeros((240, 640, 3), dtype=np.uint8)
                draw_hud(frame, state, False, binocular=stereo)
                self.assertTrue(frame.any())
                if stereo:
                    np.testing.assert_array_equal(frame[:, :320], frame[:, 320:])
