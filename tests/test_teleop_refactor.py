"""Hardware-free checks for teleoperation motion sequencing and recording."""
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from teleop import teleop_lifecycle as lifecycle
from teleop.robot_control import arm_motion
from teleop.teleop_config import parse_args
from teleop.utils.pose_utils import align_wrist_pose_to_start
from teleop.utils.teleop_recording import build_recording_payload


class TeleopRefactorTests(unittest.TestCase):
    def test_startup_raises_before_starting_and_opening_hands(self):
        events = []
        session = Mock()
        session.start.side_effect = lambda: events.append('start_hand')
        session.set_grasp.side_effect = lambda close, settle: events.append(('grasp', close))
        with patch.object(lifecycle, 'move_dual_arm_to_safety_pose', side_effect=lambda *a, **k: events.append('raise')):
            lifecycle.run_startup_sequence(parse_args([]), object(), session)
        self.assertEqual(events, ['raise', 'start_hand', ('grasp', False)])

    def test_disabled_startup_does_not_start_deferred_hand(self):
        session = Mock()
        with patch.object(lifecycle, 'move_dual_arm_to_safety_pose') as move:
            lifecycle.run_startup_sequence(parse_args(['--disable-hand-safety-sequence']), object(), session)
        session.start.assert_not_called()
        session.set_grasp.assert_not_called()
        move.assert_not_called()

    def test_shutdown_order_and_partial_initialization(self):
        events = []
        arm = Mock()
        arm.release_arm_sdk_mode.side_effect = lambda **k: events.append('release')
        switcher = Mock()
        switcher.Exit_Debug_Mode.side_effect = lambda: (events.append('exit_mode') or (0, None))
        session = Mock()
        session.set_grasp.side_effect = lambda close, settle: events.append(('grasp', close))
        fake_module = SimpleNamespace(MotionSwitcher=Mock(return_value=switcher))
        with patch.dict(sys.modules, {'teleop.utils.motion_switcher': fake_module}), patch.object(lifecycle, 'hold_current_arm_pose', side_effect=lambda *a, **k: events.append('hold')), patch.object(lifecycle, 'move_dual_arm_to_safety_pose', side_effect=lambda *a, **k: events.append('raise')), patch.object(lifecycle, 'move_dual_arm_to_startup_pose', side_effect=lambda *a, **k: events.append('lower') or True):
            lifecycle.run_shutdown_sequence(parse_args(['--motion']), arm, switcher, session)
            self.assertEqual(events, ['hold', 'raise', ('grasp', True), 'lower', 'release', 'exit_mode'])
            events.clear()
            lifecycle.run_shutdown_sequence(parse_args(['--sim', '--disable-hand-safety-sequence']))
            self.assertEqual(events, [])

    def test_safety_target_and_velocity_restoration(self):
        arm = Mock()
        arm.arm_velocity_limit = 2.0
        arm._speed_gradual_max = True
        arm.get_current_dual_arm_q.return_value = np.zeros(14)
        self.assertTrue(arm_motion.move_dual_arm_to_safety_pose(arm, hold_time=0))
        q, tau = arm.ctrl_dual_arm.call_args.args
        np.testing.assert_array_equal(q, np.zeros(14))
        np.testing.assert_array_equal(tau, np.zeros(14))
        self.assertEqual(arm.arm_velocity_limit, 2.0)
        self.assertTrue(arm._speed_gradual_max)

    def test_anchor_maps_initial_controller_pose_to_robot_pose(self):
        xr = np.eye(4)
        xr[:3, 3] = [1, 2, 3]
        robot = np.eye(4)
        robot[:3, 3] = [.2, .3, .4]
        np.testing.assert_allclose(align_wrist_pose_to_start(xr, xr, robot), robot)

    def test_recording_stereo_split_and_schema(self):
        config = {'head_camera': {'binocular': True, 'image_shape': [2, 4]}, 'left_wrist_camera': {'enable_zmq': False}, 'right_wrist_camera': {'enable_zmq': False}}
        pixels = np.arange(24).reshape(2, 4, 3)
        q = np.arange(7)
        colors, depths, states, actions = build_recording_payload(config, SimpleNamespace(bgr=pixels), None, None, q, q, q, q, [1], [2], [3], [4], [], [])
        np.testing.assert_array_equal(colors['color_0'], pixels[:, :2])
        np.testing.assert_array_equal(colors['color_1'], pixels[:, 2:])
        self.assertEqual(depths, {})
        self.assertEqual(states['left_arm'], {'qpos': q.tolist(), 'qvel': [], 'torque': []})
        self.assertEqual(actions['right_ee']['qpos'], [4])
        self.assertEqual(states['body'], {'qpos': []})

class FailurePathTests(unittest.TestCase):
    def test_early_failure_does_not_acquire_robot_mode_client(self):
        constructor = Mock()
        with patch.dict(sys.modules, {'teleop.utils.motion_switcher': SimpleNamespace(MotionSwitcher=constructor)}):
            lifecycle.run_shutdown_sequence(parse_args([]))
        constructor.assert_not_called()

    def test_startup_cancellation_does_not_open_hands(self):
        session = Mock()
        arm = Mock()
        arm.get_current_dual_arm_q.return_value = np.ones(14)
        cancel = Mock(side_effect=RuntimeError('tracking lost'))
        with self.assertRaisesRegex(RuntimeError, 'tracking lost'):
            lifecycle.run_startup_sequence(parse_args([]), arm, session, check_cancel=cancel)
        session.start.assert_not_called()
        session.set_grasp.assert_not_called()

    def test_invalid_runtime_rates_rejected(self):
        for flag in ('--frequency', '--xr-tracking-timeout'):
            for value in ('0', '-1', 'nan', 'inf'):
                with self.subTest(flag=flag, value=value), patch('sys.stderr'), self.assertRaises(SystemExit):
                    parse_args([flag, value])

class EntryPointFailureTests(unittest.TestCase):
    def test_failed_usb_start_cleans_up_without_initializing_dds(self):
        # Execute the actual main function with hardware imports replaced, so
        # importing this test never initializes robot libraries or a GUI.
        import ast
        from pathlib import Path
        tree = ast.parse(Path('teleop/teleop_hand_and_arm.py').read_text())
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'main')
        session = Mock()
        session.start.side_effect = RuntimeError('USB unavailable')
        namespace = dict(parse_args=parse_args, XRSession=Mock(return_value=session),
                         logger_mp=Mock(), ChannelFactoryInitialize=Mock(),
                         run_shutdown_sequence=Mock())
        exec(compile(ast.Module(body=[main], type_ignores=[]), '<teleop main>', 'exec'), namespace)
        with self.assertRaisesRegex(RuntimeError, 'USB unavailable'):
            namespace['main'](['--xr', 'meta_quest'])
        namespace['ChannelFactoryInitialize'].assert_not_called()
        self.assertFalse(namespace['run_shutdown_sequence'].call_args.kwargs['robot_initialized'])
        session.close.assert_called_once()

    def test_failed_shutdown_still_closes_xr(self):
        import ast
        from pathlib import Path
        tree = ast.parse(Path('teleop/teleop_hand_and_arm.py').read_text())
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'main')
        session = Mock()
        session.start.side_effect = RuntimeError('USB unavailable')
        namespace = dict(parse_args=parse_args, XRSession=Mock(return_value=session),
                         logger_mp=Mock(), run_shutdown_sequence=Mock(side_effect=RuntimeError('shutdown failed')))
        exec(compile(ast.Module(body=[main], type_ignores=[]), '<teleop main>', 'exec'), namespace)
        with self.assertRaisesRegex(RuntimeError, 'USB unavailable'):
            namespace['main'](['--xr', 'meta_quest'])
        session.close.assert_called_once()

class USBEntryPointTests(unittest.TestCase):
    def test_usb_controller_reaches_ik_and_renders_camera(self):
        import ast
        from pathlib import Path
        from teleop.xr_connection import XRSession
        tree = ast.parse(Path('teleop/teleop_hand_and_arm.py').read_text())
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'main')
        pose = np.eye(4)
        sample = SimpleNamespace(left_wrist_pose=pose, right_wrist_pose=pose,
                                 left_wrist_valid=True, right_wrist_valid=True)
        sample.head_valid = True
        wrapper = Mock()
        wrapper.get_tele_data.return_value = sample
        wrapper.tvuer.head_received_at.value = 10.
        wrapper.tvuer.input_received_at.value = 10.
        arm = Mock()
        arm.get_current_dual_arm_q.return_value = np.zeros(14)
        arm.get_current_dual_arm_dq.return_value = np.zeros(14)
        ik = Mock()
        ik.get_current_ee_poses.return_value = (pose, pose)
        ik.solve_ik.return_value = (np.ones(14), np.zeros(14))
        namespace = dict(parse_args=parse_args, XRSession=XRSession, logger_mp=Mock(),
                         ChannelFactoryInitialize=Mock(), IPC_Server=Mock(), ImageClient=Mock(),
                         G1_29_ArmIK=Mock(return_value=ik), G1_29_ArmController=Mock(return_value=arm),
                         create_end_effector=Mock(), run_startup_sequence=Mock(),
                         run_shutdown_sequence=Mock(), time=Mock(), np=np,
                         align_wrist_pose_to_start=align_wrist_pose_to_start,
                         on_press=Mock(), get_state=Mock())
        namespace['ImageClient'].return_value.get_cam_config.return_value = {
            'head_camera': dict(enable_zmq=True, enable_webrtc=True, webrtc_port=60001,
                                binocular=False, image_shape=[480, 640]),
            'left_wrist_camera': {'enable_zmq': False},
            'right_wrist_camera': {'enable_zmq': False}}
        namespace['ImageClient'].return_value.get_head_frame.return_value = SimpleNamespace(bgr='camera frame')
        namespace['time'].time.return_value = 10.
        namespace['IPC_Server'].return_value.start.side_effect = lambda: namespace.update(START=True)
        arm.ctrl_dual_arm.side_effect = lambda q, tau: namespace.update(STOP=True) if np.all(q == 1) else None
        exec(compile(ast.Module(body=[main], type_ignores=[]), '<teleop main>', 'exec'), namespace)
        with patch('teleop.usb_connection.QuestUSBConnection') as transport, patch.dict(sys.modules, {'televuer': SimpleNamespace(TeleVuerWrapper=Mock(return_value=wrapper))}), patch('time.monotonic', return_value=10.1):
            namespace['main'](['--xr', 'meta_quest', '--ipc', '--motion', '--upper-body-only', '--headless'])
        namespace['ImageClient'].assert_called_once()
        wrapper.render_to_xr.assert_called_with('camera frame')
        ik.solve_ik.assert_called_once()
        wrapper.close.assert_called_once()
        transport.return_value.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
