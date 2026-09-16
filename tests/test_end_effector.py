"""Exercise real shared buffers with hardware constructors replaced by mocks."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from teleop.robot_control.end_effector import DEVICES, create_end_effector
from teleop.teleop_config import parse_args


class EndEffectorTests(unittest.TestCase):
    def make_session(self, device, mode, disabled=False):
        constructor = Mock()
        module = SimpleNamespace(**{name: constructor for _, name, _ in DEVICES.values()})
        argv = ['--input-mode', mode]
        if device:
            argv += ['--ee', device]
        if disabled:
            argv += ['--disable-hand-safety-sequence']
        with patch('teleop.robot_control.end_effector.import_module', return_value=module):
            session = create_end_effector(parse_args(argv))
        return session, constructor

    def test_all_device_modes_and_recording_widths(self):
        for device, (_, _, width) in DEVICES.items():
            for mode in ('hand', 'controller'):
                with self.subTest(device=device, mode=mode):
                    session, constructor = self.make_session(device, mode)
                    self.assertEqual(constructor.call_count, 0 if session.deferred else 1)
                    session.start()
                    session.start()
                    self.assertEqual(constructor.call_count, 1)
                    args, kwargs = constructor.call_args
                    self.assertEqual(kwargs.get('control_mode'), 'binary' if session.binary else None)
                    args[3][:] = range(2 * width)
                    args[4][:] = range(10, 10 + 2 * width)
                    snap = session.snapshot()
                    if device == 'brainco' and mode == 'controller':
                        self.assertEqual(snap.left_state, [])
                    else:
                        self.assertEqual(snap.left_state, list(range(width)))
                        self.assertEqual(snap.right_state, list(range(width, 2 * width)))
                        self.assertEqual(snap.right_action, list(range(10 + width, 10 + 2 * width)))

    def test_binary_threshold_and_lifecycle_commands(self):
        for device in ('dex3', 'inspire_ftp', 'inspire_dfx'):
            session, constructor = self.make_session(device, 'controller', disabled=True)
            left, right = constructor.call_args.args[:2]
            session.update(SimpleNamespace(left_ctrl_triggerValue=4.99, right_ctrl_triggerValue=5.0))
            self.assertEqual((left.value, right.value), (1, 0))
            session.set_grasp(False, 0)
            self.assertEqual((left.value, right.value), (0, 0))
            session.set_grasp(True, 0)
            self.assertEqual((left.value, right.value), (1, 1))

    def test_hand_positions_and_gripper_values(self):
        for device in ('dex3', 'inspire_ftp', 'inspire_dfx', 'brainco'):
            session, constructor = self.make_session(device, 'hand')
            session.update(SimpleNamespace(left_hand_pos=np.arange(75).reshape(25, 3),
                                           right_hand_pos=np.arange(75, 150).reshape(25, 3)))
            left, right = constructor.call_args.args[:2]
            self.assertEqual(left[:], list(range(75)))
            self.assertEqual(right[:], list(range(75, 150)))
            self.assertFalse(session.set_grasp(True, 0))
        for mode in ('hand', 'controller'):
            session, constructor = self.make_session('dex1', mode)
            session.update(SimpleNamespace(left_hand_pinchValue=2, right_hand_pinchValue=3,
                                           left_ctrl_triggerValue=7, right_ctrl_triggerValue=8))
            left, right = constructor.call_args.args[:2]
            self.assertEqual((left.value, right.value), (2, 3) if mode == 'hand' else (7, 8))

    def test_no_device_and_unmapped_brainco_controller(self):
        for device in (None, 'brainco'):
            session, constructor = self.make_session(device, 'controller')
            session.update(SimpleNamespace())
            self.assertEqual(session.snapshot().left_action, [])
            self.assertFalse(session.set_grasp(False, 0))
