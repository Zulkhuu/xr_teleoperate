"""Exercise the G1 publisher without importing DDS or connecting to a robot."""
import ast
from pathlib import Path
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from teleop.robot_control.command_gate import ArmCommandGate
from teleop.teleop_config import parse_args


class StopPublisher(Exception):
    pass


class G1CommandRampTests(unittest.TestCase):
    def make_arm(self, frames, on_frame=None):
        tree = ast.parse(Path('teleop/robot_control/robot_arm.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                   and n.name == 'G1_29_ArmController')
        # Execute the production publisher and limiter with only transport,
        # clock and measured state substituted.
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                    and n.name in ('clip_arm_q_target', '_ctrl_motor_state')]
        writes = []
        ticks = [0]

        def sleep(_):
            ticks[0] += 1
            if on_frame:
                on_frame(arm, ticks[0])
            if ticks[0] >= frames:
                raise StopPublisher

        namespace = dict(np=np, ArmCommandGate=ArmCommandGate,
                         time=SimpleNamespace(time=lambda: ticks[0] * .004, sleep=sleep),
                         G1_29_JointIndex=SimpleNamespace(kNotUsedJoint0=14),
                         G1_29_JointArmIndex=range(14), logger_mp=Mock())
        exec(compile(ast.Module(body=[cls], type_ignores=[]), '<G1 publisher>', 'exec'), namespace)
        arm = namespace['G1_29_ArmController']()
        arm._init_command_gate()
        arm.ctrl_lock = threading.Lock()
        arm.q_target = np.zeros(14)
        arm.tauff_target = np.zeros(14)
        arm._last_command_q = np.ones(14)
        arm._trajectory_epoch = arm._command_epoch
        arm.get_current_dual_arm_q = Mock(return_value=np.ones(14))
        arm.arm_velocity_limit, arm.control_dt = .8, .004
        arm.arm_sdk_weight = 1.
        arm.motion_mode, arm.simulation_mode = True, False
        arm._speed_gradual_max = False
        arm.msg = SimpleNamespace(motor_cmd=[SimpleNamespace(q=0.) for _ in range(15)])
        arm.crc = SimpleNamespace(Crc=lambda _: 0)
        arm.lowcmd_publisher = SimpleNamespace(
            Write=lambda msg: writes.append(np.array([m.q for m in msg.motor_cmd[:14]])))
        return arm, writes

    def test_command_reaches_zero_even_when_feedback_stalls(self):
        arm, writes = self.make_arm(400)
        with self.assertRaises(StopPublisher):
            arm._ctrl_motor_state()
        trajectory = np.vstack([np.ones(14), writes])
        self.assertLessEqual(np.max(np.abs(np.diff(trajectory, axis=0))), .0032 + 1e-12)
        np.testing.assert_allclose(writes[-1], 0., atol=1e-12)

    def test_damping_inhibits_writes_and_recovery_resets_ramp(self):
        def transition(arm, tick):
            if tick == 2:
                arm.suspend_commands()
                arm.get_current_dual_arm_q.return_value = np.full(14, .5)
            if tick == 5:
                arm.q_target = np.full(14, .5)
                arm.resume_commands()

        arm, writes = self.make_arm(6, transition)
        with self.assertRaises(StopPublisher):
            arm._ctrl_motor_state()
        self.assertEqual(len(writes), 3)
        np.testing.assert_allclose(writes[-1], .5)

    def test_debugging_safety_timeout_default_and_override(self):
        self.assertEqual(parse_args([]).arm_safety_timeout, 10.)
        self.assertEqual(parse_args(['--arm-safety-timeout', '20']).arm_safety_timeout, 20.)
