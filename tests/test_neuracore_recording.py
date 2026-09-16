"""Recorder schema, SDK lifecycle and nonblocking episode controls without cloud access."""
import importlib.util
from pathlib import Path
import queue
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np

from teleop.teleop_config import parse_args
from teleop.utils.neuracore_recorder import JOINT_NAMES, ROOT, URDF, NeuracoreRecorder, make_sample


class NeuracoreTests(unittest.TestCase):
    def sample(self):
        self.raw = np.zeros((8, 16, 3), dtype=np.uint8)
        self.raw[:, :8] = [10, 20, 30]
        self.raw[:, 8:] = [40, 50, 60]
        config = dict(head_camera=dict(enable_zmq=True, binocular=True),
                      left_wrist_camera=dict(enable_zmq=True),
                      right_wrist_camera=dict(enable_zmq=False))
        cameras = dict(head_camera=SimpleNamespace(bgr=self.raw),
                       left_wrist_camera=SimpleNamespace(bgr=self.raw[:, :8]), right_wrist_camera=None)
        arm = Mock()
        arm.get_recording_state.return_value = dict(timestamp=123., q=np.arange(29), dq=np.arange(29) / 10)
        hands = SimpleNamespace(left_state=[1.] * 6, right_state=[2.] * 6,
                                left_action=[3.] * 6, right_action=[4.] * 6)
        with patch('teleop.utils.neuracore_recorder.time.time', return_value=124.):
            return make_sample(arm, np.arange(14), hands, cameras, config)

    def test_all_29_joints_rgb_and_timestamps(self):
        sample = self.sample()
        self.assertEqual(len(sample['positions']), 29)
        self.assertTrue(all(isinstance(v, float) for v in sample['positions'].values()))
        self.assertTrue(all(isinstance(v, float) for v in sample['targets'].values()))
        self.assertEqual(sample['positions']['left_hip_pitch_joint'], 0.)
        self.assertEqual(sample['positions']['waist_pitch_joint'], 14.)
        self.assertEqual(sample['positions']['right_wrist_yaw_joint'], 28.)
        self.assertEqual(len(sample['targets']), 14)
        self.assertNotIn('left_hip_pitch_joint', sample['targets'])
        self.assertEqual(sample['timestamp'], 123.)
        self.assertEqual(sample['target_timestamp'], 124.)
        self.assertEqual(set(sample['cameras']), {'head_camera_left', 'head_camera_right', 'left_wrist_camera'})
        stamp, shape, pixels = sample['cameras']['head_camera_left']
        self.assertEqual(stamp, 124.)
        np.testing.assert_array_equal(np.frombuffer(pixels, np.uint8).reshape(shape)[0, 0], [30, 20, 10])
        np.testing.assert_array_equal(self.raw[0, 0], [10, 20, 30])
        self.assertNotIn('depth', sample)

    def test_asset_names_and_all_meshes_resolve(self):
        root = ET.parse(URDF).getroot()
        self.assertTrue(set(JOINT_NAMES) <= {j.attrib['name'] for j in root.findall('joint')})
        for mesh in root.iter('mesh'):
            self.assertTrue((URDF.parent / mesh.attrib['filename']).is_file())

    def test_options_keep_local_default(self):
        self.assertEqual(parse_args(['--record']).recording_backend, 'episode')
        args = parse_args(['--record', '--recording-backend', 'neuracore', '--ee', 'inspire_ftp'])
        self.assertEqual(args.recording_backend, 'neuracore')
        for flags in (['--recording-backend', 'neuracore'],
                      ['--record', '--recording-backend', 'neuracore', '--ee', 'dex3']):
            with patch('sys.stderr'), self.assertRaises(SystemExit):
                parse_args(flags)

    def load_worker(self):
        sdk = Mock()
        spec = importlib.util.spec_from_file_location('nc_worker', ROOT / 'integrations/neuracore/worker.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'neuracore': sdk}):
            spec.loader.exec_module(module)
        return module, sdk

    def test_worker_timestamps_and_lifecycle(self):
        worker, sdk = self.load_worker()
        sample = self.sample()
        connection = Mock()
        connection.recv.side_effect = [dict(instruction='Pick up'),
                                       dict(op='start', timestamp=122.),
                                       dict(op='sample', sample=sample),
                                       dict(op='sample', sample=sample),
                                       dict(op='stop', timestamp=125.), dict(op='close')]
        with patch.object(worker, 'setup'):
            worker.run(connection)
        sdk.start_recording.assert_called_once_with(timestamp=122.)
        sdk.stop_recording.assert_called_once_with(wait=True, timestamp=125.)
        sdk.log_joint_positions.assert_called_once_with(sample['positions'], timestamp=123.)
        for name in ('log_joint_velocities', 'log_joint_target_positions', 'log_custom_1d', 'log_rgb'):
            for call in getattr(sdk, name).call_args_list:
                self.assertIn('timestamp', call.kwargs)
        sdk.log_depth.assert_not_called()

    def test_worker_finalizes_on_logging_error(self):
        worker, sdk = self.load_worker()
        sdk.log_joint_positions.side_effect = ValueError('bad sample')
        connection = Mock()
        connection.recv.side_effect = [dict(instruction='Test'), dict(op='start', timestamp=122.),
                                       dict(op='sample', sample=self.sample())]
        with patch.object(worker, 'setup'), patch.object(worker.traceback, 'print_exc'):
            worker.run(connection)
        sdk.stop_recording.assert_called_once()
        self.assertTrue(sdk.stop_recording.call_args.kwargs['wait'])
        self.assertEqual(connection.send.call_args.args[0], {'error': 'bad sample'})

    def test_backpressure_still_allows_stop_without_blocking(self):
        recorder = NeuracoreRecorder.__new__(NeuracoreRecorder)
        recorder.commands = queue.Queue()
        recorder.ready = threading.Event()
        recorder.ready.set()
        recorder.error = None
        recorder.active = False
        recorder.episode_id = 0
        recorder.process = Mock()
        recorder.process.poll.return_value = None
        self.assertTrue(recorder.create_episode())
        for _ in range(7):
            recorder.add_sample({})
        with self.assertRaisesRegex(RuntimeError, 'backlog'):
            recorder.add_sample({})
        recorder.save_episode()
        recorder.save_episode()
        self.assertFalse(recorder.is_ready())
        self.assertEqual([c['op'] for c in recorder.commands.queue].count('stop'), 1)

    def test_login_precedes_dds(self):
        import ast
        tree = ast.parse(Path('teleop/teleop_hand_and_arm.py').read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        namespace = dict(parse_args=parse_args, XRSession=Mock(), logger_mp=Mock(),
                         ChannelFactoryInitialize=Mock(), run_shutdown_sequence=Mock())
        exec(compile(ast.Module(body=[main], type_ignores=[]), '<main>', 'exec'), namespace)
        with patch('teleop.utils.neuracore_recorder.NeuracoreRecorder', side_effect=RuntimeError('login failed')):
            with self.assertRaisesRegex(RuntimeError, 'login failed'):
                namespace['main'](['--record', '--recording-backend', 'neuracore', '--ee', 'inspire_ftp'])
        namespace['ChannelFactoryInitialize'].assert_not_called()

    def test_joint_snapshot_reads_buffer_once_and_excludes_reserved_slots(self):
        import ast
        tree = ast.parse(Path('teleop/robot_control/robot_arm.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'G1_29_ArmController')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'get_recording_state')
        namespace = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<snapshot>', 'exec'), namespace)
        arm = Mock()
        arm.lowstate_buffer.GetData.return_value = SimpleNamespace(
            timestamp=123., motor_state=[SimpleNamespace(q=i, dq=-i) for i in range(35)])
        result = namespace['get_recording_state'](arm)
        arm.lowstate_buffer.GetData.assert_called_once()
        self.assertEqual(result['q'], list(range(29)))
        self.assertEqual(result['timestamp'], 123.)

    def test_runtime_stops_recording_on_backend_error_without_arm_commands(self):
        from teleop.teleop_runtime import TeleopRuntime
        runtime = TeleopRuntime.__new__(TeleopRuntime)
        runtime.machine = SimpleNamespace(recording=True)
        runtime.args = SimpleNamespace(recording_backend='neuracore')
        runtime.camera_config = {name: dict(enable_zmq=False) for name in
                                 ('head_camera', 'left_wrist_camera', 'right_wrist_camera')}
        runtime.head = runtime.left_image = runtime.right_image = None
        runtime.hands = Mock()
        runtime.arm = Mock()
        runtime.recorder = Mock()
        runtime.recorder.add_sample.side_effect = RuntimeError('upload failed')
        with patch('teleop.utils.neuracore_recorder.make_sample', return_value={}), self.assertLogs('teleop.teleop_runtime'):
            runtime.record_frame(None, np.zeros(14), np.zeros(14), [])
        self.assertFalse(runtime.machine.recording)
        runtime.recorder.save_episode.assert_called_once()
        runtime.arm.ctrl_dual_arm.assert_not_called()


if __name__ == '__main__':
    unittest.main()
