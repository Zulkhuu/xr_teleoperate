import sys
import unittest
from unittest.mock import Mock, patch
from teleop import check_xr


class DiagnosticTests(unittest.TestCase):
    def test_no_robot_imports_video_and_cleanup_without_tracking(self):
        session = Mock()
        session.read.side_effect = RuntimeError('missing tracking')
        with patch.object(check_xr, 'XRSession', return_value=session), \
             patch.object(check_xr.time, 'monotonic', side_effect=[0, .01, .02, .025, .2]):
            self.assertEqual(check_xr.main(['--duration', '.1', '--stereo']), 1)
        self.assertEqual(session.render_to_xr.call_args.args[0].shape, (480, 1280, 3))
        session.close.assert_called_once()
        self.assertNotIn('teleop.robot_control.robot_arm', sys.modules)
        self.assertNotIn('unitree_sdk2py.core.channel', sys.modules)

    def test_animated_mono_and_stereo(self):
        import numpy as np
        self.assertEqual(check_xr.test_frame(0).shape, (480, 640, 3))
        self.assertFalse(np.array_equal(check_xr.test_frame(0), check_xr.test_frame(1)))
