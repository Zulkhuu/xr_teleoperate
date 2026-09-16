"""Hardware-free network/native backend selection and tracking checks."""
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

from teleop.teleop_config import parse_args
from teleop.xr_connection import XRSession, xr_video_options


class XRTests(unittest.TestCase):
    def test_network_hud_selects_local_frames_without_changing_backend(self):
        import sys
        factory = Mock()
        config = {'head_camera': dict(enable_zmq=True, enable_webrtc=True,
                                     webrtc_port=60001, binocular=True, image_shape=[480, 1280])}
        session = XRSession(parse_args(['--xr', 'vuer']))
        with patch.dict(sys.modules, {'televuer': SimpleNamespace(TeleVuerWrapper=factory)}):
            self.assertTrue(session.configure_display(config, hud=True))
        self.assertFalse(factory.call_args.kwargs['webrtc'])
        self.assertIsNone(session.transport)
        config['head_camera']['enable_zmq'] = False
        with self.assertRaisesRegex(ValueError, 'HUD requires'):
            session.configure_display(config, hud=True)

    def test_defaults_and_video(self):
        config = {'head_camera': dict(enable_zmq=True, enable_webrtc=True, webrtc_port=60001)}
        self.assertTrue(xr_video_options(parse_args([]), config)[0]['webrtc'])
        for backend in ('vuer_usb', 'meta_quest'):
            args = parse_args(['--xr', backend])
            self.assertEqual((args.input_mode, args.display_mode), ('controller', 'immersive'))
            options, local = xr_video_options(args, config)
            self.assertFalse(options['webrtc'])
            self.assertIsNone(options['webrtc_url'])
            self.assertTrue(local)
            config['head_camera']['enable_zmq'] = False
            with self.assertRaisesRegex(ValueError, 'enable_zmq'):
                xr_video_options(args, config)
            args.display_mode = 'pass-through'
            self.assertFalse(xr_video_options(args, config)[1])
            config['head_camera']['enable_zmq'] = True

    def test_usb_hand_tracking_and_video_wrapper(self):
        import sys
        factory = Mock()
        config = {'head_camera': dict(enable_zmq=True, enable_webrtc=True,
                                     webrtc_port=60001, binocular=True, image_shape=[480, 1280])}
        args = parse_args(['--xr', 'vuer_usb', '--input-mode', 'hand'])
        session = XRSession(args)
        session.transport = Mock()
        session.start()
        with patch.dict(sys.modules, {'televuer': SimpleNamespace(TeleVuerWrapper=factory)}):
            self.assertTrue(session.configure_display(config))
        self.assertTrue(factory.call_args.kwargs['use_hand_tracking'])
        self.assertFalse(factory.call_args.kwargs['webrtc'])
        session.render_to_xr('frame')
        factory.return_value.render_to_xr.assert_called_once_with('frame')
        session.close()
        session.transport.close.assert_called_once()


class XRSessionTests(unittest.TestCase):
    @patch('time.monotonic', return_value=10.1)
    def test_poll_retains_buttons_when_pose_invalid_but_input_fresh(self, clock):
        session = self.session()
        session.wrapper.get_tele_data.return_value.left_wrist_valid = False
        data, valid, input_fresh = session.poll()
        self.assertFalse(valid)
        self.assertTrue(input_fresh)
        self.assertIs(data, session.wrapper.get_tele_data.return_value)

    def session(self, backend='vuer'):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from teleop.xr_connection import XRSession
        session = XRSession(parse_args(['--xr', backend]))
        if session.transport is not None:
            session.transport = Mock()
        session.wrapper = Mock()
        session.wrapper.tvuer.head_received_at.value = 10.0
        session.wrapper.tvuer.input_received_at.value = 10.0
        session.wrapper.get_tele_data.return_value = SimpleNamespace(
            head_valid=True, left_wrist_valid=True, right_wrist_valid=True)
        return session

    @patch('time.monotonic', return_value=10.1)
    def test_fresh_tracking_and_invalid_pose(self, clock):
        session = self.session()
        self.assertIs(session.read(), session.wrapper.get_tele_data.return_value)
        session.wrapper.get_tele_data.return_value.left_wrist_valid = False
        with self.assertRaisesRegex(RuntimeError, 'invalid'):
            session.read()

    @patch('time.monotonic', return_value=11.0)
    def test_stale_or_missing_head_and_input(self, clock):
        for head, hand in ((10, 11), (11, 10), (0, 11), (11, 0)):
            with self.subTest(head=head, hand=hand):
                session = self.session()
                session.wrapper.tvuer.head_received_at.value = head
                session.wrapper.tvuer.input_received_at.value = hand
                with self.assertRaisesRegex(RuntimeError, 'stale'):
                    session.read()

    def test_server_failure(self):
        session = self.session()
        session.wrapper.tvuer.process.is_alive.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'server stopped'):
            session.read()

    def test_usb_cleanup_even_if_wrapper_close_fails(self):
        from unittest.mock import Mock
        session = self.session('meta_quest')
        session.transport = Mock()
        session.wrapper.close.side_effect = RuntimeError('close failed')
        with self.assertRaises(RuntimeError):
            session.close()
        session.transport.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
