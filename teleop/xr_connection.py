"""Vuer tracking and video over either the network or a USB tunnel."""
import time


def xr_video_options(args, camera_config):
    camera = camera_config['head_camera']
    usb = args.xr in ('vuer_usb', 'meta_quest')
    needs_image = args.display_mode != 'pass-through'
    if usb and needs_image and not camera['enable_zmq']:
        raise ValueError('USB video requires head_camera.enable_zmq=true on the image server')
    webrtc = camera['enable_webrtc'] and not usb
    return dict(zmq=camera['enable_zmq'], webrtc=webrtc,
                webrtc_url=f"https://{args.img_server_ip}:{camera['webrtc_port']}/offer" if webrtc else None), needs_image and not webrtc


class XRSession:
    """Own either backend and enforce fresh tracking before robot commands."""

    def __init__(self, args):
        self.args = args
        self.timeout = args.xr_tracking_timeout
        from teleop.usb_connection import QuestUSBConnection
        self.transport = (QuestUSBConnection(args.adb_path, args.quest_serial)
                          if args.xr in ('vuer_usb', 'meta_quest') else None)
        self.wrapper = None

    def start(self):
        if self.transport is not None:
            self.transport.start()

    def configure_display(self, camera_config, hud=False):
        options, needs_image = xr_video_options(self.args, camera_config)
        if hud and self.args.display_mode != 'pass-through':
            if not camera_config['head_camera']['enable_zmq']:
                raise ValueError('OpenCV XR HUD requires head_camera.enable_zmq=true')
            options.update(webrtc=False, webrtc_url=None)
            needs_image = True
        from televuer import TeleVuerWrapper
        camera = camera_config['head_camera']
        self.wrapper = TeleVuerWrapper(use_hand_tracking=self.args.input_mode == 'hand',
                                      binocular=camera['binocular'], img_shape=camera['image_shape'],
                                      display_mode=self.args.display_mode, **options)
        if self.transport is not None:
            import logging
            from teleop.usb_connection import QUEST_URL
            logging.getLogger(__name__).info('Open %s in Quest Browser, accept the local certificate, then enter VR.', QUEST_URL)
        return needs_image

    def render_to_xr(self, frame):
        if self.wrapper is not None:
            self.wrapper.render_to_xr(frame)

    def check(self):
        if self.transport is not None:
            self.transport.check()
        if self.wrapper is not None and not self.wrapper.tvuer.process.is_alive():
            raise RuntimeError('XR server stopped. Restart teleoperation.')

    def poll(self):
        """Return data, pose validity and input freshness without exiting on loss."""
        self.check()
        if self.wrapper is not None:
            tv = self.wrapper.tvuer
            received = min(tv.head_received_at.value, tv.input_received_at.value)
            data = self.wrapper.get_tele_data()
            valid = all((data.head_valid, data.left_wrist_valid, data.right_wrist_valid))
        else:
            raise RuntimeError('XR backend has not started')
        now = time.monotonic()
        fresh = received > 0 and now - received <= self.timeout
        input_time = tv.input_received_at.value
        input_fresh = input_time > 0 and now - input_time <= self.timeout
        return data, fresh and valid, input_fresh

    def read(self):
        data, valid, _ = self.poll()
        if not valid:
            raise RuntimeError('XR tracking is missing, invalid, or stale. Restart with active tracking.')
        return data

    def close(self):
        try:
            if self.wrapper is not None:
                self.wrapper.close()
        finally:
            if self.transport is not None:
                self.transport.close()
