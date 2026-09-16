"""Check headset tracking and video without DDS, IK, robot, or simulator.

Run from the repository root: uv run python -m teleop.check_xr
"""
import argparse
import logging
import time

import cv2
import numpy as np

from teleop.teleop_config import parse_args as teleop_args
from teleop.xr_connection import XRSession

logger = logging.getLogger(__name__)


def test_frame(elapsed, stereo=False):
    """Animated BGR image; visible motion makes a frozen stream apparent."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:] = (45, 25, 15)
    cv2.putText(frame, 'USB XR CHECK - NO ROBOT', (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
    cv2.putText(frame, f'Host elapsed: {elapsed:.1f} s', (30, 110),
                cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
    cv2.circle(frame, (40 + int(elapsed * 120) % 560, 260), 30, (0, 220, 100), -1)
    if stereo:
        left, right = frame.copy(), frame.copy()
        for eye, label in ((left, 'LEFT EYE'), (right, 'RIGHT EYE')):
            cv2.putText(eye, label, (30, 420), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return np.concatenate((left, right), axis=1)
    return frame


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xr', choices=['vuer_usb', 'meta_quest', 'vuer'], default='vuer_usb')
    parser.add_argument('--input-mode', choices=['controller', 'hand'], default='controller')
    parser.add_argument('--quest-serial')
    parser.add_argument('--adb-path', default='adb')
    parser.add_argument('--stereo', action='store_true', help='Generate separate labelled eye images')
    parser.add_argument('--img-server-ip', help='Use an existing image server instead of generated video')
    parser.add_argument('--duration', type=float, default=0, help='Seconds to run; 0 runs until Ctrl+C')
    opts = parser.parse_args(argv)
    if not np.isfinite(opts.duration) or opts.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    argv = ['--xr', opts.xr, '--input-mode', opts.input_mode, '--adb-path', opts.adb_path]
    if opts.quest_serial:
        argv += ['--quest-serial', opts.quest_serial]
    args = teleop_args(argv)
    if opts.img_server_ip:
        args.img_server_ip = opts.img_server_ip
    logging.basicConfig(level=logging.INFO)
    xr = XRSession(args)
    client = None
    valid_samples = 0
    try:
        xr.start()
        if opts.img_server_ip:
            from teleimager.image_client import ImageClient
            client = ImageClient(host=opts.img_server_ip, request_bgr=True)
            config = client.get_cam_config()
        else:
            config = {'head_camera': dict(enable_zmq=True, enable_webrtc=False,
                                          binocular=opts.stereo,
                                          image_shape=[480, 1280 if opts.stereo else 640])}
        local_video = xr.configure_display(config)
        logger.info('XR check only: no robot communication. Enter VR in Quest Browser; Ctrl+C exits.')
        start = time.monotonic()
        next_report = start
        while opts.duration == 0 or time.monotonic() - start < opts.duration:
            now = time.monotonic()
            xr.check()  # Transport/server errors should terminate, not look like missing tracking.
            if local_video:
                if client is None:
                    xr.render_to_xr(test_frame(now - start, opts.stereo))
                else:
                    frame = client.get_head_frame()
                    if frame is not None and frame.bgr is not None:
                        xr.render_to_xr(frame.bgr)
            try:
                data = xr.read()
            except RuntimeError as exc:
                if now >= next_report:
                    logger.info('Waiting for fresh tracking: %s', exc)
            else:
                valid_samples += 1
                if now >= next_report:
                    logger.info('TRACKING OK | left xyz=%s | right xyz=%s',
                                np.round(data.left_wrist_pose[:3, 3], 3),
                                np.round(data.right_wrist_pose[:3, 3], 3))
                    if opts.input_mode == 'controller':
                        logger.info('Triggers L/R: %.2f / %.2f | sticks L/R: %s / %s',
                                    data.left_ctrl_triggerValue, data.right_ctrl_triggerValue,
                                    data.left_ctrl_thumbstickValue, data.right_ctrl_thumbstickValue)
            if now >= next_report:
                next_report = now + 1
            time.sleep(max(0, 1 / 30 - (time.monotonic() - now)))
    except KeyboardInterrupt:
        logger.info('XR check stopped.')
    finally:
        try:
            xr.close()
        finally:
            if client is not None:
                client.close()
    logger.info('Fresh tracking samples read: %d. Verify video visually in the headset.', valid_samples)
    return 0 if valid_samples else 1


if __name__ == '__main__':
    raise SystemExit(main())
