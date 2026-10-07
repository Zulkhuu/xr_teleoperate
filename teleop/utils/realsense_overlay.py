"""Nonblocking RealSense RGB picture-in-picture source."""

import logging
import threading

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class RealSenseRgbOverlay:
    def __init__(self, serial=None, width=640, height=480, fps=30,
                 width_fraction=0.28, margin=16, capture_local=True):
        self.serial = serial
        self.width, self.height, self.fps = width, height, fps
        self.width_fraction, self.margin = width_fraction, margin
        self.capture_local = capture_local
        self._lock = threading.Lock()
        self._latest = None
        self._stop = threading.Event()
        self._thread = None
        self._pipeline = None

    def start(self):
        if not self.capture_local:
            logger.info('[D405] Using the TeleImager right-wrist RGB stream')
            return
        self._thread = threading.Thread(target=self._capture, name='d405-rgb', daemon=True)
        self._thread.start()

    def update(self, frame):
        """Accept a latest BGR frame from the remote TeleImager stream."""
        if frame is not None and frame.ndim == 3 and frame.shape[2] == 3:
            with self._lock:
                self._latest = frame.copy()

    def _capture(self):
        try:
            import pyrealsense2 as rs
            devices = list(rs.context().query_devices())
            if self.serial:
                devices = [d for d in devices if d.get_info(rs.camera_info.serial_number) == self.serial]
            else:
                d405 = [d for d in devices if 'D405' in d.get_info(rs.camera_info.name).upper()]
                devices = d405 or devices
            if not devices:
                raise RuntimeError('no matching RealSense device found')
            if len(devices) > 1:
                logger.warning('[D405] Multiple RealSense devices found; using %s',
                               devices[0].get_info(rs.camera_info.serial_number))
            device = devices[0]
            serial = device.get_info(rs.camera_info.serial_number)
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            pipeline.start(config)
            self._pipeline = pipeline
            logger.info('[D405] RGB overlay started on serial %s (%dx%d@%d)',
                        serial, self.width, self.height, self.fps)
            while not self._stop.is_set():
                frames = pipeline.wait_for_frames(timeout_ms=500)
                color = frames.get_color_frame()
                if color:
                    frame = np.asanyarray(color.get_data())
                    if frame.ndim == 3 and frame.shape[2] == 3:
                        with self._lock:
                            self._latest = frame.copy()
        except Exception as exc:
            logger.warning('[D405] RGB overlay unavailable: %s', exc)
        finally:
            if self._pipeline is not None:
                try:
                    self._pipeline.stop()
                except Exception as exc:
                    logger.debug('[D405] pipeline stop failed: %s', exc)
                self._pipeline = None

    def compose(self, image):
        with self._lock:
            frame = None if self._latest is None else self._latest.copy()
        if frame is None or image is None:
            return image
        output = image.copy()
        target_width = max(1, round(output.shape[1] * self.width_fraction))
        scale = target_width / frame.shape[1]
        target_height = max(1, round(frame.shape[0] * scale))
        if target_height > output.shape[0] - 2 * self.margin:
            target_height = output.shape[0] - 2 * self.margin
            target_width = max(1, round(frame.shape[1] * target_height / frame.shape[0]))
        overlay = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)
        x = output.shape[1] - target_width - self.margin
        y = output.shape[0] - target_height - self.margin
        if x < 0 or y < 0:
            return output
        cv2.rectangle(output, (x - 2, y - 2),
                      (x + target_width + 1, y + target_height + 1), (255, 255, 255), 2)
        output[y:y + target_height, x:x + target_width] = overlay
        return output

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
