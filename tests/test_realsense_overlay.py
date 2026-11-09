import unittest

import cv2
import numpy as np

from teleop.teleop_config import parse_args
from teleop.utils.realsense_overlay import RealSenseRgbOverlay
from teleop.utils.xr_hud import prepare_xr_image


class RealSenseOverlayTests(unittest.TestCase):
    def test_latest_rgb_is_aspect_preserving_bottom_right_overlay(self):
        source = np.zeros((100, 200, 3), dtype=np.uint8)
        source[:, :, 1] = 255
        main = np.zeros((240, 640, 3), dtype=np.uint8)
        overlay = RealSenseRgbOverlay(width_fraction=0.25, margin=10)
        overlay._latest = source
        result = overlay.compose(main)
        self.assertEqual(result.shape, main.shape)
        self.assertEqual(result[0, 0].tolist(), [0, 0, 0])
        target_width = round(640 * .25)
        target_height = round(100 * (target_width / 200))
        x = 640 - target_width - 10
        y = 240 - target_height - 10
        self.assertEqual(result[y + target_height // 2, x + target_width // 2].tolist(), [0, 255, 0])
        self.assertEqual(result[y - 3, x - 3].tolist(), [0, 0, 0])

    def test_missing_frame_is_nonfatal_and_serial_is_configurable(self):
        main = np.full((32, 48, 3), 7, dtype=np.uint8)
        overlay = RealSenseRgbOverlay(serial='123456', width_fraction=.3)
        np.testing.assert_array_equal(overlay.compose(main), main)
        self.assertEqual(parse_args(['--right-wrist-d405-serial', '123456']).right_wrist_d405_serial, '123456')

    def test_wrist_overlay_stays_inside_scaled_head_view(self):
        main = np.full((200, 400, 3), 40, dtype=np.uint8)
        wrist = np.full((50, 100, 3), 180, dtype=np.uint8)
        overlay = RealSenseRgbOverlay(width_fraction=.25, margin=10)
        overlay._latest = wrist

        composed = overlay.compose(main)
        displayed = prepare_xr_image(composed, scale=.8)

        np.testing.assert_array_equal(main, np.full_like(main, 40))
        self.assertEqual(displayed[0, 0].tolist(), [0, 0, 0])
        # The wrist view is at the head image's bottom-right, not at the
        # outer edge of the full XR canvas.
        self.assertEqual(displayed[150, 300].tolist(), [180, 180, 180])
        self.assertEqual(displayed[180, 380].tolist(), [0, 0, 0])


if __name__ == '__main__':
    unittest.main()
