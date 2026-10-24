import unittest

import cv2
import numpy as np

from lightpaint_engine import LightPainter


class LightPainterTests(unittest.TestCase):
    def test_exposure_keeps_channel_maxima_and_pause(self):
        painter = LightPainter()
        painter.filter_static = False
        painter.mode = 'exposure'
        painter.only_bright = False
        a = np.zeros((80, 100, 3), np.uint8)
        b = a.copy()
        a[20, 20] = [240, 80, 20]
        b[20, 20] = [30, 100, 255]
        painter.process(a)
        result, _ = painter.process(b)
        np.testing.assert_array_equal(result[20, 20], [240, 100, 255])
        result, _ = painter.process(np.full_like(a, 255), accumulate=False)
        np.testing.assert_array_equal(result[20, 20], [240, 100, 255])
        self.assertEqual(painter.frames, 2)

    def test_roi_survives_first_frame_and_excludes_outside(self):
        painter = LightPainter()
        painter.filter_static = False
        painter.mode = 'exposure'
        painter.only_bright = False
        painter.set_roi((20, 20, 40, 40))
        result, _ = painter.process(np.full((80, 100, 3), 255, np.uint8))
        self.assertTrue(result[30, 30].all())
        self.assertFalse(result[10, 10].any())
        self.assertEqual(painter.roi, (20, 20, 40, 40))

    def test_missing_light_breaks_stroke(self):
        painter = LightPainter()
        painter.filter_static = False
        frame = np.zeros((120, 200, 3), np.uint8)
        cv2.circle(frame, (50, 60), 5, (255, 255, 255), -1)
        _, position = painter.process(frame)
        self.assertIsNotNone(position)
        self.assertLess(np.linalg.norm(np.array(position)-[50, 60]), 2)
        painter.process(np.zeros_like(frame))
        self.assertIsNone(painter.previous)
        frame[:] = 0
        cv2.circle(frame, (85, 60), 5, (255, 255, 255), -1)
        painter.process(frame)
        self.assertFalse(painter.trail[60, 68].any(), 'Disconnected strokes must not be joined')

    def test_reset_discards_accumulation(self):
        painter = LightPainter()
        painter.filter_static = False
        painter.mode = 'exposure'
        painter.only_bright = False
        painter.process(np.full((40, 60, 3), 255, np.uint8))
        painter.reset()
        result, _ = painter.process(np.zeros((40, 60, 3), np.uint8))
        self.assertFalse(result.any())

    def test_static_lamp_rejected_in_both_modes_moving_lamp_retained(self):
        for mode in ('neon', 'exposure'):
            with self.subTest(mode=mode):
                painter = LightPainter()
                painter.mode = mode
                scene = np.full((120, 240, 3), 15, np.uint8)
                # A strong warm static lamp used to win the detector's first path.
                cv2.circle(scene, (180, 55), 12, (255, 255, 245), -1)
                painter.set_background(scene)
                for _ in range(5):
                    result, position = painter.process(scene)
                    self.assertIsNone(position)
                    self.assertFalse(result.any())
                for x in (40, 50, 60):
                    moving = scene.copy()
                    cv2.circle(moving, (x, 65), 7, (255, 255, 245), -1)
                    result, position = painter.process(moving)
                    if mode == 'neon':
                        self.assertIsNotNone(position)
                        self.assertLess(np.linalg.norm(np.array(position)-[x, 65]), 4)
                    self.assertTrue(result[:, 25:80].any())
                    self.assertFalse(result[:, 160:200].any())

    def test_preview_learns_and_global_exposure_change_does_not_paint(self):
        painter = LightPainter()
        painter.mode = 'exposure'
        scene = np.full((120, 200, 3), 25, np.uint8)
        cv2.circle(scene, (150, 60), 10, (235, 235, 235), -1)
        painter.process(scene, accumulate=False)
        brighter = np.minimum(scene.astype(np.int16)+12, 255).astype(np.uint8)
        result, _ = painter.process(brighter)
        self.assertFalse(result.any())
        self.assertEqual(painter.frames, 1)

    def test_background_survives_clear_and_first_accumulation(self):
        painter = LightPainter()
        scene = np.full((120, 200, 3), 10, np.uint8)
        cv2.circle(scene, (150, 60), 10, (255, 255, 245), -1)
        painter.set_background(scene)
        painter.reset(keep_background=True)
        self.assertIsNotNone(painter.static_background)
        result, position = painter.process(scene)
        self.assertIsNone(position)
        self.assertFalse(result.any())

    def test_disappearing_light_does_not_create_another_stroke(self):
        painter = LightPainter()
        scene = np.zeros((120, 200, 3), np.uint8)
        cv2.circle(scene, (70, 60), 8, (255, 255, 245), -1)
        painter.set_background(scene)
        _, position = painter.process(np.zeros_like(scene))
        self.assertIsNone(position)

    def test_cyan_profile_excludes_moving_white_cloth_and_blue_tube(self):
        for mode in ('neon', 'exposure'):
            with self.subTest(mode=mode):
                p = LightPainter()
                p.light_color = 'cyan'
                p.mode = mode
                scene = np.full((150, 240, 3), 10, np.uint8)
                p.set_background(scene)
                # Both moved relative to the reference; background subtraction
                # alone must not turn the bright clothing into a light source.
                scene[20:75, 140:220] = [255, 255, 255]
                scene[25:90, 35:43] = [30, 70, 255]
                result, point = p.process(scene, timestamp=1.0)
                self.assertIsNone(point)
                self.assertFalse(result.any())
                # Its grayscale luminance is below the old 230 threshold.
                cv2.circle(scene, (65, 100), 6, (60, 230, 255), -1)
                result, point = p.process(scene, timestamp=1.033)
                self.assertIsNotNone(point)
                self.assertLess(np.linalg.norm(np.array(point)-[65, 100]), 1)
                self.assertTrue(result[100, 65].any())
                self.assertFalse(result[40, 180].any())

    def test_cyan_exposure_accumulates_real_pixels_without_filling_gaps(self):
        p = LightPainter()
        p.light_color = 'cyan'
        p.mode = 'exposure'
        scene = np.zeros((120, 240, 3), np.uint8)
        p.set_background(scene)
        a = scene.copy()
        cv2.circle(a, (40, 60), 5, (60, 230, 255), -1)
        p.process(a, timestamp=1.0)
        p.process(scene, timestamp=1.033)
        b = scene.copy()
        cv2.circle(b, (75, 60), 5, (60, 230, 255), -1)
        result, _ = p.process(b, timestamp=1.067)
        self.assertFalse(result[60, 58].any(), 'Short gap must not be interpolated')
        self.assertTrue(result[60, 40].any())
        self.assertTrue(result[60, 75].any())
        c = scene.copy()
        cv2.circle(c, (110, 60), 5, (60, 230, 255), -1)
        result, _ = p.process(c, timestamp=1.5)
        self.assertFalse(result[60, 93].any(), 'Long occlusion must stay disconnected')
        p.break_stroke()
        d = scene.copy()
        cv2.circle(d, (145, 60), 5, (60, 230, 255), -1)
        result, _ = p.process(d, timestamp=1.533)
        self.assertFalse(result[60, 128].any(), 'Pause must start a new stroke')


if __name__ == '__main__':
    unittest.main()
