import time
import unittest

import numpy as np

from lightpaint_engine import LightPainter
from pipeline import LightPaintingPipeline, PipelineSettings


class PipelineInterfaceTests(unittest.TestCase):
    def test_adapter_preserves_algorithm_outputs_in_both_modes(self):
        for mode in ('neon', 'exposure'):
            with self.subTest(mode=mode):
                pipe = LightPaintingPipeline(PipelineSettings(mode=mode))
                reference = LightPainter()
                reference.mode, reference.light_color = mode, 'cyan'
                background = np.zeros((96, 128, 3), np.uint8)
                pipe.set_background(background)
                reference.set_background(background)
                for seq, x in enumerate((20, 30, None, 60), 1):
                    rgb = background.copy()
                    if x is not None:
                        rgb[40:48, x:x+8] = (0, 240, 255)
                    stamp = time.monotonic_ns()
                    actual = pipe.process_frame(rgb, sequence=seq, captured_ns=stamp)
                    image, point = reference.process(rgb, True, stamp/1e9)
                    np.testing.assert_array_equal(actual.image, image)
                    np.testing.assert_array_equal(actual.mask, reference.source_mask)
                    self.assertEqual(actual.position, point)
                    self.assertEqual(actual.sequence, seq)
                    self.assertEqual(actual.captured_ns, stamp)
                    self.assertEqual(actual.tracking_event, reference.last_tracking_event)

    def test_frame_ownership_and_pause(self):
        pipe = LightPaintingPipeline(PipelineSettings(mode='exposure', filter_static=False))
        frame = np.zeros((40, 60, 3), np.uint8)
        frame[10:16, 15:21] = (0, 240, 255)
        result = pipe.process_frame(frame)
        before = result.image.copy()
        frame[:] = 0
        paused = pipe.process_frame(frame, accumulate=False)
        self.assertEqual(paused.frames, 1)
        self.assertGreater(result.rgb.sum(), 0)
        np.testing.assert_array_equal(result.image, before)
        paused.image[:] = 0
        self.assertGreater(pipe.engine.exposure.sum(), 0)
        metadata = result.metadata()
        self.assertEqual(metadata['resolution'], [60, 40])
        self.assertEqual(metadata['source_pixels'], 36)
        self.assertAlmostEqual(metadata['normalized_position'][0], 17.5/60)

    def test_invalid_configuration_and_frame_leave_session_unchanged(self):
        pipe = LightPaintingPipeline()
        for changes in ({'threshold': 180, 'width': 0}, {'unknown': True}, {'overlay': 'false'}):
            with self.assertRaises(ValueError):
                pipe.configure(**changes)
            self.assertEqual(pipe.settings, PipelineSettings())
        with self.assertRaises(ValueError):
            pipe.process_frame(np.zeros((10, 10), np.uint8))
        with self.assertRaises(ValueError):
            pipe.process_frame(np.zeros((10, 10, 3), np.float32))
        with self.assertRaises(ValueError):
            pipe.set_roi((10, 0, 5, 10))

    def test_clear_and_mode_change_preserve_background_and_roi(self):
        pipe = LightPaintingPipeline()
        background = np.zeros((40, 60, 3), np.uint8)
        pipe.set_background(background)
        pipe.set_roi((5, 5, 50, 35))
        pipe.process_frame(background)
        pipe.configure(mode='exposure')
        self.assertEqual(pipe.engine.frames, 0)
        self.assertEqual(pipe.engine.roi, (5, 5, 50, 35))
        self.assertIsNotNone(pipe.engine.static_background)
        pipe.clear(keep_background=False)
        self.assertIsNone(pipe.engine.static_background)
