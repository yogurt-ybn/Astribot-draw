import tempfile
import unittest

import cv2
import numpy as np

from pipeline import LightPaintingPipeline, PipelineSettings
from tests.test_web import FakeWorker
from web_app import create_app
from web_controller import WebController


def scene(color, x):
    rgb = np.full((120, 160, 3), color, np.uint8)
    rgb[55:63, x:x+8] = (0, 240, 255)
    return rgb


class OverlayTests(unittest.TestCase):
    def pipeline(self, mode='neon', overlay=True):
        return LightPaintingPipeline(PipelineSettings(mode=mode, overlay=overlay, filter_static=False))

    def test_last_drawing_frame_frozen_while_camera_continues_in_both_modes(self):
        for mode in ('neon', 'exposure'):
            with self.subTest(mode=mode):
                pipe = self.pipeline(mode)
                pipe.process_frame(scene((70, 80, 90), 50), sequence=1, captured_ns=10)
                final_frame = scene((90, 70, 50), 65)
                finished = pipe.process_frame(final_frame, sequence=2, captured_ns=20)
                expected = finished.image.copy()
                final_frame[:] = 255  # Input ownership must not affect the stored base.
                pipe.break_stroke()
                for seq in (3, 4):
                    paused = pipe.process_frame(scene((180, 190, 200), 110), accumulate=False,
                                                sequence=seq, captured_ns=seq*10)
                    np.testing.assert_array_equal(paused.image, expected)
                    self.assertEqual(paused.metadata()['overlay']['base_sequence'], 2)
                    self.assertEqual(paused.metadata()['overlay']['base_captured_ns'], 20)
                    self.assertTrue(paused.overlay_frozen)
                    self.assertEqual(paused.frames, 2)
                np.testing.assert_array_equal(paused.image[0, 0], [54, 42, 30])

    def test_overlay_can_be_enabled_after_pause_without_accumulating_again(self):
        pipe = self.pipeline(overlay=False)
        drawn = pipe.process_frame(scene((100, 80, 60), 50))
        pure_trail = drawn.image.copy()
        pipe.break_stroke()
        pipe.process_frame(scene((200, 200, 200), 110), accumulate=False)
        pipe.configure(overlay=True, overlay_brightness=50)
        composited = pipe.snapshot()
        self.assertEqual(composited.frames, 1)
        np.testing.assert_array_equal(composited.image[0, 0], [50, 40, 30])
        pipe.configure(overlay_brightness=0)
        np.testing.assert_array_equal(pipe.snapshot().image, pure_trail)
        pipe.configure(overlay_brightness=100)
        np.testing.assert_array_equal(pipe.snapshot().image[0, 0], [100, 80, 60])
        pipe.configure(overlay=False)
        np.testing.assert_array_equal(pipe.snapshot().image, pure_trail)

    def test_resume_replaces_base_and_clear_discards_it(self):
        pipe = self.pipeline()
        pipe.process_frame(scene((40, 40, 40), 50), sequence=1)
        pipe.break_stroke()
        pipe.process_frame(scene((100, 80, 60), 65), sequence=2)
        pipe.break_stroke()
        self.assertEqual(pipe.overlay_state()['base_sequence'], 2)
        np.testing.assert_array_equal(pipe.snapshot().image[0, 0], [60, 48, 36])
        pipe.clear()
        self.assertIsNone(pipe.snapshot())
        self.assertIsNone(pipe.overlay_state()['base_sequence'])
        preview = pipe.process_frame(scene((50, 50, 50), 50), accumulate=False)
        np.testing.assert_array_equal(preview.image[0, 0], [30, 30, 30])
        self.assertFalse(preview.overlay_frozen)

    def test_overlay_values_are_validated_atomically(self):
        pipe = self.pipeline()
        for value in (-1, 101, True, 50.5):
            with self.assertRaises(ValueError):
                pipe.configure(overlay_brightness=value, threshold=180)
            self.assertEqual(pipe.settings.threshold, 230)
            self.assertEqual(pipe.settings.overlay_brightness, 60)

    def test_web_pause_and_overlay_recompose_immediately_and_ignore_old_packets(self):
        with tempfile.TemporaryDirectory() as root:
            controller = WebController(output_dir=root, auto_connect=False)
            controller.worker = FakeWorker()
            self.addCleanup(controller.close)
            client = create_app(controller).test_client()
            controller.pipeline.configure(filter_static=False)
            old = controller.pipeline.process_frame(scene((100, 80, 60), 50), sequence=10)
            controller._publish(old)
            response = client.post('/api/v1/action', json={'action': 'pause'})
            self.assertTrue(response.json['overlay']['frozen'])
            response = client.post('/api/v1/settings', json={'overlay': True, 'overlay_brightness': 50})
            self.assertEqual(response.status_code, 200)
            expected = controller.pipeline.snapshot().image
            controller._publish(old)  # In-flight pre-control preview must not undo composition.
            response = client.get('/api/v1/result.png')
            actual = cv2.cvtColor(cv2.imdecode(np.frombuffer(response.data, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
            np.testing.assert_array_equal(actual, expected)
            np.testing.assert_array_equal(actual[0, 0], [50, 40, 30])
            saved = client.post('/api/v1/action', json={'action': 'save'}).json
            with client.get(saved['url']) as download:
                self.assertEqual(download.data, response.data)
            self.assertEqual(controller.pipeline.engine.frames, 1)
            client.post('/api/v1/action', json={'action': 'clear'})
            self.assertIsNone(controller.pipeline.engine.last_drawing_frame)
