from pathlib import Path
import queue
import tempfile
import threading
import time
import unittest

import cv2
import numpy as np

from web_app import create_app
from web_controller import WebController


class FakeWorker:
    def __init__(self):
        self.frames = queue.Queue()
        self.events = queue.Queue()
        self.drawing = False
        self.mirrored = True
        self.recorder = None
        self.nominal_fps = 30
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.stop_event.wait)
        self.thread.start()

    def stop(self):
        self.stop_event.set()

    def settings(self):
        return dict(camera='test', mode='neon')


class WebTest(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.controller = WebController(output_dir=self.folder.name, auto_connect=False)
        self.controller.worker = FakeWorker()
        self.client = create_app(self.controller).test_client()
        self.rgb = np.zeros((120, 160, 3), np.uint8)
        self.rgb[50:60, 60:70] = (0, 240, 255)
        self.publish()

    def publish(self, seq=1):
        mask = np.zeros((120, 160), np.uint8)
        mask[50:60, 60:70] = 1
        self.controller._publish((self.rgb, seq, time.monotonic_ns(), self.rgb.copy(),
                                  (65., 55.), mask, 12.5, (10, 8, 2)))

    def tearDown(self):
        self.controller.close()
        self.folder.cleanup()

    def action(self, name, **extra):
        return self.client.post('/api/action', json=dict(action=name, **extra))

    def test_controls_background_and_normalized_roi(self):
        with self.client.get('/') as response:
            self.assertEqual(response.status_code, 200)
        self.assertTrue(self.action('start').json['drawing'])
        self.assertTrue(self.controller.worker.drawing)
        self.assertFalse(self.action('pause').json['drawing'])
        self.assertEqual(self.action('roi', coordinates=[.1, .2, .8, .9]).status_code, 200)
        self.assertEqual(self.controller.engine.roi, (16, 24, 128, 108))
        self.assertEqual(self.action('background').status_code, 200)
        np.testing.assert_array_equal(self.controller.engine.static_background, self.rgb)
        self.assertEqual(self.controller.engine.frames, 0)
        self.assertEqual(self.controller.engine.roi, (16, 24, 128, 108))

    def test_settings_reject_invalid_request_atomically(self):
        before = self.controller.engine.threshold
        result = self.client.post('/api/settings', json={'threshold': 190, 'width': 0})
        self.assertEqual(result.status_code, 409)
        self.assertEqual(self.controller.engine.threshold, before)
        self.assertEqual(self.client.post('/api/settings', json={'threshold': 190}).status_code, 200)
        self.assertEqual(self.controller.engine.threshold, 190)
        self.assertEqual(self.client.post('/api/settings', json={'filter_static': 'false'}).status_code, 409)

    def test_previews_snapshot_and_download_boundary(self):
        for name in ('camera', 'result', 'mask', 'selected'):
            response = self.client.get('/preview/'+name+'.jpg')
            decoded = cv2.imdecode(np.frombuffer(response.data, np.uint8), cv2.IMREAD_COLOR)
            self.assertEqual(decoded.shape, (120, 160, 3))
        saved = self.action('save').json
        response = self.client.get(saved['url'])
        decoded = cv2.imdecode(np.frombuffer(response.data, np.uint8), cv2.IMREAD_COLOR)
        np.testing.assert_array_equal(decoded, cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR))
        response.close()
        self.assertEqual(self.client.get('/files/captures/../../outside.png').status_code, 404)
        self.assertEqual(self.client.get('/files/diagnostics/private.txt').status_code, 404)
        self.assertEqual(self.client.get('/preview/unknown.jpg').status_code, 404)

    def test_old_preview_cannot_restore_cleared_canvas(self):
        old = (self.rgb, 1, time.monotonic_ns(), self.rgb.copy(), None,
               np.zeros((120, 160), np.uint8), 0, (10, 8, 2))
        self.controller.clear()
        self.controller._publish(old)
        self.assertIsNone(self.controller.latest_result)
        self.assertEqual(self.action('save').status_code, 409)

    def test_request_boundaries(self):
        self.assertEqual(self.client.post('/api/action', json={'action':'clear'},
                         headers={'Origin':'http://other.example'}).status_code, 403)
        self.assertEqual(self.client.post('/api/action', data='clear').status_code, 400)
        self.assertEqual(self.client.get('/api/state', base_url='http://public.example').status_code, 403)

    def test_diagnostic_lifecycle(self):
        self.assertEqual(self.action('record_start').status_code, 200)
        recorder = self.controller.recorder
        now = time.monotonic_ns()
        recorder.capture(self.rgb, 1, now)
        self.controller.engine.process(self.rgb, True)
        recorder.processed(self.rgb, self.controller.engine.source_mask, self.rgb, 1,
                           now, 10, None, self.controller.engine, {}, True)
        self.assertEqual(self.action('record_stop').status_code, 200)
        self.assertTrue(recorder.done.wait(10))
        state = self.client.get('/api/state').json
        self.assertEqual(state['recording']['result']['errors'], [])
        self.assertEqual(state['recording']['result']['written']['raw'], 1)
        with self.client.get(state['recording']['result']['url']) as response:
            self.assertEqual(response.status_code, 200)


if __name__ == '__main__':
    unittest.main()
