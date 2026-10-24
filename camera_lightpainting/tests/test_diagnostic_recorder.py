import csv
import json
from pathlib import Path
import tempfile
import time
import unittest
import zipfile

import cv2
import numpy as np

from diagnostic_recorder import DiagnosticRecorder
from lightpaint_engine import LightPainter


class DiagnosticRecorderTest(unittest.TestCase):
    def test_drain_video_frame_mapping_and_package(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = DiagnosticRecorder(directory, 'synthetic', 30, {'mode': 'neon'})
            painter = LightPainter()
            frame = np.zeros((120, 160, 3), np.uint8)
            painter.set_background(frame)
            recorder.background(painter.static_background, painter.background, frame)
            for seq in range(6):
                rgb = frame.copy()
                cv2.circle(rgb, (30+seq*9, 65), 6, (255, 255, 245), -1)
                captured_ns = time.monotonic_ns()
                self.assertTrue(recorder.capture(rgb, seq, captured_ns))
                if seq % 2 == 0:
                    result, position = painter.process(rgb)
                    recorder.processed(rgb, painter.foreground_mask, result, seq, captured_ns,
                                       3.0, position, painter, {'mode': 'neon'}, True)
            recorder.stop()
            self.assertFalse(recorder.capture(frame, 99, time.monotonic_ns()))
            self.assertTrue(recorder.done.wait(15), 'Recorder did not finalize')
            self.assertFalse(recorder.errors)
            self.assertEqual(recorder.written, {'raw': 6, 'processed': 3})
            with (recorder.folder/'processed.csv').open(encoding='utf-8-sig') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([int(row['capture_seq']) for row in rows], [0, 2, 4])
            self.assertEqual([int(row['skipped_capture_frames']) for row in rows], [0, 1, 1])
            for name, expected in [('raw.avi', 6), ('debug.avi', 3)]:
                cap = cv2.VideoCapture(str(recorder.folder/name))
                count = 0
                while True:
                    ok, _ = cap.read()
                    if not ok:
                        break
                    count += 1
                cap.release()
                self.assertEqual(count, expected)
            manifest = json.loads((recorder.folder/'manifest.json').read_text(encoding='utf-8'))
            self.assertEqual(manifest['dropped'], {'raw': 0, 'processed': 0})
            self.assertTrue((recorder.folder/'background_00_rgb.npy').exists())
            with zipfile.ZipFile(recorder.zip_path) as archive:
                self.assertIsNone(archive.testzip())
                self.assertTrue(any(name.endswith('/raw.avi') for name in archive.namelist()))


if __name__ == '__main__':
    unittest.main()
