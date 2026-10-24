from pathlib import Path
import tempfile
import threading
import unittest

import cv2
import numpy as np

from capture import CameraWorker
from lightpaint_engine import LightPainter


class CameraPipelineTest(unittest.TestCase):
    def test_processing_keeps_all_input_frames_even_when_preview_is_unread(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'input.avi'
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 60, (160, 120))
            self.assertTrue(writer.isOpened())
            for i in range(12):
                frame = np.zeros((120, 160, 3), np.uint8)
                cv2.circle(frame, (20+i*6, 60), 5, (255, 230, 60), -1)
                writer.write(frame)
            writer.release()
            painter = LightPainter()
            painter.light_color = 'cyan'
            worker = CameraWorker(str(path), painter, threading.RLock(), mirrored=False, drawing=True)
            worker.thread.join(10)
            self.assertFalse(worker.thread.is_alive())
            self.assertEqual(painter.frames, 12)
            self.assertEqual(worker.replaced_frames, 11)
            packet = worker.frames.get_nowait()
            self.assertEqual(packet[1], 12)
            self.assertEqual(packet[-1][0], 12)


if __name__ == '__main__':
    unittest.main()
