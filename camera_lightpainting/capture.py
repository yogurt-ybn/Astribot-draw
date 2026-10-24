"""Camera acquisition and frame processing independent of the Tk preview."""
import queue
import threading
import time

import cv2


class CameraWorker:
    def __init__(self, source, engine, engine_lock, mirrored=True, drawing=False):
        self.source = source
        self.engine = engine
        self.engine_lock = engine_lock
        self.mirrored = mirrored
        self.drawing = drawing
        self.frames = queue.Queue(maxsize=1)
        self.events = queue.Queue()
        self.stop_event = threading.Event()
        self.recorder = None
        self.nominal_fps = 30
        self.replaced_frames = 0
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        cap = None
        try:
            cap = cv2.VideoCapture(self.source, cv2.CAP_DSHOW) if isinstance(self.source, int) else cv2.VideoCapture(self.source)
            if not cap.isOpened():
                self.events.put(('error', '无法打开摄像头，请关闭其他占用相机的程序，或切换相机编号。'))
                return
            # Keep the driver's native camera format for USB compatibility.
            fps = cap.get(cv2.CAP_PROP_FPS) or 30
            self.nominal_fps = fps
            self.events.put(('ready', '已连接，点击“开始绘制”'))
            deadline = time.monotonic()
            seq = 0
            while not self.stop_event.is_set():
                ok, bgr = cap.read()
                if not ok:
                    self.events.put(('ended', '视频播放结束' if isinstance(self.source, str) else '摄像头画面中断，请重新连接'))
                    break
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                captured_ns = time.monotonic_ns()
                seq += 1
                recorder = self.recorder
                if recorder is not None:
                    recorder.capture(rgb, seq, captured_ns)
                if rgb.shape[1] > 960:
                    rgb = cv2.resize(rgb, (960, round(rgb.shape[0]*960/rgb.shape[1])))
                with self.engine_lock:
                    rgb = cv2.flip(rgb, 1) if self.mirrored else rgb
                    started = time.monotonic_ns()
                    result, position = self.engine.process(rgb, self.drawing, captured_ns/1e9)
                    processing_ms = (time.monotonic_ns()-started)/1e6
                    recorder = self.recorder
                    if recorder is not None:
                        recorder.processed(rgb, self.engine.source_mask, result, seq, captured_ns,
                                           processing_ms, position, self.engine, self.settings(), self.drawing)
                    stats = (self.engine.frames, self.engine.strokes, self.engine.misses)
                    packet = (rgb, seq, captured_ns, result, position, stats)
                try:
                    self.frames.get_nowait()
                    self.replaced_frames += 1
                except queue.Empty:
                    pass
                self.frames.put_nowait(packet)
                if isinstance(self.source, str):
                    deadline += 1/fps
                    self.stop_event.wait(max(0, deadline-time.monotonic()))
        except Exception as exc:
            self.events.put(('error', str(exc)))
        finally:
            if cap is not None:
                cap.release()
            self.events.put(('released', '摄像头已释放'))

    def stop(self):
        self.stop_event.set()

    def settings(self):
        e = self.engine
        return dict(camera=str(self.source), mode=e.mode, threshold=e.threshold,
                    change_threshold=e.change_threshold, line_width=e.width,
                    filter_static=e.filter_static, overlay=e.overlay, only_bright=e.only_bright,
                    mirrored=self.mirrored, roi=e.roi, light_color=e.light_color)
