"""Headless controls around the existing camera worker and light-painting engine."""
from datetime import datetime
from pathlib import Path
import queue
import threading
import time

import cv2
import numpy as np

from capture import CameraWorker
from diagnostic_recorder import DiagnosticRecorder
from lightpaint_engine import LightPainter


class ControlError(ValueError):
    pass


class WebController:
    def __init__(self, source=0, output_dir=None, auto_connect=True):
        self.engine = LightPainter()
        self.engine.light_color = 'cyan'
        self.engine_lock = threading.RLock()
        self.control_lock = threading.RLock()
        self.frame_lock = threading.Lock()
        self.source = source
        self.output_dir = Path(output_dir or Path(__file__).parent/'output').resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.worker = None
        self.recorder = None
        self.recording_result = None
        self.recording_started = None
        self.running = False
        self.mirrored = True
        self.message = '等待连接相机'
        self.revision = 0
        self.epoch_ns = 0
        self.latest_rgb = self.latest_result = None
        self.shape = None
        self.stats = (0, 0, 0)
        self.processing_ms = self.fps = 0
        self.last_frame_at = 0
        self.last_seq = 0
        self.stop_event = threading.Event()
        blank = np.zeros((480, 640, 3), np.uint8)
        self.blank_jpeg = self._jpeg(blank)
        self.previews = {name: self.blank_jpeg for name in ('camera', 'result', 'mask', 'selected')}
        self.monitor = threading.Thread(target=self._monitor, daemon=True)
        if auto_connect:
            self.connect(source)
        self.monitor.start()

    @staticmethod
    def _jpeg(rgb):
        image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if rgb.ndim == 3 else rgb
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if not ok:
            raise RuntimeError('Preview encoding failed')
        return encoded.tobytes()

    def settings(self):
        with self.engine_lock:
            e = self.engine
            return dict(mode=e.mode, light_color=e.light_color, threshold=e.threshold,
                        change_threshold=e.change_threshold, width=e.width,
                        filter_static=e.filter_static, only_bright=e.only_bright,
                        overlay=e.overlay, mirrored=self.mirrored, roi=e.roi)

    def state(self):
        self._finish_recording()
        with self.frame_lock:
            info = dict(resolution=self.shape, frames=self.stats[0], points=self.stats[1],
                        misses=self.stats[2], processing_ms=round(self.processing_ms, 1),
                        fps=round(self.fps, 1), frame_age=round(time.monotonic()-self.last_frame_at, 2))
        alive = self.worker is not None and self.worker.thread.is_alive()
        recording = self.recorder
        return dict(version='0.4.0', connected=bool(alive and self.latest_rgb is not None and info['frame_age'] < 3),
                    drawing=self.running, source=str(self.source), message=self.message,
                    settings=self.settings(), revision=self.revision, **info,
                    recording=dict(active=recording is not None and not recording.stopping.is_set(),
                                   finalizing=recording is not None and recording.stopping.is_set(),
                                   seconds=round(time.monotonic()-self.recording_started, 1) if recording and self.recording_started else 0,
                                   result=self.recording_result))

    def _record_event(self, name, details=None):
        if self.recorder:
            self.recorder.event(name, details)

    def connect(self, source):
        with self.control_lock:
            if self.recorder:
                raise ControlError('请先停止诊断录制，再切换相机')
            self._stop_worker()
            self.source = source
            self.running = False
            with self.engine_lock:
                self.engine.reset()
            self._invalidate()
            with self.frame_lock:
                self.latest_rgb = None
                self.shape = None
                self.last_frame_at = self.last_seq = self.fps = 0
                self.previews = {name: self.blank_jpeg for name in self.previews}
            self.worker = CameraWorker(source, self.engine, self.engine_lock,
                                       mirrored=self.mirrored, preview_masks=True)
            self.message = '正在连接相机…'

    def _stop_worker(self):
        if self.worker:
            self.worker.stop()
            self.worker.thread.join(4)
            if self.worker.thread.is_alive():
                raise ControlError('相机正在释放，请稍后重试')
            self.worker = None

    def _invalidate(self):
        self.epoch_ns = time.monotonic_ns()
        with self.frame_lock:
            self.latest_result = None
            self.stats = (0, 0, 0)
            self.previews['result'] = self.blank_jpeg
        self.revision += 1

    def clear(self, keep_background=True):
        with self.control_lock, self.engine_lock:
            roi = self.engine.roi
            self.engine.reset(keep_background=keep_background)
            self.engine.roi = roi
            self._invalidate()
            self._record_event('clear', dict(keep_background=keep_background))
            self.message = '画布已清空'

    def set_drawing(self, enabled):
        with self.control_lock, self.engine_lock:
            if enabled and (self.latest_rgb is None or not self.worker or not self.worker.thread.is_alive()):
                raise ControlError('相机尚未就绪')
            self.running = enabled
            if self.worker:
                self.worker.drawing = enabled
            if not enabled:
                self.engine.break_stroke()
            self._record_event('drawing_started' if enabled else 'pause')
            self.message = '正在绘制' if enabled else '已暂停，光轨已保留'

    def capture_background(self):
        with self.control_lock, self.engine_lock:
            with self.frame_lock:
                rgb = self.latest_rgb.copy() if self.latest_rgb is not None else None
            if rgb is None:
                raise ControlError('请等待相机画面')
            self.engine.set_background(rgb)
            self.clear()
            self._record_event('background_captured')
            if self.recorder:
                self.recorder.background(self.engine.static_background, self.engine.background, rgb)
            self.message = '背景已采集，画布已清空；可以移入光源开始绘制'

    def update_settings(self, changes):
        numeric = {'threshold': (100, 254), 'change_threshold': (8, 80), 'width': (1, 16)}
        boolean = {'filter_static', 'only_bright', 'overlay', 'mirrored'}
        choices = {'mode': {'neon', 'exposure'}, 'light_color': {'cyan', 'white'}}
        for key, value in changes.items():
            if key in numeric:
                lo, hi = numeric[key]
                if type(value) is not int or not lo <= value <= hi:
                    raise ControlError(f'{key} 必须是 {lo}–{hi} 的整数')
            elif key in boolean:
                if type(value) is not bool:
                    raise ControlError(f'{key} 必须是布尔值')
            elif key in choices:
                if not isinstance(value, str) or value not in choices[key]:
                    raise ControlError(f'{key} 的选项无效')
            else:
                raise ControlError('未知设置：'+key)
        with self.control_lock, self.engine_lock:
            previous = self.settings()
            for key, value in changes.items():
                if key == 'mirrored':
                    self.mirrored = value
                    if self.worker:
                        self.worker.mirrored = value
                else:
                    setattr(self.engine, key, value)
            changed = {k for k, v in changes.items() if previous[k] != v}
            if 'mirrored' in changed:
                self.engine.roi = None
                self.clear(keep_background=False)
            elif changed & {'mode', 'light_color', 'filter_static'}:
                self.clear()
            self._record_event('settings_changed', changes)
            self.revision += 1

    def set_roi(self, coordinates):
        with self.control_lock, self.engine_lock:
            roi = None
            if coordinates is not None:
                if (not isinstance(coordinates, list) or len(coordinates) != 4
                        or any(type(v) not in (int, float) or not 0 <= v <= 1 for v in coordinates)):
                    raise ControlError('区域坐标必须是 0–1 之间的四个数')
                if not self.shape:
                    raise ControlError('请等待相机画面')
                w, h = self.shape
                x0, y0, x1, y1 = coordinates
                if x1 <= x0 or y1 <= y0:
                    raise ControlError('识别区域无效')
                roi = (round(x0*w), round(y0*h), round(x1*w), round(y1*h))
                if roi[2]-roi[0] < 8 or roi[3]-roi[1] < 8:
                    raise ControlError('识别区域太小')
            self.engine.set_roi(roi)
            self.revision += 1
            self._record_event('roi_changed', roi)

    def save(self):
        with self.frame_lock:
            if self.latest_result is None or self.stats[0] == 0:
                raise ControlError('请先绘制，再保存图片')
            result = self.latest_result.copy()
        folder = self.output_dir/'captures'
        folder.mkdir(exist_ok=True)
        name = 'lightpainting_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.png'
        ok, encoded = cv2.imencode('.png', cv2.cvtColor(result, cv2.COLOR_RGB2BGR))
        if not ok:
            raise RuntimeError('PNG encoding failed')
        (folder/name).write_bytes(encoded.tobytes())
        return dict(name=name, url='/files/captures/'+name)

    def start_recording(self):
        with self.control_lock:
            self._finish_recording()
            if self.recorder:
                raise ControlError('诊断录制正在进行或打包中')
            if self.latest_rgb is None or not self.worker or not self.worker.thread.is_alive():
                raise ControlError('相机尚未就绪')
            with self.engine_lock:
                recorder = DiagnosticRecorder(self.output_dir/'diagnostics', self.source,
                                               self.worker.nominal_fps, self.worker.settings())
                recorder.background(self.engine.static_background, self.engine.background, self.latest_rgb)
                recorder.event('recording_started')
                self.recorder = recorder
                self.worker.recorder = recorder
            self.recording_result = None
            self.recording_started = time.monotonic()

    def stop_recording(self):
        with self.control_lock:
            if self.recorder and not self.recorder.stopping.is_set():
                self.recorder.event('recording_stopped')
                if self.worker:
                    self.worker.recorder = None
                self.recorder.stop()

    def _finish_recording(self):
        with self.control_lock:
            if self.recorder and self.recorder.done.is_set():
                recorder = self.recorder
                self.recording_result = dict(name=recorder.zip_path.name,
                                             url='/files/diagnostics/'+recorder.zip_path.name,
                                             errors=list(recorder.errors), written=dict(recorder.written),
                                             dropped=dict(recorder.dropped))
                if self.worker:
                    self.worker.recorder = None
                self.recorder = None

    def _publish(self, packet):
        rgb, seq, captured_ns, result, position, mask, processing_ms, stats = packet
        if captured_ns <= self.epoch_ns:
            return
        source = rgb.copy()
        if position is not None:
            cv2.circle(source, tuple(np.rint(position).astype(int)), 11, (70, 255, 120), 2, cv2.LINE_AA)
        encoded = dict(camera=self._jpeg(source), result=self._jpeg(result),
                       mask=self._jpeg(mask*255), selected=self._jpeg(rgb*mask[:, :, None]))
        now = time.monotonic()
        with self.frame_lock:
            if captured_ns <= self.epoch_ns:
                return
            elapsed = now-self.last_frame_at
            if self.last_frame_at and seq > self.last_seq and elapsed:
                measured = (seq-self.last_seq)/elapsed
                self.fps = self.fps*.7+measured*.3 if self.fps else measured
            self.latest_rgb, self.latest_result = rgb, result
            self.shape = (rgb.shape[1], rgb.shape[0])
            self.stats, self.processing_ms = stats, processing_ms
            self.last_frame_at, self.last_seq = now, seq
            self.previews = encoded

    def preview(self, view):
        with self.frame_lock:
            return self.previews[view]

    def _monitor(self):
        while not self.stop_event.wait(.075):
            worker = self.worker
            if worker is None:
                continue
            try:
                while True:
                    kind, message = worker.events.get_nowait()
                    if worker is not self.worker:
                        break
                    if kind in ('error', 'ended'):
                        self.set_drawing(False)
                        self.message = message
                    elif kind == 'ready':
                        self.message = '相机已连接，等待开始绘制'
            except queue.Empty:
                pass
            try:
                packet = worker.frames.get_nowait()
            except queue.Empty:
                continue
            if worker is self.worker:
                try:
                    self._publish(packet)
                except Exception as exc:
                    self.message = '预览处理失败：'+str(exc)
            self._finish_recording()

    def close(self):
        self.stop_event.set()
        self.stop_recording()
        try:
            with self.control_lock:
                self._stop_worker()
        except ControlError:
            pass
        self.monitor.join(3)
        if self.recorder:
            self.recorder.done.wait(25)
