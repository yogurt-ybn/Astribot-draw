"""Public, camera/HTTP independent interface around the existing algorithm.

All inputs/outputs use RGB uint8, H x W x 3. Masks use uint8 values 0/1.
Coordinates refer to the input frame, after any caller resize/mirror.
"""
from dataclasses import asdict, dataclass, replace
import threading
import time
from typing import Protocol

import numpy as np

if __package__:
    from .lightpaint_engine import LightPainter
else:
    from lightpaint_engine import LightPainter


@dataclass(frozen=True)
class PipelineSettings:
    mode: str = 'neon'
    light_color: str = 'cyan'
    threshold: int = 230
    change_threshold: int = 25
    width: int = 4
    filter_static: bool = True
    only_bright: bool = True
    overlay: bool = False

    @classmethod
    def validate(cls, changes):
        if not isinstance(changes, dict):
            raise ValueError('设置必须为对象')
        numeric = {'threshold': (100, 254), 'change_threshold': (8, 80), 'width': (1, 16)}
        choices = {'mode': {'neon', 'exposure'}, 'light_color': {'cyan', 'white'}}
        for key, value in changes.items():
            if key in numeric:
                lo, hi = numeric[key]
                if type(value) is not int or not lo <= value <= hi:
                    raise ValueError(f'{key} 必须是 {lo}–{hi} 的整数')
            elif key in choices:
                if not isinstance(value, str) or value not in choices[key]:
                    raise ValueError(f'{key} 的选项无效')
            elif key in {'filter_static', 'only_bright', 'overlay'}:
                if type(value) is not bool:
                    raise ValueError(f'{key} 必须是布尔值')
            else:
                raise ValueError('未知设置：'+str(key))

    def __post_init__(self):
        self.validate(asdict(self))


@dataclass(frozen=True)
class FrameResult:
    sequence: int
    captured_ns: int
    rgb: np.ndarray
    image: np.ndarray
    mask: np.ndarray
    position: tuple[float, float] | None
    processing_ms: float
    frames: int
    points: int
    misses: int
    tracking_event: str
    settings: PipelineSettings
    roi: tuple[int, int, int, int] | None

    def metadata(self):
        h, w = self.rgb.shape[:2]
        return dict(sequence=self.sequence, captured_ns=self.captured_ns,
                    clock='monotonic_ns', resolution=[w, h], color_space='RGB',
                    coordinate_space='processed_frame', position=list(self.position) if self.position else None,
                    normalized_position=[self.position[0]/w, self.position[1]/h] if self.position else None,
                    processing_ms=round(self.processing_ms, 3), source_pixels=int(self.mask.sum()),
                    frames=self.frames, points=self.points, misses=self.misses,
                    tracking_event=self.tracking_event, settings=asdict(self.settings), roi=self.roi)


class FrameProcessor(Protocol):
    """Frame-processing extension point; the caller owns camera acquisition."""
    def process_frame(self, rgb: np.ndarray, *, accumulate: bool = True,
                      sequence: int | None = None, captured_ns: int | None = None) -> FrameResult: ...


class LightPaintingPipeline:
    """Stateful, serialized session. Use a separate instance per camera/session.

    Returned arrays are owned snapshots and remain valid after the next frame.
    Algorithms can be extended behind this boundary without changing consumers.
    """
    def __init__(self, settings: PipelineSettings | None = None):
        self.lock = threading.RLock()
        self.engine = LightPainter()
        self._settings = settings or PipelineSettings()
        for name, value in asdict(self._settings).items():
            setattr(self.engine, name, value)
        self._sequence = 0

    @property
    def settings(self):
        with self.lock:
            return self._settings

    @staticmethod
    def _validate_frame(rgb):
        if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) < 2:
            raise ValueError('输入必须是 uint8 RGB 图像，形状为 H×W×3，宽高至少为 2')

    def configure(self, **changes):
        PipelineSettings.validate(changes)
        with self.lock:
            settings = replace(self._settings, **changes)
            reset = any(getattr(settings, k) != getattr(self._settings, k)
                        for k in ('mode', 'light_color', 'filter_static'))
            for name, value in changes.items():
                setattr(self.engine, name, value)
            self._settings = settings
            if reset:
                self.clear()
            return settings

    def clear(self, keep_background=True):
        with self.lock:
            roi = self.engine.roi
            self.engine.reset(keep_background=keep_background)
            self.engine.roi = roi

    def break_stroke(self):
        with self.lock:
            self.engine.break_stroke()

    def set_background(self, rgb):
        self._validate_frame(rgb)
        with self.lock:
            self.engine.set_background(rgb)

    def set_roi(self, roi):
        if roi is not None:
            if (not isinstance(roi, (list, tuple)) or len(roi) != 4
                    or any(type(v) is not int or v < 0 for v in roi)
                    or roi[0] >= roi[2] or roi[1] >= roi[3]):
                raise ValueError('ROI 必须为有效的非负整数像素边界 (x0,y0,x1,y1)')
            roi = tuple(roi)
        with self.lock:
            self.engine.set_roi(roi)

    def process_frame(self, rgb, *, accumulate=True, sequence=None, captured_ns=None):
        self._validate_frame(rgb)
        if type(accumulate) is not bool:
            raise ValueError('accumulate 必须是布尔值')
        if sequence is not None and (type(sequence) is not int or sequence < 0):
            raise ValueError('sequence 必须为非负整数')
        if captured_ns is not None and (type(captured_ns) is not int or captured_ns < 0):
            raise ValueError('captured_ns 必须为非负整数单调时钟时间')
        with self.lock:
            self._sequence = self._sequence+1 if sequence is None else sequence
            stamp = time.monotonic_ns() if captured_ns is None else captured_ns
            started = time.monotonic_ns()
            image, point = self.engine.process(rgb, accumulate, stamp/1e9)
            elapsed = (time.monotonic_ns()-started)/1e6
            e = self.engine
            return FrameResult(self._sequence, stamp, rgb.copy(), image.copy(), e.source_mask.copy(),
                               tuple(map(float, point)) if point is not None else None,
                               elapsed, e.frames, e.strokes, e.misses, e.last_tracking_event,
                               self._settings, e.roi)
