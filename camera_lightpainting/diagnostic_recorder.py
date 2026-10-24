"""Write capture and processing evidence without blocking the preview thread."""
import csv
from datetime import datetime
import json
from pathlib import Path
import platform
import queue
import shutil
import threading
import time
import zipfile

import cv2
import numpy as np
from PIL import Image


class DiagnosticRecorder:
    def __init__(self, output_root, source, fps, settings):
        self.folder = Path(output_root)/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.folder.mkdir(parents=True, exist_ok=False)
        self.zip_path = self.folder.with_suffix('.zip')
        self.started_ns = time.monotonic_ns()
        self.fps = float(fps) if 1 <= float(fps) <= 240 else 30.0
        self.jobs = queue.Queue(maxsize=96)
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.done = threading.Event()
        self.errors = []
        self.offered = {'raw': 0, 'processed': 0}
        self.dropped = {'raw': 0, 'processed': 0}
        self.written = {'raw': 0, 'processed': 0}
        self.source = source
        self.settings = settings
        self.last_capture_seq = None
        self.manifest = dict(source=source, nominal_video_fps=self.fps,
                             raw_is_mirrored=False, timestamp_clock='time.monotonic_ns',
                             started_ns=self.started_ns, initial_settings=settings,
                             python=platform.python_version(), opencv=cv2.__version__, numpy=np.__version__,
                             note='AVI uses nominal FPS. Use CSV timestamps and capture_seq for actual timing; processed video contains only frames processed by the app.')
        for name in ('lightpaint_engine.py', 'lightpaint_app.py', 'capture.py', 'diagnostic_recorder.py',
                     'web_app.py', 'web_controller.py', 'pipeline.py', 'startup.py',
                     'arguments.py', 'session_launcher.py'):
            shutil.copy2(Path(__file__).parent/name, self.folder/name)
        self.thread = threading.Thread(target=self._run, daemon=False)
        self.thread.start()

    def _offer(self, kind, payload):
        with self.lock:
            if self.stopping.is_set():
                return False
            if kind in self.offered:
                self.offered[kind] += 1
            try:
                self.jobs.put_nowait((kind, payload))
                return True
            except queue.Full:
                if kind in self.dropped:
                    self.dropped[kind] += 1
                else:
                    self.errors.append(f'Queue full; lost {kind} event')
                return False

    def capture(self, rgb, seq, captured_ns):
        if captured_ns < self.started_ns:
            return False
        return self._offer('raw', (rgb.copy(), seq, captured_ns))

    def processed(self, rgb, mask, result, seq, captured_ns, processing_ms, position, engine, settings, drawing):
        if captured_ns < self.started_ns:
            return False
        gap = max(0, seq-self.last_capture_seq-1) if self.last_capture_seq is not None else 0
        self.last_capture_seq = seq
        metadata = dict(capture_seq=seq, captured_ns=captured_ns,
                        captured_seconds=(captured_ns-self.started_ns)/1e9,
                        processed_seconds=(time.monotonic_ns()-self.started_ns)/1e9,
                        processing_ms=processing_ms, skipped_capture_frames=gap,
                        drawing=drawing, detected_x=position[0] if position is not None else '',
                        detected_y=position[1] if position is not None else '',
                        filtered_x=float(engine.filtered[0]) if engine.filtered is not None else '',
                        filtered_y=float(engine.filtered[1]) if engine.filtered is not None else '',
                        foreground_pixels=int(engine.foreground_mask.sum()) if engine.foreground_mask is not None else 0,
                        source_pixels=int(engine.source_mask.sum()) if engine.source_mask is not None else 0,
                        tracking_event=engine.last_tracking_event, settings=json.dumps(settings, ensure_ascii=False))
        return self._offer('processed', (rgb.copy(), mask.copy() if mask is not None else None,
                                       result.copy(), metadata))

    def event(self, name, details=None):
        self._offer('event', dict(seconds=(time.monotonic_ns()-self.started_ns)/1e9, event=name, details=details))

    def background(self, rgb_background, gray_background, preview=None):
        self._offer('background', (rgb_background.copy() if rgb_background is not None else None,
                                  gray_background.copy() if gray_background is not None else None,
                                  preview.copy() if preview is not None else None))

    def stop(self):
        with self.lock:
            self.stopping.set()

    def _video(self, name, size):
        writer = cv2.VideoWriter(str(self.folder/name), cv2.VideoWriter_fourcc(*'MJPG'), self.fps, size)
        if not writer.isOpened():
            writer.release()
            raise RuntimeError(f'Cannot create {name}; check available disk space and MJPG codec')
        return writer

    @staticmethod
    def _panel(rgb, text, size=(480, 270)):
        panel = cv2.resize(rgb, size)
        cv2.rectangle(panel, (0, 0), (size[0], 27), (8, 12, 18), -1)
        cv2.putText(panel, text, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, .48, (230, 240, 255), 1, cv2.LINE_AA)
        return panel

    def _run(self):
        raw_writer = debug_writer = None
        raw_file = processed_file = events_file = None
        last_result = None
        try:
            raw_file = (self.folder/'capture.csv').open('w', newline='', encoding='utf-8-sig')
            raw_csv = csv.writer(raw_file)
            raw_csv.writerow(['video_frame', 'capture_seq', 'captured_ns', 'seconds'])
            processed_file = (self.folder/'processed.csv').open('w', newline='', encoding='utf-8-sig')
            processed_csv = None
            events_file = (self.folder/'events.jsonl').open('w', encoding='utf-8')
            background_count = 0
            raw_size = None
            while not (self.stopping.is_set() and self.jobs.empty()):
                try:
                    kind, payload = self.jobs.get(timeout=.1)
                except queue.Empty:
                    continue
                if kind == 'raw':
                    rgb, seq, captured_ns = payload
                    size = (rgb.shape[1], rgb.shape[0])
                    if raw_writer is None:
                        raw_size = size
                        raw_writer = self._video('raw.avi', size)
                        self.manifest['raw_resolution'] = list(size)
                    if size != raw_size:
                        raise RuntimeError('Camera resolution changed during recording')
                    raw_writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                    raw_csv.writerow([self.written['raw'], seq, captured_ns, (captured_ns-self.started_ns)/1e9])
                    self.written['raw'] += 1
                elif kind == 'processed':
                    rgb, mask, result, metadata = payload
                    if processed_csv is None:
                        processed_csv = csv.DictWriter(processed_file, fieldnames=['video_frame']+list(metadata))
                        processed_csv.writeheader()
                    source = rgb.copy()
                    if metadata['detected_x'] != '':
                        center = (round(metadata['detected_x']), round(metadata['detected_y']))
                        cv2.circle(source, center, 10, (70, 255, 100), 2, cv2.LINE_AA)
                    mask_rgb = np.repeat((mask*255)[:, :, None], 3, axis=2) if mask is not None else np.zeros_like(rgb)
                    foreground = rgb*(mask[:, :, None] if mask is not None else 0)
                    seq = metadata['capture_seq']
                    top = np.concatenate([self._panel(source, f'INPUT / detected center | frame {seq}'),
                                          self._panel(mask_rgb, 'SOURCE MASK / background + color + brightness')], axis=1)
                    bottom = np.concatenate([self._panel(foreground, 'SELECTED LIGHT / accepted source pixels'),
                                             self._panel(result, f'OUTPUT | {metadata["tracking_event"]} | {metadata["processing_ms"]:.0f} ms')], axis=1)
                    debug = np.concatenate([top, bottom], axis=0)
                    if debug_writer is None:
                        debug_writer = self._video('debug.avi', (debug.shape[1], debug.shape[0]))
                    debug_writer.write(cv2.cvtColor(debug, cv2.COLOR_RGB2BGR))
                    processed_csv.writerow(dict(video_frame=self.written['processed'], **metadata))
                    self.written['processed'] += 1
                    last_result = result
                elif kind == 'event':
                    events_file.write(json.dumps(payload, ensure_ascii=False)+'\n')
                    events_file.flush()
                elif kind == 'background':
                    rgb_bg, gray_bg, preview = payload
                    prefix = f'background_{background_count:02d}'
                    if rgb_bg is not None:
                        np.save(self.folder/(prefix+'_rgb.npy'), rgb_bg)
                        Image.fromarray(np.clip(rgb_bg, 0, 255).astype(np.uint8)).save(self.folder/(prefix+'.png'))
                    if gray_bg is not None:
                        np.save(self.folder/(prefix+'_gray.npy'), gray_bg)
                    if preview is not None:
                        Image.fromarray(preview).save(self.folder/(prefix+'_preview.png'))
                    events_file.write(json.dumps(dict(event='background_snapshot', prefix=prefix))+'\n')
                    background_count += 1
        except Exception as exc:
            self.errors.append(str(exc))
        finally:
            self.stopping.set()
            for writer in (raw_writer, debug_writer):
                if writer is not None:
                    writer.release()
            for stream in (raw_file, processed_file, events_file):
                if stream is not None:
                    stream.close()
            try:
                if last_result is not None:
                    Image.fromarray(last_result).save(self.folder/'final.png')
                self.manifest.update(offered=self.offered, dropped=self.dropped, written=self.written,
                                     elapsed_seconds=(time.monotonic_ns()-self.started_ns)/1e9, errors=self.errors)
                (self.folder/'manifest.json').write_text(json.dumps(self.manifest, ensure_ascii=False, indent=2), encoding='utf-8')
                with zipfile.ZipFile(self.zip_path, 'w', compression=zipfile.ZIP_STORED) as archive:
                    for path in self.folder.iterdir():
                        archive.write(path, arcname=self.folder.name+'/'+path.name)
            except Exception as exc:
                self.errors.append(str(exc))
            self.done.set()
