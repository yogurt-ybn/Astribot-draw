"""USB camera light painting with a local Tk preview."""
import argparse
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from tkinter import font as tkfont

import cv2
import numpy as np
from PIL import Image, ImageTk

from capture import CameraWorker
from lightpaint_engine import LightPainter
from diagnostic_recorder import DiagnosticRecorder


class LightPaintApp:
    def __init__(self, root, source=None, smoke_ms=None, diagnostic=False, camera='0'):
        self.root = root
        self.root.title('光绘实验室 · USB Camera · v0.3.0')
        available_fonts = set(tkfont.families(root))
        self.ui_font = next((font for font in ('Microsoft YaHei UI', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei', 'DejaVu Sans') if font in available_fonts), 'sans-serif')
        self.root.geometry('1280x820')
        self.root.minsize(960, 650)
        self.root.configure(bg='#10141f')
        self.engine = LightPainter()
        self.engine.light_color = 'cyan'
        self.engine_lock = threading.RLock()
        self.worker = None
        self.pending_connect = False
        self.running = False
        self.closing = False
        self.rgb = self.result = None
        self.photo_refs = []
        self.drag_start = None
        self.preview_geometry = None
        self.source_override = source
        self.diagnostic = diagnostic
        self.recorder = None
        self.last_recording_folder = None
        self.record_var = tk.StringVar(value='未录制 · 先录 3 秒空背景，再采集背景并绘制 15–30 秒')
        self.camera_var = tk.StringVar(value=str(camera))
        self.mode_var = tk.StringVar(value='彩虹轨迹')
        self.threshold_var = tk.IntVar(value=230)
        self.width_var = tk.IntVar(value=4)
        self.overlay_var = tk.BooleanVar(value=False)
        self.bright_var = tk.BooleanVar(value=True)
        self.mirror_var = tk.BooleanVar(value=True)
        self.filter_var = tk.BooleanVar(value=True)
        self.change_var = tk.IntVar(value=25)
        self.light_color_var = tk.StringVar(value='青蓝光棒')
        self.status_var = tk.StringVar(value='等待连接')
        self.stats_var = tk.StringVar(value='尚未绘制')
        self.build_ui()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.bind('<space>', self.toggle_key)
        self.root.bind('<Control-s>', lambda event: self.save())
        self.root.bind('<Escape>', lambda event: self.pause())
        self.root.after(30, self.tick)
        self.connect()
        if smoke_ms:
            self.root.after(100, self.smoke_begin)
            self.root.after(smoke_ms, self.smoke_finish)

    def build_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('.', font=(self.ui_font, 10))
        style.configure('TFrame', background='#10141f')
        style.configure('TLabel', background='#10141f', foreground='#dce5f5')
        style.configure('TCheckbutton', background='#10141f', foreground='#dce5f5')
        style.configure('TButton', padding=(12, 8))
        outer = ttk.Frame(self.root, padding=18)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='光绘实验室', font=(self.ui_font, 23, 'bold')).pack(anchor='w')
        ttk.Label(outer, text='移动光点，留下轨迹。左侧为相机画面，右侧为绘制结果。').pack(anchor='w', pady=(4, 15))
        if self.diagnostic:
            recording = ttk.Frame(outer)
            recording.pack(fill='x', pady=(0, 10))
            self.record_btn = ttk.Button(recording, text='开始诊断录制', command=self.toggle_recording)
            self.record_btn.pack(side='left')
            ttk.Button(recording, text='打开录制目录', command=self.open_recording_folder).pack(side='left', padx=8)
            ttk.Label(recording, textvariable=self.record_var, wraplength=800).pack(side='left', padx=8)
        toolbar = ttk.Frame(outer)
        toolbar.pack(fill='x')
        ttk.Label(toolbar, text='相机').pack(side='left')
        ttk.Combobox(toolbar, textvariable=self.camera_var, values=['0', '1', '2', '3'], width=14).pack(side='left', padx=8)
        self.connect_btn = ttk.Button(toolbar, text='重新连接', command=self.connect)
        self.connect_btn.pack(side='left', padx=(0, 20))
        ttk.Label(toolbar, text='模式').pack(side='left')
        mode = ttk.Combobox(toolbar, textvariable=self.mode_var, values=['彩虹轨迹', '取亮长曝光'], state='readonly', width=14)
        mode.pack(side='left', padx=8)
        mode.bind('<<ComboboxSelected>>', self.change_mode)
        self.start_btn = ttk.Button(toolbar, text='开始绘制', command=self.toggle)
        self.start_btn.pack(side='right', padx=4)
        ttk.Button(toolbar, text='保存 PNG', command=self.save).pack(side='right', padx=4)
        ttk.Button(toolbar, text='清空', command=self.clear).pack(side='right', padx=4)
        source_options = ttk.Frame(outer)
        source_options.pack(fill='x', pady=(12, 0))
        ttk.Label(source_options, text='光源类型').pack(side='left')
        light_color = ttk.Combobox(source_options, textvariable=self.light_color_var,
                                  values=['青蓝光棒', '白光 / 手电'], state='readonly', width=14)
        light_color.pack(side='left', padx=8)
        light_color.bind('<<ComboboxSelected>>', self.change_light_color)
        ttk.Label(source_options, text='青蓝光棒按颜色和发光强度筛选，排除白衣服、窗户和蓝色管身。').pack(side='left')
        settings = ttk.Frame(outer)
        settings.pack(fill='x', pady=12)
        ttk.Label(settings, text='亮度阈值').pack(side='left')
        tk.Scale(settings, variable=self.threshold_var, from_=100, to=254, orient='horizontal', length=165,
                 bg='#10141f', fg='#dce5f5', highlightthickness=0, troughcolor='#303a50').pack(side='left', padx=8)
        ttk.Label(settings, text='线宽').pack(side='left')
        ttk.Spinbox(settings, from_=1, to=16, textvariable=self.width_var, width=4).pack(side='left', padx=8)
        for text, var, command in [('相机背景', self.overlay_var, None), ('仅累积亮部', self.bright_var, None), ('镜像', self.mirror_var, self.mirror_changed)]:
            ttk.Checkbutton(settings, text=text, variable=var, command=command).pack(side='left', padx=8)
        filtering = ttk.Frame(outer)
        filtering.pack(fill='x', pady=(0, 12))
        ttk.Checkbutton(filtering, text='过滤静止光源', variable=self.filter_var, command=self.filter_changed).pack(side='left')
        ttk.Label(filtering, text='变化阈值').pack(side='left', padx=(18, 0))
        tk.Scale(filtering, variable=self.change_var, from_=8, to=80, orient='horizontal', length=140,
                 bg='#10141f', fg='#dce5f5', highlightthickness=0, troughcolor='#303a50').pack(side='left', padx=8)
        ttk.Label(filtering, text='数值越大，过滤越强。固定相机，先移走光源采集背景。').pack(side='left', padx=8)
        panels = ttk.Frame(outer)
        panels.pack(fill='both', expand=True)
        panels.columnconfigure((0, 1), weight=1, uniform='preview')
        panels.rowconfigure(1, weight=1)
        ttk.Label(panels, text='相机画面 · 拖框限定识别区域').grid(row=0, column=0, sticky='w', pady=(0, 7))
        ttk.Label(panels, text='光绘画布').grid(row=0, column=1, sticky='w', padx=(12, 0), pady=(0, 7))
        self.source_canvas = tk.Canvas(panels, bg='#070a10', highlightthickness=0)
        self.output_canvas = tk.Canvas(panels, bg='#000000', highlightthickness=0)
        self.source_canvas.grid(row=1, column=0, sticky='nsew', padx=(0, 6))
        self.output_canvas.grid(row=1, column=1, sticky='nsew', padx=(6, 0))
        self.source_canvas.bind('<ButtonPress-1>', self.begin_roi)
        self.source_canvas.bind('<B1-Motion>', self.move_roi)
        self.source_canvas.bind('<ButtonRelease-1>', self.finish_roi)
        actions = ttk.Frame(outer)
        actions.pack(fill='x', pady=12)
        ttk.Button(actions, text='取消识别区域', command=lambda: self.set_roi(None)).pack(side='left')
        ttk.Button(actions, text='采集背景', command=self.background).pack(side='left', padx=8)
        ttk.Label(actions, text='采集背景会清空画布。相机移动或灯光变化后，请重新采集。').pack(side='left', padx=8)
        ttk.Label(outer, textvariable=self.status_var, font=(self.ui_font, 11)).pack(anchor='w')
        ttk.Label(outer, textvariable=self.stats_var).pack(anchor='w', pady=4)
        ttk.Label(outer, text='空格：开始 / 暂停    Esc：暂停    Ctrl+S：保存    关闭窗口后释放摄像头').pack(anchor='w', pady=(8, 0))

    def connect(self):
        if self.recorder is not None:
            self.stop_recording()
        self.pause()
        if self.worker and self.worker.thread.is_alive():
            self.worker.stop()
            self.pending_connect = True
            self.status_var.set('正在释放上一台摄像头…')
            return
        self.pending_connect = False
        self.rgb = self.result = None
        with self.engine_lock:
            self.engine.reset()
        choice = self.camera_var.get().strip()
        source = self.source_override if self.source_override is not None else int(choice) if choice.isdecimal() else choice
        self.sync_settings()
        self.worker = CameraWorker(source, self.engine, self.engine_lock, self.mirror_var.get())
        self.status_var.set('正在连接…')

    def pause(self):
        self.running = False
        with self.engine_lock:
            if self.worker:
                self.worker.drawing = False
            self.engine.break_stroke()
        self.start_btn.configure(text='开始绘制')
        self.record_event('pause')

    def toggle_key(self, event):
        if isinstance(event.widget, (ttk.Combobox, ttk.Spinbox)):
            return
        self.toggle()
        return 'break'

    def toggle(self):
        if self.running:
            self.pause()
            self.status_var.set('已暂停，画面仍可预览和保存')
        elif self.rgb is not None and self.worker and self.worker.thread.is_alive():
            self.running = True
            with self.engine_lock:
                self.worker.drawing = True
            self.start_btn.configure(text='暂停绘制')
            self.status_var.set('正在绘制 · 移动光源即可留下轨迹')
            self.record_event('drawing_started')

    def clear(self, keep_background=True):
        self.record_event('clear', dict(keep_background=keep_background))
        with self.engine_lock:
            roi = self.engine.roi
            self.engine.reset(keep_background=keep_background)
            self.engine.roi = roi
        self.result = None
        self.output_canvas.delete('all')
        self.status_var.set('画布已清空')

    def mirror_changed(self):
        with self.engine_lock:
            if self.worker:
                self.worker.mirrored = self.mirror_var.get()
            self.engine.roi = None
            self.clear(keep_background=False)

    def filter_changed(self):
        self.sync_settings()
        self.clear()
        self.status_var.set('静止光源过滤已开启，画布已清空' if self.filter_var.get() else '静止光源过滤已关闭，画布已清空')

    def change_mode(self, event=None):
        with self.engine_lock:
            self.engine.mode = 'neon' if self.mode_var.get() == '彩虹轨迹' else 'exposure'
            self.clear()

    def change_light_color(self, event=None):
        with self.engine_lock:
            self.engine.light_color = 'cyan' if self.light_color_var.get() == '青蓝光棒' else 'white'
            self.clear()

    def set_roi(self, roi):
        with self.engine_lock:
            self.engine.set_roi(roi)

    def sync_settings(self):
        with self.engine_lock:
            self.engine.threshold = self.threshold_var.get()
            try:
                self.engine.width = max(1, min(16, self.width_var.get()))
            except tk.TclError:
                pass
            self.engine.overlay = self.overlay_var.get()
            self.engine.only_bright = self.bright_var.get()
            self.engine.filter_static = self.filter_var.get()
            self.engine.change_threshold = self.change_var.get()

    def background(self):
        if self.rgb is not None:
            with self.engine_lock:
                self.engine.set_background(self.rgb)
                self.clear()
                if self.recorder is not None:
                    self.recorder.background(self.engine.static_background, self.engine.background, self.rgb)
            self.status_var.set('背景已采集，画布已清空；现在可将绘制光源移回画面')
            self.record_event('background_captured')

    def recording_settings(self):
        with self.engine_lock:
            return self.worker.settings()

    def record_event(self, name, details=None):
        if self.recorder is not None:
            self.recorder.event(name, details)

    def toggle_recording(self):
        if self.recorder is not None:
            self.stop_recording()
            return
        if self.rgb is None or not self.worker or not self.worker.thread.is_alive():
            self.record_var.set('请先连接摄像头，再开始录制')
            return
        try:
            self.recorder = DiagnosticRecorder(Path(__file__).resolve().parent/'output'/'diagnostics',
                                               self.worker.source, self.worker.nominal_fps, self.recording_settings())
            with self.engine_lock:
                self.recorder.background(self.engine.static_background, self.engine.background, self.rgb)
            self.recorder.event('recording_started')
            self.worker.recorder = self.recorder
            self.record_btn.configure(text='停止录制并打包')
            self.record_var.set('正在录制原始画面和诊断数据；停止后自动生成 ZIP')
        except Exception as exc:
            self.recorder = None
            messagebox.showerror('录制启动失败', str(exc), parent=self.root)

    def stop_recording(self):
        if self.recorder is None or self.recorder.stopping.is_set():
            return
        self.recorder.event('recording_stopped')
        if self.worker:
            self.worker.recorder = None
        self.recorder.stop()
        self.record_btn.configure(text='正在打包…', state='disabled')
        self.record_var.set('正在写完视频并打包，请稍候…')
        self.root.after(100, self.recording_finished)

    def recording_finished(self):
        if self.recorder is None:
            return
        if not self.recorder.done.is_set():
            self.root.after(100, self.recording_finished)
            return
        recorder = self.recorder
        self.last_recording_folder = recorder.folder
        dropped = sum(recorder.dropped.values())
        if recorder.errors:
            self.record_var.set(f'录制存在错误，原文件已保留：{recorder.errors[0]}')
        else:
            self.record_var.set(f'已保存：{recorder.zip_path.name} · 原始 {recorder.written["raw"]} 帧 / 处理 {recorder.written["processed"]} 帧 / 写盘丢帧 {dropped}')
        print(f'DIAGNOSTIC_SAVED: {recorder.zip_path}', flush=True)
        self.recorder = None
        self.record_btn.configure(text='开始诊断录制', state='normal')

    def open_recording_folder(self):
        folder = self.last_recording_folder.parent if self.last_recording_folder else Path(__file__).resolve().parent/'output'/'diagnostics'
        folder.mkdir(parents=True, exist_ok=True)
        if sys.platform == 'win32':
            os.startfile(str(folder))
        else:
            subprocess.Popen(['open' if sys.platform == 'darwin' else 'xdg-open', str(folder)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def image_point(self, event):
        if self.preview_geometry is None or self.rgb is None:
            return None
        x0, y0, scale = self.preview_geometry
        h, w = self.rgb.shape[:2]
        return (int(np.clip((event.x-x0)/scale, 0, w-1)), int(np.clip((event.y-y0)/scale, 0, h-1)))

    def begin_roi(self, event):
        self.drag_start = self.image_point(event)
        self.drag_current = self.drag_start

    def move_roi(self, event):
        self.drag_current = self.image_point(event)

    def finish_roi(self, event):
        end = self.image_point(event)
        if self.drag_start and end:
            x0, x1 = sorted([self.drag_start[0], end[0]])
            y0, y1 = sorted([self.drag_start[1], end[1]])
            if x1-x0 > 15 and y1-y0 > 15:
                self.set_roi((x0, y0, x1, y1))
                self.status_var.set('已限定识别区域')
        self.drag_start = None

    def paint_canvas(self, canvas, rgb):
        cw, ch = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        h, w = rgb.shape[:2]
        scale = min(cw/w, ch/h)
        size = (max(1, round(w*scale)), max(1, round(h*scale)))
        photo = ImageTk.PhotoImage(Image.fromarray(rgb).resize(size, Image.Resampling.BILINEAR))
        self.photo_refs.append(photo)
        canvas.delete('all')
        x0, y0 = (cw-size[0])/2, (ch-size[1])/2
        canvas.create_image(x0, y0, image=photo, anchor='nw')
        return x0, y0, scale

    def tick(self):
        if self.closing:
            return
        if self.pending_connect and not self.worker.thread.is_alive():
            self.connect()
        if self.worker:
            self.sync_settings()
            if self.recorder is not None and self.recorder.done.is_set():
                self.worker.recorder = None
                self.recording_finished()
            while True:
                try:
                    kind, text = self.worker.events.get_nowait()
                except queue.Empty:
                    break
                if kind in ('error', 'ended'):
                    self.pause()
                    self.status_var.set(text)
                elif kind == 'ready':
                    self.status_var.set(text)
            try:
                packet = self.worker.frames.get_nowait()
            except queue.Empty:
                packet = None
            if packet is not None:
                self.rgb, seq, captured_ns, self.result, position, stats = packet
                source = self.rgb.copy()
                if position is not None:
                    cv2.circle(source, tuple(np.rint(position).astype(int)), 12, (70, 255, 120), 2, cv2.LINE_AA)
                self.photo_refs = []
                self.preview_geometry = self.paint_canvas(self.source_canvas, source)
                self.paint_canvas(self.output_canvas, self.result)
                roi = self.engine.roi
                if self.drag_start and self.drag_current:
                    roi = (*self.drag_start, *self.drag_current)
                if roi:
                    x0, y0, scale = self.preview_geometry
                    self.source_canvas.create_rectangle(x0+roi[0]*scale, y0+roi[1]*scale,
                        x0+roi[2]*scale, y0+roi[3]*scale, outline='#55e9d0', width=2)
                h, w = self.rgb.shape[:2]
                self.stats_var.set(f'{w} × {h}  |  已累积 {stats[0]} 帧  |  轨迹点 {stats[1]}  |  未检出 {stats[2]} 帧')
        self.root.after(25, self.tick)

    def save(self):
        if self.result is None or self.engine.frames == 0:
            self.status_var.set('请先开始绘制，再保存图片')
            return
        folder = Path(__file__).resolve().parent/'output'/'captures'
        folder.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime('lightpainting_%Y%m%d_%H%M%S.png')
        path = filedialog.asksaveasfilename(parent=self.root, title='保存光绘图片', initialdir=str(folder),
                                          initialfile=name, defaultextension='.png', filetypes=[('PNG 图片', '*.png')])
        if path:
            try:
                Image.fromarray(self.result).save(path)
                self.status_var.set(f'图片已保存：{path}')
            except OSError as exc:
                messagebox.showerror('保存失败', str(exc), parent=self.root)

    def smoke_begin(self):
        if self.closing:
            return
        if self.rgb is not None:
            if self.diagnostic:
                self.toggle_recording()
            self.toggle()
        else:
            self.root.after(100, self.smoke_begin)

    def smoke_finish(self):
        if self.rgb is None or self.engine.frames == 0:
            print(f'SMOKE_FAILED: no processed frames; status={self.status_var.get()}', flush=True)
        else:
            print(f'SMOKE_OK: frames={self.engine.frames}, shape={self.result.shape}, points={self.engine.strokes}', flush=True)
        self.close()

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.stop_recording()
        if self.worker:
            self.worker.stop()
        self.wait_closed()

    def wait_closed(self):
        if self.worker and self.worker.thread.is_alive():
            self.root.after(50, self.wait_closed)
        elif self.recorder is not None and not self.recorder.done.is_set():
            self.root.after(50, self.wait_closed)
        else:
            if self.recorder is not None:
                self.recording_finished()
            self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', help='Use a video as the preview input')
    parser.add_argument('--camera', default='0', help='Camera index or Linux /dev/video device path')
    parser.add_argument('--smoke-ms', type=int, help=argparse.SUPPRESS)
    parser.add_argument('--diagnostic', action='store_true', help='Show diagnostic recording controls')
    args = parser.parse_args()
    root = tk.Tk()
    LightPaintApp(root, args.video, args.smoke_ms, args.diagnostic, args.camera)
    root.mainloop()


if __name__ == '__main__':
    main()
