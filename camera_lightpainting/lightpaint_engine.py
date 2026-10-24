"""Camera-independent light-painting processing, shared by preview and video export."""
import colorsys
import math
import time

import cv2
import numpy as np


class LightPainter:
    def __init__(self):
        self.mode = 'neon'
        self.threshold = 230
        self.width = 4
        self.only_bright = True
        self.overlay = False
        self.filter_static = True
        self.change_threshold = 25
        self.light_color = 'white'
        self.reset()

    def reset(self, keep_background=False):
        background = getattr(self, 'background', None) if keep_background else None
        static_background = getattr(self, 'static_background', None) if keep_background else None
        self.trail = None
        self.exposure = None
        self.background = background
        self.static_background = static_background
        self.foreground_mask = None
        self.source_mask = None
        self._glow_result = None
        self._glow_strokes = -1
        self.last_tracking_event = 'paused'
        self.previous = None
        self.filtered = None
        self.misses = 0
        self.strokes = 0
        self.frames = 0
        self.started = time.monotonic()
        self.roi = None

    def break_stroke(self):
        self.previous = None
        self.filtered = None

    def light_mask(self, rgb, allowed):
        if self.light_color == 'cyan':
            hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
            channels = rgb.astype(np.int16)
            value = hsv[:, :, 2]
            # Bright cyan emission has strong green AND blue. Dark blue tube
            # plastic, white clothing and pale cyan window reflections fail.
            mask = ((hsv[:, :, 0] >= 75) & (hsv[:, :, 0] <= 112)
                    & (hsv[:, :, 1] >= 60) & (value >= self.threshold)
                    & (channels[:, :, 1] >= value*.65)
                    & (channels[:, :, 2] >= channels[:, :, 0]+25)).astype(np.uint8)
        else:
            mask = (cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY) >= self.threshold).astype(np.uint8)
        if allowed is not None:
            mask &= allowed
        if self.roi:
            x0, y0, x1, y1 = self.roi
            mask[:y0] = mask[y1:] = 0
            mask[:, :x0] = mask[:, x1:] = 0
        return mask

    def detect_colored_light(self, rgb):
        n, labels, stats, _ = cv2.connectedComponentsWithStats(self.source_mask)
        candidates = []
        h, w = rgb.shape[:2]
        for label in range(1, n):
            x, y, bw, bh, area = stats[label]
            if area < 2 or area > h*w*.03:
                continue
            local = labels[y:y+bh, x:x+bw] == label
            patch = rgb[y:y+bh, x:x+bw].astype(np.float32)
            weights = patch[:, :, 1]**2 * local
            yy, xx = np.mgrid[y:y+bh, x:x+bw]
            point = (float((xx*weights).sum()/weights.sum()), float((yy*weights).sum()/weights.sum()))
            candidates.append((float(weights.sum()), label, point))
        if not candidates:
            self.source_mask[:] = 0
            return None
        _, label, point = max(candidates)
        # Only the selected emitter participates in accumulation.
        self.source_mask = (labels == label).astype(np.uint8)
        return point

    def set_background(self, rgb):
        self.background = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(np.float32)
        self.static_background = rgb.astype(np.float32).copy()
        self.break_stroke()

    def moving_light_mask(self, rgb, accumulate):
        """Reject unchanged light before every detection and accumulation path.

        Only positive change is accepted: a disappearing light is not a new
        stroke. Preview learns quickly; recording learns slowly so a briefly
        stationary drawing light is retained. Darker changes recover quickly
        when a moving lamp uncovers the scene.
        """
        current = rgb.astype(np.float32)
        if self.static_background is None or self.static_background.shape != rgb.shape:
            self.static_background = current.copy()
            return np.zeros(rgb.shape[:2], np.uint8)
        delta = current-self.static_background
        # Compensate modest scene-wide exposure/color drift, ignoring local lamps.
        shift = np.clip(np.median(delta[::12, ::12], axis=(0, 1)), -40, 40)
        # Avoid a reduction over interleaved RGB for every pixel; separate
        # channel maxima are substantially cheaper on the Windows runtime.
        positive = np.maximum(np.maximum(delta[:, :, 0]-shift[0], delta[:, :, 1]-shift[1]),
                              delta[:, :, 2]-shift[2])
        mask = (positive >= self.change_threshold).astype(np.uint8)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # Do not expand the mask into adjacent static highlights.
        alpha = 0.008 if accumulate else 0.15
        cv2.accumulateWeighted(rgb, self.static_background, alpha)
        np.minimum(delta, 0, out=delta)
        delta *= np.float32(.2-alpha)
        self.static_background += delta
        return mask

    def set_roi(self, roi):
        self.roi = roi
        self.break_stroke()

    def detect(self, rgb, allowed=None):
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        if self.background is None:
            self.background = gray.astype(np.float32)
        channels = rgb.astype(np.int16)
        warm_mask = ((rgb.min(axis=2) >= self.threshold)
                     & (channels[:, :, 0] >= channels[:, :, 1]-2)
                     & (channels[:, :, 0] >= channels[:, :, 2]+2)).astype(np.uint8)
        mask = (gray >= self.threshold).astype(np.uint8)
        if allowed is not None:
            warm_mask &= allowed
            mask &= allowed
        if self.roi:
            x0, y0, x1, y1 = self.roi
            mask[:y0] = 0
            mask[y1:] = 0
            mask[:, :x0] = 0
            mask[:, x1:] = 0
            warm_mask[:y0] = 0
            warm_mask[y1:] = 0
            warm_mask[:, :x0] = 0
            warm_mask[:, x1:] = 0
        # Prefer a compact warm-white lamp core over a cyan window. If it
        # merges with the window, temporal change provides a second estimate.
        n_warm, warm_labels, warm_stats, _ = cv2.connectedComponentsWithStats(warm_mask)
        warm_candidates = []
        h, w = gray.shape
        for label in range(1, n_warm):
            x, y, bw, bh, area = warm_stats[label]
            if x < 2 or y < 2 or x+bw >= w-2 or y+bh >= h-2:
                continue
            if area >= 70 and area < h*w*0.12 and max(bw, bh)/max(1, min(bw, bh)) < 2.5:
                local = (warm_labels[y:y+bh, x:x+bw] == label).astype(np.uint8)
                distance = cv2.distanceTransform(np.pad(local, 1), cv2.DIST_L2, 5)[1:-1, 1:-1]
                _, radius, _, center = cv2.minMaxLoc(distance)
                if radius >= 4:
                    warm_candidates.append((area, (x+center[0], y+center[1])))
        chosen = max(warm_candidates)[1] if warm_candidates else None
        if chosen is None:
            change = np.maximum(gray.astype(np.float32)-self.background, 0)
            if allowed is not None:
                change *= allowed
            if self.roi:
                change *= (mask > 0)
            change = cv2.GaussianBlur(change, (0, 0), max(2, w/160))
            if allowed is not None:
                change *= allowed
            change[:8] = change[-8:] = 0
            change[:, :8] = change[:, -8:] = 0
            _, peak, _, point = cv2.minMaxLoc(change)
            if peak > 70:
                chosen = point
        if chosen is not None:
            cx, cy = chosen
            r = max(8, round(w/45))
            x0, x1 = max(0, cx-r), min(w, cx+r+1)
            y0, y1 = max(0, cy-r), min(h, cy+r+1)
            patch = gray[y0:y1, x0:x1].astype(np.float32)
            weights = np.maximum(patch-patch.max()*0.65, 0)**2
            if allowed is not None:
                weights *= allowed[y0:y1, x0:x1]
            yy, xx = np.mgrid[y0:y1, x0:x1]
            if weights.sum() > 0:
                chosen = (float((xx*weights).sum()/weights.sum()), float((yy*weights).sum()/weights.sum()))
            cv2.accumulateWeighted(gray, self.background, 0.008)
            return chosen
        n, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
        best = None
        h, w = gray.shape
        for label in range(1, n):
            x, y, bw, bh, area = stats[label]
            # Window-sized and edge-connected highlights are usually background.
            if area < 3 or area > h*w*0.09:
                continue
            if x < 2 or y < 2 or x+bw >= w-2 or y+bh >= h-2:
                continue
            aspect = max(bw, bh)/max(1, min(bw, bh))
            if aspect > 3.5:
                continue
            local_mask = labels[y:y+bh, x:x+bw] == label
            distance = cv2.distanceTransform(np.pad(local_mask.astype(np.uint8), 1), cv2.DIST_L2, 5)[1:-1, 1:-1]
            _, radius, _, center = cv2.minMaxLoc(distance)
            cx, cy = x+center[0], y+center[1]
            outer = max(12, int(max(bw, bh)*1.2))
            patch = gray[max(0, cy-outer):min(h, cy+outer+1), max(0, cx-outer):min(w, cx+outer+1)]
            contrast = max(0, float(gray[cy, cx])-float(np.percentile(patch, 35)))
            change = max(0, float(gray[cy, cx])-float(self.background[cy, cx]))
            compactness = min(1.0, 4*math.pi*radius*radius/max(1, area))
            # Saturated LEDs and warm lamps receive a small tie-break against cyan windows.
            red, green, blue = map(float, rgb[cy, cx])
            warmth = max(-15, min(15, red-blue))
            score = contrast + change*0.25 + compactness*35 + min(radius, 25) + warmth
            if self.filtered is not None:
                gap = math.dist((cx, cy), self.filtered)
                score += max(0, 25-gap*0.15)
            if best is None or score > best[0]:
                best = (score, (float(cx), float(cy)))
        # Slow background adaptation lets newly introduced light stand out.
        cv2.accumulateWeighted(gray, self.background, 0.008)
        return best[1] if best is not None and best[0] >= 30 else None

    def process(self, rgb, accumulate=True, timestamp=None):
        h, w = rgb.shape[:2]
        if self.trail is None or self.trail.shape != rgb.shape:
            self.trail = np.zeros_like(rgb)
            self.exposure = np.zeros_like(rgb)
            self._glow_result = None
            if self.background is not None and self.background.shape != (h, w):
                self.background = None
                self.break_stroke()
                self.roi = None
        self.foreground_mask = self.moving_light_mask(rgb, accumulate)
        allowed = self.foreground_mask if self.filter_static else None
        self.source_mask = self.light_mask(rgb, allowed)
        position = None
        self.last_tracking_event = 'paused'
        if accumulate:
            self.frames += 1
            if self.light_color == 'cyan':
                position = self.detect_colored_light(rgb)
            if self.mode == 'exposure':
                self.last_tracking_event = 'exposure'
                if self.light_color == 'cyan':
                    light = rgb*self.source_mask[:, :, None]
                    self.last_tracking_event = 'miss' if position is None else 'new_stroke'
                    if position is not None:
                        self.strokes += 1
                    else:
                        self.misses += 1
                elif self.only_bright:
                    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
                    alpha = np.clip((gray.astype(np.float32)-self.threshold)/max(1, 255-self.threshold), 0, 1)
                    light = (rgb*alpha[:, :, None]).astype(np.uint8)
                else:
                    light = rgb
                if allowed is not None:
                    light = light*allowed[:, :, None]
                if self.roi:
                    x0, y0, x1, y1 = self.roi
                    cropped = np.zeros_like(light)
                    cropped[y0:y1, x0:x1] = light[y0:y1, x0:x1]
                    light = cropped
                np.maximum(self.exposure, light, out=self.exposure)
            else:
                if self.light_color != 'cyan':
                    position = self.detect(rgb, allowed)
                if position is None:
                    self.last_tracking_event = 'miss'
                    self.misses += 1
                    self.previous = None
                    self.filtered = None
                else:
                    point = np.asarray(position, dtype=np.float32)
                    if self.filtered is not None:
                        point = self.filtered*0.3+point*0.7
                    self.filtered = point
                    point_int = tuple(np.rint(point).astype(int))
                    hue = (0.5+self.strokes/220) % 1
                    color = tuple(round(v*255) for v in colorsys.hsv_to_rgb(hue, 1, 1))
                    if self.previous is not None and math.dist(point, self.previous) < w*0.22:
                        self.last_tracking_event = 'joined'
                        cv2.line(self.trail, self.previous, point_int, color, self.width, cv2.LINE_AA)
                    else:
                        self.last_tracking_event = 'new_stroke' if self.previous is None else 'jump_rejected'
                        cv2.circle(self.trail, point_int, max(1, self.width//2), color, -1, cv2.LINE_AA)
                    self.previous = point_int
                    self.strokes += 1
        if self.mode == 'neon':
            if self._glow_result is None or self._glow_strokes != self.strokes:
                near = cv2.GaussianBlur(self.trail, (0, 0), 2)
                small = cv2.resize(self.trail, (max(1, w//2), max(1, h//2)), interpolation=cv2.INTER_AREA)
                halo = cv2.GaussianBlur(small, (0, 0), 3.5)
                halo = cv2.resize(halo, (w, h), interpolation=cv2.INTER_LINEAR)
                glow = cv2.addWeighted(self.trail, 1, near, 1.6, 0)
                self._glow_result = cv2.addWeighted(glow, 1, halo, 2.5, 0)
                self._glow_strokes = self.strokes
            result = self._glow_result
        else:
            result = self.exposure.copy()
        if self.overlay:
            result = np.maximum(result, (rgb*0.28).astype(np.uint8))
        return result, position
