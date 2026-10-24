# Uniform measured-result renderer

Run **after** the benchmark worker confirms reports/execution files are final and releases the model-memory slot:

```sh
# From svg_planner/, after regenerating full-rate execution.npz files:
pip install cairosvg imageio imageio-ffmpeg matplotlib
python visualizer/render_results.py runs/frozen-v3-analytic-time \
  --inputs inputs --out visuals --video --robot
```

- Same rendering code, camera, canvas scale, frame count and output format for every input. No design-specific branches.
- Verifies original SVG bytes against the benchmark freeze and successful report hashes.
- Original SVG is rendered directly using CairoSVG. Actual overlay comes only from successful `execution.npz` measured `actual` arrays.
- Robot frames replay recorded `qpos`; rendering does not execute dynamics or manufacture a successful trajectory from planned qdes.
- Canvas mapping is the calibrated T1 mapping: u=(x+0.77)/0.0025, v=(1.89-z)/0.0025. Only `draw_both` samples leave a visible trace; it is a geometric tip projection, not camera-visible laser/pen detection.
- Full measured approach/draw/return is compressed to 8 seconds, 120 frames per success. Simulated duration and speed multiplier recorded in JSON and video.
- Missing/failed actual execution produces a clearly marked source-only panel, no pretend actual replay.
- Outputs are derived views, not an assertion of contact-force control, continuous collision safety, or readiness for physical hardware.

## Final v3 visual results

Ginkgo and koi passed actual dynamics, replayed for 8 s each from measured qpos. Full simulated durations are 238.755 s and 295.635 s (29.844375x and 36.954375x playback). Wings passed static SVG validation but exhausted the deterministic coupled IK search; no execution exists, and its panel is explicitly source-only. A search rejection is not a proof of physical impossibility. The 16 s combined MP4 contains successful cases only.

System dependencies: install `ffmpeg` on PATH and DejaVu Sans fonts (Debian/Ubuntu: `sudo apt-get install ffmpeg fonts-dejavu-core`). The renderer invokes the system ffmpeg binary; the optional Python imageio-ffmpeg package does not replace that requirement.
