"""Static validation plus actual SVG rendering, not robot execution."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageFilter
from svg_input import parse_svg, validate_masks
from smoothness import check_smoothness


def validate(source, out, canvas):
    source = Path(source)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    report = dict(candidate=source.stem, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  static='FAIL', mechanical_feasibility='UNVALIDATED', ik='not_run',
                  full_plan='not_run', simulation='not_run', modification_log=[])
    dest = out / (source.stem + '.report.json')
    try:
        import cairosvg
        data = parse_svg(source)
        report['masks'] = validate_masks(data, canvas)
        report['format_and_masks'] = 'PASS'
        report['smoothness'] = check_smoothness(source)
        if report['smoothness']['status'] != 'PASS':
            raise ValueError('v1.1 geometric smoothness rejected; see smoothness.arms errors')
        raw = source.read_bytes()
        png = cairosvg.svg2png(bytestring=raw, output_width=616, output_height=560, unsafe=False)
        # Render the exact same styling but replace source curves with the parser's
        # centerlines; compare both masks bidirectionally with one pixel dilation.
        root = ET.fromstring(raw)
        for path in root:
            side = path.get('data-arm', path.get('id'))
            points = data['arms'][side]['points_uv']
            path.set('d', 'M ' + ' L '.join(f'{u:.10f} {v:.10f}' for u, v in points))
        poly_png = cairosvg.svg2png(bytestring=ET.tostring(root), output_width=616, output_height=560, unsafe=False)
        def mask(blob):
            alpha = Image.open(io.BytesIO(blob)).convert('RGBA').getchannel('A')
            return Image.fromarray(np.uint8(np.asarray(alpha)>32)*255)
        a, b = mask(png), mask(poly_png)
        aa, bb = np.asarray(a)>0, np.asarray(b)>0
        da, db = np.asarray(a.filter(ImageFilter.MaxFilter(3)))>0, np.asarray(b.filter(ImageFilter.MaxFilter(3)))>0
        n1, n2 = int(np.count_nonzero(aa & ~db)), int(np.count_nonzero(bb & ~da))
        report['render_comparison'] = dict(method='SVG vs same-style parsed polyline alpha masks; bidirectional 1px Chebyshev dilation',
            alpha_threshold=32, pixel_tolerance=1, source_pixels=int(aa.sum()), parsed_pixels=int(bb.sum()),
            source_unmatched=n1, parsed_unmatched=n2, status='PASS' if not(n1 or n2) and aa.any() and bb.any() else 'FAIL',
            limitation='Raster agreement at this resolution, not a mathematical geometric equivalence proof')
        if report['render_comparison']['status'] != 'PASS':
            raise ValueError('SVG/polyline rendering mismatch or invisible paths')
        (out/(source.stem+'.png')).write_bytes(png)
        report['arms'] = {}
        for side, arm in data['arms'].items():
            points = np.array(arm['points_uv'])
            report['arms'][side] = dict(bounds_uv=[points.min(axis=0).tolist(),points.max(axis=0).tolist()],
                length_m=arm['length_m'], closed=arm['closed'], flattened_points=len(points),
                curve_to_polyline_error_bound_px=arm['curve_to_polyline_error_bound_px'],
                removed_zero_length_segments=arm['removed_zero_length_segments'])
        report['static'] = 'PASS'
    except Exception as exc:
        report['reason'] = f'{type(exc).__name__}: {exc}'
    dest.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(report=str(dest), static=report['static'], reason=report.get('reason')), ensure_ascii=False))
    return 0 if report['static']=='PASS' else 2

if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('svg')
    p.add_argument('--out',default='generated')
    p.add_argument('--canvas',default=str(Path(__file__).parent/'t1-canvas'))
    a=p.parse_args()
    raise SystemExit(validate(a.svg,a.out,a.canvas))
