import tempfile
import unittest
from pathlib import Path
import numpy as np
from svg_input import parse_svg, validate_masks, SVGInputError

ROOT='<svg xmlns="http://www.w3.org/2000/svg" width="616" height="560" viewBox="0 0 616 560">'
def svg(left='M 400,200 L 410,210',right='M 200,200 L 190,210'):
    return ROOT+f'<path id="left" fill="none" d="{left}"/><path id="right" fill="none" d="{right}"/></svg>'

class ParserTests(unittest.TestCase):
    def parse(self,s,**kw):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'input.svg';p.write_text(s);return parse_svg(p,**kw)
    def bad(self,s):
        with self.assertRaises(SVGInputError):self.parse(s)
    def test_lines_exact(self):
        o=self.parse(svg())
        np.testing.assert_array_equal(o['arms']['left']['points_uv'][0],[400,200])
        np.testing.assert_allclose(o['arms']['left']['points_world'][0],[.23,-.42,1.39])
        for arm in o['arms'].values():
            self.assertLessEqual(np.linalg.norm(np.diff(arm['points_uv'],axis=0),axis=1).max(),1)
            self.assertTrue(np.all(np.diff(arm['progress'])>0))
    def test_relative_and_implicit(self):
        a=self.parse(svg('m400 200 5 0 l0 5'))['arms']['left']['points_uv']
        self.assertEqual(a[-1],[405,205])
    def test_quadratic_and_cubic(self):
        o=self.parse(svg('M400 200 Q420 180 440 200 C460 220 480 180 500 200'))
        self.assertEqual(o['arms']['left']['points_uv'][-1],[500,200])
        self.assertEqual(o['arms']['left']['curve_to_polyline_error_bound_px'],.1)
    def test_curve_error(self):
        o=self.parse(svg('M400 200 Q440 120 480 200'))
        p=np.array(o['arms']['left']['points_uv']);t=np.linspace(0,1,1001)
        true=(1-t[:,None])**2*np.array([400,200])+2*(1-t[:,None])*t[:,None]*np.array([440,120])+t[:,None]**2*np.array([480,200])
        a=p[:-1];v=np.diff(p,axis=0)
        for q in true:
            u=np.clip(np.sum((q-a)*v,axis=1)/np.sum(v*v,axis=1),0,1)
            self.assertLessEqual(np.linalg.norm(q-a-u[:,None]*v,axis=1).min(),.10000001)
    def test_close(self):
        o=self.parse(svg('M400 200 L405 200 L405 205 z'))
        self.assertEqual(o['arms']['left']['points_uv'][0],o['arms']['left']['points_uv'][-1])
    def test_no_implicit_closing(self):
        o=self.parse(svg());self.assertNotEqual(o['arms']['left']['points_uv'][0],o['arms']['left']['points_uv'][-1])
    def test_invalid_dimensions(self):
        for s in [svg().replace('width="616"','width="800"'),svg().replace('height="560"','height="560mm"'),svg().replace('0 0 616 560','0 0 600 560')]:self.bad(s)
    def test_labels(self):
        for s in [svg().replace('id="left"','id="arm"'),svg().replace('id="right"','id="left"'),svg().replace('id="left"','id="left" data-arm="right"')]:self.bad(s)
    def test_external_content(self):
        for s in ['<!DOCTYPE svg [<!ENTITY e SYSTEM "file:///etc/passwd">]>'+svg(),svg().replace('</svg>','<script>alert(1)</script></svg>'),svg().replace('fill="none"','fill="url(http://evil)"'),svg().replace('id="left"','id="left" onclick="alert(1)"'),svg().replace('</svg>','<image href="http://evil"/></svg>')]:self.bad(s)
    def test_disconnected_unsupported(self):
        for d in ['M400 200 L410 210 M420 210 L430 210','M400 200 A10 10 0 0 0 420 210','M400 200 H420','M400 200 L410','M400 200 L410 210 Z L410 220','L400 200']:
            self.bad(svg(d))
        self.bad(svg().replace('id="left"','id="left" transform="scale(2)"'))
    def test_out_of_bounds(self):
        for d in ['M400 200 L700 200','M400 200 L400 -1','M400 200 L616 200']:
            self.bad(svg(d))
    def test_invalid_separators(self):
        for d in ['M,400 200 L410 210','M400,,200 L410 210','M400 200 L410 210,','M400 200,L410 210']:
            self.bad(svg(d))
    def test_encoded_xml_attack(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'attack.svg';p.write_bytes(('<!DOCTYPE svg [<!ENTITY e \"M400 200 L410 210\">]>'+svg().replace('M 400,200 L 410,210','&e;')).encode('utf-16'))
            with self.assertRaises(SVGInputError):parse_svg(p)
    def test_zero_length(self):self.bad(svg('M400 200 L400 200'))
    def test_mask_supercover(self):
        from PIL import Image
        o=self.parse(svg('M400.2 200.2 L401.8 201.8'))
        with tempfile.TemporaryDirectory() as d:
            mask=np.ones((560,616),dtype=np.uint8)*255
            for side in ['left','right']:Image.fromarray(mask).save(Path(d)/f'mask-{side}.png')
            validate_masks(o,d)
            mask[200,401]=0 # exact corner contact must not skip this cell
            Image.fromarray(mask).save(Path(d)/'mask-left.png')
            with self.assertRaises(SVGInputError):validate_masks(o,d)

if __name__=='__main__':unittest.main()
