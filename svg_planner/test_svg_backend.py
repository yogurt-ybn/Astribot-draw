import unittest
from pathlib import Path
import tempfile
import numpy as np
from svg_input import parse_svg
from svg_plan import shared_targets, torso_candidates, validate_calibration, PlanningError, main
ROOT=Path(__file__).resolve().parent
class BackendTests(unittest.TestCase):
 def test_calibration_rejects_wrong_scale(self):
  import json
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);cfg=json.loads((ROOT/'t1-canvas/calibration.json').read_text());cfg['meters_per_pixel']=.01
   (p/'calibration.json').write_text(json.dumps(cfg))
   with self.assertRaises(PlanningError):validate_calibration(p)
 def test_failed_run_removes_stale_success_files(self):
  import contextlib,io
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);(p/'plan.npz').write_bytes(b'stale');bad=p/'bad.svg';bad.write_text('<svg/>')
   with contextlib.redirect_stdout(io.StringIO()):code=main([str(bad),'--out',str(p),'--mode','static'])
   self.assertEqual(code,2);self.assertFalse((p/'plan.npz').exists());self.assertTrue((p/'report.json').exists())
 def test_union_preserves_vertices(self):
  d=parse_svg(ROOT/'examples/short-curves.svg');u,p=shared_targets(d)
  for si,s in enumerate(('left','right')):
   a=d['arms'][s]
   for t,world in zip(a['progress'],a['points_world']):
    k=np.flatnonzero(u==t);self.assertEqual(len(k),1);np.testing.assert_allclose(p[si][k[0]],world,atol=1e-14)
 def test_mirrored_wing_knots_are_numerically_stable(self):
  d=parse_svg(ROOT/'examples/wings.svg');u,p=shared_targets(d)
  self.assertGreater(np.min(np.diff(u)),1e-12)
  for si,side in enumerate(('left','right')):
   a=d['arms'][side];reconstructed=np.column_stack([np.interp(a['progress'],u,p[si][:,j]) for j in range(3)])
   self.assertLess(np.max(np.linalg.norm(reconstructed-np.array(a['points_world']),axis=1)),1e-10)
 def test_torso_is_shared_finite_family(self):
  d=parse_svg(ROOT/'examples/short-curves.svg');u,p=shared_targets(d)
  variants=list(torso_candidates(u,p));self.assertEqual(len(variants),4)
  for _,q in variants:self.assertEqual(q.shape,(len(u),4));self.assertTrue(np.isfinite(q).all())
 def test_all_supplied_examples_parse(self):
  for p in (ROOT/'examples').rglob('*.svg'):self.assertEqual(set(parse_svg(p)['arms']),{'left','right'})
if __name__=='__main__':unittest.main()
