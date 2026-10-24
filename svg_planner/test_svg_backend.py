import unittest
from pathlib import Path
import tempfile
import numpy as np
from svg_input import parse_svg
from svg_plan import shared_targets, torso_candidates, validate_calibration, PlanningError, main
ROOT=Path(__file__).resolve().parent
class BackendTests(unittest.TestCase):
 def test_uniform_targets_ignore_redundant_and_nearly_coincident_source_vertices(self):
  import copy
  d=parse_svg(ROOT/'examples/short-curves.svg')
  u,p=shared_targets(d,sampling='uniform')
  changed=copy.deepcopy(d)
  for arm in changed['arms'].values():
   v=np.array(arm['progress']);uv=np.array(arm['points_uv']);extra=v[0]+(v[1]-v[0])*1e-8
   arm['progress']=np.insert(v,1,extra).tolist()
   arm['points_uv']=np.insert(uv,1,uv[0]+(uv[1]-uv[0])*1e-8,axis=0).tolist()
  uu,pp=shared_targets(changed,sampling='uniform')
  np.testing.assert_array_equal(u,uu)
  for a,b in zip(p,pp):np.testing.assert_allclose(a,b,atol=1e-14)
  self.assertLess(np.ptp(np.diff(u)),1e-14)
  self.assertLessEqual(d['metadata']['shared_target_sampling']['maximum_arc_step_m'],.0025)
 def test_uniform_target_refinement_preserves_sharp_source_corners(self):
  uv=np.array([[400.,250.],[401.37,250.],[401.37,260.]])
  arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(uv,axis=0),axis=1))]
  d={'arms':{s:dict(points_uv=uv.tolist(),progress=(arc/arc[-1]).tolist(),length_m=arc[-1]*.0025)
             for s in ('left','right')},'metadata':{}}
  u,p=shared_targets(d,sampling='uniform',max_step_m=.01,max_error_m=.0001)
  self.assertGreater(len(u),max(2,int(np.ceil(arc[-1]*.0025/.01)))+1)
  dense=np.linspace(0,1,10001)
  original=np.column_stack([np.interp(dense,arc/arc[-1],uv[:,j]) for j in range(2)])
  expected=np.c_[-.77+.0025*original[:,0],np.full(len(dense),-.42),1.89-.0025*original[:,1]]
  for points in p:
   actual=np.column_stack([np.interp(dense,u,points[:,j]) for j in range(3)])
   self.assertLessEqual(np.linalg.norm(actual-expected,axis=1).max(),.0001+1e-12)
  with self.assertRaisesRegex(PlanningError,'sampling budget'):
   shared_targets(d,sampling='uniform',max_step_m=.01,max_error_m=1e-12,max_intervals=8)
 def test_uniform_target_option_rejects_invalid_budgets(self):
  d=parse_svg(ROOT/'examples/short-curves.svg')
  for kw in ({'sampling':'unknown'},{'max_step_m':0.},{'max_error_m':float('nan')}):
   with self.assertRaises(PlanningError):shared_targets(d,**dict({'sampling':'uniform'},**kw))
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
