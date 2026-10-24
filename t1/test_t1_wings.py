"""Regression checks for exported large wings and recorded execution summary."""
import json,unittest
from pathlib import Path
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parent
class Wings(unittest.TestCase):
 def test_geometry(self):
  d=json.loads((ROOT/'t1-results/wings/paths.json').read_text())
  self.assertEqual((d['width'],d['height']),(616,560))
  points=[]
  for side in ['left','right']:
   p=np.asarray(d['arms'][side]['points_uv']);points.append(p)
   self.assertEqual(p.shape,(1001,2));self.assertTrue(np.isfinite(p).all())
   self.assertTrue((np.diff(d['arms'][side]['progress'])>0).all())
   mask=np.asarray(Image.open(ROOT/'t1-canvas'/f'mask-{side}.png'))>0
   ij=np.floor(p).astype(int);self.assertTrue(mask[ij[:,1],ij[:,0]].all())
   self.assertGreater(np.ptp(p[:,0])*.0025,.47)
   self.assertGreater(np.ptp(p[:,1])*.0025,.65)
   self.assertGreater(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()*.0025,1.9)
  np.testing.assert_allclose(points[0][:,0]+points[1][:,0],616,atol=1e-9)
  np.testing.assert_allclose(points[0][:,1],points[1][:,1],atol=1e-9)
 def test_recorded_execution(self):
  r=json.loads((ROOT/'t1-results/wings/results.json').read_text())
  self.assertEqual(r['robot'],'Astribot T1')
  self.assertGreaterEqual(r['planned_minimum_m'],.002)
  self.assertGreater(r['actual_minimum']['distance_m'],0)
  self.assertEqual(r['max_contacts'],0)
  self.assertFalse(any(r['warning_counts']))
  for side in ['left','right']:
   self.assertLess(r['arms'][side]['max_m'],.005)
   self.assertGreater(r['arms'][side]['min_joint_margin_rad'],0)
if __name__=='__main__':unittest.main()
