"""Lightweight artifact regression tests; no mesh assets required."""
import unittest,json
from pathlib import Path
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parent
class Outputs(unittest.TestCase):
 def test_calibration(self):
  c=json.loads((ROOT/'t1-canvas/calibration.json').read_text());self.assertEqual((c['width'],c['height']),(616,560));self.assertEqual(c['drawing_plane_world_y_m'],-.42)
 def test_paths_masks_and_progress(self):
  for name in ['waves','leaves','spiral']:
   r=json.loads((ROOT/f't1-results/{name}/paths.json').read_text())
   for side in ['left','right']:
    a=r['arms'][side];p=np.array(a['points_uv']);t=np.array(a['progress']);self.assertTrue(np.isfinite(p).all());self.assertEqual(t[0],0);self.assertEqual(t[-1],1);self.assertTrue((np.diff(t)>0).all());self.assertLessEqual(np.linalg.norm(np.diff(p,axis=0),axis=1).max(),1)
    ij=np.floor(p).astype(int);mask=np.array(Image.open(ROOT/f't1-canvas/mask-{side}.png').convert('L'));self.assertTrue((mask[ij[:,1],ij[:,0]]>0).all())
 def test_measured_reports(self):
  for name in ['waves','leaves','spiral']:
   r=json.loads((ROOT/f't1-results/{name}/results.json').read_text());self.assertEqual(r['robot'],'Astribot T1');self.assertGreater(r['actual_minimum']['distance_m'],.002);self.assertEqual(r['max_contacts'],0);self.assertFalse(any(r['warning_counts']));self.assertGreater(np.ptp(r['torso_z_min_max_m']),.015)
   for a in r['arms'].values():self.assertLess(a['max_m'],.001)
if __name__=='__main__':unittest.main()
