import unittest
import numpy as np
from scipy.interpolate import CubicSpline, PPoly
from svg_plan import spline_derivative_bound
class TimingTests(unittest.TestCase):
 def test_stationary_extremum(self):
  s=PPoly(np.array([[[-1.]],[[1.5]],[[0.]],[[0.]]]),[0.,1.])
  self.assertAlmostEqual(spline_derivative_bound(s),.75)
 def test_narrow_peak_missed_by_old_uniform_grid(self):
  x=np.array([0,.5001,.50011,.50012,1.]);y=np.array([0,0,.001,0,0.])
  s=CubicSpline(x,y,bc_type='clamped');bound=spline_derivative_bound(s)
  sampled=np.max(abs(s(np.linspace(0,1,2001),1)))
  self.assertGreater(bound,sampled)
  points=np.concatenate([np.linspace(a,b,2001) for a,b in zip(x[:-1],x[1:])])
  self.assertGreaterEqual(bound+1e-8,np.max(abs(s(points,1))))
 def test_linear_and_constant(self):
  self.assertAlmostEqual(spline_derivative_bound(CubicSpline([0,.3,1],[[1,2],[1.6,2],[3,2]])),2.)
 def test_random_multiaxis_bound(self):
  rng=np.random.default_rng(1);x=np.r_[0,np.sort(rng.random(30)),1];s=CubicSpline(x,rng.normal(size=(len(x),18)))
  self.assertGreaterEqual(spline_derivative_bound(s)+1e-8,np.max(abs(s(np.linspace(0,1,100001),1))))
if __name__=='__main__':unittest.main()
