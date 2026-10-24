import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.interpolate import CubicSpline, PPoly
from svg_plan import spline_derivative_bound, schedule
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
 def test_stop_at_drawing_end_preserves_draw_without_return(self):
  model=SimpleNamespace(nq=18,opt=SimpleNamespace(timestep=.01),jnt_qposadr=np.arange(18),
      jnt_range=np.tile([-2.,2.],(18,1)))
  u=np.array([0.,1.]);home=np.zeros(18);torso=np.zeros((2,4))
  chains=[SimpleNamespace(qids=np.arange(4,11),lo=np.full(7,-2.),hi=np.full(7,2.)),
      SimpleNamespace(qids=np.arange(11,18),lo=np.full(7,-2.),hi=np.full(7,2.))]
  solved=[np.tile(np.array([.01,.03])[:,None],(1,7)) for _ in chains]
  targets=[np.array([[0.,0.,0.],[.01,0.,0.]]) for _ in chains]
  with patch('svg_plan.collision_pairs',return_value=[]),patch('svg_plan.rrt_transfer',
      side_effect=lambda m,c,start,goal,*args:[start,goal]):
   full=schedule(model,u,targets,torso,home,chains,solved)
   stopped=schedule(model,u,targets,torso,home,chains,solved,return_home=False)
  q,dq,phase,desired=stopped
  self.assertEqual(set(phase),{'approach_both','draw_both'})
  self.assertEqual(phase[-1],'draw_both')
  np.testing.assert_allclose(q[-1,4:],.03)
  np.testing.assert_allclose(dq[-1],0.,atol=1e-12)
  self.assertGreater(np.max(abs(q[-1]-home)),.02)
  for actual,original in zip(stopped,full):
   np.testing.assert_array_equal(actual,original[:len(q)])
  self.assertEqual(full[2][-1],'return_both')
  np.testing.assert_array_equal(full[0][-1],home)
if __name__=='__main__':unittest.main()
