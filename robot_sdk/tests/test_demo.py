import unittest

import numpy as np

from astribot_draw_sdk import ExecutionPolicy
from astribot_draw_sdk.backends import MOCK_HOME
from astribot_draw_sdk.cli import demo_trajectory


class DemoTests(unittest.TestCase):
    def test_visible_motion_has_two_peak_holds_and_returns_to_start(self):
        plan = demo_trajectory(MOCK_HOME)
        self.assertAlmostEqual(plan.time[-1], 12.)
        np.testing.assert_allclose(plan.position[[0, -1]], np.tile(MOCK_HOME, (2, 1)), atol=1e-12)
        np.testing.assert_allclose(plan.velocity[[0, -1]], 0., atol=1e-12)
        delta = plan.position-MOCK_HOME
        np.testing.assert_allclose(delta[:, 7], delta[:, 14])
        self.assertAlmostEqual(delta[:, 7].max(), np.deg2rad(10.), places=12)
        other = [i for i in range(18) if i not in (7, 14)]
        np.testing.assert_allclose(delta[:, other], 0.)
        holds = np.array(plan.phase) == 'demo_hold'
        starts = np.flatnonzero(holds & ~np.r_[False, holds[:-1]])
        self.assertEqual(len(starts), 2)
        self.assertGreaterEqual(holds.sum()/125, 1.98)

    def test_larger_demo_remains_within_execution_speed_and_step_policy(self):
        plan = demo_trajectory(MOCK_HOME)
        plan.validate(ExecutionPolicy(), np.full(18, -3.), np.full(18, 3.), np.ones(18), MOCK_HOME)
        self.assertLess(np.abs(plan.velocity).max(), .17)
        # The analytical desired velocity agrees with the interpolated position derivative.
        for t in (1., 4., 7., 10.):
            before = plan.sample(t-1e-5)[0]
            after = plan.sample(t+1e-5)[0]
            np.testing.assert_allclose((after-before)/2e-5, plan.sample(t)[1], atol=1e-7)
