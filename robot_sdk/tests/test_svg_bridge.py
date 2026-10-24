import importlib.util
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

import numpy as np

from astribot_draw_sdk import RobotSession, T1_JOINTS, Trajectory
from astribot_draw_sdk.backends import MOCK_HOME, MockBackend
from astribot_draw_sdk.ros_feedback import AUXILIARY_PARTS
from astribot_draw_sdk.svg_runner import FixedPartsGuard, full_pose, validate_auxiliary, validate_native_contract


def auxiliary():
    return {name: dict(position=[0.]*size, received_at=time.perf_counter())
            for name, size in AUXILIARY_PARTS.items()}


class SvgBoundaryTests(unittest.TestCase):
    def test_native_numerical_validation_rejects_limits_and_nonfinite_targets(self):
        model = SimpleNamespace(jnt_limited=np.ones(18), jnt_qposadr=np.arange(18),
                                jnt_range=np.tile([-1., 1.], (18, 1)))
        q, dq = np.zeros((3, 18)), np.zeros((3, 18))
        phase, desired = np.array(['approach_both', 'draw_both', 'draw_both']), np.zeros((3, 2, 3))
        validate_native_contract(model, q, dq, phase, desired)
        q[1, 0] = 1.1
        with self.assertRaisesRegex(RuntimeError, 'joint limits'):
            validate_native_contract(model, q, dq, phase, desired)
        q[:] = 0
        dq[1, 0] = .21
        with self.assertRaisesRegex(RuntimeError, 'joint limits'):
            validate_native_contract(model, q, dq, phase, desired)
        dq[:] = 0
        desired[-1, 0, 0] = np.nan
        with self.assertRaisesRegex(RuntimeError, 'Nonfinite'):
            validate_native_contract(model, q, dq, phase, desired)
    def test_measured_pose_maps_names_and_preserves_head(self):
        names = list(T1_JOINTS)+['astribot_head_joint_1', 'astribot_head_joint_2',
            'astribot_gripper_left_joint_L1', 'astribot_gripper_right_joint_L1']
        indices = np.random.default_rng(15).permutation(len(names))
        model = SimpleNamespace(qpos0=np.zeros(len(names)), jnt_qposadr=indices,
            joint=lambda name: SimpleNamespace(id=names.index(name)))
        aux = auxiliary()
        aux['astribot_head']['position'] = [.1, .2]
        values = full_pose(model, MOCK_HOME, aux)
        np.testing.assert_allclose(values[indices[:18]], MOCK_HOME)
        np.testing.assert_allclose(values[indices[18:20]], [.1, .2])
        aux['astribot_gripper_left']['position'] = [.1]
        with self.assertRaisesRegex(RuntimeError, 'nearly closed'):
            full_pose(model, MOCK_HOME, aux)

    def test_missing_stale_or_moving_auxiliary_is_rejected(self):
        aux = auxiliary()
        guard = FixedPartsGuard(aux)
        guard(None, aux)
        aux['astribot_head']['position'][0] = .01
        with self.assertRaisesRegex(RuntimeError, 'Uncommanded part'):
            guard(None, aux)
        aux = auxiliary()
        aux['astribot_head']['received_at'] -= 1.
        with self.assertRaisesRegex(RuntimeError, 'fresh complete'):
            validate_auxiliary(aux)
        with self.assertRaisesRegex(RuntimeError, 'fresh complete'):
            validate_auxiliary({})

    def test_execution_guard_stops_motion_and_releases_lease(self):
        class Backend(MockBackend):
            def __init__(self):
                super().__init__()
                self.changed = False
            def extra_state(self):
                result = auxiliary()
                result['astribot_head']['position'][0] = .02 if self.changed else 0.
                return result
            def command(self, *args, **kwargs):
                super().command(*args, **kwargs)
                self.changed = True
        backend = Backend()
        plan = Trajectory(np.arange(126)/125, np.tile(MOCK_HOME, (126, 1)))
        with RobotSession(backend_factory=lambda: backend) as robot:
            result = robot.execute(plan, state_guard=FixedPartsGuard(auxiliary())).wait(3)
        self.assertEqual(result['status'], 'aborted')
        self.assertIn('Uncommanded part', result['error'])
        self.assertEqual(result['commands_sent'], 1)
        self.assertFalse(backend.held)

    def test_vectorized_resampling_matches_scalar_for_irregular_knots(self):
        rng = np.random.default_rng(8)
        plan = Trajectory([0., .01, .019, .07], rng.normal(0., .01, (4, 18)),
                          velocity=np.vstack([rng.normal(0., .01, (3, 18)), np.zeros(18)]))
        times, positions, velocities = plan.control_samples(125)
        for i, t in enumerate(times):
            q, dq, _ = plan.sample(t)
            np.testing.assert_allclose(positions[i], q, atol=1e-12)
            np.testing.assert_allclose(velocities[i], dq, atol=1e-12)


@unittest.skipUnless(importlib.util.find_spec('mujoco'), 'MuJoCo is available in the simulator container')
class CollisionBroadPhaseTests(unittest.TestCase):
    def test_broad_phase_matches_exhaustive_distances_including_plane_and_overlap(self):
        import mujoco
        from astribot_draw_sdk.geometry import CollisionChecker
        xml = '''<mujoco><worldbody>
          <geom name="floor" type="plane" size="2 2 .1"/>
          <geom name="fixed" type="box" size=".1 .1 .1" pos="0 0 .2"/>
          <body pos=".3 0 .2"><joint type="slide" axis="1 0 0" range="-1 1"/>
            <geom name="moving" type="sphere" size=".08"/></body>
          <geom name="far" type="sphere" pos="3 0 1" size=".1"/>
        </worldbody><contact>
          <pair geom1="fixed" geom2="moving"/><pair geom1="floor" geom2="moving"/>
          <pair geom1="far" geom2="moving"/>
        </contact></mujoco>'''
        model = mujoco.MjModel.from_xml_string(xml)
        data, checker = mujoco.MjData(model), CollisionChecker(model)
        for x in np.linspace(-.5, .5, 41):
            data.qpos[0] = x
            mujoco.mj_forward(model, data)
            expected = min(.05, *(mujoco.mj_geomDistance(model, data, int(a), int(b), .05, None)
                                  for a, b in zip(model.pair_geom1, model.pair_geom2)))
            self.assertAlmostEqual(checker.check(data)['distance_m'], expected, places=10)
            fast = mujoco.MjData(model)
            fast.qpos[0] = x
            mujoco.mj_kinematics(model, fast)
            np.testing.assert_allclose(fast.geom_xpos, data.geom_xpos, atol=1e-12)
            np.testing.assert_allclose(fast.geom_xmat, data.geom_xmat, atol=1e-12)
            self.assertAlmostEqual(checker.minimum_distance(fast), expected, places=10)
