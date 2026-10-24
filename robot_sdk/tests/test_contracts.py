import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from astribot_draw_sdk import ExecutionPolicy, T1_JOINTS, T1_PARTS, Trajectory
from astribot_draw_sdk.backends import MOCK_HOME


class ContractTests(unittest.TestCase):
    def plan(self, **changes):
        values = dict(time=np.arange(5)/125, position=np.tile(MOCK_HOME, (5, 1)))
        values.update(changes)
        return Trajectory(**values)

    def test_named_joint_mapping_and_array_ownership(self):
        positions = np.tile(MOCK_HOME, (5, 1))
        plan = self.plan(position=positions[:, ::-1], joint_names=T1_JOINTS[::-1])
        np.testing.assert_array_equal(plan.position[0], MOCK_HOME)
        positions[:] = 0
        np.testing.assert_array_equal(plan.position[0], MOCK_HOME)
        self.assertFalse(plan.position.flags.writeable)

    def test_bad_shapes_time_names_nan_rejected(self):
        for changes in [dict(time=[0, 0, 1, 2, 3]), dict(position=np.zeros((5, 17))),
                        dict(position=np.full((5, 18), np.nan)), dict(joint_names=('same',)*18)]:
            with self.subTest(changes=list(changes)):
                with self.assertRaises(ValueError):
                    self.plan(**changes)

    def test_cadence_limits_speed_and_start_are_not_clipped(self):
        limits = (np.full(18, -3.), np.full(18, 3.), np.ones(18), MOCK_HOME)
        self.plan().validate(ExecutionPolicy(), *limits)
        q = np.tile(MOCK_HOME, (5, 1))
        q[1, 0] += .01
        with self.assertRaisesRegex(ValueError, 'speed'):
            self.plan(position=q).validate(ExecutionPolicy(), *limits)
        self.plan(time=np.arange(5)/50).validate(ExecutionPolicy(), *limits)
        with self.assertRaisesRegex(ValueError, 'start pose'):
            self.plan(position=np.tile(MOCK_HOME+.04, (5, 1))).validate(ExecutionPolicy(), *limits)
        with self.assertRaisesRegex(ValueError, 'limits'):
            self.plan(position=np.full((5, 18), 4.)).validate(ExecutionPolicy(), *limits)

    def test_planner_npz_requires_exact_hash_bound_approval(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'plan.npz'
            np.savez(path, time=np.arange(5)/125, command_position=np.tile(MOCK_HOME, (5, 1)),
                     command_velocity=np.zeros((5, 18)),
                     command_joint_names=T1_JOINTS, part_names=T1_PARTS, part_sizes=[4, 7, 7], robot='t1')
            self.assertEqual(Trajectory.from_npz(path).scope, 'mock_only')
            approval = dict(schema='astribot-draw-approval/v1', robot='t1',
                            plan_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                            collision_gate_passed=True, scope='simulation')
            report = Path(root)/'approval.json'
            report.write_text(json.dumps(approval))
            self.assertEqual(Trajectory.from_npz(path, evidence=report).scope, 'simulation')
            approval['plan_sha256'] = 'wrong'
            report.write_text(json.dumps(approval))
            with self.assertRaises(ValueError):
                Trajectory.from_npz(path, evidence=report)
            diagnostic = Path(root)/'ik-candidate.npz'
            np.savez(diagnostic, diagnostic_only=True)
            with self.assertRaisesRegex(ValueError, 'diagnostic'):
                Trajectory.from_npz(diagnostic)

    def test_invalid_policy(self):
        for kwargs in ({'control_hz': 0}, {'max_step_rad': float('nan')}, {'allow_hardware_motion': 1}):
            with self.assertRaises(ValueError):
                ExecutionPolicy(**kwargs)

    def test_200hz_plan_interpolates_paired_position_and_velocity_at_125hz(self):
        t = np.arange(201)/200
        q = np.tile(MOCK_HOME, (len(t), 1))
        q[:, 7] += .02*(3*t*t-2*t**3)
        v = np.zeros_like(q)
        v[:, 7] = .02*(6*t-6*t*t)
        plan = Trajectory(t, q, velocity=v)
        times, positions, velocities = plan.control_samples(125)
        self.assertEqual(len(times), 126)
        np.testing.assert_allclose(positions[:, 7]-MOCK_HOME[7], .02*(3*times**2-2*times**3), atol=1e-12)
        np.testing.assert_allclose(velocities[:, 7], .02*(6*times-6*times**2), atol=1e-12)
        self.assertFalse(plan.velocity.flags.writeable)

    def test_complete_export_roundtrip_and_missing_velocity_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'named.npz'
            original = self.plan()
            original.save_npz(path)
            loaded = Trajectory.from_npz(path)
            self.assertEqual(loaded.scope, 'mock_only')
            np.testing.assert_array_equal(loaded.velocity, original.velocity)
            self.plan(scope='simulation').save_npz(path)
            with self.assertRaisesRegex(ValueError, 'validation report'):
                Trajectory.from_npz(path)
            np.savez(path, time=[0., .008], command_position=np.zeros((2, 18)))
            with self.assertRaisesRegex(ValueError, 'missing'):
                Trajectory.from_npz(path)
