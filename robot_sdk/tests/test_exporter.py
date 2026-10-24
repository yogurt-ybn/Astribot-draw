from pathlib import Path
import tempfile
import json
from types import SimpleNamespace
import unittest

import numpy as np

from astribot_draw_sdk import T1_JOINTS, Trajectory, export_model_trajectory


class ExportTests(unittest.TestCase):
    def test_velocity_geometry_callback_receives_paired_full_pose_velocity(self):
        checked, speeds = [], []
        def state(q):
            checked.append(q.copy())
            return .004
        def velocity(dq):
            self.assertEqual(len(checked), len(speeds)+1)
            speeds.append(dq.copy())
        with tempfile.TemporaryDirectory() as tmp:
            path, _ = export_model_trajectory(Path(tmp)/'plan.npz', self.model, self.q,
                self.dq, .005, ['draw']*201, validate_state=state, validate_velocity=velocity)
            plan = Trajectory.from_npz(path)
            np.testing.assert_allclose(np.asarray(speeds)[:, self.indices], plan.velocity)
            np.testing.assert_allclose(np.asarray(checked)[:, self.indices], plan.position)
            other = np.setdiff1d(np.arange(22), self.indices)
            np.testing.assert_array_equal(np.asarray(speeds)[:, other], 0.)
    def test_phase_boundaries_and_final_frame_keep_full_geometry_coverage(self):
        phases = ['approach']*100+['draw']*100+['end']
        checked = []
        with tempfile.TemporaryDirectory() as tmp:
            path, evidence = export_model_trajectory(Path(tmp)/'plan.npz', self.model,
                self.q, self.dq, .005, phases, validate_state=lambda q: checked.append(q.copy()) or .004)
            loaded = Trajectory.from_npz(path)
            self.assertEqual(loaded.phase[0], 'approach')
            self.assertEqual(loaded.phase[63], 'draw')
            self.assertEqual(loaded.phase[-1], 'end')
            self.assertEqual(len(checked), len(loaded.time))
            self.assertTrue(json.loads(evidence.read_text())['collision_gate_passed'])
    def setUp(self):
        self.indices = np.random.default_rng(7).permutation(22)[:18]
        self.model = SimpleNamespace(nq=22, nv=22, jnt_qposadr=self.indices,
                                     joint=lambda name: SimpleNamespace(id=T1_JOINTS.index(name)))
        self.q = np.zeros((201, 22))
        t = np.arange(201)/200
        self.q[:, self.indices[0]] = .01*(3*t*t-2*t**3)
        self.dq = np.zeros_like(self.q)
        self.dq[:, self.indices[0]] = .01*(6*t-6*t*t)

    def test_named_export_resampling_checks_full_poses_and_binds_evidence(self):
        checked = []
        def clearance(q):
            checked.append(q.copy())
            return .004
        with tempfile.TemporaryDirectory() as tmp:
            plan, evidence = export_model_trajectory(Path(tmp)/'sdk-plan.npz', self.model,
                self.q, self.dq, .005, ['draw_both']*201, validate_state=clearance)
            loaded = Trajectory.from_npz(plan)
            self.assertEqual(loaded.scope, 'simulation')
            self.assertEqual(len(checked), 126)
            self.assertEqual(checked[0].shape, (22,))
            self.assertAlmostEqual(loaded.position[-1, 0], .01)
            self.assertTrue(evidence.is_file())
            with plan.open('ab') as f:
                f.write(b'changed')
            with self.assertRaisesRegex(ValueError, 'exact plan'):
                Trajectory.from_npz(plan)

    def test_resampling_geometry_failures_do_not_export_a_command(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'sdk-plan.npz'
            with self.assertRaisesRegex(ValueError, 'rechecking'):
                export_model_trajectory(path, self.model, self.q, self.dq, .005, ['draw']*201)
            with self.assertRaisesRegex(ValueError, 'clearance'):
                export_model_trajectory(path, self.model, self.q, self.dq, .005, ['draw']*201,
                                        validate_state=lambda _: -.001)
            self.assertFalse(path.exists())

    def test_hidden_joint_motion_cannot_be_dropped(self):
        other = next(i for i in range(22) if i not in self.indices)
        self.q[50, other] = .1
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError, 'outside'):
                export_model_trajectory(Path(tmp)/'sdk-plan.npz', self.model, self.q,
                                        self.dq, .005, ['draw']*201, scope='mock_only')
