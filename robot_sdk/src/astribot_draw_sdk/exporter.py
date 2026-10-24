"""Planning boundary: named q/dq export, separate from the vendor runtime."""
import hashlib
import json
from pathlib import Path

import numpy as np

from .contracts import T1_JOINTS, Trajectory


def export_model_trajectory(path, model, position, velocity, timestep, phase, *,
                            final_home=None, control_hz=125., validate_state=None,
                            scope='simulation', provenance=None, validate_velocity=None):
    """Convert planner qpos-space q/dq to a named, resampled T1 command file.

    ``velocity`` is the planner's analytical dq, never measured qvel. The
    optional final_home appends the checked terminal pose with zero velocity.
    validate_state(full_qpos) returns a minimum clearance in meters for each
    resampled pose; it must cover the same collision pairs as the planner.
    Optional validate_velocity(full_qvel) runs immediately after validate_state
    at the paired pose, for example to check tool-tip Jacobian velocity limits.
    """
    if not np.isfinite(timestep) or timestep <= 0 or not np.isfinite(control_hz) or control_hz <= 0:
        raise ValueError('timestep and control_hz must be finite and positive')
    q = np.asarray(position, dtype=float)
    dq = np.asarray(velocity, dtype=float)
    if q.ndim != 2 or q.shape[1] != model.nq or dq.shape != q.shape or len(q) < 2:
        raise ValueError('expected planner position and analytical velocity arrays (N, model.nq)')
    if model.nq != model.nv or not np.isfinite(q).all() or not np.isfinite(dq).all():
        raise ValueError('only finite fixed-base hinge-joint plans are supported')
    phases = tuple(phase)
    if len(phases) != len(q):
        raise ValueError('phase length must match planner samples')
    ids = [model.joint(name).id for name in T1_JOINTS]
    indices = np.asarray(model.jnt_qposadr)[ids]
    other = np.setdiff1d(np.arange(model.nq), indices)
    if not np.allclose(q[:, other], q[0, other], atol=1e-9, rtol=0):
        raise ValueError('plan moves joints outside the 18-joint drawing contract')
    if final_home is not None:
        home = np.asarray(final_home, dtype=float)
        if home.shape != (model.nq,) or not np.isfinite(home).all():
            raise ValueError('final_home must be a finite model.nq pose')
        if not np.allclose(home[other], q[0, other], atol=1e-9, rtol=0):
            raise ValueError('final_home changes non-commanded joints')
        q, dq = np.vstack([q, home]), np.vstack([dq, np.zeros(model.nq)])
        phases += ('hold',)
    native = Trajectory(np.arange(len(q))*timestep, q[:, indices], phase=phases,
                        scope=scope, velocity=dq[:, indices])
    times, positions, velocities = native.control_samples(control_hz)
    phase_indices = np.clip(np.searchsorted(native.time, times, side='right')-1, 0, len(native.time)-1)
    phases = tuple(np.asarray(native.phase)[phase_indices])
    minimum = None
    if scope != 'mock_only' and validate_state is None:
        raise ValueError('simulation/hardware export requires rechecking resampled full poses')
    if validate_state is not None:
        for command, command_velocity in zip(positions, velocities):
            full = q[0].copy()
            full[indices] = command
            distance = validate_state(full)
            if isinstance(distance, (bool, np.bool_)) or not np.isfinite(distance) or distance < .002:
                raise ValueError('resampled state fails the 2 mm collision-clearance gate')
            minimum = float(distance) if minimum is None else min(minimum, float(distance))
            if validate_velocity is not None:
                full_velocity = np.zeros(model.nv)
                full_velocity[indices] = command_velocity
                validate_velocity(full_velocity)
    elif validate_velocity is not None:
        raise ValueError('velocity geometry validation requires validate_state at the same pose')
    plan = Trajectory(times, positions, phase=phases, scope=scope, velocity=velocities)
    path = plan.save_npz(path)
    report = {'schema': 'astribot-draw-validation/v1', 'robot': 't1', 'scope': scope,
              'plan_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
              'collision_gate_passed': minimum is not None and minimum >= .002,
              'minimum_resampled_clearance_m': minimum, 'sample_count': len(times),
              'control_hz': control_hz, 'source_timestep_s': timestep,
              'provenance': provenance or {},
              'scope_note': 'Sampled position geometry only; SDK simulation tracking is validated separately'}
    report_path = path.with_suffix('.validation.json')
    report_path.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    return path, report_path
