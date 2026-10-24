"""Reuse a canvas drawing trajectory with a freshly checked measured-start approach."""
import argparse
from contextlib import redirect_stdout
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np

from .contracts import T1_JOINTS, Trajectory
from .cli import drawing_policy
from .geometry import checked_transfer
from .pattern_catalog import select_pattern
from .session import RobotSession
from .svg_runner import SvgJob, validate_native_contract


def cached_drawing(archived, entry):
    """Discard the archived approach and restore the exact zero-speed draw entry."""
    q0 = np.asarray(entry['position_rad'], dtype=float)
    t0 = float(entry['time_in_archived_plan_s'])
    if q0.shape != (18,) or not np.isfinite(q0).all() or not 0 < t0 < archived.time[-1]:
        raise ValueError('Invalid exact drawing-entry metadata')
    indices = np.flatnonzero(np.asarray(archived.phase) == 'draw_both')
    if len(indices) < 2 or not np.array_equal(indices, np.arange(indices[0], len(archived.time))):
        raise ValueError('Cached trajectory must end in one contiguous drawing phase')
    t, q, v = archived.time[indices]-t0, archived.position[indices], archived.velocity[indices]
    if t[0] < -1e-9 or t[0] > .008+1e-9 or np.max(np.abs(q[0]-q0)) > .01:
        raise ValueError('Drawing entry does not match the archived command trajectory')
    if t[0] > 1e-9:
        t, q, v = np.r_[0., t], np.vstack([q0, q]), np.vstack([np.zeros(18), v])
    else:
        if not np.allclose(q[0], q0, atol=1e-8, rtol=0) or np.max(np.abs(v[0])) > 1e-6:
            raise ValueError('Expected a stationary exact first drawing frame')
        t = t.copy(); t[0] = 0.
    return Trajectory(t, q, velocity=v, phase=('draw_both',)*len(t), scope='simulation')


def nearest_source(point, points):
    """Nearest point on the original flattened source, not on measured feedback."""
    a, b = points[:-1], points[1:]
    delta = b-a
    length2 = np.einsum('ij,ij->i', delta, delta)
    fraction = np.divide(np.einsum('ij,ij->i', point-a, delta), length2,
                         out=np.zeros_like(length2), where=length2 > 0)
    projected = a+np.clip(fraction, 0., 1.)[:, None]*delta
    index = int(np.argmin(np.sum((projected-point)**2, axis=1)))
    return projected[index]


class PatternJob(SvgJob):
    def __init__(self, robot, library, index, output, **kwargs):
        query, record, files = select_pattern(library, index)
        entries = query.get('canvas_drawing_entries', {})
        if record['pattern_id'] not in entries:
            raise ValueError('Library is missing exact canvas drawing-entry metadata')
        entry = entries[record['pattern_id']]
        plan = Trajectory.from_npz(files['sdk-plan.npz'])
        if entry['plan_sha256'] != plan.plan_sha256:
            raise ValueError('Drawing entry and cached plan SHA256 differ')
        self.query, self.record, self.entry = query, record, entry
        self.archived, self.cached = plan, cached_drawing(plan, entry)
        super().__init__(robot, files['source.svg'], output, joint_speed=1., tip_speed=.2,
                         skip_gripper_checks=True, **kwargs)
        self.plan_timestep = 1/125
        if robot.policy.control_hz != 125 or robot.policy.command_preview_s != .02:
            raise ValueError('Cached pattern execution requires 125 Hz and the recorded 20 ms preview')
        qualification = query['qualification']
        if qualification['model_revision'] != self.report['model_revision'] or self.report['model_revision'] == 'unknown':
            raise ValueError('Runtime model revision differs from the validated pattern library')
        for name, digest in query['canvas_calibration_sha256'].items():
            if hashlib.sha256((self.planner_dir/'t1-canvas'/name).read_bytes()).hexdigest() != digest:
                raise ValueError('Canvas calibration/mask changed: '+name)
        validation = json.loads(files['sdk-plan.validation.json'].read_text(encoding='utf-8'))
        if (validation['provenance']['svg_sha256'] != self.report['svg_sha256'] or
                validation['provenance']['model_revision'] != self.report['model_revision'] or
                validation['control_hz'] != 125):
            raise ValueError('Cached command validation is incompatible with this pattern')
        self.report['pattern_cache'] = dict(index=index, pattern_id=record['pattern_id'],
            name=record['name_en'], original_plan_sha256=plan.plan_sha256,
            archived_approach_discarded=True, fresh_approach_from_measured_state=True,
            drawing_ik_recomputed=False, exact_canvas_entry_position_rad=entry['position_rad'],
            archived_drawing_duration_s=float(self.cached.time[-1]),
            canvas='original fixed SVG world calibration; no translation or rescaling')

    def plan_motion(self, parsed):
        m, initial, mj, indices = self.model, self.initial, self.mujoco, self.guard.indices
        self.checkpoint('model_and_state')
        snapshot = self.report['initial_snapshot']
        if np.max(np.abs(snapshot['joints']['velocity'])) > .05:
            raise RuntimeError('Wait for stationary robot feedback before planning cached drawing entry')
        baseline = self.record.get('required_auxiliary_positions') or self.record['cached_trajectory']['recorded_auxiliary_positions']
        for part, tolerance in [('astribot_head', .005), ('astribot_chassis', .001)]:
            if np.max(np.abs(np.array(self.auxiliary[part]['position'])-baseline[part])) > tolerance:
                raise RuntimeError('Cached canvas geometry requires the recorded head/chassis pose: '+part)
        goal = initial.copy(); goal[indices] = self.cached.position[0]
        data = mj.MjData(m); data.qpos[:] = goal; mj.mj_kinematics(m, data)
        for side in ('left', 'right'):
            uv = np.array(parsed['arms'][side]['points_uv'][0])
            expected = np.array([-.77+.0025*uv[0], -.42, 1.89-.0025*uv[1]])
            if np.linalg.norm(data.site(side+'_light').xpos-expected) > .001:
                raise RuntimeError('Cached entry no longer matches the exact canvas SVG start: '+side)
        lo, hi = [], []
        for name in T1_JOINTS:
            jid = m.joint(name).id; margin = .001 if 'torso' in name else .04
            lo.append(m.jnt_range[jid, 0]+margin); hi.append(m.jnt_range[jid, 1]-margin)
        chain = SimpleNamespace(qids=indices, lo=np.array(lo), hi=np.array(hi), side='whole_body_18dof')
        if np.any(initial[indices] < chain.lo-1e-9) or np.any(initial[indices] > chain.hi+1e-9):
            raise RuntimeError('Measured start is outside the approach planning margins')
        print('PATTERN_APPROACH current feedback -> exact canvas drawing start', flush=True)
        route = checked_transfer(m, chain, initial[indices], goal[indices], goal[:4], initial,
                                 [(a,b) for a,b,_ in self.planner.collision_pairs(m)],
                                 np.random.default_rng(12345))
        q, dq = [], []
        jac = np.zeros((3, m.nv)); dt = self.plan_timestep
        for a, b in zip(route[:-1], route[1:]):
            delta = b-a
            if np.max(np.abs(delta)) < 1e-10:
                continue
            duration = max(4., np.max(np.abs(delta))*1.875/self.joint_speed)
            peak = 0.
            for r in np.linspace(0, 1, 65):
                s, ds = 10*r**3-15*r**4+6*r**5, 30*r*r*(1-r)**2
                data.qpos[:] = initial; data.qpos[indices] = a+s*delta
                mj.mj_kinematics(m, data); mj.mj_comPos(m, data)
                velocity = np.zeros(m.nv); velocity[indices] = delta*ds
                for side in ('left', 'right'):
                    mj.mj_jacSite(m, data, jac, None, m.site(side+'_light').id)
                    peak = max(peak, float(np.linalg.norm(jac@velocity)))
            duration = max(duration, 1.02*peak/self.tip_speed)
            count = int(np.ceil(duration/dt)); r = np.arange(count)/count
            s = 10*r**3-15*r**4+6*r**5; ds = (30*r**2-60*r**3+30*r**4)/(count*dt)
            full = np.tile(initial, (count,1)); vel = np.zeros_like(full)
            full[:,indices] = a+s[:,None]*delta; vel[:,indices] = ds[:,None]*delta
            q.extend(full); dq.extend(vel)
        approach_count = len(q)
        count = int(np.ceil(self.cached.time[-1]/dt))+1
        draw_q, draw_v = [], []
        for t in np.arange(count)*dt:
            position, velocity, _ = self.cached.sample(float(t))
            full = initial.copy(); vel = np.zeros(m.nv)
            full[indices], vel[indices] = position, velocity
            draw_q.append(full); draw_v.append(vel)
        q.extend(draw_q); dq.extend(draw_v)
        q, dq = np.array(q), np.array(dq)
        phase = np.array(['approach_both']*approach_count+['draw_both']*count)
        desired = np.full((len(q), 2, 3), np.nan)
        source = {}
        for side in ('left', 'right'):
            uv = np.array(parsed['arms'][side]['points_uv'])
            source[side] = np.c_[-.77+.0025*uv[:,0], np.full(len(uv),-.42), 1.89-.0025*uv[:,1]]
        for i in range(approach_count, len(q)):
            data.qpos[:] = q[i]; mj.mj_kinematics(m, data)
            for j, side in enumerate(('left', 'right')):
                desired[i,j] = nearest_source(data.site(side+'_light').xpos, source[side])
        self.report['target_sampling'] = dict(method='cached coupled joint drawing, checked against original source polylines',
            maximum_source_polyline_deviation_m=max(a['curve_to_polyline_error_bound_px'] for a in parsed['arms'].values())*.0025)
        self.report['pattern_cache'].update(new_approach_duration_s=approach_count*dt,
            new_drawing_duration_s=(count-1)*dt,
            endpoint_hold_rounding_s=(count-1)*dt-float(self.cached.time[-1]),
            shape_reference='nearest source-polyline points at nominal cached command poses; never fitted to measured feedback')
        self.report['native_plan'] = validate_native_contract(m, q, dq, phase, desired, joint_speed=1.)
        self.report['selected_torso_candidate'] = 'cached_coupled_torso_and_arms'
        self.report['stages']['planning'] = 'passed'
        self.checkpoint('cached_drawing_and_new_approach')
        print(f'PATTERN_CACHE drawing reused, fresh approach {approach_count*dt:.2f}s, drawing {(count-1)*dt:.2f}s', flush=True)
        return q, dq, phase, desired


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--index', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plan-only', action='store_true')
    args = parser.parse_args()
    # Verify selected paths/hashes before opening even a read-only SDK session.
    select_pattern(args.library, args.index)
    args.output.mkdir(parents=True, exist_ok=True)
    session = job = None; report = {}; started = time.perf_counter()
    try:
        with redirect_stdout(sys.stderr):
            session = RobotSession('sdk-sim', transport='ros', policy=drawing_policy(125,1), log_dir=args.output)
            session.open()
            job = PatternJob(session, args.library, args.index, args.output,
                             plan_only=args.plan_only, pipeline_started_at=started)
            report = job.run()
    except BaseException as exc:
        if job and job.handle:
            job.handle.cancel()
            try: job.handle.wait(3)
            except Exception: pass
        report = dict(status='cancelled' if isinstance(exc,KeyboardInterrupt) else 'failed', error=f'{type(exc).__name__}: {exc}')
    finally:
        if session:
            try:
                with redirect_stdout(sys.stderr): session.close()
                if session.state()['error']: report.update(status='failed',cleanup_error=session.state()['error'])
            except Exception as exc: report.update(status='failed',cleanup_error=str(exc))
    summary = {k:report[k] for k in ('status','error','execution','pattern_cache','timings_s','measured_drawing') if k in report}
    (args.output/'pattern-result.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary), flush=True)
    return 0 if report.get('status') in ('planned','completed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
