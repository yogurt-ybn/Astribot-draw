"""Explicit backend selection; every command defaults to the offline mock."""
import argparse
from contextlib import redirect_stdout
import json
from pathlib import Path
import sys
import time

import numpy as np

from .contracts import ExecutionPolicy, Trajectory
from .session import RobotSession


def demo_trajectory(position, control_hz=125., duration_s=12., scope='mock_only'):
    """Two visible 10-degree elbow excursions with a one-second peak hold."""
    if not np.isfinite(duration_s) or duration_s <= 0 or not np.isfinite(control_hz) or control_hz <= 0:
        raise ValueError('demo duration and control rate must be finite and positive')
    t = np.arange(round(duration_s*control_hz)+1)/control_hz
    if len(t) < 2:
        raise ValueError('demo duration must include at least two control samples')
    q = np.tile(position, (len(t), 1))
    cycle = t[-1]/2
    u = np.mod(t, cycle)/cycle
    rising, falling = u < 1/3, (u >= 1/2) & (u < 5/6)
    peak = (u >= 1/3) & (u < 1/2)
    amplitude = np.deg2rad(10.)
    offset, speed = np.zeros(len(t)), np.zeros(len(t))
    offset[peak] = amplitude
    for mask, s, direction in ((rising, 3*u[rising], 1),
                                (falling, 3*(u[falling]-.5), -1)):
        smooth = 10*s**3-15*s**4+6*s**5
        offset[mask] = amplitude*(smooth if direction == 1 else 1-smooth)
        speed[mask] = direction*amplitude*(30*s**2-60*s**3+30*s**4)*3/cycle
    q[:, 7] += offset
    q[:, 14] += offset
    v = np.zeros_like(q)
    v[:, 7] = v[:, 14] = speed
    phase = np.full(len(t), 'demo_rest', dtype='<U16')
    phase[rising], phase[peak], phase[falling] = 'demo_lift', 'demo_hold', 'demo_return'
    return Trajectory(t, q, phase=tuple(phase), scope=scope, velocity=v)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['probe', 'demo', 'run', 'draw'])
    parser.add_argument('plan', nargs='?', type=Path, help='SVG for draw; command NPZ for legacy run')
    parser.add_argument('--backend', choices=['mock', 'sdk-sim', 'sdk-hardware'], default='mock')
    parser.add_argument('--peers', help='Explicit Link endpoint; sdk-sim requires tcp/127.0.0.1:PORT')
    parser.add_argument('--transport', choices=['ros', 'link'], default='link',
                        help='ros uses the official astribot_simulation local SDK runtime')
    parser.add_argument('--validation', '--approval', dest='validation', type=Path,
                        help='Optional hash-bound validation report for the exact plan')
    parser.add_argument('--control-hz', type=float, default=125.)
    parser.add_argument('--output', type=Path, default=Path('output/robot_sdk'))
    parser.add_argument('--plan-only', action='store_true', help='Plan SVG from current simulation state without motion')
    parser.add_argument('--seeds', type=int, default=12)
    parser.add_argument('--speed-scale', type=float, default=1.)
    parser.add_argument('--planner-dir', type=Path)
    parser.add_argument('--assets', type=Path)
    args = parser.parse_args()
    if args.command == 'demo' and args.backend == 'sdk-hardware':
        parser.error('demo is limited to mock or the explicitly configured simulation service')
    if args.command == 'run' and args.plan is None:
        parser.error('run requires a T1 plan.npz')
    if args.command == 'run' and args.backend == 'sdk-hardware':
        parser.error('CLI hardware motion is disabled; explicit integration/review is required in the Python caller')
    if args.command == 'draw' and (args.plan is None or args.backend != 'sdk-sim' or args.transport != 'ros'):
        parser.error('draw requires an SVG and --backend sdk-sim --transport ros')
    if args.command == 'draw':
        args.output = args.output/('svg-'+str(time.time_ns()))
    report = dict(status='failed', backend=args.backend)
    session = handle = svg_job = None
    pipeline_started_at = time.perf_counter()
    try:
        # Validate file identity before opening any SDK connection.
        trajectory = Trajectory.from_npz(args.plan, evidence=args.validation) if args.command == 'run' else None
        with redirect_stdout(sys.stderr):
            session = RobotSession(args.backend, peers=args.peers, transport=args.transport,
                                   policy=ExecutionPolicy(control_hz=args.control_hz), log_dir=args.output)
            session.open()
            if args.command == 'draw':
                from .svg_runner import SvgJob
                svg_job = SvgJob(session, args.plan, args.output, planner_dir=args.planner_dir,
                                 assets=args.assets, seeds=args.seeds, speed_scale=args.speed_scale,
                                 plan_only=args.plan_only, pipeline_started_at=pipeline_started_at)
                report = svg_job.run()
            elif args.command == 'probe':
                first = session.state()['joints']['sequence']
                time.sleep(.2)
                report = session.state()
                if report['joints']['sequence'] <= first:
                    raise RuntimeError('state stream is not refreshing')
                report.update(status='ready', control_requests=0, motion_commands=0)
            else:
                if trajectory is None:
                    trajectory = demo_trajectory(session.state()['joints']['position'], args.control_hz,
                                                 scope='simulation' if args.backend == 'sdk-sim' else 'mock_only')
                    print('Demo: both elbows move 10 degrees and return, twice in 12 seconds; '
                          'one-second hold at each peak.', file=sys.stderr, flush=True)
                handle = session.execute(trajectory)
                report = handle.wait(timeout=float(trajectory.time[-1])+10)
    except KeyboardInterrupt:
        handle = handle or (svg_job.handle if svg_job else None)
        if handle:
            handle.cancel()
            try:
                report = handle.wait(3)
            except Exception as exc:
                report = dict(status='failed', error=f'cancellation not confirmed: {exc}')
        else:
            report = dict(status='cancelled', error='interrupted')
    except Exception as exc:
        report = dict(status='failed', error=f'{type(exc).__name__}: {exc}', backend=args.backend)
    finally:
        if session:
            try:
                with redirect_stdout(sys.stderr):
                    session.close()
                if session.state()['error']:
                    report.update(status='failed', cleanup_error=session.state()['error'])
            except Exception as exc:
                report.update(status='failed', cleanup_error=str(exc))
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output/((report.get('execution_id') or args.command+'-'+str(time.time_ns()))+'.json')
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    print(json.dumps(dict(report=report, report_file=str(report_path)), ensure_ascii=True), flush=True)
    return 0 if report.get('status') in ('ready', 'completed', 'planned') else 1
