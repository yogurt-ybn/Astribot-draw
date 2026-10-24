"""Named T1 joint contract; trajectory validation has no transport side effects."""
from dataclasses import dataclass
import hashlib
import io
import json
import math
import time
from pathlib import Path

import numpy as np

T1_PARTS = ('astribot_torso', 'astribot_arm_left', 'astribot_arm_right')
T1_SIZES = (4, 7, 7)
T1_JOINTS = tuple(f'{part}_joint_{i}' for part, size in zip(T1_PARTS, T1_SIZES)
                  for i in range(1, size+1))


def vector(value, name):
    array = np.array(value, dtype=float, copy=True)
    if array.shape != (18,) or not np.isfinite(array).all():
        raise ValueError(f'{name} must contain 18 finite T1 joint values')
    array.setflags(write=False)
    return array


def split_parts(value):
    return [value[:4].tolist(), value[4:11].tolist(), value[11:18].tolist()]


def flatten_parts(groups, name):
    if len(groups) != 3 or any(np.asarray(row).shape != (n,) for row, n in zip(groups, T1_SIZES)):
        raise ValueError(f'{name}: expected torso/left/right groups of 4/7/7')
    return vector(np.concatenate(groups), name)


@dataclass(frozen=True)
class ExecutionPolicy:
    """Application ceilings, not manufacturer ratings; live SDK limits also apply."""
    control_hz: float = 125.
    command_preview_s: float = 0.
    max_speed_rad_s: float = .3
    max_step_rad: float = .03
    start_tolerance_rad: float = .03
    tracking_tolerance_rad: float = .15
    final_tolerance_rad: float = .02
    max_feedback_age_s: float = .25
    max_lateness_s: float = .05
    settle_timeout_s: float = 2.
    settle_velocity_rad_s: float = .05
    allow_hardware_motion: bool = False

    def __post_init__(self):
        for key, value in vars(self).items():
            if key == 'allow_hardware_motion':
                if type(value) is not bool:
                    raise ValueError('allow_hardware_motion must be bool')
            elif key == 'command_preview_s':
                if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= .03:
                    raise ValueError('command_preview_s must be finite and between 0 and 0.03 seconds')
            elif type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if self.control_hz > 250:
            raise ValueError('control_hz must not exceed the supported 250 Hz scheduling ceiling')


@dataclass(frozen=True)
class JointState:
    position: np.ndarray
    velocity: np.ndarray
    received_at: float
    sequence: int

    def __post_init__(self):
        object.__setattr__(self, 'position', vector(self.position, 'feedback position'))
        object.__setattr__(self, 'velocity', vector(self.velocity, 'feedback velocity'))
        if not math.isfinite(self.received_at):
            raise ValueError('invalid feedback receipt time')

    def to_dict(self):
        return dict(position=self.position.tolist(), velocity=self.velocity.tolist(),
                    joint_names=list(T1_JOINTS), received_at=self.received_at,
                    feedback_age_s=max(0., time.perf_counter()-self.received_at),
                    sequence=self.sequence, position_unit='rad', velocity_unit='rad/s')


@dataclass(frozen=True)
class Trajectory:
    time: np.ndarray
    position: np.ndarray
    joint_names: tuple = T1_JOINTS
    phase: tuple = ()
    scope: str = 'mock_only'
    plan_sha256: str = ''
    velocity: np.ndarray = None

    def __post_init__(self):
        t = np.array(self.time, dtype=float, copy=True)
        q = np.array(self.position, dtype=float, copy=True)
        names = tuple(self.joint_names)
        if len(names) != 18 or len(set(names)) != 18 or set(names) != set(T1_JOINTS):
            raise ValueError('trajectory must name exactly the 18 T1 torso/left/right joints')
        if t.ndim != 1 or len(t) < 2 or q.shape != (len(t), 18):
            raise ValueError('trajectory requires time[N] and position[N,18], N >= 2')
        if not np.isfinite(q).all() or not np.isfinite(t).all() or abs(t[0]) > 1e-9 or np.any(np.diff(t) <= 0):
            raise ValueError('trajectory must be finite with strictly increasing time starting at zero')
        if self.scope not in ('mock_only', 'simulation', 'hardware'):
            raise ValueError('unknown execution scope')
        phases = tuple(self.phase) or ('trajectory',)*len(t)
        if len(phases) != len(t):
            raise ValueError('phase length must match time')
        q = q[:, [names.index(name) for name in T1_JOINTS]]
        if self.velocity is None:
            v = np.gradient(q, t, axis=0)
            v[[0, -1]] = 0.
        else:
            v = np.array(self.velocity, dtype=float, copy=True)
            if v.shape != q.shape or not np.isfinite(v).all():
                raise ValueError('velocity must contain finite values with shape (N, 18)')
            v = v[:, [names.index(name) for name in T1_JOINTS]]
        if np.max(np.abs(v[-1])) > 1e-6:
            raise ValueError('trajectory must end at zero desired velocity')
        t.setflags(write=False)
        q.setflags(write=False)
        v.setflags(write=False)
        object.__setattr__(self, 'time', t)
        object.__setattr__(self, 'position', q)
        object.__setattr__(self, 'joint_names', T1_JOINTS)
        object.__setattr__(self, 'phase', phases)
        object.__setattr__(self, 'velocity', v)

    @classmethod
    def from_npz(cls, path, *, evidence=None):
        """Load the existing T1 planner format; diagnostics are never executable."""
        path = Path(path)
        raw = path.read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        if evidence is None and path.with_suffix('.validation.json').is_file():
            evidence = path.with_suffix('.validation.json')
        with np.load(io.BytesIO(raw), allow_pickle=False) as plan:
            if ('diagnostic_only' in plan and bool(plan['diagnostic_only'].item())) or path.name == 'ik-candidate.npz':
                raise ValueError('diagnostic IK candidates are not execution plans')
            required = {'time', 'command_position', 'command_velocity', 'command_joint_names', 'part_names', 'part_sizes', 'robot'}
            if not required <= set(plan.files):
                raise ValueError('missing T1 command-plan fields')
            if str(plan['robot'].item()).lower() != 't1':
                raise ValueError('this SDK adapter only supports T1')
            if list(plan['part_names']) != list(T1_PARTS) or list(plan['part_sizes']) != list(T1_SIZES):
                raise ValueError('plan part contract must be torso/left/right 4/7/7')
            scope = str(plan['scope'].item()) if 'scope' in plan else 'mock_only'
            if scope != 'mock_only' and evidence is None:
                raise ValueError('simulation/hardware command files require a hash-bound validation report')
            if evidence is not None:
                report = json.loads(Path(evidence).read_text(encoding='utf-8'))
                if (report.get('schema') not in ('astribot-draw-validation/v1', 'astribot-draw-approval/v1')
                        or report.get('plan_sha256') != digest
                        or report.get('robot') != 't1'
                        or (report.get('scope') != 'mock_only' and report.get('collision_gate_passed') is not True)
                        or report.get('scope') not in ('simulation', 'hardware', 'mock_only')):
                    raise ValueError('validation report does not match this exact plan')
                scope = report['scope']
            return cls(plan['time'], plan['command_position'], tuple(plan['command_joint_names']),
                       tuple(plan['phase']) if 'phase' in plan else (), scope, digest,
                       velocity=plan['command_velocity'])

    def save_npz(self, path):
        """Write a complete command artifact, including analytical desired dq."""
        path = Path(path)
        if path.suffix.lower() != '.npz':
            raise ValueError('command artifact path must end in .npz')
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('wb') as output:
            np.savez_compressed(output, schema='astribot-draw-command/v1', robot='t1',
                                time=self.time, command_position=self.position,
                                command_velocity=self.velocity, command_joint_names=T1_JOINTS,
                                part_names=T1_PARTS, part_sizes=T1_SIZES, phase=self.phase,
                                scope=self.scope)
        return path

    def sample(self, elapsed):
        """Cubic Hermite interpolation preserves paired q/dq on one time axis."""
        t = float(np.clip(elapsed, 0., self.time[-1]))
        i = min(max(0, int(np.searchsorted(self.time, t, side='right')-1)), len(self.time)-2)
        h = self.time[i+1]-self.time[i]
        u = (t-self.time[i])/h
        p0, p1 = self.position[i], self.position[i+1]
        v0, v1 = self.velocity[i], self.velocity[i+1]
        p = (2*u**3-3*u**2+1)*p0 + (u**3-2*u**2+u)*h*v0 + (-2*u**3+3*u**2)*p1 + (u**3-u**2)*h*v1
        v = ((6*u*u-6*u)*p0 + (3*u*u-4*u+1)*h*v0 + (-6*u*u+6*u)*p1 + (3*u*u-2*u)*h*v1)/h
        return p, v, self.phase[i if t < self.time[-1] else -1]

    def control_samples(self, control_hz):
        t = np.arange(int(np.floor(self.time[-1]*control_hz))+1)/control_hz
        if t[-1] < self.time[-1]-1e-10:
            t = np.r_[t, self.time[-1]]
        i = np.clip(np.searchsorted(self.time, t, side='right')-1, 0, len(self.time)-2)
        h = (self.time[i+1]-self.time[i])[:, None]
        u = (t-self.time[i])[:, None]/h
        p0, p1, v0, v1 = self.position[i], self.position[i+1], self.velocity[i], self.velocity[i+1]
        p = (2*u**3-3*u**2+1)*p0+(u**3-2*u**2+u)*h*v0+(-2*u**3+3*u**2)*p1+(u**3-u**2)*h*v1
        v = ((6*u*u-6*u)*p0+(3*u*u-4*u+1)*h*v0+(-6*u*u+6*u)*p1+(3*u*u-2*u)*h*v1)/h
        return t, p, v

    def validate(self, policy, lower, upper, velocity_limit, current):
        lower, upper = vector(lower, 'lower limits'), vector(upper, 'upper limits')
        velocity_limit = vector(velocity_limit, 'velocity limits')
        if np.any(lower >= upper) or np.any(velocity_limit <= 0):
            raise ValueError('invalid runtime limits')
        if np.any(self.position < lower) or np.any(self.position > upper):
            raise ValueError('trajectory exceeds runtime joint limits; commands will not be clipped')
        t, q, v = self.control_samples(policy.control_hz)
        if np.any(q < lower) or np.any(q > upper):
            raise ValueError('interpolated trajectory exceeds runtime joint limits')
        dt = np.diff(t)
        step = np.abs(np.diff(q, axis=0))
        speed = step/dt[:, None]
        if np.any(step > policy.max_step_rad):
            raise ValueError('trajectory step exceeds configured ceiling')
        if np.any(speed > np.minimum(velocity_limit, policy.max_speed_rad_s)+1e-9):
            raise ValueError('trajectory speed exceeds runtime/configured ceiling')
        if np.any(np.abs(v) > np.minimum(velocity_limit, policy.max_speed_rad_s)+1e-9):
            raise ValueError('desired velocity exceeds runtime/configured ceiling')
        if np.max(np.abs(self.position[0]-current)) > policy.start_tolerance_rad:
            delta = self.position[0]-current
            index = int(np.argmax(np.abs(delta)))
            raise ValueError(f'start pose mismatch: {T1_JOINTS[index]} delta={delta[index]:+.6f} rad, '
                             f'tolerance={policy.start_tolerance_rad:.6f} rad; '
                             'plan a checked approach from measured state')

    def summary(self):
        return dict(robot='t1', samples=len(self.time), duration_s=float(self.time[-1]),
                    scope=self.scope, plan_sha256=self.plan_sha256, joint_names=list(T1_JOINTS))
