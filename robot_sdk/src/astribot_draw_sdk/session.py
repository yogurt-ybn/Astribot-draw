"""One resident thread owns backend creation, feedback, commands and teardown."""
from concurrent.futures import Future
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import queue
import threading
import time
import uuid

import numpy as np

from .backends import MockBackend, SDKBackend
from .contracts import ExecutionPolicy, Trajectory


class SessionError(RuntimeError):
    pass


class ExecutionHandle:
    def __init__(self):
        self.id = uuid.uuid4().hex
        self._future = Future()
        self._cancel = threading.Event()

    def cancel(self):
        """Request software cancellation at the next loop tick; not a physical E-stop."""
        self._cancel.set()

    def wait(self, timeout=None):
        return self._future.result(timeout=timeout)

    @property
    def done(self):
        return self._future.done()


class RobotSession:
    def __init__(self, backend='mock', *, peers=None, transport='link', policy=None, log_dir=None,
                 backend_factory=None, startup_timeout_s=10.):
        if backend not in ('mock', 'sdk-sim', 'sdk-hardware'):
            raise ValueError('backend must be mock, sdk-sim or sdk-hardware')
        if not np.isfinite(startup_timeout_s) or startup_timeout_s <= 0:
            raise ValueError('startup timeout must be finite and positive')
        if transport not in ('ros', 'link') or (backend == 'sdk-hardware' and transport != 'link'):
            raise ValueError('invalid transport for requested backend')
        if backend != 'mock' and transport == 'link' and not peers:
            raise ValueError('SDK backends require an explicit Link endpoint')
        self.backend_kind = backend
        self.peers = peers
        self.transport = transport
        self.policy = policy or ExecutionPolicy()
        self.log_dir = Path(log_dir) if log_dir else None
        self.factory = backend_factory
        self.startup_timeout_s = startup_timeout_s
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._requests = queue.Queue(maxsize=8)
        self._thread = None
        self._cached = None
        self._cached_auxiliary = {}
        self._lifecycle = 'new'
        self._error = None
        self._active_id = None

    def open(self):
        with self._lock:
            if self._lifecycle != 'new':
                raise SessionError('sessions are single-use; create another instance to reconnect')
            self._lifecycle = 'connecting'
            self._thread = threading.Thread(target=self._loop, name='astribot-draw-owner', daemon=True)
            self._thread.start()
        if not self._ready.wait(self.startup_timeout_s):
            self._stop.set()
            raise SessionError('backend startup timed out; shutdown requested, no motion was requested')
        with self._lock:
            if self._error:
                raise SessionError(self._error)
        return self

    def state(self):
        with self._lock:
            state = self._cached
            return dict(backend=self.backend_kind, lifecycle=self._lifecycle,
                        transport=self.transport if self.backend_kind != 'mock' else 'offline',
                        execution_id=self._active_id, error=self._error,
                        auxiliary=deepcopy(self._cached_auxiliary),
                        joints=state.to_dict() if state is not None else None)

    def execute(self, trajectory, *, state_guard=None):
        if not isinstance(trajectory, Trajectory):
            raise TypeError('execute requires a validated Trajectory object')
        handle = ExecutionHandle()
        with self._lock:
            if self._lifecycle not in ('ready', 'executing') or self._stop.is_set():
                raise SessionError('session is not ready')
            try:
                self._requests.put_nowait((trajectory, handle, state_guard))
            except queue.Full as exc:
                raise SessionError('execution request queue is full') from exc
        return handle

    def close(self, timeout=5.):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout)
            if self._thread.is_alive():
                raise SessionError('owner thread is still inside backend code; shutdown not confirmed')

    def __enter__(self):
        return self.open()

    def __exit__(self, *_):
        self.close()

    def _fresh(self, state):
        age = time.perf_counter()-state.received_at
        if not 0 <= age <= self.policy.max_feedback_age_s:
            raise SessionError(f'stale/invalid feedback receipt age: {age:.3f}s')

    def _begin(self, backend, trajectory, handle, state, state_guard=None):
        if self.backend_kind != 'mock':
            expected_scope = 'simulation' if self.backend_kind == 'sdk-sim' else 'hardware'
            if trajectory.scope != expected_scope:
                raise SessionError(f'trajectory scope must explicitly permit {expected_scope} execution')
            if self.backend_kind == 'sdk-hardware' and not trajectory.plan_sha256:
                raise SessionError('hardware execution requires a hash-bound validation artifact')
            if self.backend_kind == 'sdk-hardware' and not self.policy.allow_hardware_motion:
                raise SessionError('hardware motion is disabled by policy')
        self._fresh(state)
        if state_guard:
            state_guard(state, self._extra_state(backend))
        limits = backend.limits()
        trajectory.validate(self.policy, *limits, state.position)
        log = None
        if self.log_dir:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            log = (self.log_dir/(handle.id+'.jsonl')).open('w', encoding='utf-8')
        job = dict(trajectory=trajectory, handle=handle, index=0, started=time.perf_counter(),
                   due=None, last_target=None, held=False, log=log, commands=0,
                   max_error=0., max_lateness=0., last_lease_check=0., settling_at=None,
                   last_submit_at=None, limits=limits, state_guard=state_guard)
        try:
            job['held'] = True  # Cleanup even a partially denied acquisition.
            backend.acquire()
            state = backend.read_state()
            self._fresh(state)
            if state_guard:
                state_guard(state, self._extra_state(backend))
            trajectory.validate(self.policy, *backend.limits(), state.position)
            job['started'] = job['due'] = time.perf_counter()
            with self._lock:
                self._active_id = handle.id
                self._lifecycle = 'executing'
            self._trace(job, 'start', plan=trajectory.summary(), policy=asdict(self.policy))
            return job
        except BaseException:
            try:
                backend.release()
            finally:
                if log:
                    log.close()
            raise

    @staticmethod
    def _trace(job, event, **details):
        if job['log']:
            job['log'].write(json.dumps(dict(event=event, monotonic_s=time.perf_counter(), **details), allow_nan=False)+'\n')

    def _finish(self, backend, job, status, error=None):
        if job['held']:
            try:
                # On cancellation/fault only hold a fresh measured pose while our lease is valid.
                if status != 'completed':
                    state = backend.read_state()
                    self._fresh(state)
                    if backend.has_control():
                        backend.command(state.position, 1/self.policy.control_hz, velocity=np.zeros(18))
            except Exception as hold_error:
                error = f'{error or status}; hold unavailable: {hold_error}'
            try:
                backend.release()
            except Exception as release_error:
                status, error = 'failed', f'{error or ""}; release failed: {release_error}'
        result = dict(execution_id=job['handle'].id, status=status, error=error,
                      backend=self.backend_kind, commands_sent=job['commands'],
                      transport=self.transport if self.backend_kind != 'mock' else 'offline',
                      elapsed_s=time.perf_counter()-job['started'],
                      max_tracking_error_rad=job['max_error'], max_lateness_s=job['max_lateness'],
                      plan=job['trajectory'].summary(),
                      evidence_level='mock_protocol' if self.backend_kind == 'mock' else self.backend_kind)
        try:
            try:
                self._trace(job, 'result', result=result)
            except Exception as exc:
                result.update(status='failed', error=f'could not write execution report: {exc}')
        finally:
            if job['log']:
                job['log'].close()
            with self._lock:
                self._active_id = None
                self._lifecycle = 'ready'
            job['handle']._future.set_result(result)

    def _tick(self, backend, job, state):
        now = time.perf_counter()
        if job['state_guard']:
            job['state_guard'](state, self._extra_state(backend))
        if (job['last_submit_at'] is not None
                and now-job['last_submit_at'] > self.policy.max_lateness_s+1/self.policy.control_hz):
            raise SessionError('command stream stalled; no catch-up burst')
        if now-job['last_lease_check'] >= .1:
            if not backend.has_control():
                raise SessionError('control lease lost')
            job['last_lease_check'] = now
        if job['last_target'] is not None:
            error = float(np.max(np.abs(state.position-job['last_target'])))
            job['max_error'] = max(job['max_error'], error)
            if error > self.policy.tracking_tolerance_rad:
                raise SessionError(f'tracking error exceeds configured ceiling: {error:.4f} rad')
        lateness = max(0., now-job['due'])
        job['max_lateness'] = max(job['max_lateness'], lateness)
        now = time.perf_counter()
        lateness = max(lateness, now-job['due'])
        job['max_lateness'] = max(job['max_lateness'], lateness)
        if lateness > self.policy.max_lateness_s:
            raise SessionError(f'command deadline missed by {lateness:.4f}s; no catch-up burst')
        plan = job['trajectory']
        elapsed = min(now-job['started'], float(plan.time[-1]))
        target, target_velocity, phase = plan.sample(elapsed)
        if elapsed >= plan.time[-1]:
            target_velocity = np.zeros(18)
        lower, upper, velocity_limit = job['limits']
        if (np.any(target < lower) or np.any(target > upper)
                or np.any(np.abs(target_velocity) > np.minimum(velocity_limit, self.policy.max_speed_rad_s)+1e-8)):
            raise SessionError('interpolated command exceeds runtime limits')
        if job['last_target'] is not None and np.max(np.abs(target-job['last_target'])) > self.policy.max_step_rad:
            raise SessionError('interpolated command step exceeds configured ceiling')
        backend.command(target, 1/self.policy.control_hz, velocity=target_velocity)
        job['last_submit_at'] = now
        job['last_target'] = target
        job['commands'] += 1
        self._trace(job, 'sample', index=job['index'], trajectory_time_s=elapsed, phase=phase,
                    target=target.tolist(), target_velocity=target_velocity.tolist(),
                    measured=state.position.tolist(), velocity=state.velocity.tolist(),
                    auxiliary=self._extra_state(backend) if job['state_guard'] else {},
                    feedback_sequence=state.sequence, feedback_age_s=now-state.received_at,
                    lateness_s=lateness)
        job['due'] += 1/self.policy.control_hz
        job['index'] += 1
        if elapsed >= plan.time[-1]:
            if job['settling_at'] is None:
                job['settling_at'] = now
                job['final_command_sequence'] = state.sequence
                return False
            if (state.sequence > job['final_command_sequence']
                    and float(np.max(np.abs(state.position-plan.position[-1]))) <= self.policy.final_tolerance_rad
                    and float(np.max(np.abs(state.velocity))) <= self.policy.settle_velocity_rad_s):
                return True
            if now-job['settling_at'] > self.policy.settle_timeout_s:
                raise SessionError('final pose did not settle within the configured timeout')
        return False

    @staticmethod
    def _extra_state(backend):
        return backend.extra_state() if hasattr(backend, 'extra_state') else {}

    def _loop(self):
        backend = job = None
        try:
            backend = self.factory() if self.factory else (
                MockBackend() if self.backend_kind == 'mock' else SDKBackend(
                    kind=self.backend_kind, peers=self.peers, transport=self.transport, control_hz=self.policy.control_hz))
            if backend.kind != self.backend_kind:
                raise SessionError('backend factory identity does not match requested backend')
            state = backend.read_state()
            self._fresh(state)
            with self._lock:
                self._cached, self._lifecycle = state, 'ready'
                self._cached_auxiliary = self._extra_state(backend)
            self._ready.set()
            next_tick = time.perf_counter()
            while not self._stop.is_set():
                state = backend.read_state()
                self._fresh(state)
                with self._lock:
                    self._cached = state
                    self._cached_auxiliary = self._extra_state(backend)
                try:
                    trajectory, handle, state_guard = self._requests.get_nowait()
                except queue.Empty:
                    pass
                else:
                    if job is not None or handle._cancel.is_set():
                        handle._future.set_result(dict(status='rejected', error='busy or cancelled before start', execution_id=handle.id))
                    else:
                        try:
                            job = self._begin(backend, trajectory, handle, state, state_guard)
                            state = backend.read_state()
                            self._fresh(state)
                            next_tick = job['due']
                        except Exception as exc:
                            handle._future.set_result(dict(status='rejected', error=str(exc), execution_id=handle.id))
                if job:
                    if job['handle']._cancel.is_set():
                        self._finish(backend, job, 'cancelled')
                        job = None
                    else:
                        try:
                            if self._tick(backend, job, state):
                                self._finish(backend, job, 'completed')
                                job = None
                        except Exception as exc:
                            self._finish(backend, job, 'aborted', str(exc))
                            job = None
                next_tick += 1/self.policy.control_hz
                delay = next_tick-time.perf_counter()
                if delay < 0:
                    now = time.perf_counter()
                    skipped = int((now-next_tick)*self.policy.control_hz)+1
                    next_tick += skipped/self.policy.control_hz
                    if job:
                        job['due'] = next_tick
                # Event.wait on Windows can have a 15.6 ms timer granularity.
                # Python's high-resolution sleep keeps 125 Hz from becoming
                # alternating long waits and zero-delay command bursts.
                while not self._stop.is_set():
                    remaining = next_tick-time.perf_counter()
                    if remaining <= 0:
                        break
                    time.sleep(min(remaining, .002))
        except BaseException as exc:
            with self._lock:
                self._error = f'{type(exc).__name__}: {exc}'
            if job:
                self._finish(backend, job, 'aborted', self._error)
                job = None
        finally:
            if job:
                self._finish(backend, job, 'cancelled', 'session closed')
            while True:
                try:
                    _, handle, _ = self._requests.get_nowait()
                except queue.Empty:
                    break
                handle._future.set_result(dict(status='rejected', error='session closed', execution_id=handle.id))
            if backend:
                try:
                    backend.close()
                except Exception as exc:
                    with self._lock:
                        self._error = f'backend cleanup failed: {exc}'
            with self._lock:
                self._lifecycle = 'failed' if self._error else 'closed'
            self._ready.set()
