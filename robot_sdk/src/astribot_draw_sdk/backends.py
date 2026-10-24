"""Vendor compatibility is isolated here; mock never imports the vendor runtime."""
from importlib import import_module
from importlib.metadata import version
import math
import inspect
import os
import threading
import time
from typing import Protocol

import numpy as np

from .contracts import JointState, T1_PARTS, T1_SIZES, flatten_parts, split_parts, vector

MOCK_HOME = np.array([.137015, -.607721, .734286, 0.,
                     .473063, -.672368, -1.770881, 2.126753, -.115575, -.433513, .095747,
                     -.473063, -.672368, 1.770881, 2.126753, .115575, -.433513, -.095747])


class RobotBackend(Protocol):
    kind: str
    def read_state(self) -> JointState: ...
    def limits(self) -> tuple: ...
    def acquire(self) -> None: ...
    def has_control(self) -> bool: ...
    def command(self, position: np.ndarray, dt: float, velocity=None) -> None: ...
    def release(self) -> None: ...
    def close(self) -> None: ...


class MockBackend:
    """First-order joint follower, not a collision or rigid-body simulator."""
    kind = 'mock'

    def __init__(self, initial=None, time_constant_s=.025):
        if not math.isfinite(time_constant_s) or time_constant_s <= 0:
            raise ValueError('mock time constant must be positive')
        self.position = vector(MOCK_HOME if initial is None else initial, 'mock initial').copy()
        self.target = self.position.copy()
        self.time_constant = time_constant_s
        self.updated_at = time.perf_counter()
        self.sequence = 0
        self.held = False
        self.closed = False

    def read_state(self):
        now = time.perf_counter()
        dt = max(now-self.updated_at, 1e-9)
        before = self.position.copy()
        self.position += (self.target-self.position)*(1-math.exp(-dt/self.time_constant))
        self.updated_at = now
        self.sequence += 1
        return JointState(self.position, (self.position-before)/dt, now, self.sequence)

    def limits(self):
        # Deliberately mock-only bounds. Vendor backend always queries actual limits.
        return np.full(18, -3.), np.full(18, 3.), np.full(18, 1.)

    def acquire(self):
        self.held = True

    def has_control(self):
        return self.held and not self.closed

    def command(self, position, dt, velocity=None):
        if not self.has_control():
            raise RuntimeError('mock lease not held')
        self.target = vector(position, 'command').copy()

    def release(self):
        self.target = self.position.copy()
        self.held = False

    def close(self):
        self.release()
        self.closed = True


class SDKBackend:
    """Verified SDK Link/official ROS simulation adapters on one owner thread.

    The private state subscription is version-pinned because public SDK getters
    do not expose receipt age or an atomic multi-part state snapshot.
    """
    def __init__(self, *, kind, peers=None, transport='link', control_hz=125., state_timeout_s=5.):
        if kind not in ('sdk-sim', 'sdk-hardware'):
            raise ValueError('invalid SDK backend kind')
        if transport not in ('ros', 'link') or (transport == 'ros' and kind != 'sdk-sim'):
            raise ValueError('ROS transport is limited to the official SDK simulation')
        if transport == 'link' and (not isinstance(peers, str) or not peers.startswith('tcp/') or any(c in peers for c in ';, \n')):
            raise ValueError('supply one explicit Link endpoint, e.g. tcp/127.0.0.1:7447')
        if transport == 'link' and kind == 'sdk-sim' and peers.rsplit(':', 1)[0] not in ('tcp/127.0.0.1', 'tcp/localhost'):
            raise ValueError('sdk-sim requires a local simulator endpoint; no hardware discovery fallback')
        sdk_version = version('astribot_sdk')
        supported = ('3.4.2', '3.4.8') if transport == 'ros' else ('3.4.2',)
        if sdk_version not in supported:
            raise RuntimeError(f'SDK {sdk_version} has not been verified for {transport}; supported: {supported}')
        self.sdk_version = sdk_version
        self.kind = kind
        self.transport = transport
        self.feedback = None
        self.owner = threading.get_ident()
        self.robot = None
        self.lease_id = None
        self.acquisition_attempted = False
        os.environ['ROBOT_TYPE'] = 'T1'
        os.environ['ASTRIBOT_COMMU_BACKEND'] = transport
        os.environ['ASTRIBOT_ON_SIMULATION_MODE'] = '1' if transport == 'ros' else '0'
        if transport == 'link':
            os.environ['ASTRIBOT_LINK_PEERS'] = peers
        # Link must load its paired native runtime before SDK loads its bundled SO.
        import_module('astribot_link')
        Astribot = import_module('astribot_sdk').Astribot
        read_only_init = ({'auto_acquire_control': False}
                          if 'auto_acquire_control' in inspect.signature(Astribot).parameters else {})

        class ReadOnlyOnConnect(Astribot):
            def __init__(client):
                client._draw_initializing = True
                client._draw_received_at = None
                client._draw_sequence = 0
                try:
                    super().__init__(freq=control_hz, node_name='astribot_draw_sdk',
                                     remote_wbc=transport != 'ros', **read_only_init)
                except BaseException:
                    try:
                        client.shutdown()
                    except Exception:
                        pass
                    raise
                finally:
                    client._draw_initializing = False

            def acquire_control(client, *args, **kwargs):
                if client._draw_initializing:
                    return {'granted': False, 'lease_id': None}
                return super().acquire_control(*args, **kwargs)

            def _setup_state_subscription(client):
                if transport == 'ros':
                    if client._proxy is not None:
                        raise RuntimeError('simulation ROS must use the local SDK runtime')
                    return
                if client._proxy is None or client._link_backend is None:
                    raise RuntimeError('SDK Link state transport required; refusing a local/hardware fallback')

                def receive(snapshot):
                    with client._state_cache_lock:
                        client._state_cache = snapshot
                        client._draw_received_at = time.perf_counter()
                        client._draw_sequence += 1
                        client._state_cache_event.set()

                client._link_backend.subscribe('stream_robot_state', receive)

        try:
            # Check the real ROS stream before SDK orchestration, which can
            # otherwise block inside native initialization when the viewer stalls.
            if transport == 'ros':
                from .ros_feedback import RosSimulationFeedback
                self.feedback = RosSimulationFeedback()
                deadline = time.perf_counter()+state_timeout_s
                while self.feedback.read() is None:
                    if time.perf_counter() > deadline:
                        raise RuntimeError('no complete ROS simulation joint feedback; check domain/environment')
                    time.sleep(.01)
            self.robot = ReadOnlyOnConnect()
            if 'velocity' not in inspect.signature(self.robot.set_joints_position).parameters:
                raise RuntimeError('installed SDK lacks verified paired position/velocity support')
            if transport == 'link' and not self.robot._state_cache_event.wait(state_timeout_s):
                raise RuntimeError('no fresh stream_robot_state from simulator/driver')
            if list(self.robot.get_dof(names=list(T1_PARTS))) != list(T1_SIZES):
                raise RuntimeError('connected robot does not expose the expected T1 4/7/7 DOFs')
            self.read_state()
        except BaseException:
            self.close()
            raise

    def _check_owner(self):
        if threading.get_ident() != self.owner:
            raise RuntimeError('SDK calls must remain on the owning thread')

    def read_state(self):
        self._check_owner()
        if self.feedback is not None:
            state = self.feedback.read()
            if state is None:
                raise RuntimeError('ROS simulation feedback unavailable')
            return state
        if not self.robot.is_robot_alive():
            raise RuntimeError('robot driver unavailable')
        with self.robot._state_cache_lock:
            snap = self.robot._state_cache
            stamp, seq = self.robot._draw_received_at, self.robot._draw_sequence
            if not isinstance(snap, dict) or stamp is None:
                raise RuntimeError('state cache unavailable')
            names = snap['names']
            if len(names) != len(set(names)) or not set(T1_PARTS) <= set(names):
                raise RuntimeError('incomplete or duplicate body parts in feedback')
            indices = [names.index(name) for name in T1_PARTS]
            position = flatten_parts([snap['cur_jpos'][i] for i in indices], 'feedback position')
            velocity = flatten_parts([snap['cur_jvel'][i] for i in indices], 'feedback velocity')
            return JointState(position, velocity, stamp, seq)

    def limits(self):
        self._check_owner()
        lower, upper = self.robot.get_joints_position_limit(names=list(T1_PARTS))
        velocity = self.robot.get_joints_velocity_limit(names=list(T1_PARTS))
        return (flatten_parts(lower, 'lower limits'), flatten_parts(upper, 'upper limits'),
                flatten_parts(velocity, 'velocity limits'))

    def extra_state(self):
        self._check_owner()
        return self.feedback.extra_state() if self.feedback is not None else {}

    def acquire(self):
        self._check_owner()
        status = self.robot.get_control_rights_status().get('server_state')
        if self.transport == 'link' and (not isinstance(status, dict) or any(not isinstance(status.get(p), dict) for p in T1_PARTS)):
            raise RuntimeError('control-rights server state incomplete')
        if self.transport == 'link' and any(status[p].get('writer') for p in T1_PARTS):
            raise RuntimeError('another controller owns requested parts; release it before execution')
        self.acquisition_attempted = True
        grant = self.robot.acquire_control(objects=list(T1_PARTS), priority=3, sub_priority=3,
                                          reason='T1 drawing trajectory session')
        if not isinstance(grant, dict) or grant.get('granted') is not True or not grant.get('lease_id'):
            raise RuntimeError(f'control request denied: {grant}')
        self.lease_id = grant['lease_id']
        if not self.has_control():
            raise RuntimeError('server did not confirm control of all requested parts')

    def has_control(self):
        self._check_owner()
        if self.transport == 'ros':
            # Local SDK simulation leases do not provide cross-process arbitration.
            lease = self.robot.get_control_rights_status().get('leases', {}).get(self.lease_id, {})
            return bool(self.lease_id and set(T1_PARTS) <= set(lease.get('objects', [])))
        status = self.robot.get_control_rights_status().get('server_state', {})
        return bool(self.lease_id and isinstance(status, dict) and all(
            isinstance(status.get(p), dict) and status[p].get('lease_id') == self.lease_id for p in T1_PARTS))

    def command(self, position, dt, velocity=None):
        self._check_owner()
        velocity = np.zeros(18) if velocity is None else vector(velocity, 'desired velocity')
        self.robot.set_joints_position(names=list(T1_PARTS), position=split_parts(position),
                                       control_way='direct', use_wbc=False,
                                       add_default_torso=False, dt=dt,
                                       velocity=split_parts(velocity))

    def release(self):
        self._check_owner()
        if self.robot is not None and self.acquisition_attempted:
            if self.robot.release_control(objects=list(T1_PARTS)) is not True:
                raise RuntimeError('SDK did not confirm release of requested parts')
            self.acquisition_attempted = False
            self.lease_id = None

    def close(self):
        self._check_owner()
        try:
            if self.feedback is not None:
                self.feedback.close()
        finally:
            self.feedback = None
            if self.robot is not None:
                try:
                    self.release()
                finally:
                    self.robot.shutdown()
                    self.robot = None
