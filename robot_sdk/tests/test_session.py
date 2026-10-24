from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import numpy as np

from astribot_draw_sdk import ExecutionPolicy, RobotSession, Trajectory
from astribot_draw_sdk.backends import MOCK_HOME, MockBackend
from astribot_draw_sdk.session import SessionError


class RecordingBackend(MockBackend):
    def __init__(self, fault=None):
        super().__init__()
        self.calls = [('create', threading.get_ident())]
        self.sent = []
        self.fault = fault

    def read_state(self):
        self.calls.append(('read', threading.get_ident()))
        state = super().read_state()
        if self.sent and self.fault == 'stale':
            return replace(state, received_at=time.perf_counter()-1)
        if self.sent and self.fault == 'tracking':
            return replace(state, position=state.position+.5)
        return state

    def acquire(self):
        self.calls.append(('acquire', threading.get_ident()))
        super().acquire()

    def has_control(self):
        if self.sent and self.fault == 'lease':
            return False
        return super().has_control()

    def command(self, position, dt, velocity=None):
        self.calls.append(('command', threading.get_ident()))
        super().command(position, dt, velocity=velocity)
        self.sent.append(position.copy())
        if self.fault == 'deadline' and len(self.sent) == 1:
            time.sleep(.08)

    def release(self):
        self.calls.append(('release', threading.get_ident()))
        super().release()

    def close(self):
        self.calls.append(('close', threading.get_ident()))
        super().close()


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.backends = []

    def factory(self, fault=None, kind='mock'):
        def make():
            backend = RecordingBackend(fault)
            backend.kind = kind
            self.backends.append(backend)
            return backend
        return make

    def trajectory(self, n=12, **kwargs):
        return Trajectory(np.arange(n)/50, np.tile(MOCK_HOME, (n, 1)), **kwargs)

    def session(self, **kwargs):
        return RobotSession(policy=ExecutionPolicy(control_hz=50), backend_factory=self.factory(), **kwargs)

    def test_read_only_connect_and_all_calls_on_owner_thread(self):
        with self.session() as session:
            self.assertEqual(session.state()['lifecycle'], 'ready')
            self.assertFalse(any(name == 'acquire' for name, _ in self.backends[0].calls))
            result = session.execute(self.trajectory()).wait(3)
            self.assertEqual(result['status'], 'completed')
            self.assertGreaterEqual(result['commands_sent'], 13)  # A newer feedback frame confirms the final command.
        calls = self.backends[0].calls
        self.assertEqual(len({thread for _, thread in calls}), 1)
        self.assertNotEqual(calls[0][1], threading.get_ident())
        self.assertTrue(self.backends[0].closed)

    def test_invalid_start_rejected_before_acquiring_control(self):
        with self.session() as session:
            plan = Trajectory(np.arange(5)/50, np.tile(MOCK_HOME+.1, (5, 1)))
            result = session.execute(plan).wait(2)
            self.assertEqual(result['status'], 'rejected')
            self.assertFalse(any(name == 'acquire' for name, _ in self.backends[0].calls))
            self.assertEqual(self.backends[0].sent, [])

    def test_cancel_and_busy_request(self):
        with self.session() as session:
            first = session.execute(self.trajectory(n=100))
            second = session.execute(self.trajectory())
            self.assertEqual(second.wait(2)['status'], 'rejected')
            first.cancel()
            self.assertEqual(first.wait(2)['status'], 'cancelled')
            self.assertFalse(self.backends[0].held)

    def test_feedback_lease_tracking_and_deadline_faults_release_control(self):
        for fault in ('stale', 'lease', 'tracking', 'deadline'):
            with self.subTest(fault=fault):
                with RobotSession(policy=ExecutionPolicy(control_hz=50), backend_factory=self.factory(fault)) as session:
                    report = session.execute(self.trajectory(n=25)).wait(3)
                    self.assertEqual(report['status'], 'aborted')
                    self.assertTrue(report['error'])
                    self.assertFalse(self.backends[-1].held)

    def test_mock_scope_never_executes_on_sdk_backend(self):
        with RobotSession('sdk-sim', peers='tcp/127.0.0.1:7447',
                          policy=ExecutionPolicy(control_hz=50), backend_factory=self.factory(kind='sdk-sim')) as session:
            report = session.execute(self.trajectory()).wait(2)
            self.assertEqual(report['status'], 'rejected')
            self.assertIn('scope', report['error'])
            self.assertFalse(self.backends[0].held)

    def test_hardware_motion_requires_explicit_policy(self):
        with RobotSession('sdk-hardware', peers='tcp/192.0.2.1:7447',
                          policy=ExecutionPolicy(control_hz=50), backend_factory=self.factory(kind='sdk-hardware')) as session:
            report = session.execute(self.trajectory(scope='hardware', plan_sha256='a'*64)).wait(2)
            self.assertEqual(report['status'], 'rejected')
            self.assertIn('disabled', report['error'])
            self.assertFalse(self.backends[0].held)

    def test_trace_contains_command_feedback_and_terminal_result(self):
        with tempfile.TemporaryDirectory() as root:
            with self.session(log_dir=root) as session:
                report = session.execute(self.trajectory()).wait(3)
            rows = [json.loads(line) for line in (Path(root)/(report['execution_id']+'.jsonl')).read_text().splitlines()]
            samples = [row for row in rows if row['event'] == 'sample']
            self.assertEqual(len(samples), report['commands_sent'])
            self.assertEqual(len(samples[0]['target']), 18)
            self.assertEqual(len(samples[0]['measured']), 18)
            self.assertEqual(len(samples[0]['target_velocity']), 18)
            self.assertEqual(rows[-1]['result']['status'], 'completed')

    def test_mock_does_not_import_vendor_sdk_or_link(self):
        code = "import sys; from astribot_draw_sdk import RobotSession; s=RobotSession().open(); s.close(); assert 'astribot_sdk' not in sys.modules and 'astribot_link' not in sys.modules"
        subprocess.run([sys.executable, '-c', code], check=True, timeout=10)

    def test_startup_error_and_single_use_lifecycle(self):
        def fail():
            raise RuntimeError('backend unavailable')
        session = RobotSession(backend_factory=fail)
        with self.assertRaisesRegex(SessionError, 'unavailable'):
            session.open()
        session.close()
        with self.assertRaises(SessionError):
            session.open()
