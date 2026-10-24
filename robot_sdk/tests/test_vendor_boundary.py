import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np

from astribot_draw_sdk.backends import SDKBackend
from astribot_draw_sdk.contracts import T1_PARTS, split_parts


class VendorBoundaryTests(unittest.TestCase):
    def test_command_uses_verified_sdk_signature_and_paired_q_dq(self):
        backend = SDKBackend.__new__(SDKBackend)
        backend.owner = threading.get_ident()
        backend.robot = Mock()
        q, dq = np.arange(18)/100, np.arange(18)/1000
        backend.command(q, .008, velocity=dq)
        backend.robot.set_joints_position.assert_called_once_with(
            names=list(T1_PARTS), position=split_parts(q), velocity=split_parts(dq),
            control_way='direct', use_wbc=False, add_default_torso=False, dt=.008)

    def test_hold_explicitly_clears_desired_velocity(self):
        backend = SDKBackend.__new__(SDKBackend)
        backend.owner = threading.get_ident()
        backend.robot = Mock()
        backend.command(np.zeros(18), .008)
        self.assertEqual(backend.robot.set_joints_position.call_args.kwargs['velocity'],
                         [[0.]*4, [0.]*7, [0.]*7])

    def test_off_thread_vendor_calls_are_rejected(self):
        backend = SDKBackend.__new__(SDKBackend)
        backend.owner = -1
        backend.robot = Mock()
        with self.assertRaisesRegex(RuntimeError, 'owning thread'):
            backend.command(np.zeros(18), .008)
        backend.robot.assert_not_called()

    def test_simulation_requires_explicit_local_endpoint_before_importing_vendor(self):
        with patch('astribot_draw_sdk.backends.import_module') as importer:
            with self.assertRaisesRegex(ValueError, 'local simulator'):
                SDKBackend(kind='sdk-sim', peers='tcp/192.168.0.11:7447', control_hz=125)
            importer.assert_not_called()

    def test_unverified_sdk_version_fails_before_vendor_import(self):
        with patch('astribot_draw_sdk.backends.version', return_value='9.0.0'), \
                patch('astribot_draw_sdk.backends.import_module') as importer:
            with self.assertRaisesRegex(RuntimeError, 'has not been verified'):
                SDKBackend(kind='sdk-sim', transport='ros')
            importer.assert_not_called()
