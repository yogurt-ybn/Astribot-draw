import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from astribot_draw_sdk.backends import SDKBackend
from astribot_draw_sdk.contracts import T1_JOINTS, T1_PARTS
from astribot_draw_sdk.ros_feedback import RosSimulationFeedback


class RosFeedbackTests(unittest.TestCase):
    def reader(self):
        reader = RosSimulationFeedback.__new__(RosSimulationFeedback)
        reader.executor = Mock()
        reader.parts, reader.error, reader.sequence = {}, None, 0
        return reader

    def message(self, part, frame='simulation'):
        names = [n for n in T1_JOINTS if n.startswith(part+'_joint_')]
        return SimpleNamespace(header=SimpleNamespace(frame_id=frame), name=names[::-1],
                               position=list(range(len(names)))[::-1], velocity=[0.]*len(names))

    def test_partial_feedback_waits_and_named_mapping_preserves_receipt_age(self):
        reader = self.reader()
        reader.receive(T1_PARTS[0], self.message(T1_PARTS[0]))
        self.assertIsNone(reader.read())
        first_stamp = reader.parts[T1_PARTS[0]][2]
        for part in T1_PARTS[1:]:
            reader.receive(part, self.message(part))
        state = reader.read()
        self.assertEqual(state.position.tolist(), list(range(4))+list(range(7))*2)
        self.assertEqual(state.received_at, first_stamp)
        self.assertEqual(reader.read().sequence, state.sequence)  # Reads do not fake fresh frames.

    def test_real_robot_frame_and_bad_names_rejected(self):
        for bad in ('real', 'bad_names'):
            reader = self.reader()
            msg = self.message(T1_PARTS[0], frame='real' if bad == 'real' else 'simulation')
            if bad == 'bad_names':
                msg.name = ['unknown']*4
            reader.receive(T1_PARTS[0], msg)
            with self.assertRaises(RuntimeError):
                reader.read()

    def test_official_hal_empty_name_uses_t1_part_order_and_checks_width(self):
        reader = self.reader()
        for part in T1_PARTS:
            msg = self.message(part)
            msg.name = []
            msg.position.reverse()
            reader.receive(part, msg)
        self.assertEqual(reader.read().position.tolist(), list(range(4))+list(range(7))*2)
        msg.position = [0.]
        reader.receive(T1_PARTS[-1], msg)
        with self.assertRaisesRegex(RuntimeError, 'complete T1'):
            reader.read()

    def test_local_simulation_lease_checks_all_parts(self):
        backend = SDKBackend.__new__(SDKBackend)
        backend.owner, backend.transport, backend.lease_id = threading.get_ident(), 'ros', 'local-1'
        backend.robot = Mock()
        backend.robot.get_control_rights_status.return_value = {'leases': {'local-1': {'objects': list(T1_PARTS)}}}
        self.assertTrue(backend.has_control())
        backend.robot.get_control_rights_status.return_value = {'leases': {'local-1': {'objects': [T1_PARTS[0]]}}}
        self.assertFalse(backend.has_control())
