"""[ADDED] Pure mocked register-verification tests; no ROS graph, no motor commands."""
from concurrent.futures import Future
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from hardware.torque_evidence import TorqueEvidence
from hardware.xr_contract import ARM_DXL_IDS, motor_read_health


class FakeClient:
    def __init__(self):
        self.responsive = True
        self.states = {i: 1 for side in ARM_DXL_IDS.values() for i in side}
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append((request.id, request.item_name))
        f = Future()
        if self.responsive:
            f.set_result(SimpleNamespace(result=True, item_data=self.states[request.id]))
        return f


class FakeNode:
    def __init__(self):
        self.client = FakeClient()
        self.warnings = []

    def create_client(self, _type, name):
        assert name == '/ffw_follower/get_dxl_data'
        return self.client

    def get_logger(self):
        return SimpleNamespace(warn=self.warnings.append)


class TorqueEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.clock = 100.
        self.patch = patch('hardware.torque_evidence.time.monotonic',
                           side_effect=lambda: self.clock)
        self.patch.start()
        self.node = FakeNode()
        self.evidence = TorqueEvidence(self.node)

    def tearDown(self):
        self.patch.stop()

    def step(self, n=1, spacing=.06):
        for _ in range(n):
            self.clock += spacing
            self.evidence.tick()

    def test_all_fourteen_positive_reads_required(self):
        self.assertFalse(self.evidence.verified())
        self.step(30)
        self.assertTrue(self.evidence.verified())
        self.assertEqual(set(i for i, _ in self.node.client.requests),
                         set(self.evidence.ids))
        self.assertEqual({name for _, name in self.node.client.requests},
                         {'Torque Enable'})

    def test_one_motor_off_refuses_proof(self):
        self.node.client.states[34] = 0
        self.step(40)
        self.assertFalse(self.evidence.verified())

    def test_stale_proof_expires_without_new_service_data(self):
        self.step(30)
        self.assertTrue(self.evidence.verified())
        self.node.client.responsive = False
        self.clock += 7.
        self.assertFalse(self.evidence.verified())

    def test_bus_check_does_not_depend_on_faulty_torque_bit(self):
        ids = ARM_DXL_IDS['right'] + ARM_DXL_IDS['left']
        self.assertTrue(motor_read_health(ids, [0]*14, 0))
        self.assertFalse(motor_read_health(ids, [0]*14, 1))
        self.assertFalse(motor_read_health(ids[:-1], [0]*13, 0))
        fault = [0]*14
        fault[3] = 1
        self.assertFalse(motor_read_health(ids, fault, 0))


if __name__ == '__main__':
    unittest.main()
