"""[ADDED] ROS-message-only unit tests for reduced ready-pose precheck.

No ROS graph, no real motors, no services called. Checks that reducing the
native-driver readiness, strict torque policy, receipt age and bus checks.
"""
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from dynamixel_interfaces.msg import DynamixelState

from hardware.ready_pose_precheck import ReadyPosePrecheck
from hardware.xr_contract import ARM_DXL_IDS


class Fake:
    def __init__(self, readiness_policy='driver_managed'):
        self.readiness_policy = readiness_policy
        self.expected = set(ARM_DXL_IDS['left'] + ARM_DXL_IDS['right'])
        self.bus_ok = False
        self.reported_torque_good = False
        self.motor_rx = self.motor_stamp = float('-inf')
        self.good_since = None
        self.reason = 'waiting'
        self.unverified_since = None
        self.proof = SimpleNamespace(verified=lambda: False, reason='no torque proof')
        self.count_publishers = lambda topic: 1
        self.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000_000))

    def motor(self, msg):
        return ReadyPosePrecheck.motor(self, msg)

    def ready(self):
        return ReadyPosePrecheck.ready(self)


def msg(t=1000.0, torque=True):
    data = DynamixelState()
    data.header.stamp.sec = int(t)
    data.header.stamp.nanosec = round((t-int(t)) * 1e9)
    data.id = ARM_DXL_IDS['right'] + ARM_DXL_IDS['left']
    data.comm_state = 0
    data.dxl_hw_state = [0]*14
    data.torque_state = [torque]*14
    return data


class PrecheckTests(unittest.TestCase):
    def test_fresh_healthy_motor_only(self):
        f = Fake()
        with patch('hardware.ready_pose_precheck.time.monotonic', return_value=1000):
            f.motor(msg())
            self.assertTrue(f.bus_ok)
            self.assertTrue(f.reported_torque_good)
            self.assertFalse(f.ready())  # 0.3 second stable proof still needed
        with patch('hardware.ready_pose_precheck.time.monotonic', return_value=1000.4):
            self.assertTrue(f.ready())

    def test_strict_policy_does_not_accept_buffered_register_proof(self):
        f = Fake('strict_torque')
        with patch('hardware.ready_pose_precheck.time.monotonic', return_value=1000):
            f.motor(msg(torque=False))
            self.assertTrue(f.bus_ok)
            self.assertFalse(f.reported_torque_good)
            self.assertFalse(f.ready())
            f.proof.verified=lambda:True
            self.assertFalse(f.ready())
        with patch('hardware.ready_pose_precheck.time.monotonic', return_value=1000.4):
            self.assertFalse(f.ready())

    def test_driver_managed_policy_accepts_healthy_read_with_unknown_torque(self):
        f = Fake()
        with patch('hardware.ready_pose_precheck.time.monotonic', return_value=1000):
            f.motor(msg(torque=False))
            self.assertFalse(f.ready())
        with patch('hardware.ready_pose_precheck.time.monotonic', return_value=1000.4):
            self.assertTrue(f.ready())

    def test_remote_epoch_is_ignored_but_receipt_expiry_and_replay_are_not(self):
        f = Fake()
        with patch('hardware.ready_pose_precheck.time.monotonic', return_value=1000):
            f.motor(msg(10.0))
            self.assertTrue(f.bus_ok)
            self.assertFalse(f.ready())
        with patch('hardware.ready_pose_precheck.time.monotonic', return_value=1000.4):
            self.assertTrue(f.ready())
            f.motor(msg(10.0))
            self.assertEqual(f.motor_rx, 1000)
        with patch('hardware.ready_pose_precheck.time.monotonic', return_value=1000.6):
            self.assertFalse(f.ready())

    def test_multiple_feedback_publishers_refuse_readiness(self):
        f = Fake()
        f.count_publishers = lambda topic: 2
        with patch('hardware.ready_pose_precheck.time.monotonic', return_value=1000):
            f.motor(msg())
            self.assertFalse(f.ready())
        with patch('hardware.ready_pose_precheck.time.monotonic', return_value=1000.4):
            self.assertFalse(f.ready())

    def test_comm_or_hardware_error_cannot_proceed(self):
        f = Fake()
        d = msg()
        d.comm_state = 1
        f.motor(d)
        self.assertFalse(f.bus_ok)
        f2 = Fake()
        d2 = msg()
        d2.dxl_hw_state[0] = 1
        f2.motor(d2)
        self.assertFalse(f2.bus_ok)


if __name__ == '__main__':
    unittest.main()
