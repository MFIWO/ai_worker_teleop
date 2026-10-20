"""Hardware boot-epoch feedback tests on isolated domain232; no actuator I/O."""
import os
import time
import unittest
import rclpy
from sensor_msgs.msg import JointState
from dynamixel_interfaces.msg import DynamixelState
from hardware.capture_snapshot import Snapshot
from hardware.ready_pose_precheck import ReadyPosePrecheck
from hardware.xr_contract import ARM_NAMES, ARM_DXL_IDS


class ReceiptTimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '232':
            raise unittest.SkipTest('isolated domain232 only')
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def motor(self):
        msg=DynamixelState()
        msg.header.stamp.sec=10
        msg.id=ARM_DXL_IDS['left']+ARM_DXL_IDS['right']
        msg.comm_state=0;msg.dxl_hw_state=[0]*14;msg.torque_state=[True]*14
        return msg

    def test_precheck_accepts_boot_epoch_and_expires_by_receipt(self):
        node=ReadyPosePrecheck('driver_managed')
        try:
            msg=self.motor()
            node.motor(msg)
            self.assertTrue(node.bus_ok)
            first=node.motor_rx
            node.motor(msg)
            self.assertEqual(node.motor_rx,first)
            node.count_publishers=lambda topic:1
            node.good_since=time.monotonic()-1.
            self.assertTrue(node.ready())
            node.motor_rx=time.monotonic()-1.
            self.assertFalse(node.ready())
        finally:
            node.destroy_node()

    def test_snapshot_accepts_remote_epoch_without_refreshing_duplicates(self):
        node=Snapshot('sg2',readiness_policy='driver_managed')
        try:
            names=[k for group in ARM_NAMES.values() for k in group]+['lift_joint']
            msg=JointState(name=names,position=[0.]*len(names))
            msg.header.stamp.sec=10
            node.joints(msg)
            self.assertIsNotNone(node.state)
            first=node.js_rx
            node.joints(msg)
            self.assertEqual(node.js_rx,first)
            node.motor(self.motor())
            self.assertTrue(node.motor_snapshot_ok)
        finally:
            node.destroy_node()


if __name__=='__main__':
    unittest.main()
