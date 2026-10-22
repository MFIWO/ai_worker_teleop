"""[ADDED] Actual rclpy node, no Vuer server, synthetic feedback guard tests.

Run only with ROS_DOMAIN_ID=232 in robotis-applications Docker. Numerical
fixtures replace feedback for these unit tests; this is not a live test.
"""
import os
import time
import unittest

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped

from robotis_vuer.vr_publisher_sg2_xr import XRDeltaPublisher


class XRPublisherSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '232':
            raise unittest.SkipTest('Safety fixtures require isolated test domain 232')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = XRDeltaPublisher(start_server=False)

    def tearDown(self):
        self.node._set_arm_control_enabled(False, 'unit fixture cleanup')
        self.node.destroy_node()

    def test_active_XA_does_not_call_capture(self):
        self.node.arm_control_enabled = True
        def forbidden_capture():
            self.fail('ACTIVE start must not capture commanded FK')
        self.node._capture = forbidden_capture
        self.node._set_arm_control_enabled(True, 'repeated X+A')
        self.assertTrue(self.node.arm_control_enabled)

    def test_stop_clears_old_controller_feedback(self):
        self.node._robot_poses = {'left': (np.eye(4), time.monotonic(), 1.0)}
        self.node._set_arm_control_enabled(False, 'Y+B fixture')
        self.assertEqual(self.node._robot_poses, {})
        self.assertIsNone(self.node._mapper)

    def test_stale_message_header_is_rejected(self):
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.pose.orientation.w = 1.0
        self.node._robot_pose(msg, 'left')
        self.assertNotIn('left', self.node._robot_poses)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.node._robot_pose(msg, 'left')
        self.assertIn('left', self.node._robot_poses)

    def test_fresh_by_age_but_before_stop_is_rejected(self):
        now = time.monotonic()
        self.node._stopped_at = now - .30
        self.node._robot_poses = {s: (np.eye(4), now - .31, self.node._stopped_ros - .01)
                                 for s in ('left', 'right')}
        self.node._graph_check = lambda: None
        self.node._arm_tracking_is_fresh = lambda now=None: True
        self.node._snapshot = lambda: {s: {'wrist': np.eye(4)} for s in ('left', 'right')}
        with self.assertRaisesRegex(ValueError, 'missing inactive Cyclo'):
            self.node._capture()

    def test_delayed_pre_stop_message_is_not_a_measured_anchor(self):
        now = time.monotonic()
        now_ros = self.node.get_clock().now().nanoseconds * 1e-9
        self.node._stopped_at = now - .30
        self.node._stopped_ros = now_ros - .30
        # Received after the stop and numerically recent, but generated before
        # the stop while the controller was still ACTIVE/commanded-FK.
        self.node._robot_poses = {s: (np.eye(4), now - .10, now_ros - .31)
                                 for s in ('left', 'right')}
        self.node._graph_check = lambda: None
        self.node._arm_tracking_is_fresh = lambda now=None: True
        self.node._snapshot = lambda: {s: {'wrist': np.eye(4)} for s in ('left', 'right')}
        with self.assertRaisesRegex(ValueError, 'missing inactive Cyclo'):
            self.node._capture()


if __name__ == '__main__':
    unittest.main()
