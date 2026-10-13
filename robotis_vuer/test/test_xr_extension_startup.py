"""[ADDED] Reproduce real Quest startup drift with real rclpy, synthetic feedback.

No browser, simulator, or joint-command publisher. Run only on test domain232
when no live controller owns that domain. Original fixed-resume tests stay intact.
"""
from copy import deepcopy
import os
import time
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robotis_vuer.vr_publisher_sg2_xr_extension import XRExtensionPublisher
from test_xr_resume import fixture
from xr_resume_smoke import signature


class ExtensionStartupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '232':
            raise unittest.SkipTest('isolated test domain232 required')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = XRExtensionPublisher(start_server=False)
        self.head, self.inputs, self.robots, self.links = fixture()
        # Explicit numerical fixtures; the integration test uses real endpoints.
        self.node._graph_check = lambda: None
        self.node._arm_tracking_is_fresh = lambda now=None: True
        self.node._require_still_robot = lambda now: None
        self.node._snapshot = lambda: deepcopy(self.links)
        now = time.monotonic()
        ros = self.node.get_clock().now().nanoseconds*1e-9
        self.node._stopped_at, self.node._stopped_ros = now-1, ros-1
        self.node._robot_poses = {s: (p.copy(), now-.01, ros-.01) for s,p in self.robots.items()}
        self.node.head_transform_matrix = self.head.copy()
        for s, p in self.inputs.items():
            setattr(self.node, s+'_controller_matrix', p.copy())
        self.node._set_arm_control_enabled(True, 'fixture X+A')
        self.assertEqual(self.node._phase, 'SETTLING')
        self.held = deepcopy(self.node._goals)
        self.sig = signature(self.node._session)

    def tearDown(self):
        self.node._set_arm_control_enabled(False, 'unit cleanup')
        self.node.destroy_node()

    def publish(self):
        self.node._last_pub = None
        self.node._publish_synced_pose_frame_if_ready()

    def warmup(self, completed=False):
        now = time.monotonic()
        self.node._phase = 'WARMUP'
        self.node._arming_until = None
        self.node._enable_at = now - (12.5 if completed else 5.)
        self.node._first_command_rx = {s: now-(8.2 if completed else 1.) for s in self.inputs}
        self.node._command_rx = {s: now for s in self.inputs}

    def assert_held(self):
        self.assertEqual(signature(self.node._session), self.sig)
        for s, p in self.held.items():
            np.testing.assert_array_equal(self.node._goals[s], p)

    def wait_start(self):
        self.warmup(completed=True)
        self.node.left_controller_matrix[2, 3] -= .0251
        self.publish()
        self.assertEqual(self.node._phase, 'WAIT_START')

    def test_observed_20_to_25mm_drift_does_not_cancel_warmup(self):
        self.warmup()
        for distance in (.0202, .0206, .0209, .0251, .080):
            self.node.left_controller_matrix = self.inputs['left'].copy()
            self.node.left_controller_matrix[2, 3] -= distance
            self.publish()
            self.assertEqual(self.node._phase, 'WARMUP')
            self.assertTrue(self.node.arm_control_enabled)
            self.assert_held()

    def test_unmoved_hands_keep_existing_automatic_ready_path(self):
        self.warmup(completed=True)
        self.publish()
        self.assertEqual(self.node._phase, 'ACTIVE')
        self.assert_held()

    def test_drift_latches_manual_confirmation_even_after_hand_returns(self):
        self.warmup()
        self.node.left_controller_matrix[2, 3] -= .025
        self.publish()
        self.node.left_controller_matrix = self.inputs['left'].copy()
        self.warmup(completed=True)
        self.publish()
        self.assertEqual(self.node._phase, 'WAIT_START')
        self.assertFalse(self.node.arm_start_armed)
        self.assert_held()

    def test_wrong_hand_confirmation_holds_without_recapture(self):
        self.wait_start()
        self.node._capture = lambda: self.fail('must not recapture during confirmation')
        for _ in range(3):
            self.node._set_arm_control_enabled(True, 'wrong X+A')
            self.assertEqual(self.node._phase, 'WAIT_START')
            self.assertTrue(self.node.arm_control_enabled)
            self.assert_held()

    def test_aligned_confirmation_no_reactivate_or_new_reference(self):
        self.wait_start()
        self.node.left_controller_matrix = self.inputs['left'].copy()
        self.node._capture = lambda: self.fail('must not recapture')
        self.node._publish_reactivate = lambda *a, **k: self.fail('must not toggle reactivate')
        self.node._set_arm_control_enabled(True, 'fresh aligned X+A')
        self.assertEqual(self.node._phase, 'ACTIVE')
        self.assert_held()
        # Restore before tearDown, which legitimately sends False.
        del self.node._publish_reactivate

    def test_returning_hands_alone_never_starts_motion(self):
        self.wait_start()
        self.node.left_controller_matrix = self.inputs['left'].copy()
        self.publish()
        self.assertEqual(self.node._phase, 'WAIT_START')
        self.assert_held()

    def test_measured_motion_still_stops_before_any_publication(self):
        self.warmup()
        self.links['left']['wrist'][0, 3] += .021
        count = self.node.published_pairs
        self.publish()
        self.assertFalse(self.node.arm_control_enabled)
        self.assertEqual(self.node.published_pairs, count)
        self.assertIn('measured pose moved', self.node._last_reason)

    def test_measured_reset_while_wait_start_stops(self):
        self.wait_start()
        self.links['right']['wrist'][0, 3] += .1
        count = self.node.published_pairs
        self.publish()
        self.assertFalse(self.node.arm_control_enabled)
        self.assertEqual(self.node.published_pairs, count)

    def test_clutch_limit_is_not_relaxed_in_warmup(self):
        self.warmup()
        self.node.left_controller_matrix[2, 3] -= .501
        count = self.node.published_pairs
        self.publish()
        self.assertFalse(self.node.arm_control_enabled)
        self.assertEqual(self.node.published_pairs, count)
        self.assertIn('0.5m clutch', self.node._last_reason)

    def test_stale_command_feedback_still_stops_wait_start(self):
        self.wait_start()
        self.node._command_rx['right'] = time.monotonic() - .6
        self.node.left_controller_matrix = self.inputs['left'].copy()
        self.node._set_arm_control_enabled(True, 'X+A but stale controller')
        self.assertFalse(self.node.arm_control_enabled)
        self.assertIn('stale', self.node._last_reason)

    def test_fault_latch_cannot_be_cleared_by_wait_start_XA(self):
        self.wait_start()
        self.node._reference_fault(Bool(data=True))
        self.node._set_arm_control_enabled(True, 'retry after fault')
        self.assertFalse(self.node.arm_control_enabled)
        self.assertTrue(self.node._controller_fault_latched)
        self.assertEqual(signature(self.node._session), self.sig)

    def test_YB_stops_held_controller(self):
        self.wait_start()
        self.node._set_arm_control_enabled(False, 'Y+B buttons')
        self.assertFalse(self.node.arm_control_enabled)
        self.assertEqual(self.node._goals, {})
        self.assertEqual(signature(self.node._session), self.sig)

    def test_bilateral_callback_pending_stage_is_preserved(self):
        self.warmup()
        self.node._enable_at = 100.
        self.node._first_command_rx.clear()
        for s, short in (('left','l'),('right','r')):
            msg = JointTrajectory()
            msg.joint_names = [f'arm_{short}_joint{i}' for i in range(1,8)]
            p = JointTrajectoryPoint()
            p.positions = [0.,0.,0.,-1.57,0.,0.,0.]
            msg.points = [p]
            with patch('robotis_vuer.vr_publisher_sg2_xr_resume.time.monotonic', return_value=102.):
                self.node._command_feedback(msg, s)
            self.assertNotIn(s, self.node._first_command_rx)
            with patch('robotis_vuer.vr_publisher_sg2_xr_resume.time.monotonic', return_value=103.2):
                self.node._command_feedback(msg, s)
            self.assertEqual(self.node._first_command_rx[s], 103.2)

    def test_tracking_loss_during_hold_stops(self):
        self.wait_start()
        self.node._arm_tracking_is_fresh = lambda now=None: False
        count = self.node.published_pairs
        self.publish()
        self.assertFalse(self.node.arm_control_enabled)
        self.assertEqual(self.node.published_pairs, count)

    def test_held_XA_does_not_confirm_until_release_and_new_press(self):
        self.wait_start()
        self.node.left_controller_matrix = self.inputs['left'].copy()
        self.node.both_a_buttons_pressed_prev = True
        self.node.left_controller_state = {'aButton': True, 'bButton': False}
        self.node.right_controller_state = {'aButton': True, 'bButton': False}
        self.node._update_arm_activation()
        self.assertEqual(self.node._phase, 'WAIT_START')
        self.node.left_controller_state['aButton'] = False
        self.node.right_controller_state['aButton'] = False
        self.node._update_arm_activation()
        self.node.left_controller_state['aButton'] = True
        self.node.right_controller_state['aButton'] = True
        self.node._update_arm_activation()
        self.assertEqual(self.node._phase, 'ACTIVE')
        self.assert_held()


if __name__ == '__main__':
    unittest.main()
