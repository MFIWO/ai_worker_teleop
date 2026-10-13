"""[ADDED] Fixed-reference restarts; real rclpy with synthetic feedback only.

Domain232, no simulator/controller/browser required. No joint commands are
published. Dependencies are provided by existing robotis-applications Docker.
"""
from copy import deepcopy
import os
import time
import unittest

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from unittest.mock import patch

from robotis_vuer.xr_resume_session import FixedResumeSession
from robotis_vuer.vr_publisher_sg2_xr_resume import XRResumePublisher


def fixture():
    head = np.eye(4)
    head[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    head[:3, 3] = [0, 1.6, 0]
    inputs, robots, links = {}, {}, {}
    for s, sign in (('left', 1), ('right', -1)):
        inputs[s] = np.eye(4)
        inputs[s][:3, 3] = [-sign*.2, 1.1, -.3]
        robots[s] = np.eye(4)
        robots[s][:3, 3] = [.36, sign*.2275, 1.09]
        links[s] = {k: np.eye(4) for k in ('shoulder', 'elbow', 'wrist')}
        links[s]['shoulder'][:3, 3] = [0, sign*.2275, 1.4316]
        links[s]['elbow'][:3, 3] = [.041004, sign*.2275, 1.1316]
        links[s]['wrist'] = robots[s].copy()
    return head, inputs, robots, links


class SessionMathTests(unittest.TestCase):
    def setUp(self):
        self.head, self.inputs, self.robots, self.links = fixture()
        self.session = FixedResumeSession(self.head, self.inputs, self.robots, self.links, 1.)

    def test_same_input_always_same_goal_over_30_resume_hint_seeds(self):
        reference = {s: self.session.mapper.target(s, self.inputs[s]) for s in self.inputs}
        bounds = {s: h.maximum for s, h in self.session.templates.items()}
        for i in range(30):
            measured = deepcopy(self.links)
            measured['left']['elbow'][0, 3] += .03*np.sin(i)
            h = self.session.fresh_hints(measured)
            h['left'].previous += .01
            for s in self.inputs:
                np.testing.assert_array_equal(reference[s], self.session.mapper.target(s, self.inputs[s]))
                self.assertEqual(h[s].maximum, bounds[s])
        np.testing.assert_array_equal(self.session.templates['left'].previous,
                                      self.links['left']['elbow'][:3, 3])

    def test_different_head_after_pause_does_not_change_basis(self):
        original = self.session.mapper.basis.copy()
        self.head[:3, :3] = Rotation.from_euler('y', 50, degrees=True).as_matrix() @ self.head[:3, :3]
        self.session.require_alignment(self.inputs, self.robots)
        np.testing.assert_array_equal(self.session.mapper.basis, original)

    def test_aligned_resume_preserves_mapping(self):
        moved = deepcopy(self.inputs)
        moved['left'][2, 3] -= .04
        actual = {s: self.session.mapper.target(s, moved[s]) for s in moved}
        self.session.require_alignment(moved, actual)
        np.testing.assert_array_equal(self.session.mapper.anchors['left'].input_pose, self.inputs['left'])

    def test_far_hand_is_rejected_even_when_projection_matches_actual(self):
        moved = deepcopy(self.inputs)
        moved['left'][2, 3] -= .4
        actual = deepcopy(self.robots)
        raw = self.session.mapper.target('left', moved['left'])
        actual['left'] = raw.copy()
        actual['left'][:3, 3], _ = self.session.templates['left'].project_wrist(raw[:3, 3])
        with self.assertRaisesRegex(ValueError, 'ALIGN_REQUIRED'):
            self.session.require_alignment(moved, actual)

    def test_base_shift_cannot_silently_reanchor(self):
        moved = deepcopy(self.links)
        moved['left']['shoulder'][0, 3] += .02
        with self.assertRaisesRegex(ValueError, 'shoulder/base moved'):
            self.session.fresh_hints(moved)


class ResumeNodeTests(unittest.TestCase):
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
        self.node = XRResumePublisher(start_server=False)
        self.head, self.inputs, self.robots, self.links = fixture()
        self.node._graph_check = lambda: None
        self.node._arm_tracking_is_fresh = lambda now=None: True
        self.node._require_still_robot = lambda now: None
        self.node._snapshot = lambda: deepcopy(self.links)
        self.prepare_feedback()

    def prepare_feedback(self):
        now = time.monotonic()
        now_ros = self.node.get_clock().now().nanoseconds*1e-9
        self.node._stopped_at, self.node._stopped_ros = now-1, now_ros-1
        self.node._robot_poses = {s: (self.robots[s].copy(), now-.01, now_ros-.01) for s in self.inputs}
        self.node.head_transform_matrix = self.head.copy()
        for s in self.inputs:
            setattr(self.node, s+'_controller_matrix', self.inputs[s].copy())

    def tearDown(self):
        self.node._set_arm_control_enabled(False, 'unit cleanup')
        self.node.destroy_node()

    def start(self):
        self.node._set_arm_control_enabled(True, 'fixture manual start')
        self.assertTrue(self.node.arm_control_enabled)

    def test_stop_clears_transients_but_keeps_reference(self):
        self.start()
        session = self.node._session
        self.node._set_arm_control_enabled(False, 'range fault')
        self.assertIs(self.node._session, session)
        for name in ('_goals', '_elbows', '_hints', '_last_input', '_robot_poses'):
            self.assertEqual(getattr(self.node, name), {})
        self.assertIsNone(self.node._mapper)
        self.assertIsNone(self.node._enable_at)

    def test_resume_wrong_hand_does_not_recalibrate(self):
        self.start()
        session = self.node._session
        anchor = session.mapper.anchors['left'].input_pose.copy()
        self.node._set_arm_control_enabled(False, 'range fault')
        self.inputs['left'][2, 3] -= .1
        self.prepare_feedback()
        self.node._set_arm_control_enabled(True, 'wrong-position X+A')
        self.assertFalse(self.node.arm_control_enabled)
        self.assertIs(self.node._session, session)
        self.assertIn('ALIGN_REQUIRED', self.node._last_reason)
        np.testing.assert_array_equal(anchor, session.mapper.anchors['left'].input_pose)

    def test_aligned_repeated_resume_never_changes_reference(self):
        self.start()
        session = self.node._session
        basis = session.mapper.basis.copy()
        for i in range(4):
            self.node._set_arm_control_enabled(False, 'pause')
            self.prepare_feedback()
            self.node._set_arm_control_enabled(True, 'aligned X+A')
            self.assertTrue(self.node.arm_control_enabled)
            self.assertIs(self.node._session, session)
            np.testing.assert_array_equal(self.node._mapper.basis, basis)

    def test_fresh_inactive_stamp_is_still_required(self):
        self.node._robot_poses['left'] = (self.robots['left'], time.monotonic(), self.node._stopped_ros-.01)
        self.node._set_arm_control_enabled(True, 'delayed active message')
        self.assertFalse(self.node.arm_control_enabled)
        self.assertIsNone(self.node._session)

    def test_controller_reference_fault_latches_across_XA(self):
        self.start()
        session = self.node._session
        self.node._reference_fault(Bool(data=True))
        self.prepare_feedback()
        self.node._set_arm_control_enabled(True, 'retry after reference fault')
        self.assertFalse(self.node.arm_control_enabled)
        self.assertTrue(self.node._controller_fault_latched)
        self.assertIs(self.node._session, session)

    def test_warmup_does_not_integrate_small_human_motion(self):
        self.start()
        goals = deepcopy(self.node._goals)
        self.node.left_controller_matrix[2, 3] -= .005
        self.node._publish_synced_pose_frame_if_ready()
        np.testing.assert_array_equal(self.node._goals['left'], goals['left'])

    def test_warmup_large_motion_stops_without_new_calibration(self):
        self.start()
        session = self.node._session
        self.node.left_controller_matrix[2, 3] -= .10
        self.node._publish_synced_pose_frame_if_ready()
        self.assertFalse(self.node.arm_control_enabled)
        self.assertIs(self.node._session, session)
        self.assertIn('ALIGN_REQUIRED', self.node._last_reason)

    def test_ready_requires_observed_commands_and_ramp_hold(self):
        self.start()
        self.node._phase = 'WARMUP'
        self.node._enable_at = time.monotonic()-5
        self.node._arming_until = None
        self.node._publish_synced_pose_frame_if_ready()
        self.assertEqual(self.node._phase, 'WARMUP')
        # Separate later fixture: first commands occur3.9s after enable,
        # then8.1s of ramp holding. No contradictory activation timestamps.
        self.node._enable_at = time.monotonic()-12
        self.node._first_command_rx = {s: time.monotonic()-8.1 for s in self.inputs}
        self.node._command_rx = {s: time.monotonic() for s in self.inputs}
        self.node._last_pub = None
        self.node._publish_synced_pose_frame_if_ready()
        self.assertEqual(self.node._phase, 'ACTIVE')

    def test_nonzero_failed_frame_never_publishes_partial_goals(self):
        self.start()
        self.node._phase = 'ACTIVE'
        self.node._command_rx = {s: time.monotonic() for s in self.inputs}
        self.node.left_controller_matrix[2, 3] -= .01
        self.node.right_controller_matrix[2, 3] -= .60
        count = self.node.published_pairs
        self.node._publish_synced_pose_frame_if_ready()
        self.assertEqual(self.node.published_pairs, count)
        self.assertFalse(self.node.arm_control_enabled)
        self.assertEqual(self.node._goals, {})

    def test_measured_reset_during_warmup_stops_before_goal_publication(self):
        self.start()
        self.node._phase = 'WARMUP'
        self.node._enable_at = time.monotonic()-12
        self.node._first_command_rx = {s: time.monotonic()-9 for s in self.inputs}
        self.node._command_rx = {s: time.monotonic() for s in self.inputs}
        self.links['left']['wrist'][0, 3] += .10
        count = self.node.published_pairs
        self.node._publish_synced_pose_frame_if_ready()
        self.assertFalse(self.node.arm_control_enabled)
        self.assertEqual(self.node.published_pairs, count)
        self.assertIn('measured pose moved', self.node._last_reason)

    def test_failed_large_seed_never_changes_checker_history_on_retry(self):
        self.start()
        self.node._publish_pair()
        count = self.node.published_pairs
        session = self.node._session
        self.node._set_arm_control_enabled(False, 'pause')
        self.robots['left'][0, 3] += .15
        self.links['left']['wrist'] = self.robots['left'].copy()
        for _ in range(2):
            self.prepare_feedback()
            self.node._set_arm_control_enabled(True, 'large seed retry')
            self.assertFalse(self.node.arm_control_enabled)
            self.assertEqual(self.node.published_pairs, count)
            self.assertIn('SEED_JUMP_REFUSED', self.node._last_reason)
            self.assertIs(self.node._session, session)

    def test_stability_window_rejects_moving_measured_joints(self):
        now = time.monotonic()
        ros = self.node.get_clock().now().nanoseconds*1e-9
        self.node._stopped_at, self.node._stopped_ros = now-1, ros-1
        for dt in (.42, .3, .15, .02):
            self.node._joint_history.append((now-dt, ros-dt, np.ones(14)*dt))
        with self.assertRaisesRegex(ValueError, 'still moving'):
            XRResumePublisher._require_still_robot(self.node, now)
        self.node._joint_history.clear()
        for dt in (.42, .3, .15, .02):
            self.node._joint_history.append((now-dt, ros-dt, np.zeros(14)))
        XRResumePublisher._require_still_robot(self.node, now)

    def test_repeated_source_stamp_is_not_a_stability_window(self):
        now = time.monotonic()
        ros = self.node.get_clock().now().nanoseconds*1e-9
        self.node._stopped_at, self.node._stopped_ros = now-1, ros-1
        for dt in (.42, .3, .15, .02):
            self.node._joint_history.append((now-dt, ros-.45, np.zeros(14)))
        with self.assertRaisesRegex(ValueError, 'post-stop stable'):
            XRResumePublisher._require_still_robot(self.node, now)

    def test_controller_model_tolerance_cannot_hide_measured_misalignment(self):
        self.start()
        session = self.node._session
        self.node._set_arm_control_enabled(False, 'pause')
        # Commanded/measurement model agreement passes its existing4cm check,
        # but raw-vs-PhysX3cm exceeds the tighter2cm resume alignment.
        self.links['left']['wrist'][0, 3] += .03
        self.prepare_feedback()
        self.node._set_arm_control_enabled(True, 'physical mismatch')
        self.assertFalse(self.node.arm_control_enabled)
        self.assertIs(self.node._session, session)
        self.assertIn('ALIGN_REQUIRED', self.node._last_reason)

    def test_reference_fault_before_first_start_also_latches(self):
        self.node._reference_fault(Bool(data=True))
        self.prepare_feedback()
        self.node._set_arm_control_enabled(True, 'attempt after inactive fault')
        self.assertFalse(self.node.arm_control_enabled)
        self.assertIsNone(self.node._session)
        self.assertTrue(self.node._controller_fault_latched)
        self.node._reference_fault(Bool(data=False))
        self.assertTrue(self.node._controller_fault_latched)

    def test_command_callbacks_cannot_skip_three_second_pending_stage(self):
        self.start()
        self.node._phase = 'WARMUP'
        self.node._enable_at = 100.0
        for s, short in (('left', 'l'), ('right', 'r')):
            msg = JointTrajectory()
            msg.joint_names = [f'arm_{short}_joint{i}' for i in range(1,8)]
            point = JointTrajectoryPoint()
            point.positions = [0.]*7
            msg.points = [point]
            with patch('robotis_vuer.vr_publisher_sg2_xr_resume.time.monotonic', return_value=101.0):
                self.node._command_feedback(msg, s)
            self.assertNotIn(s, self.node._first_command_rx)
            with patch('robotis_vuer.vr_publisher_sg2_xr_resume.time.monotonic', return_value=103.2):
                self.node._command_feedback(msg, s)
            self.assertEqual(self.node._first_command_rx[s], 103.2)


if __name__ == '__main__':
    unittest.main()
