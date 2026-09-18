"""Test neck-frame geometry and real callback routing without hardware commands."""

import asyncio
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, Mock, patch

from builtin_interfaces.msg import Time
import numpy as np
from robotis_vuer.arm_tracking_guard import ArmTrackingGuard
from robotis_vuer.gravity_arm_frame import GravityArmFrame
from robotis_vuer.vr_publisher_hand_neck import NeckVRTrajectoryPublisher
from robotis_vuer.vr_publisher_sh5 import BODY_JOINT_KEYS, VRTrajectoryPublisher
from scipy.spatial.transform import Rotation as R


B = VRTrajectoryPublisher.BODY_HEAD_TO_ROS_POSITION
V = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])


def head_pose(yaw=0., pitch=0., roll=0.):
    head = np.eye(4)
    head[:3, :3] = (R.from_euler('y', yaw, degrees=True).as_matrix()
                    @ R.from_euler('x', pitch, degrees=True).as_matrix()
                    @ V.T @ B @ R.from_euler('y', roll, degrees=True).as_matrix())
    head[:3, 3] = [1., 1.3, -2.]
    return head


def original_relative(head, position, orientation):
    basis = B @ head[:3, :3].T
    return (basis @ (position - head[:3, 3]),
            R.from_matrix(basis @ orientation).as_quat())


class GravityFrameTests(unittest.TestCase):
    def test_upright_matches_original(self):
        frame = GravityArmFrame(B)
        self.assertTrue(frame.update(head_pose()))
        np.testing.assert_allclose(frame.correction, np.eye(3), atol=1e-12)

    def test_forward_up_and_wrist_orientation_independent_of_head_tilt(self):
        for yaw in (-130., 0., 75.):
            world_basis = R.from_euler('y', yaw, degrees=True).as_matrix() @ V.T
            for pitch, roll in ((-65., 0.), (-40., 25.), (35., -30.), (0., 0.)):
                head = head_pose(yaw, pitch, roll)
                frame = GravityArmFrame(B)
                self.assertTrue(frame.update(head))
                wrist_rotation = R.from_euler('xyz', [15., -20., 30.], degrees=True)
                for offset in ([.4, 0., 0.], [0., 0., .3], [.2, -.25, -.1]):
                    position = head[:3, 3] + world_basis @ offset
                    raw = original_relative(head, position,
                                            world_basis @ wrist_rotation.as_matrix())
                    transformed, quat = frame.transform(*raw)
                    np.testing.assert_allclose(transformed, offset, atol=1e-12)
                    np.testing.assert_allclose(R.from_quat(quat).as_matrix(),
                                               wrist_rotation.as_matrix(), atol=1e-12)

    def test_head_translation_does_not_change_relative_pose(self):
        frame = GravityArmFrame(B)
        head = head_pose(pitch=-45.)
        world_hand = np.array([1.2, 1.1, -2.4])
        results = []
        for shift in (np.zeros(3), np.array([4., -2., 1.])):
            shifted = head.copy()
            shifted[:3, 3] += shift
            frame.update(shifted)
            results.append(frame.transform(*original_relative(
                shifted, world_hand + shift, V.T)))
        np.testing.assert_allclose(results[0][0], results[1][0], atol=1e-12)
        np.testing.assert_allclose(results[0][1], results[1][1], atol=1e-12)

    def test_vertical_head_needs_prior_heading_and_then_retains_it(self):
        frame = GravityArmFrame(B)
        self.assertFalse(frame.update(head_pose(pitch=-90.)))
        self.assertTrue(frame.update(head_pose(yaw=35., pitch=-45.)))
        previous = frame.reference[:3, :3].copy()
        self.assertTrue(frame.update(head_pose(yaw=35., pitch=-90.)))
        np.testing.assert_allclose(frame.reference[:3, :3], previous, atol=1e-12)

    def test_invalid_head_cannot_reuse_previous_correction(self):
        frame = GravityArmFrame(B)
        for invalid in (np.zeros((4, 4)), np.full((4, 4), np.nan), np.eye(3)):
            frame.update(head_pose())
            self.assertFalse(frame.update(invalid))
            with self.assertRaises(RuntimeError):
                frame.transform([0., 0., 0.], [0., 0., 0., 1.])


class NeckCallbackTests(unittest.TestCase):
    def setUp(self):
        self.node = NeckVRTrajectoryPublisher.__new__(NeckVRTrajectoryPublisher)
        self.node.arm_frame = GravityArmFrame(B)
        self.node.tracking_guard = ArmTrackingGuard()
        self.node.vr_publishing_enabled = False

    def test_all_arm_roles_use_same_correction_and_preserve_original_options(self):
        head = head_pose(pitch=-45., roll=20.)
        self.node.arm_frame.update(head)
        raw = original_relative(head, head[:3, 3] + V.T @ [.4, .1, -.2], V.T)
        with patch.object(VRTrajectoryPublisher, 'publish_relative_pose') as publish:
            for role in ('wrist', 'elbow', 'shoulder'):
                for side in ('left', 'right'):
                    self.node.publish_relative_pose(
                        *raw, None, vr_scale=1.4, z_offset=-.25,
                        apply_right_z_flip=side == 'right', pose_role=role, side=side)
                    args, kwargs = publish.call_args
                    np.testing.assert_allclose(args[0], [.4, .1, -.2], atol=1e-12)
                    np.testing.assert_allclose(R.from_quat(args[1]).as_matrix(),
                                               np.eye(3), atol=1e-12)
                    self.assertEqual(kwargs['apply_right_z_flip'], side == 'right')
                    self.assertEqual(kwargs['z_offset'], -.25)
                    self.assertEqual(kwargs['vr_scale'], 1.4)

    def test_missing_reference_sends_no_arm_pose(self):
        with patch.object(VRTrajectoryPublisher, 'publish_relative_pose') as publish:
            self.node.publish_relative_pose([0., 0., 0.], [0., 0., 0., 1.], None)
            publish.assert_not_called()

    def test_body_callback_updates_frame_before_original_processing(self):
        matrices = np.tile(np.eye(4).flatten(order='F'), len(BODY_JOINT_KEYS))
        start = 16 * BODY_JOINT_KEYS.index('head')
        matrices[start:start + 16] = head_pose(pitch=-45.).flatten(order='F')
        event = SimpleNamespace(value={'body': matrices})
        with patch.object(VRTrajectoryPublisher, 'on_body_tracking_move',
                          new_callable=AsyncMock) as original:
            asyncio.run(self.node.on_body_tracking_move(event, None))
            self.assertIsNotNone(self.node.arm_frame.correction)
            original.assert_awaited_once_with(event, None)

    def test_invalid_head_does_not_forward_body_event(self):
        self.node.arm_frame.update(head_pose())
        event = SimpleNamespace(value={'body': [0.] * (16 * len(BODY_JOINT_KEYS))})
        with patch.object(VRTrajectoryPublisher, 'on_body_tracking_move',
                          new_callable=AsyncMock) as original:
            asyncio.run(self.node.on_body_tracking_move(event, None))
            original.assert_not_awaited()
            self.assertIsNone(self.node.arm_frame.correction)

    def test_diagnostics_include_raw_head_and_gravity_up_without_activation(self):
        self.node.head_diagnostic_pub = Mock()
        self.node.reference_diagnostic_pub = Mock()
        self.node.get_clock = Mock()
        self.node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        head = head_pose(pitch=-45., roll=20.)
        matrices = np.tile(np.eye(4).flatten(order='F'), len(BODY_JOINT_KEYS))
        start = 16 * BODY_JOINT_KEYS.index('head')
        matrices[start:start + 16] = head.flatten(order='F')
        event = SimpleNamespace(value={'body': matrices})
        with patch.object(VRTrajectoryPublisher, 'on_body_tracking_move',
                          new_callable=AsyncMock), patch('rclpy.ok', return_value=True):
            asyncio.run(self.node.on_body_tracking_move(event, None))
        for publisher in (self.node.head_diagnostic_pub, self.node.reference_diagnostic_pub):
            publisher.publish.assert_called_once()
            self.assertEqual(publisher.publish.call_args.args[0].header.frame_id, 'xr_world_y_up')
        raw = self.node.head_diagnostic_pub.publish.call_args.args[0].pose.orientation
        reference = self.node.reference_diagnostic_pub.publish.call_args.args[0].pose.orientation
        np.testing.assert_allclose(R.from_quat([raw.x, raw.y, raw.z, raw.w]).as_matrix(),
                                   head[:3, :3], atol=1e-12)
        up = R.from_quat([reference.x, reference.y, reference.z, reference.w]).as_matrix()[:, 2]
        np.testing.assert_allclose(up, [0., 1., 0.], atol=1e-12)


if __name__ == '__main__':
    unittest.main(verbosity=2)
