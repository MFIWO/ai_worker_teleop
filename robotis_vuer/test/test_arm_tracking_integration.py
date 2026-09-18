"""Exercise actual hand/body callbacks on isolated ROS domain 231."""

import asyncio
import copy
import os
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from robotis_vuer.vr_publisher_hand_neck import NeckVRTrajectoryPublisher
from robotis_vuer.vr_publisher_sh5 import BODY_JOINT_KEYS


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID') == '231', 'Requires isolated domain 231')
class ActualCallbackTests(unittest.TestCase):
    def test_dropout_jump_and_recovery_publish_complete_continuous_batches(self):
        rclpy.init()
        with patch.object(NeckVRTrajectoryPublisher, 'start_vuer_server'):
            node = NeckVRTrajectoryPublisher()
        try:
            publishers = []
            for side in ('left', 'right'):
                for suffix in ('wrist_rviz_pub', 'elbow_pub', 'shoulder_pub'):
                    publisher = Mock()
                    setattr(node, side + '_' + suffix, publisher)
                    publishers.append(publisher)
            node.reactivate_pub = Mock()
            node.vr_publishing_enabled = True
            body = np.tile(np.eye(4).flatten(order='F'), len(BODY_JOINT_KEYS))
            head = np.eye(4)
            head[:3, :3] = node.vr_to_ros_matrix.T @ node.BODY_HEAD_TO_ROS_POSITION
            start = BODY_JOINT_KEYS.index('head') * 16
            body[start:start + 16] = head.flatten(order='F')
            body_event = SimpleNamespace(value={'body': body})

            def send(now, left_x=0.):
                hands = {}
                for side in ('left', 'right'):
                    matrices = np.tile(np.eye(4), (25, 1, 1))
                    matrices[:, 0, 3] = left_x if side == 'left' else 0.
                    hands[side] = np.concatenate([m.flatten(order='F') for m in matrices])
                with patch('time.monotonic', return_value=now):
                    asyncio.run(node.on_hand_move(SimpleNamespace(value=hands), None))
                    asyncio.run(node.on_body_tracking_move(body_event, None))

            for now in np.arange(0., .51, .05):
                send(float(now))
            counts = [p.publish.call_count for p in publishers]
            self.assertGreater(min(counts), 0)
            self.assertEqual(len(set(counts)), 1)

            # BODY_MOVE must not recycle a hand cache that stopped receiving HAND_MOVE.
            with patch('time.monotonic', return_value=1.):
                asyncio.run(node.on_body_tracking_move(body_event, None))
            self.assertEqual([p.publish.call_count for p in publishers], counts)
            self.assertEqual(node.tracking_guard.reason, 'tracking_missing')

            for now in (1.01, 1.12, 1.23, 1.34):
                send(now)
            self.assertFalse(node.tracking_guard.holding)
            counts = [p.publish.call_count for p in publishers]
            filters = copy.deepcopy(node.pose_filters)
            for now in (1.4, 1.5, 2.):
                send(now, left_x=.3)
            self.assertEqual([p.publish.call_count for p in publishers], counts)
            self.assertEqual(node.tracking_guard.reason, 'return_to_held_pose')
            for key in filters:
                np.testing.assert_array_equal(node.pose_filters[key]['pos'], filters[key]['pos'])

            for now in (2.1, 2.21, 2.32, 2.43):
                send(now)
            self.assertFalse(node.tracking_guard.holding)
            self.assertGreater(publishers[0].publish.call_count, counts[0])
            self.assertEqual(len({p.publish.call_count for p in publishers}), 1)

            with patch('time.monotonic', return_value=3.):
                asyncio.run(node.on_hand_move(
                    SimpleNamespace(value={'left': [0.] * 400}), None))
            self.assertIsNone(node.left_hand_data)
            self.assertIsNone(node.hand_received['left'])
            self.assertTrue(node.tracking_guard.holding)
            node.reactivate_pub.publish.assert_not_called()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main(verbosity=2)
