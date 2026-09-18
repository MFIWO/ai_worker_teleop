"""Run real hand/body callbacks with fake publishers, never robot hardware."""

import asyncio
import json
import os
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from robotis_vuer.operator_hand import ConsoleNeckPublisher, ConsoleOriginalPublisher
from robotis_vuer.vr_publisher_sh5 import BODY_JOINT_KEYS
from std_msgs.msg import String


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID') == '231', 'Requires isolated domain 231')
class OperatorHandTests(unittest.TestCase):
    def exercise(self, cls):
        rclpy.init()
        with patch.object(cls, 'start_vuer_server'):
            node = cls()
        try:
            outputs = {}
            for name in node.operator_outputs:
                outputs[name] = Mock()
                getattr(node, name).publisher = outputs[name]
            node.reactivate_pub = Mock()
            node.vr_publishing_enabled = True
            body = np.tile(np.eye(4).flatten(order='F'), len(BODY_JOINT_KEYS))
            head = np.eye(4)
            head[:3, :3] = node.vr_to_ros_matrix.T @ node.BODY_HEAD_TO_ROS_POSITION
            start = BODY_JOINT_KEYS.index('head') * 16
            body[start:start+16] = head.flatten(order='F')

            def send(now, offset=0., hands=True):
                event = {}
                for side in ('left', 'right'):
                    matrices = np.tile(np.eye(4), (25, 1, 1))
                    matrices[:, 0, 3] = offset
                    event[side] = np.concatenate([m.flatten(order='F') for m in matrices])
                with patch('time.monotonic', return_value=now):
                    if hands:
                        asyncio.run(node.on_hand_move(SimpleNamespace(value=event), None))
                    asyncio.run(node.on_body_tracking_move(
                        SimpleNamespace(value={'body': body}), None))

            def command(command, request_id):
                node.operator_request(String(data=json.dumps(
                    {'command': command, 'id': request_id})))

            for now in np.arange(0., .6, .05):
                send(float(now))
            self.assertIsNotNone(node.operator_guard.last)
            counts = {name: p.publish.call_count for name, p in outputs.items()}
            command('pause', 'pause1')
            self.assertFalse(node.reactivate_pub.publish.call_args.args[0].data)
            for now in (1., 1.2, 1.4):
                send(now, .3)
            self.assertEqual(counts, {n: p.publish.call_count for n, p in outputs.items()})
            command('pause', 'pause1')  # retry must not toggle
            self.assertTrue(node.operator_paused)
            command('resume', 'resume1')
            self.assertTrue(node.operator_resuming)
            for now in (2., 2.2, 2.4, 2.6):
                send(now, .06)  # raw displacement exceeds 5cm even if a low-pass filter hides it
            self.assertTrue(node.operator_resuming)
            self.assertEqual(counts, {n: p.publish.call_count for n, p in outputs.items()})
            send(4., hands=False)
            self.assertTrue(node.operator_resuming)
            for now in np.arange(4.1, 5.6, .05):
                send(float(now))
            self.assertFalse(node.operator_resuming)
            self.assertTrue(node.reactivate_pub.publish.call_args.args[0].data)
            self.assertGreater(outputs['left_wrist_rviz_pub'].publish.call_count,
                               counts['left_wrist_rviz_pub'])
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_neck_pause_and_safe_resume(self):
        self.exercise(ConsoleNeckPublisher)

    def test_original_pause_and_safe_resume(self):
        self.exercise(ConsoleOriginalPublisher)


if __name__ == '__main__':
    unittest.main()
