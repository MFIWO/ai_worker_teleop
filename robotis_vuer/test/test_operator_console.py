"""Offline key routing and bounded robot commands; requires isolated ROS domain 231."""

import json
import os
import time
import unittest
from unittest.mock import Mock

import rclpy
from robotis_vuer.operator_console import (
    bounded_step, key_route, motion_vector, OperatorConsole,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class KeyRoutingTests(unittest.TestCase):
    def test_recording_and_head_a_are_exclusive(self):
        self.assertEqual(key_route('a', False), 'loop')
        self.assertEqual(key_route('a', True), 'motion')
        self.assertEqual(key_route('a', True, True), 'loop')
        self.assertEqual(key_route('b', True), 'loop')
        self.assertEqual(key_route('c', True), 'loop')
        for key in ('w', 's', 'd', 'q', 'e', 'o', 'p', 'up', 'down', 'left', 'right'):
            self.assertEqual(key_route(key, False), 'ignore')
            self.assertEqual(key_route(key, True), 'motion')

    def test_pedal_a_never_moves_head_while_paused(self):
        for key in ('a', 'b', 'c'):
            self.assertEqual(key_route(key, True, loop_input=True), 'loop')
        self.assertEqual(key_route('u', True, loop_input=True), 'toggle')

    def test_directions_and_speed_bound(self):
        self.assertEqual(motion_vector({'up'}), (.15, 0., 0.))
        self.assertEqual(motion_vector({'left', 'q'}), (0., .15, .3))
        self.assertEqual(motion_vector({'up', 'down', 'q', 'e'}), (0., 0., 0.))
        x, y, _ = motion_vector({'up', 'left'})
        self.assertAlmostEqual(x*x + y*y, .15**2)

    def test_stalled_feedback_and_limits_bound_target(self):
        target = 0.
        for _ in range(1000):
            target = bounded_step(0., target, 1, .25, .05, (-.35, .35), .05)
        self.assertAlmostEqual(target, .05)
        self.assertEqual(bounded_step(-.001, -.001, 1, .04, .05, (-.5, 0.), .01), 0.)
        with self.assertRaises(ValueError):
            bounded_step(float('nan'), 0., 1, .25, .05, (-.35, .35), .05)


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID') == '231', 'Requires isolated domain 231')
class ConsoleNodeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = OperatorConsole()
        for name in ('request_pub', 'base_pub', 'head_pub', 'lift_pub'):
            setattr(self.node, name, Mock())

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def hold(self):
        self.node.receive_status(String(data=json.dumps({
            'paused': True, 'request_id': '', 'reason': 'paused'})))
        self.node.receive_joints(JointState(
            name=['head_joint1', 'head_joint2', 'lift_joint'], position=[.1, .0, -.25]))

    def test_no_movement_without_ack_and_fresh_feedback(self):
        n = self.node
        n.keys = {'up', 'w', 'o'}
        n.tick()
        n.base_pub.publish.assert_not_called()
        n.head_pub.publish.assert_not_called()
        n.lift_pub.publish.assert_not_called()
        self.hold()
        n.request('resume')
        n.keys = {'up', 'w', 'o'}
        n.tick()
        n.base_pub.publish.assert_not_called()

    def test_release_and_stale_status_stop_base(self):
        self.hold()
        n = self.node
        n.keys = {'up', 'q'}
        n.tick()
        self.assertEqual(n.base_pub.publish.call_args.args[0].linear.x, .15)
        n.keys.clear()
        n.tick()
        self.assertEqual(n.base_pub.publish.call_args.args[0].linear.x, 0.)
        n.keys = {'up'}
        n.tick()
        n.status_time = time.monotonic() - 1.
        n.tick()
        self.assertEqual(n.base_pub.publish.call_args.args[0].linear.x, 0.)

    def test_head_lift_measured_start_release_and_stale_feedback(self):
        self.hold()
        n = self.node
        n.keys = {'w', 'a', 'o'}
        n.last_tick -= .05
        n.tick()
        head = n.head_pub.publish.call_args.args[0].points[0].positions
        self.assertLess(head[0], .1)  # negative pitch looks up
        self.assertGreater(head[1], 0.)
        self.assertGreater(n.lift_pub.publish.call_args.args[0].points[0].positions[0], -.25)
        n.keys.clear()
        n.tick()
        self.assertEqual(list(n.head_pub.publish.call_args.args[0].points[0].positions), [.1, 0.])
        self.assertEqual(list(n.lift_pub.publish.call_args.args[0].points[0].positions), [-.25])
        n.joints.clear()
        n.head_pub.publish.reset_mock()
        n.keys = {'w'}
        n.tick()
        n.head_pub.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
