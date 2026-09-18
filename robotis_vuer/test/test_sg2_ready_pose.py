"""Ready-pose tests. ROS integration cases require isolated domain 231."""

import math
import os
from pathlib import Path
import sys
import threading
import unittest

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robotis_vuer.prepare_sg2_ready_pose import (  # noqa: E402, I100
    ARM_NAMES, make_trajectory, READY_Q, ReadyPoseMover,
)


class TrajectoryTests(unittest.TestCase):
    def test_ready_pose_and_smooth_endpoints(self):
        self.assertEqual(READY_Q, [0., 0., 0., -1.57, 0., 0., 0.])
        msg, duration = make_trajectory(ARM_NAMES['l'], [0.] * 7, READY_Q, 5.)
        self.assertEqual(duration, 5.)
        self.assertEqual(len(msg.points), 100)
        self.assertEqual(msg.joint_names, ARM_NAMES['l'])
        self.assertEqual(list(msg.points[0].positions), [0.] * 7)
        self.assertEqual(list(msg.points[-1].positions), READY_Q)
        for point in (msg.points[0], msg.points[-1]):
            self.assertEqual(list(point.velocities), [0.] * 7)
            self.assertEqual(list(point.accelerations), [0.] * 7)
        times = [p.time_from_start.sec + p.time_from_start.nanosec / 1e9 for p in msg.points]
        self.assertEqual(times[0], 0.)
        self.assertEqual(times[-1], 5.)
        self.assertTrue(all(b > a for a, b in zip(times, times[1:])))
        self.assertTrue(all(-1.57 <= p.positions[3] <= 0. for p in msg.points))
        self.assertTrue(all(abs(v) <= 3. for p in msg.points for v in p.velocities))

    def test_speed_cap_extends_short_duration(self):
        msg, duration = make_trajectory(ARM_NAMES['r'], [0.] * 7, READY_Q, .1)
        self.assertGreater(duration, .1)
        self.assertTrue(all(abs(v) <= 3. for p in msg.points for v in p.velocities))

    def test_invalid_inputs_rejected(self):
        for start, duration in (([math.nan] * 7, 5.), ([0.] * 6, 5.), ([0.] * 7, -1.)):
            with self.assertRaises(ValueError):
                make_trajectory(ARM_NAMES['l'], start, READY_Q, duration)


class FakeFollowers(Node):
    def __init__(self, follow=True):
        super().__init__('fake_sg2_followers')
        self.follow = follow
        self.received = {}
        self.q = {name: 0. for names in ARM_NAMES.values() for name in names}
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(.02, self.publish_state)
        self.servers = [ActionServer(
            self, FollowJointTrajectory, f'/arm_{side}_controller/follow_joint_trajectory',
            lambda handle, side=side: self.execute(handle, side),
        ) for side in ARM_NAMES]

    def publish_state(self):
        msg = JointState(name=list(self.q), position=list(self.q.values()),
                         velocity=[0.] * 14)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def execute(self, handle, side):
        trajectory = handle.request.trajectory
        self.received[side] = trajectory
        if self.follow:
            self.q.update(zip(trajectory.joint_names, trajectory.points[-1].positions))
        handle.succeed()
        return FollowJointTrajectory.Result(error_code=0)


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID') == '231', 'Requires isolated domain 231')
class IntegrationTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.server = FakeFollowers()
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.server)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        self.mover = ReadyPoseMover()

    def tearDown(self):
        self.mover.destroy_node()
        self.executor.shutdown()
        self.thread.join()
        for server in self.server.servers:
            server.destroy()
        self.server.destroy_node()
        rclpy.shutdown()

    def test_both_arm_actions_and_feedback(self):
        self.mover.move(5.)
        self.assertEqual(set(self.server.received), {'l', 'r'})
        for side, msg in self.server.received.items():
            self.assertEqual(msg.joint_names, ARM_NAMES[side])
            self.assertEqual(list(msg.points[-1].positions), READY_Q)
            self.assertEqual(msg.points[-1].time_from_start.sec, 5)

    def test_success_result_without_reaching_pose_is_rejected(self):
        self.server.follow = False
        with self.assertRaisesRegex(RuntimeError, 'measured ready pose'):
            self.mover.move(5.)

    def test_existing_vr_controller_prevents_motion(self):
        other = Node('vr_controller')
        try:
            with self.assertRaisesRegex(RuntimeError, 'Stop the existing'):
                self.mover.move(5.)
            self.assertFalse(self.server.received)
        finally:
            other.destroy_node()


if __name__ == '__main__':
    unittest.main(verbosity=2)
