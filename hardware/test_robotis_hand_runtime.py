"""Original Cyclo mode-1 entrypoint in isolated ROS domain232; no hardware."""
import os
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np
import rclpy
from geometry_msgs.msg import Point32
from robotis_interfaces.msg import HandJoints
from trajectory_msgs.msg import JointTrajectory
from rclpy.qos import QoSProfile, ReliabilityPolicy
from hardware.sh5_controller_hands import HAND_TOPICS
from hardware.test_thumb_flexion import skeleton


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID') == '232', 'isolated domain232 only')
class RobotisHandRuntimeTests(unittest.TestCase):
    def test_mode1_entrypoint_publishes_native_hand_trajectories(self):
        rclpy.init()
        node = rclpy.create_node('robotis_thumb_test_peer')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        got = {}
        subs = [node.create_subscription(JointTrajectory, topic,
                lambda msg, side=side: got.update({side: msg}), qos) for side, topic in HAND_TOPICS.items()]
        pubs = {s: node.create_publisher(HandJoints, f'/{s}_hand/hand_joint_pos', qos) for s in HAND_TOPICS}
        output = tempfile.TemporaryFile(mode='w+')
        process = subprocess.Popen([sys.executable, '-B', '-m', 'hardware.cyclo_hand_retarget_node'], stdout=output, stderr=output)
        try:
            deadline = time.monotonic()+15.
            while time.monotonic() < deadline and len(got) < 2:
                for pub in pubs.values():
                    msg = HandJoints(joints=[Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in skeleton(.2, .8, 21)])
                    pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=.03)
                if process.poll() is not None:
                    output.seek(0);self.fail(output.read())
            self.assertEqual(set(got), set(HAND_TOPICS))
            for side, msg in got.items():
                self.assertEqual(len(msg.joint_names), 20)
                self.assertEqual(len(msg.points[0].positions), 20)
                self.assertTrue(np.isfinite(msg.points[0].positions).all())
                self.assertEqual(msg.points[0].time_from_start.sec, 0)
                self.assertEqual(msg.points[0].time_from_start.nanosec, 0)
            output.flush();output.seek(0)
            self.assertIn('[HAND MODE 1] Cyclo original', output.read())
        finally:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
            output.close();node.destroy_node();rclpy.try_shutdown()


if __name__ == '__main__':unittest.main()
