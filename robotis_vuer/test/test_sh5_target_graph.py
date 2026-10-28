"""Actual SH5 publishers and real graph checker, isolated domain232 only."""
import os,time,unittest
from unittest.mock import patch
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robotis_vuer.vr_publisher_sg2_xr import XRDeltaPublisher
from robotis_vuer.vr_publisher_sh5_hands_hardware import SH5HandsHardwarePublisher

class SH5TargetGraphTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID')!='232':raise unittest.SkipTest('isolated domain232 required')
        rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.env=patch.dict(os.environ,{'AI_WORKER_XR_MODEL':'sh5','AI_WORKER_SH5_INPUT':'controllers','AI_WORKER_SH5_HAND_MODE':'2','AI_WORKER_XR_EXECUTION':'validation'})
        self.env.start();self.node=SH5HandsHardwarePublisher(start_server=False)
    def tearDown(self):self.node.destroy_node();self.env.stop()
    def test_real_hardware_topics_pass_without_legacy_subgoal_writer(self):
        n=self.node;queried=[];original=n.count_publishers
        def count(topic):queried.append(topic);return original(topic)
        n.count_publishers=count
        XRDeltaPublisher._graph_check(n)
        self.assertIn('/l_elbow_pose',queried);self.assertIn('/r_elbow_pose',queried)
        self.assertNotIn('/l_subgoal_pose',queried);self.assertNotIn('/r_subgoal_pose',queried)
    def test_missing_actual_target_writer_still_blocks(self):
        n=self.node;original=n.count_publishers
        n.count_publishers=lambda topic:0 if topic=='/l_elbow_pose' else original(topic)
        with self.assertRaisesRegex(ValueError,'/l_elbow_pose: expected one target writer, found 0'):
            XRDeltaPublisher._graph_check(n)
    def test_duplicate_actual_target_writer_still_blocks(self):
        other=Node('duplicate_elbow_test_writer')
        pub=other.create_publisher(PoseStamped,'/l_elbow_pose',10)
        try:
            deadline=time.monotonic()+2.
            while self.node.count_publishers('/l_elbow_pose')<2 and time.monotonic()<deadline:
                rclpy.spin_once(self.node,timeout_sec=.01)
            with self.assertRaisesRegex(ValueError,'/l_elbow_pose: expected one target writer, found 2'):
                XRDeltaPublisher._graph_check(self.node)
        finally:other.destroy_node()

if __name__=='__main__':unittest.main()
