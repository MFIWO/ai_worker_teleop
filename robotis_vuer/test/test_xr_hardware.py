"""[ADDED] Real rclpy with numerical fixtures on TEST domain232, no simulator/motors."""
from copy import deepcopy
import os
import time
import unittest
from unittest.mock import patch

import numpy as np
import rclpy

from robotis_vuer.vr_publisher_ai_worker_hardware import HardwareXRPublisher
from robotis_vuer.vr_publisher_sg2_xr import XRDeltaPublisher
from robotis_vuer.xr_path_workspace import AnchorRayHint
from test_xr_resume import fixture
from xr_resume_smoke import signature


class HardwarePublisherTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '232':
            raise unittest.SkipTest('test-domain232 only')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.env=patch.dict(os.environ,{'AI_WORKER_XR_EXECUTION':'validation',
                                     'AI_WORKER_XR_MODEL':'sg2','AI_WORKER_XR_ALLOW_ARM':'0'})
        self.env.start()
        self.node=HardwareXRPublisher(start_server=False)

    def tearDown(self):
        self.node._set_arm_control_enabled(False,'test cleanup')
        self.node.destroy_node()
        self.env.stop()

    def capture(self):
        n=self.node
        h,inputs,robots,links=fixture()
        n._graph_check=lambda: None
        n._arm_tracking_is_fresh=lambda now=None: True
        n._require_still_robot=lambda now: None
        n._snapshot=lambda: deepcopy(links)
        now=time.monotonic(); ros=n.get_clock().now().nanoseconds*1e-9
        n._stopped_at,n._stopped_ros=now-1,ros-1
        n._robot_poses={s:(p.copy(),now-.01,ros-.01) for s,p in robots.items()}
        n.head_transform_matrix=h
        for s,p in inputs.items(): setattr(n,s+'_controller_matrix',p.copy())
        n.hardware_allow_arm=True
        n._set_arm_control_enabled(True,'fixture X+A')
        self.assertEqual(n._phase,'SETTLING')
        return inputs,robots,links

    def test_observe_only_cannot_enable(self):
        self.node._capture=lambda: self.fail('observe-only must never capture/enable')
        self.node._set_arm_control_enabled(True,'X+A')
        self.assertFalse(self.node.arm_control_enabled)
        self.assertIsNone(self.node._session)

    def test_hardware_capture_does_not_expand_elbow_or_shell(self):
        self.capture()
        self.assertEqual(self.node.goal_speed,.06)
        self.assertEqual(self.node.angular_speed,.35)
        for hint in self.node._session.templates.values():
            self.assertIsInstance(hint,AnchorRayHint)
            self.assertLessEqual(hint.maximum,.94*(hint.upper+hint.forearm)+1e-8)

    def test_hardware_real_mode_cannot_use_test_domain(self):
        with patch.dict(os.environ,{'AI_WORKER_XR_EXECUTION':'hardware','ROS_DOMAIN_ID':'232'}):
            with self.assertRaises(RuntimeError): self.node._validate_runtime_domain()

    def test_sim_class_still_refuses_robot_domain(self):
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'30'}):
            with self.assertRaises(RuntimeError): XRDeltaPublisher._validate_runtime_domain(self.node)

    def test_far_robot_tracking_error_does_not_publish(self):
        _,_,links=self.capture()
        self.node._phase='ACTIVE'
        links['left']['wrist'][0,3]+=.06
        count=self.node.published_pairs
        self.node._publish_synced_pose_frame_if_ready()
        self.assertFalse(self.node.arm_control_enabled)
        self.assertEqual(count,self.node.published_pairs)

    def test_no_reference_change_on_repeated_active_XA(self):
        self.capture()
        before=signature(self.node._session)
        self.node._phase='ACTIVE'
        for _ in range(3): self.node._set_arm_control_enabled(True,'active X+A')
        self.assertEqual(before,signature(self.node._session))


if __name__ == '__main__': unittest.main()
