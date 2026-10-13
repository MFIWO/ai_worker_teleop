"""[ADDED] Real rclpy node / synthetic feedback only, isolated domain232."""
from copy import deepcopy
import os
import time
import unittest
import numpy as np
import rclpy
from std_msgs.msg import Bool

from robotis_vuer.vr_publisher_sg2_xr_overhead import XROverheadPublisher
from robotis_vuer.xr_path_workspace import AnchorRayHint
from test_xr_resume import fixture
from xr_resume_smoke import signature


class OverheadNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '232':
            raise unittest.SkipTest('isolated test domain232 required')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node=XROverheadPublisher(start_server=False)
        self.head,self.inputs,self.robots,self.links=fixture()
        self.node._graph_check=lambda: None
        self.node._arm_tracking_is_fresh=lambda now=None: True
        self.node._require_still_robot=lambda now: None
        self.node._snapshot=lambda: deepcopy(self.links)
        self.prepare()
        self.node._set_arm_control_enabled(True,'fixture X+A')
        self.assertEqual(self.node._phase,'SETTLING')
        self.sig=signature(self.node._session)

    def prepare(self):
        now=time.monotonic(); ros=self.node.get_clock().now().nanoseconds*1e-9
        self.node._stopped_at,self.node._stopped_ros=now-1,ros-1
        self.node._robot_poses={s:(p.copy(),now-.01,ros-.01) for s,p in self.robots.items()}
        self.node.head_transform_matrix=self.head.copy()
        for s,p in self.inputs.items(): setattr(self.node,s+'_controller_matrix',p.copy())

    def tearDown(self):
        self.node._set_arm_control_enabled(False,'unit cleanup')
        self.node.destroy_node()

    def publish(self):
        self.node._last_pub=None
        self.node._command_rx={s:time.monotonic() for s in self.inputs}
        self.node._publish_synced_pose_frame_if_ready()

    def test_arms_can_move_above_start_by_over_half_meter(self):
        self.node._phase='ACTIVE'
        for _ in range(180):
            self.node.left_controller_matrix[1,3] += .004
            self.publish()
            self.assertTrue(self.node.arm_control_enabled, self.node._last_reason)
            self.links['left']['wrist']=self.node._goals['left'].copy()
        self.assertGreater(self.node._path_frame['left']['anchor_displacement_m'],.5)
        self.assertGreater(self.node._goals['left'][2,3],self.robots['left'][2,3]+.5)
        self.assertEqual(signature(self.node._session),self.sig)

    def test_warmup_overhead_input_waits_held_not_clutch_stop(self):
        self.node._phase='WARMUP'; now=time.monotonic()
        self.node._enable_at=now-12
        self.node._first_command_rx={s:now-8.1 for s in self.inputs}
        saved=deepcopy(self.node._goals)
        self.node.left_controller_matrix[1,3]+=.65
        self.publish()
        self.assertEqual(self.node._phase,'WAIT_START')
        for s in saved: np.testing.assert_array_equal(saved[s],self.node._goals[s])

    def test_fault_and_tracking_checks_still_stop(self):
        self.node._phase='ACTIVE'
        self.node._reference_fault(Bool(data=True))
        self.assertFalse(self.node.arm_control_enabled)
        self.assertTrue(self.node._controller_fault_latched)

    def test_follow_error_not_disabled(self):
        self.node._phase='ACTIVE'
        self.links['left']['wrist'][0,3]-=.3
        count=self.node.published_pairs
        self.publish()
        self.assertFalse(self.node.arm_control_enabled)
        self.assertEqual(count,self.node.published_pairs)

    def test_no_partial_pair_on_corrupt_right_input(self):
        self.node._phase='ACTIVE'
        self.node.left_controller_matrix[2,3]-=.01
        self.node.right_controller_matrix[0,0]=float('nan')
        count=self.node.published_pairs
        self.publish()
        self.assertEqual(count,self.node.published_pairs)
        self.assertFalse(self.node.arm_control_enabled)

    def test_resume_preserves_ray_anchor_and_guidance(self):
        session=self.node._session
        anchors={s:h.path_anchor.copy() for s,h in session.templates.items()}
        for _ in range(3):
            self.node._set_arm_control_enabled(False,'Y+B buttons')
            self.prepare()
            self.node._set_arm_control_enabled(True,'aligned X+A')
            self.assertTrue(self.node.arm_control_enabled)
            self.assertIs(self.node._session,session)
            for s,h in session.templates.items():
                self.assertIsInstance(h,AnchorRayHint)
                np.testing.assert_array_equal(h.path_anchor,anchors[s])
        state=self.node._overlay_snapshot()
        self.assertEqual(set(state['arms']),{'left','right'})


if __name__=='__main__': unittest.main()
