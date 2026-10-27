"""Immediate SH5 hand callbacks in isolated ROS; no Vuer server or hardware."""
import asyncio
import os
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock,Mock,patch

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation as R

from robotis_vuer.stock_sh5_hand import StockSH5HandInput,OriginalSH5
from robotis_vuer.vr_publisher_ai_worker_hardware import HardwareXRPublisher
from robotis_vuer.vr_publisher_sh5_hands_hardware import SH5HandsHardwarePublisher


def hand(scale=1.):
    poses=np.tile(np.eye(4),(25,1,1))
    for i in range(25):poses[i,:3,3]=np.array([.4,1.1,-.2])+scale*i*np.array([.001,.002,.003])
    return np.concatenate([p.flatten(order='F') for p in poses]).tolist()


def body(head=None):
    poses=np.tile(np.eye(4),(7,1,1))
    if head is not None:poses[6]=head
    return SimpleNamespace(value={'body':np.concatenate([p.flatten(order='F') for p in poses]).tolist()})


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID')=='232','isolated domain232 only')
class StockHandTests(unittest.TestCase):
    def setUp(self):
        self.env=patch.dict(os.environ,dict(AI_WORKER_XR_EXECUTION='validation',AI_WORKER_XR_MODEL='sh5',
            AI_WORKER_XR_ALLOW_ARM='1',AI_WORKER_LOOP_CONTROLS='1',AI_WORKER_SH5_INPUT='hands',AI_WORKER_SH5_HAND_MODE='1'))
        self.env.start();rclpy.init(args=[])
        self.node=SH5HandsHardwarePublisher(start_server=False)
        self.stock=self.node._stock_hands
        self.stock.left_hand_pos_pub=Mock();self.stock.right_hand_pos_pub=Mock()
        self.node.arm_control_enabled=True
        self.node._phase='ACTIVE'
        self.loop=asyncio.new_event_loop()

    def tearDown(self):
        self.node.destroy_node();rclpy.shutdown();self.loop.close();self.env.stop()

    def send_hand(self,**values):
        with patch.object(HardwareXRPublisher,'on_controller_move',new_callable=AsyncMock):
            self.loop.run_until_complete(self.node.on_hand_move(SimpleNamespace(value=values),None))

    def send_body(self,event=None):
        with patch.object(HardwareXRPublisher,'on_body_tracking_move',new_callable=AsyncMock) as arm:
            self.loop.run_until_complete(self.node.on_body_tracking_move(event or body(),None))
            arm.assert_awaited_once()

    def test_reuses_original_conversion_and_publishes_only_on_hand_event(self):
        self.assertIs(StockSH5HandInput.on_hand_move,OriginalSH5.on_hand_move)
        self.assertIs(StockSH5HandInput.process_hand_joints,OriginalSH5.process_hand_joints)
        self.send_hand(left=hand())
        self.stock.left_hand_pos_pub.publish.assert_called_once()
        self.send_body()
        self.stock.left_hand_pos_pub.publish.assert_called_once()
        self.stock.right_hand_pos_pub.publish.assert_not_called()

    def test_same_frame_head_and_original_landmark_order(self):
        self.send_hand(left=hand())
        head=np.eye(4);head[:3,:3]=R.from_euler('z',70,degrees=True).as_matrix();head[:3,3]=[1.,2.,3.]
        self.send_body(body(head))
        msg=self.stock.left_hand_pos_pub.publish.call_args.args[0]
        points=np.array([[p.x,p.y,p.z] for p in msg.joints])
        expected=np.array([[-.002*i,-.001*i,-.003*i] for i in self.stock.required_vr_frames])
        np.testing.assert_allclose(points,expected,atol=1e-7)
        np.testing.assert_array_equal(self.stock.head_transform_matrix,head)

    def test_partial_hand_updates_keep_independent_caches_and_original_filter(self):
        self.send_hand(left=hand(),right=hand())
        self.send_body()
        right=self.stock.right_hand_data.copy()
        self.send_hand(left=hand(2.))
        np.testing.assert_array_equal(self.stock.right_hand_data,right)
        self.send_body()
        msg=self.stock.left_hand_pos_pub.publish.call_args.args[0]
        points=np.array([[p.x,p.y,p.z] for p in msg.joints])
        expected=np.array([[-.002*i,-.001*i,-.003*i] for i in self.stock.required_vr_frames])*2.
        np.testing.assert_allclose(points,expected,atol=1e-7)
        self.assertEqual(self.stock.right_hand_pos_pub.publish.call_count,1)

    def test_session_hold_sends_no_zero_or_custom_finger_targets(self):
        self.node.arm_control_enabled=False
        self.send_hand(left=hand())
        self.send_body()
        self.stock.left_hand_pos_pub.publish.assert_not_called()
        self.stock.right_hand_pos_pub.publish.assert_not_called()

    def test_resume_resets_finger_filter_using_original_enable_method(self):
        self.assertIs(StockSH5HandInput._set_vr_publishing_enabled,OriginalSH5._set_vr_publishing_enabled)
        self.send_hand(left=hand());self.send_body()
        self.node.arm_control_enabled=False;self.send_body()
        self.node.arm_control_enabled=True;self.send_body()
        self.send_hand(left=hand(2.))
        msg=self.stock.left_hand_pos_pub.publish.call_args.args[0]
        points=np.array([[p.x,p.y,p.z] for p in msg.joints])
        expected=np.array([[-.002*i,-.001*i,-.003*i] for i in self.stock.required_vr_frames])*2.
        np.testing.assert_allclose(points,expected,atol=1e-7)

    def test_unused_finger_rotation_does_not_add_a_custom_hand_rejection(self):
        raw=hand()
        # Stock SH5 consumes finger positions and the wrist's rotation only.
        finger=np.array(raw[16:32]).reshape(4,4,order='F')
        finger[:3,:3]=0.
        raw[16:32]=finger.flatten(order='F').tolist()
        with patch.object(HardwareXRPublisher,'on_controller_move',new_callable=AsyncMock) as arm:
            self.loop.run_until_complete(self.node.on_hand_move(SimpleNamespace(value=dict(left=raw,right=hand())),None))
            arm.assert_awaited_once()
        self.send_body()
        self.stock.left_hand_pos_pub.publish.assert_called_once()


if __name__=='__main__':unittest.main()
