"""Mode 2 uses HAND events without changing the inherited arm input; domain232 only."""
import asyncio
import os
import time
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch

import numpy as np
import rclpy

from robotis_vuer.vr_publisher_ai_worker_hardware import HardwareXRPublisher
from robotis_vuer.vr_publisher_sh5_hands_hardware import SH5HandsHardwarePublisher


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID')=='232', 'isolated domain232 only')
class XRHandInputTests(unittest.TestCase):
    def setUp(self):
        self.env=patch.dict(os.environ,dict(AI_WORKER_XR_EXECUTION='validation',AI_WORKER_XR_MODEL='sh5',
            AI_WORKER_XR_ALLOW_ARM='1',AI_WORKER_LOOP_CONTROLS='1',AI_WORKER_SH5_INPUT='hands',AI_WORKER_SH5_HAND_MODE='2'))
        self.env.start();rclpy.init(args=[])
        self.node=SH5HandsHardwarePublisher(start_server=False)
        self.node._xr_hands.publishers={s:Mock() for s in ('left','right')}
        self.loop=asyncio.new_event_loop()

    def tearDown(self):
        self.node.destroy_node();rclpy.shutdown();self.loop.close();self.env.stop()

    def event(self):
        poses=np.tile(np.eye(4),(25,1,1))
        for i in range(1,25):poses[i,:3,3]=[.001*i,.002*i,-.003*i]
        raw=poses.transpose(0,2,1).reshape(-1).tolist()
        return SimpleNamespace(value=dict(left=raw,right=raw))

    def test_hand_event_publishes_twenty_five_points_and_same_arm_wrists(self):
        self.node.arm_control_enabled=True;self.node._phase='ACTIVE'
        self.node.last_body_pose_sec=time.monotonic()
        event=self.event()
        with patch.object(HardwareXRPublisher,'on_controller_move',new_callable=AsyncMock) as arm:
            self.loop.run_until_complete(self.node.on_hand_move(event,None))
        arm.assert_awaited_once()
        adapted=arm.call_args.args[1].value
        for side,pub in self.node._xr_hands.publishers.items():
            self.assertEqual(adapted[side],event.value[side][:16])
            msg=pub.publish.call_args.args[0]
            self.assertEqual(len(msg.points),25)
            self.assertEqual(msg.header.frame_id,'xr_ai_wrist_local')
        self.assertIsNone(self.node._stock_hands)

    def test_hold_and_body_events_produce_no_hand_targets(self):
        self.node.arm_control_enabled=False
        self.node.last_body_pose_sec=time.monotonic()
        with patch.object(HardwareXRPublisher,'on_controller_move',new_callable=AsyncMock):
            self.loop.run_until_complete(self.node.on_hand_move(self.event(),None))
        with patch.object(HardwareXRPublisher,'on_body_tracking_move',new_callable=AsyncMock) as arm:
            self.loop.run_until_complete(self.node.on_body_tracking_move(SimpleNamespace(value={}),None))
            arm.assert_awaited_once()
        for pub in self.node._xr_hands.publishers.values():
            pub.publish.assert_not_called()


if __name__=='__main__':unittest.main()
