"""Real PC publisher input callbacks on an isolated ROS domain; no X11 grabs."""
import json
import os
import time
import unittest
from unittest.mock import Mock,patch
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from robotis_vuer.vr_publisher_sh5_hands_hardware import SH5HandsHardwarePublisher


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID')=='232','isolated domain232 only')
class LoopOperatorTests(unittest.TestCase):
    def setUp(self):
        self.env=patch.dict(os.environ,dict(AI_WORKER_XR_EXECUTION='validation',AI_WORKER_XR_MODEL='sh5',
            AI_WORKER_XR_ALLOW_ARM='1',AI_WORKER_LOOP_CONTROLS='1'))
        self.env.start()
        rclpy.init(args=[])
        self.node=SH5HandsHardwarePublisher(start_server=False)
        self.node._hardware_manifest={'manual_auxiliary':True,'auxiliary_limits':{
            n:dict(lower=-1.,upper=1.,velocity=1.) for n in ('head_joint1','head_joint2','lift_joint')}}
        self.node._joint_rx=time.monotonic()
        self.node.current_joint_states=JointState(name=['head_joint1','head_joint2','lift_joint'],position=[0.,0.,-.2])
        self.node._set_arm_control_enabled=Mock(side_effect=lambda enabled,reason:setattr(self.node,'arm_control_enabled',enabled))

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.env.stop()

    def send(self,keys,seq=1,stop=False,focused=True):
        self.node._operator_input(String(data=json.dumps(dict(instance='fixture',sequence=seq,
            keys=keys,stop=stop,focused=focused))))

    def test_base_reposition_holds_arms_and_requires_a_new_profile(self):
        self.node.arm_control_enabled=True
        self.send(['w'])
        self.assertFalse(self.node.arm_control_enabled)
        self.assertTrue(self.node._base_profile_dirty)
        self.assertEqual(self.node._extra_intent(time.monotonic())['manual_aux']['base'],[.15,0.,0.])

    def test_head_commands_do_not_change_arm_authority_and_expire(self):
        self.node.arm_control_enabled=True
        self.send(['left'])
        self.node._operator_tick-=.05
        data=self.node._extra_intent(time.monotonic())
        self.assertTrue(self.node.arm_control_enabled)
        self.assertGreater(data['manual_aux']['joints']['head_joint2'],0.)
        self.node._operator_rx-=.31
        self.assertFalse(self.node._extra_intent(time.monotonic())['manual_enabled'])

    def test_shutdown_holds_arms_and_replay_cannot_reactivate_motion(self):
        self.node.arm_control_enabled=True
        self.send([],stop=True)
        self.assertFalse(self.node.arm_control_enabled)
        self.assertFalse(self.node._extra_intent(time.monotonic())['manual_enabled'])
        self.send(['w'],seq=1)
        self.assertEqual(self.node._operator_keys,set())
        self.send(['w'],seq=2,focused=False)
        self.assertFalse(self.node._extra_intent(time.monotonic())['manual_enabled'])


if __name__=='__main__':unittest.main()
