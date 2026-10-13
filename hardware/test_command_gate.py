"""[ADDED] Real rclpy gate tests, synthetic data only; never run on domain30."""
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from dynamixel_interfaces.msg import DynamixelState

from hardware.command_gate import HardwareCommandGate
from hardware.test_hardware_contract import fixture
from hardware.xr_contract import ARM_NAMES, ARM_DXL_IDS, sha256


class GateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '232':
            raise unittest.SkipTest('isolated domain232 only')
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.env=patch.dict(os.environ,{'AI_WORKER_XR_EXECUTION':'validation'})
        self.env.start()
        self.tmp=tempfile.TemporaryDirectory()
        root=Path(self.tmp.name)
        p={'schema':1,'mode':'hardware_commissioning','model':'sg2',
           'max_joint_speed':.35,'goal_speed':.06,
           'controller_joint_names':[k for names in ARM_NAMES.values() for k in names]+['lift_joint'],
           'limits':{k:[-2.9,-.7 if k.endswith('joint4') else 2.9] for names in ARM_NAMES.values() for k in names},
           'arm_signature':{},'frozen_joints':{},'arm_dxl_ids':ARM_DXL_IDS}
        for key,content in (('urdf',fixture()),('srdf','<robot/>'),('config','{}')):
            f=root/(key+'.txt'); f.write_text(content);p[key]=str(f);p[key+'_sha256']=sha256(f)
        self.path=root/'profile.json';self.path.write_text(json.dumps(p))
        self.gate=HardwareCommandGate(self.path)
        self.sent=[]
        self.gate.emit=lambda target:self.sent.append(dict(target))
        self.gate.graph=lambda:None
        q={k:(-1.57 if k.endswith('joint4') else 0.) for names in ARM_NAMES.values() for k in names}
        q['lift_joint']=0.
        self.gate.measured=q
        self.gate.measured_rx=self.gate.motor_rx=time.monotonic()
        self.gate.description_ok=self.gate.motor_ok=True
        self.sequence=0

    def tearDown(self):
        self.gate.destroy_node();self.tmp.cleanup();self.env.stop()

    def intent(self,enabled):
        self.sequence+=1
        self.gate.intent(String(data=json.dumps({'model':'sg2','enabled':enabled,'session_id':'fixture',
            'sequence':self.sequence,'ros_time_s':self.gate.get_clock().now().nanoseconds*1e-9})))

    def start(self):
        self.intent(False);self.intent(True)
        for side,names in ARM_NAMES.items():
            self.gate.commands[side]=({k:self.gate.measured[k] for k in names},time.monotonic())
        self.gate.tick()
        self.assertTrue(self.gate.output_active)

    def test_default_is_disarmed(self):
        self.gate.tick();self.assertEqual(self.sent,[])

    def test_new_session_cannot_start_already_enabled(self):
        self.intent(True);self.gate.tick()
        self.assertFalse(self.gate.intent_enabled);self.assertEqual(self.sent,[])

    def test_only_seven_joints_per_side_survive(self):
        self.start()
        self.assertEqual(set(self.sent[-1]),set().union(*ARM_NAMES.values()))

    def test_pc_disconnect_holds_and_latches(self):
        self.start();self.gate.intent_rx=time.monotonic()-1
        self.gate.tick()
        self.assertIn('lease expired',self.gate.fault)
        self.assertFalse(self.gate.output_active)
        self.assertEqual(self.sent[-1],self.gate.measured)

    def test_fresh_JS_cannot_hide_motor_read_failure(self):
        self.start();self.gate.motor_rx=time.monotonic()-1
        self.gate.tick();self.assertIn('motor read',self.gate.fault)

    def test_command_skew_latches(self):
        self.start()
        target,_=self.gate.commands['left'];self.gate.commands['left']=(target,time.monotonic()-.2)
        self.gate.tick();self.assertIn('out of sync',self.gate.fault)

    def test_velocity_jump_latches_before_requested_output(self):
        self.start();self.gate.commands['left'][0]['arm_l_joint1']=.03
        self.gate.tick();self.assertIn('speed bound',self.gate.fault)
        self.assertEqual(self.sent[-1]['arm_l_joint1'],0.)

    def test_replayed_motor_header_does_not_refresh(self):
        msg=DynamixelState();msg.header.stamp=self.gate.get_clock().now().to_msg()
        msg.id=ARM_DXL_IDS['right']+ARM_DXL_IDS['left'];msg.comm_state=0;msg.dxl_hw_state=[0]*14;msg.torque_state=[True]*14
        self.gate.motor_state(msg)
        first=self.gate.motor_rx
        with patch('hardware.command_gate.time.monotonic',return_value=first+1):self.gate.motor_state(msg)
        self.assertEqual(first,self.gate.motor_rx)

    def test_manual_stop_retains_latched_fault(self):
        self.start();self.gate.latch('injected test fault');self.intent(False);self.intent(True)
        count=len(self.sent);self.gate.tick()
        self.assertEqual(len(self.sent),count)
        self.assertEqual(self.gate.fault,'injected test fault')


if __name__ == '__main__':unittest.main()
