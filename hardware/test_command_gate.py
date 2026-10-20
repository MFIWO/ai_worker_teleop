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
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from hardware.command_gate import HardwareCommandGate
from hardware.test_hardware_contract import fixture
from hardware.xr_contract import ARM_NAMES, ARM_DXL_IDS, GOAL_SPEED, load_profile, sha256
from hardware.aux_controls import read_aux_limits, AUX_TOPICS


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
           'max_joint_speed':4.8,'goal_speed':GOAL_SPEED,
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
        # [FIX 2026-10-14] Synthetic native telemetry is healthy in this fixture.
        # Do not depend on the removed buffered-register fallback.
        self.gate.motor_topic_ok=True
        self.sequence=0

    def tearDown(self):
        self.gate.destroy_node();self.tmp.cleanup();self.env.stop()

    def intent(self,enabled,aux=None):
        self.sequence+=1
        self.gate.intent(String(data=json.dumps({'model':'sg2','enabled':enabled,'session_id':'fixture',
            'sequence':self.sequence,'aux':aux or {},
            'ros_time_s':self.gate.get_clock().now().nanoseconds*1e-9})))

    def start(self):
        self.intent(False);self.intent(True)
        for side,names in ARM_NAMES.items():
            self.gate.commands[side]=({k:self.gate.measured[k] for k in names},time.monotonic())
        self.gate.tick()
        self.assertTrue(self.gate.output_active)

    def test_default_is_disarmed(self):
        self.gate.tick();self.assertEqual(self.sent,[])

    def test_clock_offset_ignored_but_lease_order_and_watchdog_preserved(self):
        self.intent(False)
        data={'model':'sg2','enabled':True,'session_id':'fixture','sequence':self.sequence+1,
              'ros_time_s':10.}
        self.gate.intent(String(data=json.dumps(data)))
        self.assertTrue(self.gate.intent_enabled)
        self.assertGreater(self.gate.intent_rx,0.)
        self.gate.intent_rx=time.monotonic()-5.01
        last=self.gate.intent_rx
        self.gate.intent(String(data=json.dumps(data)))
        self.assertEqual(self.gate.intent_rx,last)
        self.gate.tick()
        self.assertIn('lease expired',self.gate.fault)

    def test_boot_epoch_joint_and_motor_headers_preserve_receipt_order(self):
        names=list(self.gate.measured)
        joint=JointState(name=names,position=[self.gate.measured[k] for k in names])
        joint.header.stamp.sec=10
        self.gate.joints(joint)
        self.assertEqual(self.gate.measured_stamp,10.)
        first=self.gate.measured_rx
        self.gate.joints(joint)
        self.assertEqual(self.gate.measured_rx,first)
        motor=DynamixelState()
        motor.header.stamp.sec=10
        motor.id=ARM_DXL_IDS['right']+ARM_DXL_IDS['left']
        motor.comm_state=0;motor.dxl_hw_state=[0]*14;motor.torque_state=[True]*14
        self.gate.motor_state(motor)
        self.assertTrue(self.gate.motor_ok)
        first=self.gate.motor_rx
        self.gate.motor_state(motor)
        self.assertEqual(self.gate.motor_rx,first)

    def test_near_zero_solver_noise_is_clamped_without_latching_or_stopping(self):
        self.gate.profile['limits']['arm_l_joint2'] = [0., 3.14]
        self.gate.readiness_policy = 'driver_managed'
        self.start()
        names = list(ARM_NAMES['left'])
        for value in (1.5e-7, -1.5e-7, -4.9e-7, -1e-5, 0., 1e-7):
            msg = JointTrajectory(joint_names=names)
            positions = [self.gate.measured[k] for k in names]
            positions[1] = value
            msg.points = [JointTrajectoryPoint(positions=positions)]
            self.gate.command(msg, 'left')
            self.gate.tick()
            self.assertIsNone(self.gate.fault)
            self.assertTrue(self.gate.intent_enabled)
            self.assertTrue(self.gate.output_active)
            self.assertGreaterEqual(self.sent[-1]['arm_l_joint2'], 0.)

    def test_actual_joint_limit_violation_still_latches(self):
        self.gate.profile['limits']['arm_l_joint2'] = [0., 3.14]
        self.start()
        names = list(ARM_NAMES['left'])
        msg = JointTrajectory(joint_names=names)
        positions = [self.gate.measured[k] for k in names]
        positions[1] = -.0001
        msg.points = [JointTrajectoryPoint(positions=positions)]
        self.gate.command(msg, 'left')
        self.assertIn('numerical_tolerance=1e-05rad', self.gate.fault)
        self.assertFalse(self.gate.intent_enabled)
        self.assertFalse(self.gate.output_active)

    def test_new_session_cannot_start_already_enabled(self):
        self.intent(True);self.gate.tick()
        self.assertFalse(self.gate.intent_enabled);self.assertEqual(self.sent,[])

    def test_only_seven_joints_per_side_survive(self):
        self.start()
        self.assertEqual(set(self.sent[-1]),set().union(*ARM_NAMES.values()))

    def test_pc_disconnect_holds_and_latches(self):
        self.start();self.gate.intent_rx=time.monotonic()-5.01
        self.gate.tick()
        self.assertIn('lease expired',self.gate.fault)
        self.assertFalse(self.gate.output_active)
        self.assertEqual(self.sent[-1],self.gate.measured)

    def test_short_pc_intent_gap_holds_and_resumes(self):
        self.start()
        held = dict(self.sent[-1])
        self.gate.intent_rx = time.monotonic()-1.
        self.gate.tick()
        self.assertIsNone(self.gate.fault)
        self.assertTrue(self.gate.feedback_waiting)
        self.assertTrue(self.gate.intent_enabled)
        self.assertEqual(self.sent[-1], held)
        self.intent(True)
        self.gate.tick()
        self.assertIsNone(self.gate.fault)
        self.assertFalse(self.gate.feedback_waiting)

    def test_fresh_JS_cannot_hide_motor_read_loss_over_five_seconds(self):
        self.start();self.gate.motor_rx=time.monotonic()-5.01
        self.gate.tick();self.assertIn('motor read',self.gate.fault)

    def test_feedback_gaps_hold_last_output_then_recover_without_disarming(self):
        for channel in ('measured', 'motor', 'command'):
            with self.subTest(channel=channel):
                self.start()
                held = dict(self.sent[-1])
                now = time.monotonic()
                self.gate.measured_rx = self.gate.motor_rx = now
                for values, _ in self.gate.commands.values():
                    values['arm_l_joint1' if 'arm_l_joint1' in values else 'arm_r_joint1'] = .1
                if channel == 'command':
                    self.gate.commands = {s:(v,now-1.) for s,(v,_) in self.gate.commands.items()}
                else:
                    setattr(self.gate, channel+'_rx', now-1.)
                self.gate.tick()
                self.assertIsNone(self.gate.fault)
                self.assertTrue(self.gate.intent_enabled)
                self.assertTrue(self.gate.feedback_waiting)
                self.assertEqual(self.sent[-1], held)
                self.gate.measured_rx = self.gate.motor_rx = time.monotonic()
                self.gate.commands = {s:({k:self.gate.measured[k] for k in names},time.monotonic())
                                      for s,names in ARM_NAMES.items()}
                self.gate.tick()
                self.assertIsNone(self.gate.fault)
                self.assertFalse(self.gate.feedback_waiting)

    def test_feedback_and_output_loss_over_five_seconds_latches(self):
        for channel in ('measured', 'command'):
            with self.subTest(channel=channel):
                self.start()
                if channel == 'measured':
                    self.gate.measured_rx = time.monotonic()-5.01
                else:
                    self.gate.commands = {s:(v,time.monotonic()-5.01)
                                          for s,(v,_) in self.gate.commands.items()}
                self.gate.tick()
                self.assertIsNotNone(self.gate.fault)
                # Reset only fixture state before checking the other channel.
                self.gate.fault = None
                self.gate.measured_rx = time.monotonic()

    def test_command_skew_waits_for_a_coherent_pair(self):
        self.start()
        target,_=self.gate.commands['left'];self.gate.commands['left']=(target,time.monotonic()-.2)
        self.gate.tick()
        self.assertIsNone(self.gate.fault)
        self.assertTrue(self.gate.feedback_waiting)

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

    def test_manual_reset_clears_latch_only_after_disarmed_healthy_feedback(self):
        self.start()
        self.gate.latch('test fault')
        refused=self.gate.reset_fault(Trigger.Request(),Trigger.Response())
        self.assertFalse(refused.success)
        self.intent(False)
        reply=self.gate.reset_fault(Trigger.Request(),Trigger.Response())
        self.assertTrue(reply.success)
        self.assertIsNone(self.gate.fault)
        self.assertFalse(self.gate.intent_enabled)
        self.assertFalse(self.gate.output_active)
        self.assertEqual(self.gate.last_cleared_fault['fault'],'test fault')
        self.start()
        self.assertTrue(self.gate.output_active)

    def test_fault_reset_refuses_persisting_motor_or_feedback_failure(self):
        self.start();self.gate.latch('motor fault');self.intent(False)
        self.gate.motor_ok=False
        response=self.gate.reset_fault(Trigger.Request(),Trigger.Response())
        self.assertFalse(response.success)
        self.assertIsNotNone(self.gate.fault)
        self.gate.motor_ok=True
        self.gate.measured_rx=time.monotonic()-1.
        response=self.gate.reset_fault(Trigger.Request(),Trigger.Response())
        self.assertFalse(response.success)
        self.assertIsNotNone(self.gate.fault)

    def test_false_torque_telemetry_cannot_use_buffered_evidence(self):
        # [ADDED] Actual message types, isolated domain232; no motor commands.
        msg=DynamixelState()
        msg.header.stamp=self.gate.get_clock().now().to_msg()
        msg.id=ARM_DXL_IDS['right']+ARM_DXL_IDS['left']
        msg.comm_state=0
        msg.dxl_hw_state=[0]*14
        msg.torque_state=[False]*14
        self.gate.motor_state(msg)
        self.assertFalse(self.gate.motor_ok)
        self.assertFalse(self.gate.torque_evidence.verified())
        self.intent(False)
        self.intent(True)
        self.gate.tick()
        self.assertFalse(self.gate.output_active)
        self.assertIsNotNone(self.gate.fault)
        self.assertEqual(self.sent,[])

    def test_native_stream_timing_is_not_replaced_with_100ms_ramp(self):
        # [ADDED 2026-10-14] Exercise the actual emitter, not the fixture lambda.
        messages = []
        class Sink:
            def publish(self, msg):
                messages.append(msg)
        original = self.gate.pub
        self.gate.pub = {side: Sink() for side in ARM_NAMES}
        try:
            HardwareCommandGate.emit(self.gate, self.gate.measured)
        finally:
            self.gate.pub = original
        self.assertEqual(len(messages), 2)
        for msg, names in zip(messages, ARM_NAMES.values()):
            self.assertEqual(list(msg.joint_names), list(names))
            self.assertEqual((msg.header.stamp.sec, msg.header.stamp.nanosec), (0, 0))
            self.assertEqual(len(msg.points), 1)
            point = msg.points[0]
            self.assertEqual((point.time_from_start.sec, point.time_from_start.nanosec), (0, 0))
            self.assertEqual(list(point.positions), [self.gate.measured[k] for k in names])

    def test_future_trajectory_is_not_collapsed_to_immediate_target(self):
        self.intent(False)
        self.intent(True)
        msg = JointTrajectory(joint_names=list(ARM_NAMES['left']))
        point = JointTrajectoryPoint(positions=[self.gate.measured[k] for k in msg.joint_names])
        point.time_from_start.nanosec = 100_000_000
        msg.points = [point]
        self.gate.command(msg, 'left')
        self.assertIn('one native immediate', self.gate.fault)
        self.assertEqual(self.sent, [])

    def test_rejected_joint6_values_survive_hold_and_first_fault_is_retained(self):
        self.start()
        self.gate.commands['left'][0]['arm_l_joint6'] = .481
        self.gate.tick()
        self.assertIn('commanded/measured error exceeds URDF velocity tracking budget', self.gate.fault)
        self.assertIn('target=+0.481000', self.gate.fault)
        self.assertFalse(self.gate.output_active)
        item = self.gate.fault_detail['joints']['arm_l_joint6']
        self.assertEqual(item['target_rad'], .481)
        self.assertEqual(item['measured_rad'], 0.)
        self.assertEqual(item['previous_output_rad'], 0.)
        encoded = json.dumps(self.gate.fault_detail, allow_nan=False)
        self.gate.latch('later symptom')
        self.assertEqual(encoded, json.dumps(self.gate.fault_detail, allow_nan=False))
        self.assertEqual(self.sent[-1]['arm_l_joint6'], 0.)

    def test_driver_managed_joint5_error_above_old_bound_keeps_following(self):
        self.gate.readiness_policy = 'driver_managed'
        self.start()
        self.gate.commands['left'][0]['arm_l_joint5'] = .12
        self.gate.last_output = time.monotonic() - .02
        self.gate.tick()
        self.assertIsNone(self.gate.fault)
        self.assertTrue(self.gate.output_active)
        self.assertGreater(self.sent[-1]['arm_l_joint5'], 0.)
        self.assertLess(self.sent[-1]['arm_l_joint5'], .12)

    def test_normal_gate_limits_speed_without_following_error_fault(self):
        self.gate.readiness_policy = 'driver_managed'
        self.start()
        self.gate.commands['left'][0]['arm_l_joint5'] = .9
        self.gate.last_output = time.monotonic() - .02
        self.gate.tick()
        self.assertIsNone(self.gate.fault)
        self.assertTrue(self.gate.output_active)
        self.assertGreater(self.sent[-1]['arm_l_joint5'], 0.)
        self.assertLess(self.sent[-1]['arm_l_joint5'], .11)

    def enable_auxiliary_fixture(self):
        bounds = {'head_joint1':(-.2317,.6951,4.8), 'head_joint2':(-.35,.35,4.8),
                  'lift_joint':(-.5,0.,4.8), 'gripper_l_joint1':(0.,1.1,6.5),
                  'gripper_r_joint1':(0.,1.1,6.5)}
        self.gate.profile['auxiliary_limits']=read_aux_limits('<robot>'+''.join(
            f'<joint name="{k}"><limit lower="{lo}" upper="{hi}" velocity="{v}"/></joint>'
            for k,(lo,hi,v) in bounds.items())+'</robot>')
        self.gate.measured.update({k:(-.2 if k=='lift_joint' else 0.) for k in bounds})
        self.aux_messages=[]; self.base_messages=[]; self.arm_messages=[]
        class Sink:
            def __init__(self, messages): self.messages=messages
            def publish(self,msg): self.messages.append(msg)
        self.gate.aux_pub={group:Sink(self.aux_messages) for group in AUX_TOPICS}
        self.gate.base_pub=Sink(self.base_messages)
        self.gate.pub={side:Sink(self.arm_messages) for side in ARM_NAMES}

    def test_auxiliary_and_gripper_outputs_stop_on_pc_disconnect(self):
        self.enable_auxiliary_fixture()
        self.gate.readiness_policy='driver_managed'
        self.start()
        self.gate.emit=HardwareCommandGate.emit.__get__(self.gate)
        self.intent(True, {'joints':{'head_joint1':.1,'head_joint2':.1,'lift_joint':-.25,
                                   'gripper_l_joint1':1.1,'gripper_r_joint1':.55},
                           'base':[.1,0.,.1]})
        self.gate.last_output=time.monotonic()-.02
        self.gate.tick()
        self.assertIsNone(self.gate.fault)
        self.assertEqual(len(self.arm_messages), 2)
        self.assertIn('gripper_l_joint1', self.arm_messages[0].joint_names)
        self.assertGreater(self.arm_messages[0].points[0].positions[-1], 0.)
        self.assertLess(self.arm_messages[0].points[0].positions[-1], .15)
        self.assertTrue(any('head_joint1' in msg.joint_names for msg in self.aux_messages))
        self.assertTrue(any('lift_joint' in msg.joint_names for msg in self.aux_messages))
        self.assertEqual(self.base_messages[-1].linear.x, .1)
        self.gate.intent_rx=time.monotonic()-5.01
        self.gate.tick()
        self.assertIn('lease expired', self.gate.fault)
        self.assertEqual(self.base_messages[-1].linear.x, 0.)
        self.assertEqual(self.base_messages[-1].angular.z, 0.)
        self.assertEqual(self.gate.aux_previous,{})

    def test_out_of_bounds_auxiliary_target_stops_gate(self):
        self.enable_auxiliary_fixture()
        self.start()
        self.intent(True, {'joints':{'lift_joint':.2}})
        self.assertIn('outside configured range',self.gate.fault)
        self.assertFalse(self.gate.intent_enabled)
        self.assertEqual(self.base_messages[-1].linear.x,0.)

    def test_legacy_manifest_derives_per_joint_limits_from_hashed_urdf(self):
        profile = load_profile(self.path)
        self.assertEqual(profile['joint_velocity_limits']['arm_l_joint5'], 4.8)
        self.assertAlmostEqual(profile['tracking_error_limits']['arm_l_joint5'], .48)
        self.assertEqual(profile['limits']['arm_l_joint4'], [-2.9, -.7])

    def test_profile_cannot_override_urdf_position_or_speed(self):
        original = json.loads(self.path.read_text())
        for key, value in [('limits', {**original['limits'], 'arm_l_joint1': [-3., 3.]}),
                           ('joint_velocity_limits', {k: 10. for k in original['limits']})]:
            changed = dict(original)
            changed[key] = value
            self.path.write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError, 'differ from verified URDF'):
                load_profile(self.path)


if __name__ == '__main__':unittest.main()
