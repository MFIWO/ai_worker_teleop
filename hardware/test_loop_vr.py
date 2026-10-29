"""Offline data contract and operator mapping regressions; no ROS or recording."""
from copy import deepcopy
from contextlib import redirect_stderr
import io
import math
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch
from hardware.loop_clock import PCRecordingClock,ClockedRobotStreamer

from hardware.keyboard_controls import keyboard_aux
from hardware.loop_vr_contract import GROUPS, RecordingStatus, vr_parameters, recording_payload
from hardware.teleop_cli import parser, command


class LoopVRTests(unittest.TestCase):
    def test_hold_and_late_lift_keep_numeric_parquet_columns_fixed(self):
        prefix='ffw_sh5_vr.'
        payload={}
        for group,names in GROUPS.items():
            suffix='velocity' if group=='mobile' else 'joint_position'
            payload[prefix+f'observation.{group}.{suffix}']=[.1]*len(names)
            payload[prefix+f'action.{group}.{suffix}']=None
        pressure=prefix+'observation.right_hand.tactile_pressure'
        payload[pressure]=None
        initial=recording_payload(payload,'ffw_sh5_vr',dict(action_valid=[0.]*7))
        self.assertTrue(all(value is not None for value in initial.values()))
        self.assertNotIn(pressure,initial)
        self.assertEqual(initial[prefix+'action.lift.joint_position'],[.1])
        payload[pressure]=list(range(45))
        payload[prefix+'action.lift.joint_position']=[.3]
        active=recording_payload(payload,'ffw_sh5_vr',dict(action_valid=[1.]*7))
        self.assertEqual(set(initial),set(active))
        self.assertEqual({k:len(v) for k,v in initial.items()}, {k:len(v) for k,v in active.items()})
        self.assertEqual(active[prefix+'action.lift.joint_position'],[.3])
        held=recording_payload(payload,'ffw_sh5_vr',dict(action_valid=[0.]*7))
        self.assertEqual(held[prefix+'action.lift.joint_position'],[.1])
        self.assertNotIn(pressure,held)
        self.assertEqual(held[prefix+'teleop.action_valid'],[0.]*7)

    def test_missing_feedback_is_explicitly_invalid_and_never_zero_filled(self):
        prefix='ffw_sh5_vr.'
        payload=recording_payload({},'ffw_sh5_vr',dict(action_valid=[1.]*7))
        self.assertEqual(payload[prefix+'teleop.observation_valid'],[0.]*7)
        self.assertEqual(payload[prefix+'teleop.action_valid'],[0.]*7)
        self.assertTrue(all(math.isnan(v) for v in payload[prefix+'observation.left_hand.joint_position']))

    def test_robot_speed_flags_are_validated_and_forwarded(self):
        args=parser().parse_args(['robot','--pc-ip','192.168.6.56','--robot-ip','192.168.6.2',
                                 '--base-speed','.1','--turn-speed','.15'])
        remote=command(args,False)[-1]
        self.assertIn('--base-speed 0.1',remote)
        self.assertIn('--turn-speed 0.15',remote)
        # The host entry point runs the file directly, not python -m hardware.
        smoke=subprocess.run([sys.executable,str(Path(__file__).with_name('teleop_cli.py')),
            'robot','--pc-ip','192.168.6.56','--robot-ip','192.168.6.2',
            '--base-speed','.1','--turn-speed','.15','--dry-run'],
            text=True,capture_output=True,check=True)
        self.assertIn('--base-speed 0.1',smoke.stdout)
        for flag,value in (('--base-speed','nan'),('--base-speed','.3'),('--turn-speed','-1')):
            with self.assertRaises(SystemExit),redirect_stderr(io.StringIO()):
                parser().parse_args(['robot','--pc-ip','192.168.6.56','--robot-ip','192.168.6.2',flag,value])

    def test_camera_configuration_and_leader_layout_are_preserved(self):
        original={'/**':{'ros__parameters':{'ffw_sg2_rev1':{
            'camera_topic_list':['cam_head_left:/loop/left','cam_wrist_left:/camera/left'],
            'joint_order':{'leader_left':['arm_l_joint1','gripper_l_joint1']}},
            'jpeg_quality':75,'camera_rotation':'','head_camera_fps':15.}}}
        before=deepcopy(original)
        output=vr_parameters(original,loop_addr='192.168.5.2:50051',robot_ip='192.168.5.101')
        self.assertEqual(original,before)
        params=output['/**']['ros__parameters']
        self.assertEqual(params['ffw_sg2_rev1'],before['/**']['ros__parameters']['ffw_sg2_rev1'])
        self.assertEqual(params['head_camera_fps'],15.)
        self.assertEqual(params['ffw_sh5_vr']['camera_topic_list'],params['ffw_sg2_rev1']['camera_topic_list'])
        self.assertEqual(len(params['ffw_sh5_vr']['joint_order']['leader_left_arm']),7)
        self.assertEqual(len(params['ffw_sh5_vr']['joint_order']['leader_left_hand']),20)
        self.assertEqual(params['source_key'],'robotis-vr-sh5')

    def ready(self):
        state=RecordingStatus()
        state.status('gate',dict(model='sh5',gate_instance='g',status_sequence=1,
                     forwarding=True,feedback_waiting=False,fault=None),10.)
        state.status('intent',dict(model='sh5',session_id='pc',sequence=1,enabled=True,phase='ACTIVE'),10.)
        state.action_rx={name:10. for name in GROUPS}
        return state

    def test_hold_and_manual_motion_keep_a_complete_fixed_metadata_layout(self):
        state=self.ready()
        active=state.values(10.1)
        self.assertEqual(active['flags'],[1.,0.,0.,0.,1.])
        state.status('intent',dict(model='sh5',session_id='pc',sequence=2,enabled=False,phase='WAIT_START'),10.2)
        state.status('gate',dict(model='sh5',gate_instance='g',status_sequence=2,
                     forwarding=False,manual_active=True,fault=None),10.2)
        held=state.values(10.3)
        self.assertEqual(set(active),set(held))
        self.assertEqual(held['flags'],[0.,1.,1.,0.,1.])
        self.assertEqual(held['action_valid'][:4],[0.]*4)
        self.assertEqual(held['action_valid'][4:],[1.]*3)

    def test_stale_and_replayed_status_never_claim_active(self):
        state=self.ready()
        state.status('intent',dict(model='sh5',session_id='pc',sequence=1,enabled=True,phase='ACTIVE'),10.6)
        self.assertEqual(state.intent_rx,10.)
        stale=state.values(10.6)
        self.assertEqual(stale['flags'][0],0.)
        self.assertEqual(stale['action_valid'],[0.]*7)
        self.assertTrue(all(math.isfinite(x) for values in stale.values() for x in values))

    def test_actions_require_fresh_commands_independently_of_status(self):
        state=self.ready()
        state.action_rx['left_hand']=None
        self.assertEqual(state.values(10.1)['action_valid'][2],0.)
        self.assertEqual(state.values(10.1)['action_valid'][0],1.)

    def test_keyboard_axes_and_opposing_keys(self):
        limits={n:dict(lower=-1.,upper=1.,velocity=1.) for n in ('head_joint1','head_joint2','lift_joint')}
        measured={n:0. for n in limits}
        out=keyboard_aux({'w','a','up','left','u','o'},measured,limits,{},.05)
        self.assertAlmostEqual(math.hypot(*out['base'][:2]),.15)
        self.assertEqual(out['base'][2],.3)
        self.assertLess(out['joints']['head_joint1'],0.)
        self.assertGreater(out['joints']['head_joint2'],0.)
        self.assertGreater(out['joints']['lift_joint'],0.)
        self.assertEqual(keyboard_aux({'w','s','a','d','o','p'},measured,limits,{},.05)['base'],[0.,0.,0.])
        limits['lift_joint']['upper']=0.
        self.assertEqual(keyboard_aux({'u'},measured,limits,{},.05)['joints']['lift_joint'],0.)
        with self.assertRaises(ValueError):keyboard_aux({'unknown'},measured,limits,{},.05)

    def test_two_commands_quote_remote_arguments_and_keep_bringup_explicit(self):
        args=parser().parse_args(['robot','--pc-ip','192.168.5.2','--robot-ip','ffw-SNPR48A1114.local','--loop','vr'])
        cmd=command(args,False)
        self.assertEqual(cmd[0],'ssh')
        self.assertNotIn('--bringup',cmd[-1])
        self.assertIn('--loop-channels training',cmd[-1])
        args.bringup=True
        self.assertIn('--bringup',command(args,False)[-1])
        args.loop_channels='qpos'
        args.loop_no_tactile=True
        remote=command(args,False)[-1]
        self.assertIn('--loop-channels qpos',remote)
        self.assertIn('--loop-no-tactile',remote)
        with self.assertRaises(SystemExit),redirect_stderr(io.StringIO()):
            parser().parse_args(['pc','--pc-ip','192.168.5.2','--robot-ip','x;touch /tmp/unsafe'])

    def test_pc_clock_epoch_is_used_even_when_robot_wall_clock_is_months_away(self):
        clock=PCRecordingClock()
        self.assertIsNone(clock.timestamp(10.))
        epoch=1792660000000000
        clock.receive(epoch,10.)
        self.assertAlmostEqual(clock.timestamp(10.05),epoch+50000,delta=1)
        previous=clock.timestamp(10.05)
        self.assertGreater(clock.timestamp(10.04),previous)
        with self.assertRaises(ValueError):clock.receive(epoch+5000000,10.1)
        class Producer:
            def send_robot(self,*args):self.received=args
        streamer=ClockedRobotStreamer('unused',None,clock)
        streamer.producer=Producer()
        with patch('hardware.loop_clock.time.monotonic',return_value=10.1):
            streamer.send({'state':[.1],'missing':None})
        timestamp,sequence,payload=streamer.producer.received
        self.assertAlmostEqual(timestamp,epoch+100000,delta=1)
        self.assertEqual(sequence,0)
        self.assertIsNone(payload['missing'])


if __name__=='__main__':unittest.main()
