"""Isolated real ROS callbacks + actual SDK schema, with a fake recording sink."""
import os
from types import SimpleNamespace
import unittest
from unittest.mock import patch


class LoopRuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        expected={'ROS_DOMAIN_ID':'232','ROS_LOCALHOST_ONLY':'1','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp'}
        if any(os.environ.get(k)!=v for k,v in expected.items()):
            raise unittest.SkipTest('requires isolated localhost/domain232/FastDDS')

    def test_hold_does_not_interrupt_samples_or_turn_cached_actions_into_valid_actions(self):
        import json,time
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from sensor_msgs.msg import JointState
        from control_msgs.msg import DynamicJointState, InterfaceValue
        from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
        from loop_sdk import SourceProducer, RobotFrame
        from loop_sdk.source.outbound.proto_mapping import _sample, frame_from_sample
        from hardware.loop_vr_contract import GROUPS,TOPICS
        from hardware.hx5_tactile import SENSOR_NAMES, PRESSURE_NAMES
        from hardware.loop_vr_streamer import main
        frames=[]
        tactile_frames=[]
        schemas=[]
        class Sink:
            def __init__(self, kind):self.kind=kind
            def stats(self):return SimpleNamespace(connected=True)
            def send_robot(self,stamp,sequence,payload):
                assert self.kind=='robot'
                # Use the installed SDK's real protobuf codec: None would be
                # omitted here and reproduce the observed Parquet column drift.
                sample=_sample(RobotFrame(timestamp_us=stamp,sequence=sequence,state=payload))
                wire=type(sample).FromString(sample.SerializeToString())
                decoded=frame_from_sample('fixture',wire)
                frames.append((stamp,sequence,decoded.state))
            def send_tactile(self,*args):
                assert self.kind=='tactile'
                tactile_frames.append(args)
            def close(self):pass
        def connect(addr,schema,**kwargs):
            schemas.append(schema)
            return Sink(schema.kind.value)
        epoch=1792660000000000
        def spin(streamer):
            fixture=Node('loop_contract_fixture')
            status=fixture.create_publisher(String,'/xr_hardware/gate_status',10)
            intent=fixture.create_publisher(String,'/xr_hardware/intent',10)
            feedback=fixture.create_publisher(JointState,'/joint_states',10)
            pressure=fixture.create_publisher(DynamicJointState,'/dynamic_joint_states',10)
            commands={n:fixture.create_publisher(JointTrajectory,t,10) for n,t in TOPICS.items() if n!='mobile'}
            begin=time.monotonic()
            seq=0
            try:
                while time.monotonic()-begin<4.5:
                    now=time.monotonic();elapsed=now-begin;seq+=1
                    active=elapsed<1.3 or elapsed>3.6
                    manual=2.6<elapsed<3.6
                    names=[n for group,names in GROUPS.items() if group!='mobile' for n in names]
                    feedback.publish(JointState(name=names,position=[.1]*len(names)))
                    if .7<elapsed<3.6:
                        sensor_names=[name for side in ('left','right') for name in SENSOR_NAMES[side]]
                        msg=DynamicJointState(joint_names=sensor_names,interface_values=[
                            InterfaceValue(interface_names=list(PRESSURE_NAMES),values=[float(v) for v in range(9)])
                            for _ in sensor_names])
                        msg.header.stamp=fixture.get_clock().now().to_msg()
                        pressure.publish(msg)
                    intent.publish(String(data=json.dumps(dict(model='sh5',session_id='fixture',sequence=seq,
                        enabled=active,phase='ACTIVE' if active else 'WAIT_START',
                        recording_epoch_us=epoch+int(elapsed*1e6)))))
                    status.publish(String(data=json.dumps(dict(model='sh5',gate_instance='fixture',
                        status_sequence=seq,enabled=active,forwarding=active,manual_active=manual,fault=None))))
                    if active:
                        for name,pub in commands.items():
                            if name in ('head','lift') and elapsed<3.6:continue
                            pub.publish(JointTrajectory(joint_names=list(GROUPS[name]),
                                points=[JointTrajectoryPoint(positions=[.2]*len(GROUPS[name]))]))
                    for _ in range(6):rclpy.spin_once(streamer,timeout_sec=0.)
                    time.sleep(.01)
            finally:fixture.destroy_node()
        with patch.object(SourceProducer,'connect',side_effect=connect),patch.object(rclpy,'spin',side_effect=spin),\
             patch('sys.argv',['loop-vr','--loop-addr','unused:50051','--robot-ip','127.0.0.1','--no-camera']):
            main()
        self.assertEqual(len(schemas),2)
        # CHANGED(2026-10-28): one Robot State, typed Tactile; no fake pressure.
        self.assertEqual([s.kind.value for s in schemas],['robot','tactile'])
        self.assertEqual(schemas[0].robot.name,'aiworker')
        self.assertGreater(len(tactile_frames),3)
        self.assertEqual(tactile_frames[0][2][0].raw,bytes(list(range(9))*5))
        declared={c.key for c in schemas[0].robot.channels}
        self.assertIn('ffw_sh5_vr.observation.left_hand.joint_position',declared)
        self.assertIn('ffw_sh5_vr.observation.left_arm.joint_position',declared)
        self.assertIn('ffw_sh5_vr.observation.right_hand.tactile_pressure',declared)
        self.assertIn('ffw_sh5_vr.observation.left_hand.tactile_status',declared)
        self.assertGreater(len(frames),50)
        prefix='ffw_sh5_vr.'
        active=[f for f in frames if f[2][prefix+'teleop.flags'][0]]
        held=[f for f in frames if f[2][prefix+'teleop.flags'][1] and f[2][prefix+'teleop.flags'][4]]
        self.assertTrue(active)
        self.assertGreater(len(held),10)
        self.assertTrue(any(f[2][prefix+'teleop.flags'][2] for f in held))
        self.assertNotIn(prefix+'teleop.action_fresh',frames[0][2])
        self.assertTrue(all(f[2][prefix+'teleop.action_valid'][0]==0 for f in held))
        for stamp,_,payload in held:
            self.assertEqual(payload[prefix+'observation.left_arm.joint_position'],[.1]*7)
            self.assertEqual(payload[prefix+'observation.left_hand.joint_position'],[.1]*20)
            if payload[prefix+'action.left_arm.joint_position'] is not None:
                self.assertEqual(payload[prefix+'action.left_arm.joint_position'],[.1]*7)
            self.assertLess(abs(stamp-epoch),10000000)
        stamps=[f[0] for f in frames]
        self.assertTrue(all(a<b for a,b in zip(stamps,stamps[1:])))
        # Every declared field exists from the first tick, before lift/pressure arrive.
        self.assertTrue(all(set(frame[2])==declared for frame in frames))
        self.assertTrue(all(all(v is not None for v in frame[2].values()) for frame in frames))
        widths={key:len(value) for key,value in frames[0][2].items()}
        self.assertTrue(all({k:len(v) for k,v in frame[2].items()}==widths for frame in frames))
        self.assertEqual(frames[0][2][prefix+'action.lift.joint_position'],[.1])
        self.assertTrue(any(f[2][prefix+'observation.right_hand.tactile_pressure']==list(range(9))*5 for f in held))


if __name__=='__main__':unittest.main()
