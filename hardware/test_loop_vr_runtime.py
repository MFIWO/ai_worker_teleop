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
        from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
        from loop_sdk import SourceProducer
        from hardware.loop_vr_contract import GROUPS,TOPICS
        from hardware.loop_vr_streamer import main
        frames=[]
        hand_frames=[]
        schemas=[]
        class Sink:
            def __init__(self, target):self.target=target
            def stats(self):return SimpleNamespace(connected=True)
            def send_robot(self,stamp,sequence,payload):self.target.append((stamp,sequence,payload))
            def send_tactile(self,*args):raise AssertionError('fixture has no pressure; do not invent samples')
            def close(self):pass
        def connect(addr,schema,**kwargs):
            schemas.append(schema)
            return Sink(hand_frames if schema.source_key.endswith('-hand') else frames)
        epoch=1792660000000000
        def spin(streamer):
            fixture=Node('loop_contract_fixture')
            status=fixture.create_publisher(String,'/xr_hardware/gate_status',10)
            intent=fixture.create_publisher(String,'/xr_hardware/intent',10)
            feedback=fixture.create_publisher(JointState,'/joint_states',10)
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
                    intent.publish(String(data=json.dumps(dict(model='sh5',session_id='fixture',sequence=seq,
                        enabled=active,phase='ACTIVE' if active else 'WAIT_START',
                        recording_epoch_us=epoch+int(elapsed*1e6)))))
                    status.publish(String(data=json.dumps(dict(model='sh5',gate_instance='fixture',
                        status_sequence=seq,enabled=active,forwarding=active,manual_active=manual,fault=None))))
                    if active:
                        for name,pub in commands.items():
                            pub.publish(JointTrajectory(joint_names=list(GROUPS[name]),
                                points=[JointTrajectoryPoint(positions=[.2]*len(GROUPS[name]))]))
                    for _ in range(6):rclpy.spin_once(streamer,timeout_sec=0.)
                    time.sleep(.01)
            finally:fixture.destroy_node()
        with patch.object(SourceProducer,'connect',side_effect=connect),patch.object(rclpy,'spin',side_effect=spin),\
             patch('sys.argv',['loop-vr','--loop-addr','unused:50051','--robot-ip','127.0.0.1','--no-camera']):
            main()
        self.assertEqual(len(schemas),3)
        self.assertEqual([s.kind.value for s in schemas],['robot','robot','tactile'])
        self.assertEqual(schemas[2].tactile.name,'HX5-D20')
        self.assertEqual([(f[0],f[1]) for f in frames],[(f[0],f[1]) for f in hand_frames])
        self.assertTrue(all(not any('.left_hand.' in k or '.right_hand.' in k for k in f[2]) for f in frames))
        self.assertTrue(all(not any('.left_arm.' in k or '.mobile.' in k for k in f[2]) for f in hand_frames))
        self.assertGreater(len(frames),50)
        prefix='ffw_sh5_vr.'
        active=[f for f in frames if f[2][prefix+'teleop.flags'][0]]
        held=[f for f in frames if f[2][prefix+'teleop.flags'][1] and f[2][prefix+'teleop.flags'][4]]
        self.assertTrue(active)
        self.assertGreater(len(held),10)
        self.assertTrue(any(f[2][prefix+'teleop.flags'][2] for f in held))
        self.assertTrue(any(f[2][prefix+'teleop.action_fresh'][0]==0 for f in held))
        self.assertTrue(all(f[2][prefix+'teleop.action_valid'][0]==0 for f in held))
        for stamp,_,payload in held:
            self.assertEqual(payload[prefix+'observation.left_arm.joint_position'],[.1]*7)
            if payload[prefix+'action.left_arm.joint_position'] is not None:
                self.assertEqual(payload[prefix+'action.left_arm.joint_position'],[.2]*7)
            self.assertLess(abs(stamp-epoch),10000000)
        stamps=[f[0] for f in frames]
        self.assertTrue(all(a<b for a,b in zip(stamps,stamps[1:])))


if __name__=='__main__':unittest.main()
