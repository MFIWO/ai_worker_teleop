"""TEST ONLY: DDS simulator state -> ROS publisher + SYNTHETIC DXL health.

Hardcoded validation/domain232. This does not read motors or certify hardware.
Never installed by install_robot.sh.
"""
import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from dynamixel_interfaces.msg import DynamixelState
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory
from hardware.xr_contract import ARM_DXL_IDS


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '232' or os.environ.get('AI_WORKER_XR_EXECUTION') != 'validation':
        raise SystemExit('SYNTHETIC driver fixture: validation/domain232 ONLY')
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--description',required=True)
    p.add_argument('--kinematic-profile',help='TEST ONLY: model FK follower, NOT PhysX or motor hardware')
    p.add_argument('--snapshot');a=p.parse_args()
    rclpy.init(args=[])
    n=Node('synthetic_driver_fixture')
    pub=n.create_publisher(JointState,'/joint_states',10)
    health=n.create_publisher(DynamixelState,'/ffw_follower/dxl_state',10)
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    desc=n.create_publisher(String,'/robot_description',latched)
    desc.publish(String(data=Path(a.description).read_text()))
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    sub=n.create_subscription(JointState,'/validation_sim/joint_states',pub.publish,qos)
    kinematic_timer=None;target_subs=[]
    if a.kinematic_profile:
        import pinocchio as pin
        import numpy as np
        if not a.snapshot:raise ValueError('kinematic fixture requires an explicit synthetic joint snapshot')
        profile=json.loads(Path(a.kinematic_profile).read_text())
        if not profile.get('validation_fixture'):raise ValueError('kinematic fixture needs validation-marked profile')
        model=pin.buildModelFromUrdf(profile['urdf']);data=model.createData()
        joints=json.loads(Path(a.snapshot).read_text());targets=dict(joints)
        tf_pub=n.create_publisher(TFMessage,'/tf',10)
        def receive(msg):
            if msg.points:
                targets.update(dict(zip(msg.joint_names,msg.points[-1].positions)))
        from hardware.xr_contract import FOLLOWER_TOPICS
        target_subs=[n.create_subscription(JointTrajectory,t,receive,10) for t in FOLLOWER_TOPICS.values()]
        def sample():
            for name in joints:
                delta=targets[name]-joints[name]
                joints[name]+=float(np.clip(delta,-.35*.02,.35*.02))
            stamp=n.get_clock().now().to_msg()
            state=JointState();state.header.stamp=stamp;state.name=list(joints);state.position=list(joints.values());state.velocity=[0.]*len(joints)
            pub.publish(state)
            q=pin.neutral(model)
            for i in range(1,model.njoints):q[model.joints[i].idx_q]=joints[model.names[i]]
            pin.forwardKinematics(model,data,q);pin.updateFramePlacements(model,data)
            base=data.oMf[model.getFrameId('base_link')];frames=[]
            for i,f in enumerate(model.frames):
                if f.type!=pin.FrameType.BODY or f.name=='base_link':continue
                pose=base.inverse()*data.oMf[i];rotation=pin.Quaternion(pose.rotation)
                msg=TransformStamped();msg.header.stamp=stamp;msg.header.frame_id='base_link';msg.child_frame_id=f.name
                msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=map(float,pose.translation)
                msg.transform.rotation.x=float(rotation.x);msg.transform.rotation.y=float(rotation.y)
                msg.transform.rotation.z=float(rotation.z);msg.transform.rotation.w=float(rotation.w)
                frames.append(msg)
            tf_pub.publish(TFMessage(transforms=frames))
        kinematic_timer=n.create_timer(.02,sample)
        print('[KINEMATIC FIXTURE] FK-only follower; not independent physics validation',flush=True)
    def tick():
        m=DynamixelState();m.header.stamp=n.get_clock().now().to_msg()
        m.id=ARM_DXL_IDS['right']+ARM_DXL_IDS['left'];m.torque_state=[True]*14;m.dxl_hw_state=[0]*14;m.comm_state=0
        health.publish(m)
    timer=n.create_timer(.05,tick)
    print('[SYNTHETIC] driver health is fabricated for isolated integration test, NOT real hardware',flush=True)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
