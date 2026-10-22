"""Native ROBOTIS SH5 21-landmark -> D20 retarget -> PhysX transport probe.

Run in ai_worker Docker on isolated ROS domain 232 together with its stock
retargeting_teleop node. Synthetic wrist-local landmarks (not live Quest).
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Point32
from robotis_interfaces.msg import HandJoints
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def landmarks(spread=1., curl=0.):
    # 21 points, standard wrist, thumb (4), index/middle/ring/little (4 each).
    points = [[0., 0., 0.]]
    points += [[.025,-.025,.004],[.046,-.035,.012],[.065,-.044,.021],[.081,-.047,.027]]
    for y in (-.032,-.011,.011,.031):
        for i in range(4):
            z = -.002 + .018*i + .015*curl*(i/3.)
            points.append([.023 + .002*i,y*spread,z])
    return [Point32(x=float(a),y=float(b),z=float(c)) for a,b,c in points]


def main():
    import os
    if os.environ.get('ROS_DOMAIN_ID')!='232' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':
        raise SystemExit('Requires simulator-only FastDDS domain 232')
    rclpy.init()
    node=Node('sh5_native_hand_optical_probe')
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    pubs={s:node.create_publisher(HandJoints,f'/{s}_hand/hand_joint_pos',qos) for s in ('left','right')}
    output={}; joint_feedback={}; seen={}
    for s in ('left','right'):
        topic=f'/leader/joint_trajectory_command_broadcaster_{s}_hand/joint_trajectory'
        def cb(m,s=s):
            if m.points: output[s]=dict(zip(m.joint_names,m.points[-1].positions));seen[s]=time.monotonic()
        node.create_subscription(JointTrajectory,topic,cb,qos)
    def state(m):
        if len(m.name)==len(m.position): joint_feedback.update(zip(m.name,m.position))
    node.create_subscription(JointState,'/joint_states',state,qos)
    def run(t,spread,curl):
        end=time.monotonic()+t
        while time.monotonic()<end:
            for s,p in pubs.items():
                m=HandJoints();m.header.stamp=node.get_clock().now().to_msg();m.joints=landmarks(spread,curl);p.publish(m)
            rclpy.spin_once(node,timeout_sec=.05)
    try:
        run(3.,1.,0.)
        baseline={s:dict(output.get(s,{})) for s in pubs}
        feedback_before=dict(joint_feedback)
        run(3.,1.35,.45)
        end={s:dict(output.get(s,{})) for s in pubs}
        changes={s:max((abs(v-baseline[s].get(k,v)) for k,v in end[s].items()),default=0.) for s in pubs}
        feedback={s:sum(1 for k in end[s] if k in joint_feedback) for s in pubs}
        sim_moves={s:max((abs(float(joint_feedback[k])-float(feedback_before[k])) for k in end[s]
                          if k in joint_feedback and k in feedback_before),default=0.) for s in pubs}
        ok=all(len(end[s])==20 and changes[s]>1e-3 and feedback[s]==20 and sim_moves[s]>1e-3 for s in pubs)
        print('SH5_NATIVE_HAND_RETARGET_SIM '+json.dumps({'native_retarget_topics':{s:len(end[s]) for s in pubs},'changes_rad':changes,'sim_feedback_finger_joints':feedback,'sim_measured_motion_rad':sim_moves,'passed':ok,'note':'synthetic optical landmarks; no swept mesh collision certification'}),flush=True)
    finally:
        run(1.,1.,0.)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
