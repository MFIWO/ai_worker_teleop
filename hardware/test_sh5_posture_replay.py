"""Isolated SH5 replay against compiled native QP; never a physical follower.
Requires validation/domain232/localhost/FastDDS. Uses only recorded goal/lift
inputs and a synthetic first-order joint follower; not a motor certification.
"""
import argparse,bisect,json,os,subprocess,time
from pathlib import Path

def main():
    expected={'ROS_DOMAIN_ID':'232','ROS_LOCALHOST_ONLY':'1','AI_WORKER_XR_EXECUTION':'validation','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp'}
    if any(os.environ.get(k)!=v for k,v in expected.items()):raise SystemExit('isolated validation environment required')
    import numpy as np,pinocchio as pin,rclpy,rosbag2_py,yaml
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from geometry_msgs.msg import PoseStamped
    from trajectory_msgs.msg import JointTrajectory
    from hardware.arm_posture import apply_limits
    from hardware.xr_contract import arm_limits
    from hardware.sh5_hands_model import scalar_sensor_position
    parser=argparse.ArgumentParser();parser.add_argument('--profile',required=True);parser.add_argument('--bag',required=True);parser.add_argument('--binary',required=True);parser.add_argument('--output',required=True);parser.add_argument('--urdf',help='optional collision-covered replay URDF; defaults to the profile model');a=parser.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True);profile=json.loads(Path(a.profile).read_text())
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=a.bag,storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    types={x.name:get_message(x.type) for x in reader.get_all_topics_and_types()};rows={k:[] for k in ['/joint_states','/l_goal_pose','/r_goal_pose','/xr_hardware/body_elbows_relaxed']};t0=None
    while reader.has_next():
        topic,b,ns=reader.read_next()
        if t0 is None:t0=ns/1e9
        if topic in rows:
            msg=deserialize_message(b,types[topic]);value=dict(zip(msg.name,msg.position)) if topic=='/joint_states' else msg
            rows[topic].append((ns/1e9-t0,value))
    times={k:[r[0] for r in v] for k,v in rows.items()}
    def sample(topic,t):return rows[topic][max(0,bisect.bisect_right(times[topic],t)-1)][1]
    urdf,limits=apply_limits(Path(a.urdf or profile['urdf']).read_text(),arm_limits(Path(profile['urdf']).read_text())[0]);(out/'model.urdf').write_text(urdf)
    cfg=yaml.safe_load(Path(profile['config']).read_text());params=cfg['vr_controller']['ros__parameters'];params.update(urdf_path=str(out/'model.urdf'),srdf_path=profile['srdf'],weight_elbow_position=.1,activation_delay_seconds=0.,startup_ramp_seconds=0.)
    (out/'controller.yaml').write_text(yaml.safe_dump(cfg))
    model=pin.buildModelFromUrdf(str(out/'model.urdf'));data=model.createData();joints=dict(sample('/joint_states',26.5));target=dict(joints);q=pin.neutral(model)
    rclpy.init();node=rclpy.create_node('isolated_sh5_posture_replay')
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    pub=node.create_publisher(JointState,'/xr_hardware/sh5/joint_states',10);enable=node.create_publisher(Bool,'/reactivate',10);body=node.create_publisher(Bool,'/xr_hardware/body_elbows_relaxed',1)
    goals={s:node.create_publisher(PoseStamped,'/'+s+'_goal_pose',qos) for s in ['l','r']};elbows={s:node.create_publisher(PoseStamped,'/'+s+'_elbow_pose',qos) for s in ['l','r']}
    commands=[];violations=[];errors={s:[] for s in ('l','r')}
    def receive(msg):
        values=dict(zip(msg.joint_names,msg.points[0].positions));commands.append((time.monotonic(),values))
        for name,v in values.items():
            if not limits[name][0]-1e-5<=v<=limits[name][1]+1e-5:violations.append((name,v))
        target.update(values)
    subs=[node.create_subscription(JointTrajectory,'/xr_hardware/sh5/'+s+'/joint_trajectory',receive,10) for s in ['left','right']]
    log=(out/'native.log').open('w');process=subprocess.Popen([a.binary,'--ros-args','--params-file',str(out/'controller.yaml')],stdout=log,stderr=subprocess.STDOUT)
    start=time.monotonic();last=start;last_pub=0.;records=[];activated=False
    try:
        while time.monotonic()-start<34.5:
            now=time.monotonic();elapsed=now-start;dt=min(.03,now-last);last=now;record_t=min(57.4,26.5+max(0.,elapsed-3.))
            for name in joints:
                if name.startswith('arm_') and '_joint' in name:joints[name]+=(target[name]-joints[name])*min(1.,dt/.05)
            measured=sample('/joint_states',record_t)
            for name in joints:
                if not name.startswith('arm_'):joints[name]=measured[name]
            if now-last_pub>=.01:
                last_pub=now;state=JointState();state.header.stamp=node.get_clock().now().to_msg();state.name=list(joints);state.position=[scalar_sensor_position(n,joints[n]) for n in state.name];state.velocity=[0.]*len(joints);pub.publish(state)
                for s,publisher in goals.items():publisher.publish(sample('/'+s+'_goal_pose',record_t))
                for i in range(1,model.njoints):q[model.joints[i].idx_q]=scalar_sensor_position(model.names[i],joints[model.names[i]])
                pin.forwardKinematics(model,data,q);pin.updateFramePlacements(model,data)
                if elapsed>3.:
                    for side in ('l','r'):
                        goal=sample('/'+side+'_goal_pose',record_t).pose.position
                        actual=data.oMf[model.getFrameId('arm_'+side+'_link7')].translation
                        errors[side].append(float(np.linalg.norm(actual-np.array([goal.x,goal.y,goal.z]))))
                for s,publisher in elbows.items():
                    msg=PoseStamped();msg.header.frame_id='base_link';xyz=data.oMf[model.getFrameId('arm_'+s+'_link4')].translation;msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=map(float,xyz);msg.pose.orientation.w=1.;publisher.publish(msg)
                body.publish(sample('/xr_hardware/body_elbows_relaxed',record_t))
                if elapsed>2. and not activated:enable.publish(Bool(data=True));activated=True
                records.append({'t':elapsed,'lift':joints['lift_joint'],'l4':joints['arm_l_joint4'],'r4':joints['arm_r_joint4']})
            rclpy.spin_once(node,timeout_sec=.001)
            if process.poll() is not None:raise RuntimeError('native replay node exited')
        summary={'synthetic_only':True,'output_count':len(commands),'violations':violations,
                 'left_elbow_range_rad':[min(x['l4'] for x in records),max(x['l4'] for x in records)],
                 'right_elbow_range_rad':[min(x['r4'] for x in records),max(x['r4'] for x in records)],
                 'wrist_position_error_m':{s:np.quantile(v,[.5,.95,1]).tolist() for s,v in errors.items()},
                 'max_command_gap_s':max((b[0]-a[0] for a,b in zip(commands,commands[1:])),default=0.)}
        (out/'summary.json').write_text(json.dumps(summary,indent=2));(out/'joints.json').write_text(json.dumps(records));print(json.dumps(summary),flush=True)
        if len(commands)<100 or violations:raise RuntimeError('posture replay failed')
    finally:
        enable.publish(Bool(data=False));process.terminate()
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
