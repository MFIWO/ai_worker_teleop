"""[ADDED] Real Cyclo/PhysX, synthetic controller input: line out/back + overhead.

Never publishes JointTrajectory. Fixture FK supplies only reachable Cartesian
endpoints; actual C++ QP selects joints. Domain232 only. Records raw, workspace,
published and measured traces; does not claim real Quest straightness evidence.
"""
import argparse
import asyncio
from copy import deepcopy
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from scipy.spatial.transform import Rotation as R

from robotis_vuer.vr_publisher_sg2_xr_overhead import XROverheadPublisher
from robotis_vuer.xr_calibration_overlay import controller_pose_for_robot
from xr_shoulder_smoke import FixtureFK
from xr_resume_smoke import signature


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='232':
        raise SystemExit('isolated simulation domain232 required')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture-urdf',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    fk=FixtureFK(args.fixture_urdf)
    rclpy.init()
    node=XROverheadPublisher(start_server=False)
    executor=SingleThreadedExecutor(); executor.add_node(node)
    loop=asyncio.new_event_loop()
    head=np.eye(4); head[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]; head[:3,3]=[0,1.6,0]
    inputs={s:np.eye(4) for s in ('left','right')}
    inputs['left'][:3,3]=[-.2,1.1,-.3]; inputs['right'][:3,3]=[.2,1.1,-.3]
    rows=[]; checks={}; metrics={}; stage='startup'; begin=time.monotonic()

    def step(a=False,b=False):
        for _ in range(40): executor.spin_once(timeout_sec=0.)
        body=np.tile(np.eye(4).flatten(order='F'),7); body[96:112]=head.flatten(order='F')
        loop.run_until_complete(node.on_body_tracking_move(SimpleNamespace(value={'body':body}),None))
        buttons={'aButton':a,'bButton':b}
        loop.run_until_complete(node.on_controller_move(SimpleNamespace(value={
            'left':inputs['left'].flatten(order='F').tolist(),
            'right':inputs['right'].flatten(order='F').tolist(),
            'leftState':buttons,'rightState':buttons}),None))
        for _ in range(40): executor.spin_once(timeout_sec=0.)
        row={'t':time.monotonic()-begin,'stage':stage,'phase':node._phase,
             'reason':node._last_reason,'signature':signature(node._session) if node._session else None}
        if node._path_frame is not None: row['path']=deepcopy(node._path_frame)
        if node.current_joint_states is not None:
            row['joints']=dict(zip(node.current_joint_states.name,node.current_joint_states.position))
        rows.append(row); time.sleep(.02)

    def wait(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end: step()

    def press(a=False,b=False): step();step(a,b);step()

    def ready():
        end=time.monotonic()+19
        while time.monotonic()<end and node._phase!='ACTIVE':
            step()
            if node._phase not in ('SETTLING','WARMUP','ACTIVE'):
                raise RuntimeError('startup failed: '+str(node._last_reason))
        if node._phase!='ACTIVE': raise RuntimeError('startup timeout')

    def move(start,end,duration):
        t0=time.monotonic()
        while time.monotonic()-t0<duration:
            u=min(1,(time.monotonic()-t0)/(duration-.2)); u=u*u*(3-2*u)
            for side in inputs:
                pose=start[side].copy()
                pose[:3,3]+=u*(end[side][:3,3]-start[side][:3,3])
                rv=R.from_matrix(end[side][:3,:3]@start[side][:3,:3].T).as_rotvec()
                pose[:3,:3]=R.from_rotvec(u*rv).as_matrix()@start[side][:3,:3]
                inputs[side]=controller_pose_for_robot(node._session.mapper,side,pose)
            step()
            if node._phase!='ACTIVE': raise RuntimeError(stage+': '+str(node._last_reason))

    error=None
    try:
        limit=time.monotonic()+12
        while time.monotonic()<limit:
            step()
            if node._joint_rx is not None and len(node._robot_poses)==2 and node._profile_verified: break
        wait(1);press(a=True);ready()
        original=signature(node._session)
        base=deepcopy(node._goals)
        before_q=dict(zip(node.current_joint_states.name,node.current_joint_states.position))
        out=deepcopy(base)
        for p in out.values(): p[0,3]+=.10
        stage='line_out';move(base,out,4);wait(1)
        stage='line_back';move(out,base,4);wait(2)
        line_rows=[r for r in rows if r['stage'] in ('line_out','line_back') and 'path' in r]
        for side in inputs:
            for kind in ('raw','published','actual'):
                v=np.array([r['path'][side][kind+'_pose_base'] for r in line_rows])[:,:3,3]
                metrics[f'{side}_{kind}_line_cross_track_mm']=float(np.max(np.linalg.norm(v[:,1:]-base[side][None,1:3,3],axis=1))*1000)
            p=node._snapshot()[side]['wrist']
            metrics[side+'_return_position_error_mm']=float(np.linalg.norm(p[:3,3]-base[side][:3,3])*1000)
        now_q=dict(zip(node.current_joint_states.name,node.current_joint_states.position))
        metrics['max_joint_return_difference_deg']=float(np.degrees(max(abs(now_q[n]-before_q[n])
                    for n in before_q if n.startswith('arm_') and '_joint' in n)))
        checks['raw_and_goal_lines_straight']=all(metrics[s+'_published_line_cross_track_mm']<.01 for s in inputs)
        checks['measured_line_within_10mm']=all(metrics[s+'_actual_line_cross_track_mm']<10 for s in inputs)
        checks['return_within_5mm']=all(metrics[s+'_return_position_error_mm']<5 for s in inputs)
        print('[LINE]',json.dumps(metrics),flush=True)

        over=deepcopy(base)
        for side,short,sign in (('left','l',1),('right','r',-1)):
            q0=[0,0,0,-1.57,0,0,0]; q1=[-2.,sign*.10,0,-.9,0,0,0]
            t0,t1=fk.arm(short,q0),fk.arm(short,q1)
            over[side][:3,3]+=t1[:3,3]-t0[:3,3]
            over[side][:3,:3]=t1[:3,:3]@t0[:3,:3].T@base[side][:3,:3]
        stage='overhead';move(base,over,10);wait(3)
        for side in inputs:
            p=node._snapshot()[side]['wrist']
            metrics[side+'_overhead_z_m']=float(p[2,3])
            metrics[side+'_overhead_delta_m']=float(np.linalg.norm(over[side][:3,3]-base[side][:3,3]))
            metrics[side+'_overhead_error_mm']=float(np.linalg.norm(p[:3,3]-node._goals[side][:3,3])*1000)
        checks['overhead_no_half_meter_stop']=node._phase=='ACTIVE' and all(metrics[s+'_overhead_delta_m']>.5 for s in inputs)
        checks['both_wrists_above_1_8m']=all(metrics[s+'_overhead_z_m']>1.8 for s in inputs)
        checks['overhead_within_20mm']=all(metrics[s+'_overhead_error_mm']<20 for s in inputs)
        print('[OVERHEAD]',json.dumps(metrics),flush=True)
        stage='overhead_return';move(over,base,10);wait(2)
        checks['original_mapping_unchanged']=signature(node._session)==original
        stage='pause';press(b=True);wait(1)
        for side in inputs: inputs[side][2,3]-=.08
        wait(.6);before=node.published_pairs;press(a=True)
        checks['misaligned_resume_refused']=not node.arm_control_enabled
        checks['no_publish_on_refusal']=node.published_pairs==before
        links=node._snapshot()
        for side in inputs: inputs[side]=controller_pose_for_robot(node._session.mapper,side,links[side]['wrist'])
        wait(.8);stage='resume';press(a=True);ready()
        checks['aligned_resume_works']=node._phase=='ACTIVE'
        checks['mapping_after_resume_unchanged']=signature(node._session)==original
        stage='done';press(b=True);checks['manual_stop']=not node.arm_control_enabled
    except Exception as exc:
        error=repr(exc)
    finally:
        node._set_arm_control_enabled(False,'overhead test cleanup')
        for _ in range(10): executor.spin_once(timeout_sec=0.)
        node.destroy_node(); executor.shutdown(); rclpy.shutdown(); loop.close()
    result={'passed':error is None and len(checks)>=10 and all(checks.values()),
            'error':error,'checks':checks,'metrics':metrics,'rows':rows,'synthetic_input':True,'domain':232}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2))
    print('[RESULT]',json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
    raise SystemExit(0 if result['passed'] else 2)


if __name__=='__main__': main()
