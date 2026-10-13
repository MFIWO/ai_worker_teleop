"""[ADDED] Same synthetic wrist fixture -> real Cyclo/PhysX baseline/extension.

No direct joint commands: fixture FK only generates Cartesian WebXR input.
Measures both definitions of elbow bend and fixed-calibration resume.
"""
import argparse
import asyncio
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from scipy.spatial.transform import Rotation as R

from robotis_vuer.vr_publisher_sg2_xr_resume import XRResumePublisher
from robotis_vuer.vr_publisher_sg2_xr_extension import XRExtensionPublisher
from robotis_vuer.xr_extension_math import bend_degrees
from xr_shoulder_smoke import FixtureFK
from xr_resume_smoke import signature


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '232':
        raise SystemExit('Requires isolated simulation domain232')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['baseline','extension'], required=True)
    parser.add_argument('--fixture-urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--startup-drift-test', action='store_true',
                        help='Reproduce a25mm early hand movement, then manually align/confirm.')
    args = parser.parse_args()
    fk = FixtureFK(args.fixture_urdf)
    rclpy.init()
    node = (XRExtensionPublisher if args.mode == 'extension' else XRResumePublisher)(start_server=False)
    executor = SingleThreadedExecutor(); executor.add_node(node)
    loop = asyncio.new_event_loop()
    head = np.eye(4); head[:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]; head[:3,3] = [0,1.6,0]
    inputs = {s:np.eye(4) for s in ('left','right')}
    inputs['left'][:3,3], inputs['right'][:3,3] = [-.2,1.1,-.3],[.2,1.1,-.3]
    rows, checks, metrics = [], {}, {}
    begun = time.monotonic()
    stage = 'startup'
    def step(a=False,b=False):
        for _ in range(50): executor.spin_once(timeout_sec=0.)
        body = np.tile(np.eye(4).flatten(order='F'),7); body[96:112]=head.flatten(order='F')
        loop.run_until_complete(node.on_body_tracking_move(SimpleNamespace(value={'body':body}),None))
        state = {'aButton':a,'bButton':b}
        loop.run_until_complete(node.on_controller_move(SimpleNamespace(value={
            'left':inputs['left'].flatten(order='F').tolist(),
            'right':inputs['right'].flatten(order='F').tolist(),
            'leftState':state,'rightState':state}),None))
        for _ in range(50): executor.spin_once(timeout_sec=0.)
        row={'t':time.monotonic()-begun, 'stage':stage, 'phase':node._phase,
             'active':node.arm_control_enabled, 'reason':node._last_reason,
             'session_signature':signature(node._session) if node._session else None}
        try:
            links=node._snapshot()
            q=dict(zip(node.current_joint_states.name,node.current_joint_states.position))
            row['joints']=q; row['arms']={}
            for side,short in (('left','l'),('right','r')):
                s,e,w=[links[side][k][:3,3] for k in ('shoulder','elbow','wrist')]
                p=node._tf_pose(f'arm_{short}_link5')[:3,3]
                arm={'q4_rad':q[f'arm_{short}_joint4'], 'proximal_bend_deg':bend_degrees(s,e,p),
                     'wrist_bend_deg':bend_degrees(s,e,w), 'actual_radius_m':float(np.linalg.norm(w-s))}
                if side in node._goals:
                    g=node._goals[side]
                    arm.update(position_error_mm=1000*float(np.linalg.norm(g[:3,3]-w)),
                               orientation_error_deg=float(np.degrees(R.from_matrix(g[:3,:3]@links[side]['wrist'][:3,:3].T).magnitude())))
                arm.update(node._diagnostics.get(side,{}))
                row['arms'][side]=arm
        except Exception: pass
        rows.append(row); time.sleep(.02)
    def wait(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end: step()
    def press(a=False,b=False): step();step(a,b);step()
    def ready():
        end=time.monotonic()+19
        while time.monotonic()<end and node._phase!='ACTIVE':
            step()
            if node._phase not in ('SETTLING','WARMUP','ACTIVE'):
                raise RuntimeError('READY failed: '+str(node._last_reason))
        if node._phase!='ACTIVE': raise RuntimeError('READY timeout')
    def align():
        links=node._snapshot();mapper=node._session.mapper;B=mapper.basis
        for s in inputs:
            actual=links[s]['wrist']; a=mapper.anchors[s]
            inputs[s][:3,3]=a.input_pose[:3,3]+B.T@(actual[:3,3]-a.robot_pose[:3,3])/mapper.scale
            inputs[s][:3,:3]=B.T@actual[:3,:3]@a.robot_pose[:3,:3].T@B@a.input_pose[:3,:3]
    error=None
    try:
        end=time.monotonic()+12
        while time.monotonic()<end:
            step()
            if node._joint_rx is not None and len(node._robot_poses)==2: break
        wait(1.0);press(a=True)
        if args.startup_drift_test:
            if args.mode != 'extension':
                raise RuntimeError('startup drift policy is extension-only')
            held = {s:p.copy() for s,p in node._goals.items()}
            capture_signature = signature(node._session)
            held_inputs = {s:p.copy() for s,p in inputs.items()}
            stage='startup_drift'
            wait(1.0)
            inputs['left'][2,3] -= .0251
            end=time.monotonic()+18
            while time.monotonic()<end and node._phase!='WAIT_START':
                step()
                if node._phase not in ('SETTLING','WARMUP','WAIT_START'):
                    raise RuntimeError('Startup drift still stopped: '+str(node._last_reason))
            checks['startup_drift_waits_instead_of_stopping']=node._phase=='WAIT_START'
            checks['startup_drift_keeps_original_mapping']=signature(node._session)==capture_signature
            checks['startup_drift_goals_held']=all(np.array_equal(node._goals[s],held[s]) for s in inputs)
            print('[TEST]25.1mm early hand motion: stationary warmup completed, WAIT_START',flush=True)
            press(a=True)
            checks['wrong_start_confirmation_refused']=node._phase=='WAIT_START'
            for s in inputs:inputs[s]=held_inputs[s].copy()
            wait(.7)
            checks['returning_hand_does_not_autostart']=node._phase=='WAIT_START'
            start_confirm=time.monotonic();press(a=True)
            checks['aligned_start_confirmation_succeeds']=node._phase=='ACTIVE'
            metrics['manual_confirmation_latency_s']=time.monotonic()-start_confirm
            checks['confirmation_does_not_recapture']=signature(node._session)==capture_signature
            if not all(checks.values()):
                raise RuntimeError('startup-drift confirmation checks failed')
            print('[TEST]Aligned manual X+A entered ACTIVE without restarting calibration/warmup',flush=True)
        else:
            ready()
        checks['calibration_ready']=True
        sig=signature(node._session)
        initial={s:p.copy() for s,p in inputs.items()}
        mapper=node._session.mapper;B=mapper.basis
        q0=np.array([0.,0.,0.,-1.57,0.,0.,0.])
        base={s:fk.arm(s[0],q0) for s in inputs}
        print('[TEST]',args.mode,'fixed calibration READY; starting wrist-only extension',flush=True)
        stage='extension';start=time.monotonic()
        while time.monotonic()-start<8:
            u=min(1,(time.monotonic()-start)/7);u=u*u*(3-2*u)
            for s in inputs:
                q=q0.copy();q[3]=-1.57+1.11*u;q[0]=-1.11*u
                target=fk.arm(s[0],q)
                inputs[s]=initial[s].copy()
                inputs[s][:3,3]+=B.T@(target[:3,3]-base[s][:3,3])
                inputs[s][:3,:3]=B.T@(target[:3,:3]@base[s][:3,:3].T)@B
            step()
            if node._phase!='ACTIVE': raise RuntimeError('Stopped during extension: '+str(node._last_reason))
        stage='hold';wait(3)
        early=[r for r in rows if r['stage']=='hold' and 'arms' in r][-1]
        for s in inputs:
            metrics[s+'_bend_after_3s_deg']=early['arms'][s]['proximal_bend_deg']
        wait(5)  # Record the first3s above and a distinct settled8s endpoint.
        hold=[r for r in rows if r['stage']=='hold' and 'arms' in r][-20:]
        for s in inputs:
            for key in ('q4_rad','proximal_bend_deg','wrist_bend_deg','position_error_mm','orientation_error_deg','projection_m','raw_radius_m','shell_max_m'):
                metrics[s+'_'+key]=float(np.median([r['arms'][s][key] for r in hold]))
        checks['both_wrists_follow']=all(metrics[s+'_position_error_mm']<15 and metrics[s+'_orientation_error_deg']<5 for s in inputs)
        if args.mode=='extension':
            checks['near_straight_both']=all(metrics[s+'_proximal_bend_deg']<8 for s in inputs)
            checks['original_upper_crossed_sim_only']=all(metrics[s+'_q4_rad']>-.6 for s in inputs)
            checks['not_beyond_trial_upper']=all(metrics[s+'_q4_rad']<=-.445 for s in inputs)
        print('[HOLD]',json.dumps(metrics),flush=True)
        stage='stopped';press(b=True);wait(1)
        for s in inputs: inputs[s][:3,3]+=[0,0,.08]
        wait(.6);before=node.published_pairs;press(a=True)
        checks['wrong_pose_resume_refused']=not node.arm_control_enabled and 'ALIGN_REQUIRED' in str(node._last_reason)
        checks['no_goal_on_failed_resume']=node.published_pairs==before
        checks['mapping_retained_after_refusal']=signature(node._session)==sig
        wait(.7);align();wait(.7);stage='resume';press(a=True);ready();wait(.5)
        checks['aligned_resume_ready']=node._phase=='ACTIVE'
        checks['calibration_and_shell_unchanged']=signature(node._session)==sig
        stage='finished';press(b=True);checks['YB_stops']=not node.arm_control_enabled
    except Exception as exc: error=repr(exc)
    finally:
        node._set_arm_control_enabled(False,'extension synthetic test cleanup')
        for _ in range(10): executor.spin_once(timeout_sec=0.)
        node.destroy_node();executor.shutdown();rclpy.shutdown();loop.close()
    result={'passed':error is None and bool(checks) and all(checks.values()), 'error':error,
            'mode':args.mode,'checks':checks,'metrics':metrics,'rows':rows,
            'synthetic_input':True,'domain':232,'fixture_q4_final':-.46}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2))
    print('[RESULT]',json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
    raise SystemExit(0 if result['passed'] else 2)


if __name__=='__main__': main()
