"""[ADDED] Isolated real Cyclo/gate/PhysX integration; synthetic XR and DXL health.
No test on physical domain30. Model validation is not hardware certification.
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
from trajectory_msgs.msg import JointTrajectory

from robotis_vuer.vr_publisher_ai_worker_hardware import HardwareXRPublisher
from hardware.xr_contract import ARM_NAMES, FOLLOWER_TOPICS
from xr_resume_smoke import signature


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='232' or os.environ.get('AI_WORKER_XR_EXECUTION')!='validation':
        raise SystemExit('validation/domain232 only; no physical robot')
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--backend',choices=('physx','kinematic_fixture'),default='physx');a=parser.parse_args()
    rclpy.init(args=['--ros-args','-r','__node:=ai_worker_xr_hardware'])
    n=HardwareXRPublisher(start_server=False)
    executor=SingleThreadedExecutor();executor.add_node(n)
    loop=asyncio.new_event_loop()
    head=np.eye(4);head[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];head[:3,3]=[0,1.6,0]
    inputs={s:np.eye(4) for s in ('left','right')}
    inputs['left'][:3,3]=[-.2,1.1,-.3];inputs['right'][:3,3]=[.2,1.1,-.3]
    commands=[]; command_values=[]; rows=[]
    def output(msg):
        commands.append(list(msg.joint_names))
        command_values.append({'t':time.monotonic(),'names':list(msg.joint_names),
                               'q':list(msg.points[-1].positions) if msg.points else []})
    subs=[n.create_subscription(JointTrajectory,FOLLOWER_TOPICS[s],output,10) for s in inputs]
    def step(press=False):
        for _ in range(25):executor.spin_once(timeout_sec=0.)
        body=np.tile(np.eye(4).flatten(order='F'),7);body[96:112]=head.flatten(order='F')
        loop.run_until_complete(n.on_body_tracking_move(SimpleNamespace(value={'body':body}),None))
        state={'aButton':press,'bButton':False}
        loop.run_until_complete(n.on_controller_move(SimpleNamespace(value={
            'left':inputs['left'].flatten(order='F').tolist(),'right':inputs['right'].flatten(order='F').tolist(),
            'leftState':state,'rightState':state}),None))
        for _ in range(25):executor.spin_once(timeout_sec=0.)
        try:
            links=n._snapshot()
            rows.append({'t':time.monotonic(),'phase':n._phase,'reason':n._last_reason,
                         'q':dict(zip(n.current_joint_states.name,n.current_joint_states.position)),
                         'goals':{s:p.tolist() for s,p in n._goals.items()},
                         'actual':{s:links[s]['wrist'].tolist() for s in links}})
        except Exception:pass
        time.sleep(.02)
    def wait(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:step()
    checks={};metrics={};error=None
    try:
        end=time.monotonic()+15
        while time.monotonic()<end:
            step()
            try:n._graph_check();n._snapshot();break
            except Exception:pass
        n._graph_check();wait(1.)
        print('[HIL] real gate/profile/motor-fixture graph ready',flush=True)
        step(True);step()
        end=time.monotonic()+20
        while time.monotonic()<end and n._phase!='ACTIVE':
            step()
            if n._phase not in ('SETTLING','WARMUP','WAIT_START','ACTIVE'):
                raise RuntimeError('startup stopped: '+str(n._last_reason))
        checks['manual_start_active']=n._phase=='ACTIVE'
        if not checks['manual_start_active']:raise RuntimeError('activation timeout')
        sig=signature(n._session);base=n._snapshot();original={s:p.copy() for s,p in inputs.items()}
        start=time.monotonic()
        while time.monotonic()-start<6.:
            u=min(1.,(time.monotonic()-start)/5.);u=u*u*(3-2*u)
            for s in inputs:
                inputs[s]=original[s].copy();inputs[s][2,3]-=.04*u
            step()
            if n._phase!='ACTIVE':raise RuntimeError('motion stopped: '+str(n._last_reason))
        wait(2.)
        final=n._snapshot()
        for s in inputs:
            moved=final[s]['wrist'][:3,3]-base[s]['wrist'][:3,3]
            metrics[s+'_forward_m']=float(moved[0])
            metrics[s+'_error_mm']=1000*float(np.linalg.norm(n._goals[s][:3,3]-final[s]['wrist'][:3,3]))
        checks['both_moved']=all(metrics[s+'_forward_m']>.025 for s in inputs)
        checks['both_follow_within_10mm']=all(metrics[s+'_error_mm']<10 for s in inputs)
        allowed=[list(v) for v in ARM_NAMES.values()]
        checks['only_seven_arm_joints_forwarded']=bool(commands) and all(x in allowed for x in commands)
        checks['fixed_calibration']=signature(n._session)==sig
        n._intent_timer.cancel();wait(1.2)
        checks['robot_gate_pc_lease_timeout']=bool(n._gate_state and 'lease expired' in str(n._gate_state.get('fault')))
        checks['fault_disables_pc']=not n.arm_control_enabled and n._controller_fault_latched
        print('[HIL]',json.dumps({'checks':checks,'metrics':metrics}),flush=True)
    except Exception as e:error=repr(e)
    finally:
        n._set_arm_control_enabled(False,'hardware HIL test cleanup')
        for _ in range(5):executor.spin_once(timeout_sec=.01)
        n.destroy_node();executor.shutdown();rclpy.try_shutdown();loop.close()
    result={'passed':error is None and checks and all(checks.values()),'error':error,'checks':checks,'metrics':metrics,
            'model':os.environ.get('AI_WORKER_XR_MODEL'),'domain':232,'synthetic_XR':True,'synthetic_motor_health':True,
            'physical_hardware_test':False,'backend':a.backend}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps({**result,'rows':rows,'commands':command_values},indent=2))
    print('[RESULT]',json.dumps(result),flush=True)
    raise SystemExit(0 if result['passed'] else 2)


if __name__=='__main__':main()
