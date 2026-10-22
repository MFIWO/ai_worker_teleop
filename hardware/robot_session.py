"""SH5 startup coordinator. Never starts a physical leader or a Loop recording."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time

from hardware.session_processes import start,stop,wait
from hardware.teleop_cli import ipv4,hostname


def processes():
    result=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:args=(path/'cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
        except (OSError,ProcessLookupError):continue
        if args:result.append(args)
    return result


def listening(host,port):
    try:
        with socket.create_connection((host,port),timeout=1):return True
    except OSError:return False


def ready(services,started_follower,timeout=120):
    import rclpy
    from hardware.capture_snapshot import Snapshot
    rclpy.init(args=[])
    node=Snapshot('sh5',readiness_policy='driver_managed')
    began=time.monotonic()
    try:
        while time.monotonic()-began<timeout:
            for name,process in services:
                if process.poll() is not None:raise RuntimeError(f'{name} failed during initialization')
            rclpy.spin_once(node,timeout_sec=.1)
            if not node.ready():continue
            if started_follower:
                if time.monotonic()-began<15:continue
                expected=[0.,0.,0.,-1.57,0.,0.,0.]
                if any(abs(node.state.get(f'arm_{side}_joint{i}',1e9)-value)>.1
                       for side in ('l','r') for i,value in enumerate(expected,1)):continue
                # Native initializers also move lift, head and fingers. Let their
                # action clients exit before taking the collision snapshot.
                if any('joint_trajectory_executor' in Path(a[0]).name for a in processes()):continue
            return {name for name,_ in node.get_node_names_and_namespaces()}
        raise RuntimeError('Follower not ready: '+node.error)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pc-ip',required=True,type=ipv4)
    p.add_argument('--robot-ip',required=True,type=hostname)
    p.add_argument('--bringup',action='store_true')
    p.add_argument('--loop',choices=('off','vr'),default='off')
    p.add_argument('--loop-no-camera',action='store_true')
    args=p.parse_args()
    lock=open('/tmp/sh5_vr_session.lock','w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('Another SH5 session launcher is running')
    root=Path(__file__).resolve().parents[1]
    for argv in processes():
        if (Path(argv[0]).name in ('vr_controller_node','retargeting_teleop') or
                'hardware.command_gate' in argv or 'hardware.loop_vr_streamer' in argv):
            raise SystemExit('Stop the previous VR controller/streamer before starting this session')
    subprocess.run([sys.executable,'-m','hardware.build_sh5_hands','--verify-only'],check=True)
    if args.loop=='vr':
        subprocess.run([sys.executable,'-c','import ffw_loop_streamer.loop_streamer_node; import loop_sdk'],check=True)
        if not listening(args.pc_ip,50051):raise SystemExit(f'Open Loop on {args.pc_ip}:50051 first')
    logs=Path('/workspace/ai_worker_xr_sessions')/str(time.time_ns())
    logs.mkdir(parents=True)
    services=[]
    children=[]
    files=[]
    closing=threading.Event()
    def service(name,cmd):
        output=(logs/(name+'.log')).open('w')
        files.append(output)
        process=start(cmd,output=output)
        services.append((name,process))
        (logs/(name+'.pid')).write_text(str(process.pid)+'\n')
        print(f'[SERVICE] {name}: PID {process.pid}, log {output.name}',flush=True)
    def interrupt(*_):
        closing.set()
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt)
    signal.signal(signal.SIGTERM,interrupt)
    signal.signal(signal.SIGHUP,interrupt)
    try:
        if not listening('127.0.0.1',7447):
            if not args.bringup:raise RuntimeError('Zenoh is absent; use --bringup to start missing services')
            service('zenoh',['ros2','run','rmw_zenoh_cpp','rmw_zenohd'])
            deadline=time.monotonic()+10
            while not listening('127.0.0.1',7447):
                if time.monotonic()>deadline:raise RuntimeError('Zenoh did not open port 7447')
                time.sleep(.1)
        os.environ['ZENOH_CONFIG_OVERRIDE']='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'
        follower=any(Path(argv[0]).name=='ros2_control_node' for argv in processes())
        if not follower:
            if not args.bringup:raise RuntimeError('Follower absent; --bringup starts native SH5 ready motion')
            print('[BRINGUP] Native SH5 follower initializes arms, hands, head, lift and steering.',flush=True)
            service('follower',['ros2','launch','ffw_bringup','ffw_sh5_follower_ai.launch.py'])
        print('[WAIT] Fresh stable SH5 feedback and completion of native initialization...',flush=True)
        nodes=ready(services,not follower)
        if any('leader' in name.lower() and name.endswith('controller_manager') for name in nodes):
            raise RuntimeError('Physical leader must be stopped for VR mode')
        if args.loop=='vr' and not args.loop_no_camera:
            if not {'head_left_jpeg','head_right_jpeg'}&nodes:
                loader=start(['ros2','launch','ffw_loop_streamer','head_jpeg.launch.py','jpeg_quality:=75'])
                # A composable-node loader may exit successfully after loading.
                children.append(('head JPEG loader',loader))
        controller=['bash',str(root/'hardware/run_controller.sh'),'sh5','--hands']
        if args.loop=='vr':controller.append('--manual-controls')
        children.append(('controller',start(controller)))
        if args.loop=='vr':
            loop=[sys.executable,'-m','hardware.loop_vr_streamer','--loop-addr',args.pc_ip+':50051',
                  '--robot-ip',args.robot_ip]
            if args.loop_no_camera:loop.append('--no-camera')
            children.append(('VR Loop',start(loop)))
        print('[SESSION] Ctrl+C stops controller/Loop and this session\'s router/follower, including initialization nodes. '
              f'Service logs: {logs}',flush=True)
        if args.loop=='vr':
            import rclpy
            from rclpy.node import Node
            from rclpy.callback_groups import ReentrantCallbackGroup
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.signals import SignalHandlerOptions
            from std_msgs.msg import String
            from std_srvs.srv import Trigger
            rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
            manager=Node('xr_session_manager')
            group=ReentrantCallbackGroup()
            state={'status':{},'rx':0.,'recapturing':False}
            changed=threading.Event()
            def status(msg):
                try:data=json.loads(msg.data)
                except ValueError:return
                if data.get('model')=='sh5':
                    state.update(status=data,rx=time.monotonic())
                    changed.set()
            manager.create_subscription(String,'/xr_hardware/gate_status',status,10,callback_group=group)
            def recapture(request,response):
                data=state['status']
                problems=[]
                if closing.is_set():problems.append('session is shutting down')
                if state['recapturing']:problems.append('another recapture is running')
                age=time.monotonic()-state['rx']
                if age>.5:problems.append(f'gate status age {age:.2f}s exceeds 0.5s')
                if data.get('enabled'):problems.append('arm intent is still enabled')
                if data.get('manual_active'):problems.append('manual motion is still active; release keys and wait for disarm acknowledgement')
                if data.get('fault'):problems.append('gate fault: '+str(data['fault']))
                if problems:
                    response.success=False
                    response.message='; '.join(problems)+f"; gate reason={data.get('reason','no status received')}"
                    return response
                state['recapturing']=True
                old=data.get('gate_instance')
                try:
                    index=next(i for i,item in enumerate(children) if item[0]=='controller')
                    stop(children[index][1])
                    if closing.is_set():
                        response.success=False
                        response.message='Session shutdown cancelled profile recapture'
                        return response
                    children[index]=('controller',start(controller))
                    if closing.is_set():
                        stop(children[index][1])
                        response.success=False
                        response.message='Session shutdown cancelled profile recapture'
                        return response
                    deadline=time.monotonic()+60
                    while time.monotonic()<deadline and not closing.is_set():
                        changed.wait(.2)
                        changed.clear()
                        current=state['status']
                        if current.get('gate_instance')!=old and time.monotonic()-state['rx']<.5:
                            response.success=True
                            response.message='Fresh collision profile ready; manual start gesture required'
                            return response
                        if children[index][1].poll() is not None:break
                    response.success=False
                    response.message='Profile recapture failed; inspect controller output'
                    return response
                finally:state['recapturing']=False
            manager.create_service(Trigger,'/xr_hardware/recapture_profile',recapture,callback_group=group)
            executor=MultiThreadedExecutor(num_threads=2)
            executor.add_node(manager)
            try:
                while rclpy.ok():
                    executor.spin_once(timeout_sec=.1)
                    for name,process in (*children,*services):
                        if name=='head JPEG loader' or name=='controller' and state['recapturing']:continue
                        if process.poll() is not None:raise RuntimeError(f'{name} exited ({process.returncode})')
            finally:
                closing.set()
                executor.shutdown(timeout_sec=3)
                manager.destroy_node()
                rclpy.shutdown()
        else:
            wait([item for item in children if item[0]!='head JPEG loader'],services)
    except KeyboardInterrupt:pass
    finally:
        closing.set()
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,signal.SIG_IGN)
        for name,process in (*reversed(children),*reversed(services)):
            try:
                print(f'[STOP] {name}: cleaning owned process tree',flush=True)
                stop(process)
            except Exception as error:print(f'[STOP ERROR] {name}: {error}',file=sys.stderr,flush=True)
        for file in files:file.close()
        lock.close()


if __name__=='__main__':main()
