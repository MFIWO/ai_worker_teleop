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
