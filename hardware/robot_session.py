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


class SessionLog:
    """Keep shutdown evidence even after an explicit Ctrl+C or broken terminal."""
    def __init__(self,path):
        self.file=Path(path).open('a',buffering=1)
        self.original=(sys.stdout,sys.stderr)
        self.lock=threading.Lock()
        owner=self
        class Stream:
            def __init__(self,terminal):self.terminal=terminal
            def __getattr__(self,name):return getattr(self.terminal,name)
            def write(self,value):
                with owner.lock:
                    owner.file.write(value);owner.file.flush()
                    try:self.terminal.write(value);self.terminal.flush()
                    except (OSError,ValueError):pass
                return len(value)
            def flush(self):
                with owner.lock:
                    owner.file.flush()
                    try:self.terminal.flush()
                    except (OSError,ValueError):pass
        sys.stdout,sys.stderr=(Stream(s) for s in self.original)

    def close(self):
        sys.stdout,sys.stderr=self.original
        self.file.close()


def run_logged(argv, *, timeout=30.):
    """Capture helper output in the session log, including failed/partial runs."""
    try:
        result=subprocess.run(argv,check=True,timeout=timeout,
                              stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    except (subprocess.CalledProcessError,subprocess.TimeoutExpired) as error:
        output=error.stdout or ''
        if isinstance(output,bytes):output=output.decode(errors='replace')
        if output:print(output,end='',flush=True)
        raise
    if result.stdout:print(result.stdout,end='',flush=True)


def release_hands(description):
    run_logged([sys.executable,'-m','hardware.hand_shutdown','--description',str(description)])


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


def process_identity(pid):
    try:
        fields=(Path('/proc')/str(pid)/'stat').read_text().rpartition(')')[2].split()
        return None if fields[0]=='Z' else fields[19]
    except (OSError,IndexError):return None


def find_follower_launch():
    matches=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:args=(proc/'cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
        except OSError:continue
        if all(token in args for token in ('launch','ffw_bringup','ffw_sh5_follower_ai.launch.py')):
            identity=process_identity(proc.name)
            if identity is not None:matches.append((int(proc.name),identity))
    return matches[0] if len(matches)==1 else None


def stop_follower_launch(identity):
    pid,ticks=identity
    if process_identity(pid)!=ticks:return
    os.kill(pid,signal.SIGINT)
    deadline=time.monotonic()+10.
    while process_identity(pid)==ticks and time.monotonic()<deadline:time.sleep(.1)
    if process_identity(pid)==ticks:raise RuntimeError('SH5 launch did not exit; direct port access will remain blocked')


def ready(services,description,timeout=120):
    import rclpy
    from hardware.capture_snapshot import Snapshot
    rclpy.init(args=[])
    node=Snapshot('sh5',readiness_policy='driver_managed')
    began=time.monotonic()
    saved_xml=None
    try:
        while time.monotonic()-began<timeout:
            for name,process in services:
                if process.poll() is not None:raise RuntimeError(f'{name} failed during initialization')
            rclpy.spin_once(node,timeout_sec=.1)
            if node.xml is not None and node.xml!=saved_xml:
                from hardware.hand_shutdown import configuration
                configuration(node.xml)
                description.write_text(node.xml)
                saved_xml=node.xml
            if not node.ready():continue
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
    p.add_argument('--ready-pose',action='store_true')
    p.add_argument('--loop',choices=('off','vr'),default='off')
    p.add_argument('--input',choices=('hands','controllers'),default='hands')
    p.add_argument('--hand-mode',choices=('1','2'),default='1')
    p.add_argument('--hand-trace',action='store_true')
    p.add_argument('--loop-no-camera',action='store_true')
    p.add_argument('--host-lease',action='store_true',help=argparse.SUPPRESS)
    args=p.parse_args()
    os.environ['AI_WORKER_SH5_INPUT']=args.input
    os.environ['AI_WORKER_SH5_HAND_MODE']=args.hand_mode
    lock=open('/tmp/sh5_vr_session.lock','w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('Another SH5 session launcher is running')
    root=Path(__file__).resolve().parents[1]
    for argv in processes():
        if (Path(argv[0]).name in ('vr_controller_node','retargeting_teleop') or
                'hardware.command_gate' in argv or 'hardware.loop_vr_streamer' in argv or
                'hardware.xr_hand_retarget_node' in argv):
            raise SystemExit('Stop the previous VR controller/streamer before starting this session')
    subprocess.run([sys.executable,'-m','hardware.build_sh5_hands','--verify-only'],check=True)
    if args.hand_trace:
        subprocess.run([sys.executable,'-m','hardware.build_hand_trace','--verify-only'],check=True)
    if args.input=='hands' and args.hand_mode=='2':
        # Resolve the actual hand model and dependencies before any bringup/motion.
        subprocess.run([sys.executable,'-c',
            'from hardware.xr_hand_retarget_node import make_retargeters; make_retargeters(); '
            'print("[XR AI HAND MODE 2] DexPilot dependencies and both HX5 models verified")'],check=True)
    if args.loop=='vr':
        subprocess.run([sys.executable,'-c','import ffw_loop_streamer.loop_streamer_node; import loop_sdk'],check=True)
        if not listening(args.pc_ip,50051):raise SystemExit(f'Open Loop on {args.pc_ip}:50051 first')
    logs=Path('/workspace/ai_worker_xr_sessions')/str(time.time_ns())
    logs.mkdir(parents=True)
    services=[]
    children=[]
    files=[]
    closing=threading.Event()
    follower_started=False
    reused_follower=None
    follower_candidate=None
    description=logs/'shutdown_robot_description.urdf'
    def service(name,cmd,*,env=None):
        output=(logs/(name+'.log')).open('w')
        files.append(output)
        process=start(cmd,env=env,output=output)
        services.append((name,process))
        (logs/(name+'.pid')).write_text(str(process.pid)+'\n')
        print(f'[SERVICE] {name}: PID {process.pid}, log {output.name}',flush=True)
    def interrupt(*_):
        closing.set()
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt)
    signal.signal(signal.SIGTERM,interrupt)
    signal.signal(signal.SIGHUP,interrupt)
    from hardware.host_lease import watch_host
    session_log=SessionLog(logs/'exit.log')
    try:
        watch_host(args.host_lease,logs/'exit.log',redirect_output=False)
        if not listening('127.0.0.1',7447):
            if not args.bringup:raise RuntimeError('Zenoh is absent; use --bringup to start missing services')
            service('zenoh',['ros2','run','rmw_zenoh_cpp','rmw_zenohd'])
            deadline=time.monotonic()+10
            while not listening('127.0.0.1',7447):
                if time.monotonic()>deadline:raise RuntimeError('Zenoh did not open port 7447')
                time.sleep(.1)
        os.environ['ZENOH_CONFIG_OVERRIDE']='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'
        # PC/VR lifetime is independent of this robot session. The command
        # gate holds on an expired PC intent lease; keep the follower alive
        # so a fresh, disarmed PC session can reconnect and explicitly resume.
        follower=any(Path(argv[0]).name=='ros2_control_node' for argv in processes())
        if not follower:
            if not args.bringup:raise RuntimeError('Follower absent; --bringup starts SH5 without native initialization, then raises only the arms over five seconds')
            print('[BRINGUP] SH5 follower init_position:=false; then five-second arms-only ready motion. No hand initialization trajectory.',flush=True)
            # Preserve the expected mapping even if launch is interrupted
            # before /robot_description is published. No ROS node/motion here.
            from ament_index_python.packages import get_package_share_directory
            import xacro
            source=Path(get_package_share_directory('ffw_description'))/'urdf/ffw_sh5_rev1_follower/ffw_sh5_follower.urdf.xacro'
            xml=xacro.process_file(str(source),mappings={'use_sim':'false','use_mock_hardware':'false','mock_sensor_commands':'false','port_name':'/dev/follower','model':'ffw_sh5_rev1_follower','use_head_eef_tracker':'false'}).toxml()
            from hardware.hand_shutdown import configuration
            configuration(xml)
            description.write_text(xml)
            run_logged([sys.executable,'-m','hardware.hand_transport','--description',str(description),
                        '--report',str(logs/'hand_transport.json')],timeout=15.)
            if args.hand_trace:
                from hardware.build_hand_trace import follower_environment
                follower_env=follower_environment(logs/'driver_trace')
                print('[HAND TRACE] Timing-only driver selected; capture is OFF until log_hands.sh --driver-trace.',flush=True)
            else:
                from hardware.build_hand_write_rate import follower_environment
                follower_env=follower_environment()
            print('[HAND WRITE] Existing follower driver: HX5 motor packets 30 Hz; '
                  'arm writes/read loop unchanged. No extra hand relay.',flush=True)
            follower_started=True
            service('follower',['ros2','launch','ffw_bringup','ffw_sh5_follower_ai.launch.py','init_position:=false'],env=follower_env)
        else:
            if args.hand_trace:
                from hardware.build_hand_trace import verify_running_follower
            else:
                from hardware.build_hand_write_rate import verify_running_follower
            verify_running_follower()
            follower_candidate=find_follower_launch()
            if follower_candidate is None:
                raise RuntimeError('Existing follower has no unique SH5 launch parent; stop it and restart this command with --bringup so exit can release finger torque')
        print('[WAIT] Fresh stable SH5 feedback...',flush=True)
        nodes=ready([*services,*children],description)
        if follower_candidate:
            reused_follower=follower_candidate
            print('[BRINGUP REUSED] This verified SH5 launch will also stop on session exit so the hand port can be released.',flush=True)
        if any('leader' in name.lower() and name.endswith('controller_manager') for name in nodes):
            raise RuntimeError('Physical leader must be stopped for VR mode')
        if not follower or args.ready_pose:
            mover=start([sys.executable,'-m','hardware.ready_pose','--model','sh5','--duration','5'])
            children.append(('arms-only ready',mover))
            while mover.poll() is None:
                if any(p is not mover and p.poll() is not None for _,p in (*services,*children)):
                    raise RuntimeError('Follower/router/PC session exited during arm ready motion')
                time.sleep(.1)
            if mover.returncode:raise RuntimeError('Five-second arm ready motion failed; VR remains stopped')
            children.remove(('arms-only ready',mover))
        if args.loop=='vr' and not args.loop_no_camera:
            if not {'head_left_jpeg','head_right_jpeg'}&nodes:
                loader=start(['ros2','launch','ffw_loop_streamer','head_jpeg.launch.py','jpeg_quality:=75'])
                # A composable-node loader may exit successfully after loading.
                children.append(('head JPEG loader',loader))
        controller=['bash',str(root/'hardware/run_controller.sh'),'sh5','--hands']
        if args.loop=='vr' or args.input=='controllers':controller.append('--manual-controls')
        children.append(('controller',start(controller)))
        if args.loop=='vr':
            loop=[sys.executable,'-m','hardware.loop_vr_streamer','--loop-addr',args.pc_ip+':50051',
                  '--robot-ip',args.robot_ip]
            if args.loop_no_camera:loop.append('--no-camera')
            children.append(('VR Loop',start(loop)))
        print('[SESSION] PC/VR exit holds motion and keeps this robot session/follower running. '
              'Closing this robot terminal stops SH5 follower after 3s and verifies finger torque OFF. '
              f'Service logs: {logs}',flush=True)
        wait([item for item in children if item[0]!='head JPEG loader'],services)
    except KeyboardInterrupt:pass
    finally:
        closing.set()
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,signal.SIG_IGN)
        release_at=time.monotonic()+3.
        # Stop command production first, before waiting for camera/SDK teardown.
        for name,process in children:
            if name not in ('controller','arms-only ready') or process.poll() is not None:continue
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
        for name,process in reversed(children):
            try:
                print(f'[STOP] {name}: cleaning owned process tree',flush=True)
                stop(process)
            except Exception as error:print(f'[STOP ERROR] {name}: {error}',file=sys.stderr,flush=True)
        if follower_started or reused_follower:
            print('[STOP] Motion processes stopped; finger torque release after 3-second exit grace.',flush=True)
            time.sleep(max(0.,release_at-time.monotonic()))
        # Stop follower before direct hand access; keep router until afterward.
        for name,process in reversed(services):
            if name!='follower':continue
            try:stop(process)
            except Exception as error:print(f'[STOP ERROR] {name}: {error}',file=sys.stderr,flush=True)
        if reused_follower:
            try:stop_follower_launch(reused_follower)
            except Exception as error:print(f'[STOP ERROR] reused follower: {error}',file=sys.stderr,flush=True)
        release_error=None
        if (follower_started or reused_follower) and description.exists():
            try:
                release_hands(description)
            except Exception as error:
                release_error=error
                print(f'[HAND TORQUE OFF FAILED] Physical release NOT verified: {error}',file=sys.stderr,flush=True)
        elif follower_started or reused_follower:
            release_error=RuntimeError('No verified SH5 motor mapping; torque release NOT verified')
            print(f'[HAND TORQUE OFF FAILED] {release_error}',file=sys.stderr,flush=True)
        for name,process in reversed(services):
            if name=='follower':continue
            try:stop(process)
            except Exception as error:print(f'[STOP ERROR] {name}: {error}',file=sys.stderr,flush=True)
        for file in files:file.close()
        lock.close()
        session_log.close()
        if release_error:raise RuntimeError('Finger torque release incomplete; inspect session exit log') from release_error


if __name__=='__main__':main()
