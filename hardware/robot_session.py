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


def mode2_ready_recovery_loop(children, services, controller_command):
    """[ADDED] Robot-owned, serialized recovery: disarm -> stop Cyclo -> ready -> restart.

    NEVER send a ready trajectory while Cyclo/command-gate still exists, even
    if the VR publisher reports HOLD. No PC-side direct arm action is allowed.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from hardware.ready_recovery import decode_request, gate_disarmed, recovery_event

    class ReadyListener(Node):
        def __init__(self):
            super().__init__('xr_sh5_ready_supervisor')
            self.pending = None
            self.gate = None
            self.gate_at = -float('inf')
            self.state_pub = self.create_publisher(String, '/xr_hardware/ready_state', 10)
            self.request_sub = self.create_subscription(
                String, '/xr_hardware/ready_request', self.on_request, 10)
            self.gate_sub = self.create_subscription(
                String, '/xr_hardware/gate_status', self.on_gate, 10)

        def report(self, request_id, state, message=''):
            self.last_report = (request_id, state)  # [DEBUG] no duplicate FAILED on exceptions
            self.state_pub.publish(String(data=recovery_event(request_id, state, message)))
            print(f'[READY RECOVERY {state}] {request_id}: {message}', flush=True)

        def on_request(self, msg):
            try:
                request_id = decode_request(msg.data)
            except (ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
                self.get_logger().warning('Invalid right-grip request: ' + str(error))
                return
            if self.pending is not None:
                self.report(request_id, 'FAILED', 'Recovery already pending; release right grip')
                return
            self.pending = request_id, time.monotonic()
            self.report(request_id, 'WAIT_DISARM', 'Waiting for fresh disarmed hardware gate')

        def on_gate(self, msg):
            try:
                payload = json.loads(msg.data)
                if isinstance(payload, dict) and payload.get('model') == 'sh5':
                    self.gate, self.gate_at = payload, time.monotonic()
            except (ValueError, TypeError):
                pass

    def check_processes(skip=()):
        for name, process in (*children, *services):
            if name == 'head JPEG loader' or name in skip:
                continue
            code = process.poll()
            if code is not None:
                raise RuntimeError(f'{name} exited ({code}) during right-grip recovery')

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = ReadyListener()
    try:
        while True:
            check_processes()
            rclpy.spin_once(node, timeout_sec=.05)
            if node.pending is None:
                continue
            request_id, requested_at = node.pending
            now = time.monotonic()
            if not gate_disarmed(node.gate, node.gate_at, now):
                if now - requested_at > 6.0:
                    node.report(request_id, 'FAILED',
                                'Hardware gate did not acknowledge a fresh DISARMED state')
                    raise RuntimeError('Right-grip ready recovery disarm timeout')
                continue
            previous_gate_instance = node.gate['gate_instance']
            node.report(request_id, 'MOVING', 'Disarm acknowledged; stopping old Cyclo first')
            # [CHANGED] Remove before stop; the complete owned process tree is
            # awaited by stop(), including reference checker, gate and Cyclo.
            old = next((item for item in children if item[0] == 'controller'), None)
            if old is None:
                raise RuntimeError('Robot controller process missing; ready move refused')
            children.remove(old)
            stop(old[1])
            check_processes()
            # [CHANGED] The existing original ready_pose action is allowed only
            # after the old Cyclo/gate process group is fully gone.
            move = start([sys.executable, '-m', 'hardware.ready_pose',
                          '--model', 'sh5', '--duration', '5', '--session-recovery'])
            item = ('arms-only ready', move)
            children.append(item)
            try:
                while move.poll() is None:
                    check_processes(skip=('arms-only ready',))
                    time.sleep(.1)
                if move.returncode != 0:
                    raise RuntimeError(f'Right-grip ready trajectory failed ({move.returncode})')
            finally:
                # Keep failed processes in the session cleanup if still alive.
                if move.poll() is not None:
                    children.remove(item)
            node.report(request_id, 'RESTARTING',
                        'Measured arms at ready pose; restarting fresh Cyclo/gate')
            fresh = start(controller_command)
            children.append(('controller', fresh))
            deadline = time.monotonic() + 90.0
            while True:
                check_processes()
                rclpy.spin_once(node, timeout_sec=.1)
                if (node.gate is not None
                        and node.gate['gate_instance'] != previous_gate_instance
                        and gate_disarmed(node.gate, node.gate_at, time.monotonic())):
                    node.report(request_id, 'READY',
                                'Fresh gate verified; release grips and A/X, press A+X for VR zero')
                    node.pending = None
                    break
                if time.monotonic() > deadline:
                    node.report(request_id, 'FAILED',
                                'Restarted Cyclo/gate did not reach fresh disarmed feedback')
                    raise RuntimeError('Right-grip ready controller restart timed out')
    except Exception as error:
        # [CHANGED] Never leave an operator with a silent, indefinitely
        # pending recovery after controller shutdown or action rejection.
        if (node.pending is not None
                and getattr(node, 'last_report', None) != (node.pending[0], 'FAILED')):
            node.report(node.pending[0], 'FAILED', str(error))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
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
    p.add_argument('--loop-no-camera',action='store_true')
    p.add_argument('--host-lease',action='store_true',help=argparse.SUPPRESS)
    args=p.parse_args()
    if args.hand_mode=='2' and args.input!='controllers':p.error('--hand-mode 2 requires --input controllers')
    os.environ['AI_WORKER_SH5_INPUT']=args.input
    os.environ['AI_WORKER_SH5_HAND_MODE']=args.hand_mode
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
    follower_started=False
    reused_follower=None
    follower_candidate=None
    description=logs/'shutdown_robot_description.urdf'
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
    from hardware.host_lease import watch_host
    try:
        watch_host(args.host_lease,logs/'exit.log')
        if not listening('127.0.0.1',7447):
            if not args.bringup:raise RuntimeError('Zenoh is absent; use --bringup to start missing services')
            service('zenoh',['ros2','run','rmw_zenoh_cpp','rmw_zenohd'])
            deadline=time.monotonic()+10
            while not listening('127.0.0.1',7447):
                if time.monotonic()>deadline:raise RuntimeError('Zenoh did not open port 7447')
                time.sleep(.1)
        os.environ['ZENOH_CONFIG_OVERRIDE']='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'
        # Watch PC exit during startup/ready motion as well as normal teleop.
        children.append(('PC session guard',start([sys.executable,'-m','hardware.hand_shutdown','--guard'])))
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
            follower_started=True
            service('follower',['ros2','launch','ffw_bringup','ffw_sh5_follower_ai.launch.py','init_position:=false'])
        else:
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
        print('[SESSION] Closing either terminal stops motion; after 3s, stop SH5 follower and verify finger torque OFF. '
              f'Service logs: {logs}',flush=True)
        if args.input == 'controllers' and args.hand_mode == '2':
            # [CHANGED] Supervisor exists only for the explicit SH5 mode-2
            # button mapping; mode-1 and optical control behavior is unchanged.
            mode2_ready_recovery_loop(children, services, controller)
        else:
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
                subprocess.run([sys.executable,'-m','hardware.hand_shutdown','--description',str(description)],check=True,timeout=30.)
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
        if release_error:raise RuntimeError('Finger torque release incomplete; inspect session exit log') from release_error


if __name__=='__main__':main()
