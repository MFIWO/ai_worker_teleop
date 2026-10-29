"""Two host commands for the SH5 session. --dry-run never connects or starts ROS."""
import argparse
import grp
import ipaddress
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time

# teleop.sh executes this file directly on the host, outside the ROS package path.
if __package__:
    from hardware.aux_controls import mobile_speed, BASE_LIMITS
else:
    from aux_controls import mobile_speed, BASE_LIMITS


def ipv4(value):
    try:return str(ipaddress.IPv4Address(value))
    except ValueError:raise argparse.ArgumentTypeError('expected an IPv4 address')


def hostname(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]*',value):
        raise argparse.ArgumentTypeError('invalid robot host')
    return value


def base_speed(value):
    try:return mobile_speed(value, BASE_LIMITS[0])
    except ValueError as error:raise argparse.ArgumentTypeError(str(error)) from error


def turn_speed(value):
    try:return mobile_speed(value, BASE_LIMITS[2])
    except ValueError as error:raise argparse.ArgumentTypeError(str(error)) from error


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('side',choices=('pc','robot'))
    p.add_argument('--pc-ip',type=ipv4,default=os.environ.get('PC_IP'),required='PC_IP' not in os.environ)
    p.add_argument('--robot-ip',type=hostname,default=os.environ.get('ROBOT_IP'),required='ROBOT_IP' not in os.environ)
    p.add_argument('--loop',choices=('off','vr'),default='off')
    p.add_argument('--input',choices=('hands','controllers'),default='hands')
    p.add_argument('--hand-mode',choices=('1','2'),default='1',help='hands: 1=Cyclo original, 2=xr_ai_teleop HX5 DexPilot; controllers: 1=trigger/grip, 2=button-selected poses')
    p.add_argument('--bringup',action='store_true',help='robot: start missing router/follower; lift to top, then five-second arm ready motion')
    p.add_argument('--ready-pose',action='store_true',help='robot: lift to physical top, then five-second arm ready motion before VR starts')
    p.add_argument('--base-speed',type=base_speed,default=.2,help='robot: maximum joystick XY speed in m/s (0..0.2)')
    p.add_argument('--turn-speed',type=turn_speed,default=1/3,help='robot: maximum joystick yaw speed in rad/s (0..0.333333)')
    p.add_argument('--loop-no-camera',action='store_true')
    # CHANGED(2026-10-28): Robot-side Loop recorder selection, not motion control.
    p.add_argument('--loop-channels',default='training',help='training | qpos | all | comma-separated channel keys')
    p.add_argument('--loop-no-tactile',action='store_true',help='robot: omit separate HX5-D20 Tactile source')
    p.add_argument('--hand-trace',action='store_true',help='robot: select the timing-only hand driver; recording starts separately')
    p.add_argument('--dry-run',action='store_true')
    return p


def command(args,interactive):
    # stdin carries a lease; TTY detachment alone does not stop docker exec.
    terminal='-i'
    if args.side=='pc':
        display_env=['-e',f'DISPLAY={os.environ["DISPLAY"]}'] if os.environ.get('DISPLAY') else []
        return ['docker','exec',terminal,*display_env,'robotis-applications','bash',
                '/root/ros2_ws/src/robotis_applications/hardware/run_pc_session.sh',
                '--pc-ip',args.pc_ip,'--robot-ip',args.robot_ip,'--loop',args.loop,'--input',args.input,'--hand-mode',args.hand_mode,'--host-lease']
    inner=['docker','exec',terminal,'ai_worker','bash',
           '/workspace/ai_worker_xr/hardware/run_robot_session.sh',
           '--pc-ip',args.pc_ip,'--robot-ip',args.robot_ip,'--loop',args.loop,'--input',args.input,'--hand-mode',args.hand_mode,'--host-lease']
    if args.bringup:inner.append('--bringup')
    if args.ready_pose:inner.append('--ready-pose')
    inner.extend(['--base-speed',str(args.base_speed),'--turn-speed',str(args.turn_speed)])
    if args.loop_no_camera:inner.append('--loop-no-camera')
    if args.loop=='vr':
        inner.extend(['--loop-channels',args.loop_channels])
        if args.loop_no_tactile:inner.append('--loop-no-tactile')
    if getattr(args,'hand_trace',False):inner.append('--hand-trace')
    return ['ssh','-T','-o','ServerAliveInterval=2','-o','ServerAliveCountMax=2',f'robotis@{args.robot_ip}',shlex.join(inner)]


def main():
    p=parser()
    args=p.parse_args()
    if args.side=='pc' and (args.bringup or args.loop_no_camera or args.ready_pose or args.hand_trace or
                            args.loop_no_tactile or args.loop_channels!='training' or
                            args.base_speed!=.2 or args.turn_speed!=1/3):
        p.error('--bringup, --ready-pose, --loop-no-camera, --loop-no-tactile, '
                '--loop-channels, --hand-trace, --base-speed and --turn-speed apply to the robot command')
    cmd=command(args,sys.stdin.isatty())
    if args.dry_run:
        print(shlex.join(cmd))
        return
    if args.side=='pc':
        x11=['bash',str(Path(__file__).resolve().parents[1]/'docker/allow_x11.sh')]
        if args.loop=='vr' and args.input=='hands':x11.append('--required')
        subprocess.run(x11,check=True)
    if args.side=='pc' and not os.access('/var/run/docker.sock',os.R_OK|os.W_OK):
        try:member=os.getgid()==grp.getgrnam('docker').gr_gid or os.getlogin() in grp.getgrnam('docker').gr_mem
        except (KeyError,OSError):
            import getpass
            try:member=getpass.getuser() in grp.getgrnam('docker').gr_mem
            except KeyError:member=False
        if member and shutil.which('sg'):cmd=['sg','docker','-c',shlex.join(cmd)]
    # Keep the SSH/docker client alive while the container coordinator handles
    # the terminal's Ctrl+C and shuts down its owned ROS launch trees.
    process=subprocess.Popen(cmd,stdin=subprocess.PIPE,start_new_session=True)
    stopping=False
    stop_signal=None
    def interrupt(sig,*_):
        nonlocal stopping,stop_signal
        stopping=True
        stop_signal=sig
        if process.poll() is None:
            try:print('[STOP] Waiting for container session cleanup; keep this terminal open.',flush=True)
            except OSError:pass
    saved={sig:signal.signal(sig,interrupt) for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    try:
        while process.poll() is None and not stopping:
            try:
                process.stdin.write(b'alive\n');process.stdin.flush()
            except (BrokenPipeError,OSError):break
            time.sleep(.25)
        if stop_signal==signal.SIGINT:
            try:process.stdin.write(b'stop\n');process.stdin.flush()
            except (BrokenPipeError,OSError):pass
        try:process.stdin.close()
        except (BrokenPipeError,OSError):pass
        raise SystemExit(process.wait())
    finally:
        if not process.stdin.closed:
            try:process.stdin.close()
            except (BrokenPipeError,OSError):pass
        for sig,handler in saved.items():signal.signal(sig,handler)


if __name__=='__main__':main()
