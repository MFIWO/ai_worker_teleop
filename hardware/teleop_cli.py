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


def ipv4(value):
    try:return str(ipaddress.IPv4Address(value))
    except ValueError:raise argparse.ArgumentTypeError('expected an IPv4 address')


def hostname(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]*',value):
        raise argparse.ArgumentTypeError('invalid robot host')
    return value


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('side',choices=('pc','robot'))
    p.add_argument('--pc-ip',type=ipv4,default=os.environ.get('PC_IP'),required='PC_IP' not in os.environ)
    p.add_argument('--robot-ip',type=hostname,default=os.environ.get('ROBOT_IP'),required='ROBOT_IP' not in os.environ)
    p.add_argument('--loop',choices=('off','vr'),default='off')
    p.add_argument('--input',choices=('hands','controllers'),default='hands')
    p.add_argument('--bringup',action='store_true',help='robot: start missing router/follower without native initialization; raise only arms over five seconds')
    p.add_argument('--ready-pose',action='store_true',help='robot: five-second arms-only ready motion before VR starts')
    p.add_argument('--loop-no-camera',action='store_true')
    p.add_argument('--dry-run',action='store_true')
    return p


def command(args,interactive):
    terminal='-it' if interactive else '-i'
    if args.side=='pc':
        return ['docker','exec',terminal,'robotis-applications','bash',
                '/root/ros2_ws/src/robotis_applications/hardware/run_pc_session.sh',
                '--pc-ip',args.pc_ip,'--robot-ip',args.robot_ip,'--loop',args.loop,'--input',args.input]
    inner=['docker','exec',terminal,'ai_worker','bash',
           '/workspace/ai_worker_xr/hardware/run_robot_session.sh',
           '--pc-ip',args.pc_ip,'--robot-ip',args.robot_ip,'--loop',args.loop,'--input',args.input]
    if args.bringup:inner.append('--bringup')
    if args.ready_pose:inner.append('--ready-pose')
    if args.loop_no_camera:inner.append('--loop-no-camera')
    return ['ssh',*(['-t'] if interactive else []),f'robotis@{args.robot_ip}',shlex.join(inner)]


def main():
    p=parser()
    args=p.parse_args()
    if args.side=='pc' and (args.bringup or args.loop_no_camera or args.ready_pose):
        p.error('--bringup, --ready-pose and --loop-no-camera apply to the robot command')
    cmd=command(args,sys.stdin.isatty())
    if args.dry_run:
        print(shlex.join(cmd))
        return
    if args.side=='pc' and not os.access('/var/run/docker.sock',os.R_OK|os.W_OK):
        try:member=os.getgid()==grp.getgrnam('docker').gr_gid or os.getlogin() in grp.getgrnam('docker').gr_mem
        except (KeyError,OSError):
            import getpass
            try:member=getpass.getuser() in grp.getgrnam('docker').gr_mem
            except KeyError:member=False
        if member and shutil.which('sg'):cmd=['sg','docker','-c',shlex.join(cmd)]
    # Keep the SSH/docker client alive while the container coordinator handles
    # the terminal's Ctrl+C and shuts down its owned ROS launch trees.
    process=subprocess.Popen(cmd)
    def interrupt(*_):
        if process.poll() is None:
            print('[STOP] Waiting for container session cleanup; keep this terminal open.',flush=True)
    signal.signal(signal.SIGINT,interrupt)
    try:
        raise SystemExit(process.wait())
    finally:
        signal.signal(signal.SIGINT,signal.SIG_DFL)


if __name__=='__main__':main()
