"""PC Vuer + optional Loop-focused operator in one foreground session."""
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

from hardware.session_processes import start,stop,wait
from hardware.teleop_cli import ipv4,hostname


def wait_for_router(host, timeout=120.):
    deadline=time.monotonic()+timeout
    next_notice=0.
    while True:
        try:
            with socket.create_connection((host,7447),timeout=1.):
                print(f'[ZENOH READY] {host}:7447; starting Vuer and operator inputs',flush=True)
                return
        except OSError as error:
            now=time.monotonic()
            if now>=deadline:
                raise RuntimeError(f'Zenoh {host}:7447 is unavailable after {timeout:g}s ({error}). '
                                   'Start the robot session with --bringup, then retry the PC command.') from None
            if now>=next_notice:
                print(f'[WAIT ZENOH] {host}:7447 is not ready ({max(0.,deadline-now):.0f}s remaining). '
                      'Run teleop.sh robot ... --loop vr --bringup in the second PC terminal. Ctrl+C cancels.',flush=True)
                next_notice=now+5.
            time.sleep(min(1.,max(0.,deadline-now)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pc-ip',required=True,type=ipv4)
    p.add_argument('--robot-ip',required=True,type=hostname)
    p.add_argument('--loop',choices=('off','vr'),default='off')
    p.add_argument('--input',choices=('hands','controllers'),default='hands')
    p.add_argument('--hand-mode',choices=('1','2'),default='1')
    p.add_argument('--host-lease',action='store_true',help=argparse.SUPPRESS)
    args=p.parse_args()
    root=Path(__file__).resolve().parents[1]
    cert=root/'robotis_vuer/robotis_vuer'
    subprocess.run([sys.executable,'-m','hardware.vuer_tls','--cert',str(cert/'cert.pem'),
                    '--key',str(cert/'key.pem'),'--server-ip',args.pc_ip],check=True)
    env=os.environ.copy()
    env.pop('ZENOH_CONFIG_OVERRIDE',None)
    env['AI_WORKER_SH5_INPUT']=args.input
    env['AI_WORKER_SH5_HAND_MODE']=args.hand_mode
    env['AI_WORKER_LOOP_CONTROLS']='1' if args.loop=='vr' and args.input=='hands' else '0'
    processes=[]
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt)
    signal.signal(signal.SIGTERM,interrupt)
    signal.signal(signal.SIGHUP,interrupt)
    from hardware.host_lease import watch_host
    try:
        watch_host(args.host_lease,'/tmp/sh5_pc_session_exit.log')
        # Both rclpy processes need the robot's router before context creation.
        # Waiting here lets the two host commands be entered in either order.
        wait_for_router(args.robot_ip)
        if args.loop=='vr' and args.input=='hands':
            keyboard_env=env.copy()
            keyboard_env['ZENOH_CONFIG_OVERRIDE']=f'mode="client";connect/endpoints=["tcp/{args.robot_ip}:7447"]'
            processes.append(('Loop keys',start([sys.executable,'-m','robotis_vuer.loop_operator'],env=keyboard_env)))
        processes.append(('Vuer',start(['bash',str(root/'run_xr_hardware_pc.sh'),'sh5','--arm','--hands',
            '--server-ip',args.pc_ip,'--robot-ip',args.robot_ip],env=env)))
        wait(processes)
    except KeyboardInterrupt:pass
    except RuntimeError as error:
        print('[SESSION ERROR] '+str(error),file=sys.stderr,flush=True)
        raise SystemExit(1)
    finally:
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,signal.SIG_IGN)
        for name,process in reversed(processes):
            try:stop(process)
            except Exception as error:print(f'[STOP ERROR] {name}: {error}',file=sys.stderr,flush=True)


if __name__=='__main__':main()
