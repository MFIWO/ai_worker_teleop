"""One-time HOST fix for the known SH5 eth0 Netplan/NetworkManager conflict.

Run with sudo on the robot host. Keeps its existing static-IP NM profile and
prepares the next boot; never restarts networking, reboots, or starts motors.
"""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import time

import yaml


def output(*args):
    return subprocess.check_output(args,text=True).strip()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot-ip',default='192.168.6.50')
    args=p.parse_args()
    wanted=ipaddress.IPv4Address(args.robot_ip)
    if os.geteuid()!=0:raise SystemExit('Run on ROBOT HOST: sudo python3 hardware/fix_robot_ethernet.py')
    target=Path('/etc/netplan/interfaces.yaml')
    if not target.is_file():raise SystemExit('Expected interfaces.yaml is absent; manual review required')
    files=set(Path('/etc/netplan').glob('*.yaml'))|set(Path('/etc/netplan').glob('*.yml'))
    if files!={target}:raise SystemExit('Additional Netplan files exist; manual review required')
    doc=yaml.safe_load(target.read_text())
    net=doc.get('network',{}) if isinstance(doc,dict) else {}
    if set(doc or {})!={'network'} or set(net)-{'version','renderer','ethernets'} or net.get('version')!=2:
        raise SystemExit('Unexpected Netplan layout; manual review required')
    if set(net.get('ethernets',{})) not in ({'eth0'},set()):
        raise SystemExit('Netplan contains other interfaces; manual review required')
    if net.get('renderer') not in ('networkd','NetworkManager'):
        raise SystemExit('Unexpected renderer; manual review required')
    profile=output('nmcli','-g','GENERAL.CON-UUID','device','show','eth0')
    if not profile or profile=='--':raise SystemExit('Connect the correct eth0 static-IP profile first')
    get=lambda prop:output('nmcli','-g',prop,'connection','show',profile)
    addresses=get('ipv4.addresses')
    if get('ipv4.method')!='manual' or str(wanted) not in [str(ipaddress.IPv4Interface(v.strip()).ip) for v in addresses.split(',') if v.strip()]:
        raise SystemExit('Active eth0 profile does not have the requested static robot IP')
    if output('systemctl','is-enabled','NetworkManager')!='enabled':
        raise SystemExit('NetworkManager is not enabled at boot; manual review required')
    before={'uuid':profile,'addresses':addresses,'autoconnect':get('connection.autoconnect'),
            'retries':get('connection.autoconnect-retries').split()[0]}
    if before['autoconnect'] not in ('yes','no') or not before['retries'].lstrip('-').isdigit():
        raise SystemExit('Unexpected NetworkManager property format')
    backup=Path('/var/backups/ai-worker-ethernet')/str(time.time_ns())
    backup.mkdir(parents=True,mode=0o700)
    shutil.copy2(target,backup/'interfaces.yaml')
    (backup/'connection.before.json').write_text(json.dumps(before,indent=2)+'\n')
    undo=['#!/usr/bin/env bash','set -euo pipefail',
          shlex.join(['cp','--',str(backup/'interfaces.yaml'),str(target)]),
          shlex.join(['nmcli','connection','modify',profile,'connection.autoconnect',before['autoconnect'],
                      'connection.autoconnect-retries',before['retries']]),'netplan generate']
    (backup/'restore.sh').write_text('\n'.join(undo)+'\n')
    try:
        # Remove the obsolete networkd eth0=.2 definition. The existing NM
        # profile already owns the correct .50 address, gateway and DNS.
        fd,name=tempfile.mkstemp(prefix='.ai-worker-',suffix='.tmp',dir=target.parent)
        with os.fdopen(fd,'w') as stream:
            stream.write('network:\n  version: 2\n  renderer: NetworkManager\n')
        os.chmod(name,0o600)
        os.replace(name,target)
        subprocess.run(['netplan','generate'],check=True)
        subprocess.run(['nmcli','connection','modify',profile,'connection.autoconnect','yes',
                        'connection.autoconnect-retries','0'],check=True)
    except BaseException:
        shutil.copy2(backup/'interfaces.yaml',target)
        subprocess.run(['netplan','generate'],check=False)
        subprocess.run(['nmcli','connection','modify',profile,'connection.autoconnect',before['autoconnect'],
                        'connection.autoconnect-retries',before['retries']],check=False)
        raise
    print(f'[SAVED] eth0 managed by NetworkManager at next boot; static IP retained: {addresses}')
    print('[SAVED] autoconnect=yes, retries=0 (unlimited). Current connection was not restarted.')
    print(f'[BACKUP] {backup}')
    print(f'[RESTORE] sudo bash {backup}/restore.sh')
    print('[NEXT] Check eth0 auto-connection after the next operator-controlled robot reboot.')


if __name__=='__main__':main()
