"""Start/finish observation only. Writes trace request files, never robot commands."""
import argparse
import json
import os
from pathlib import Path
import shutil
import time


def running_trace():
    found=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            args=(p/'cmdline').read_bytes().split(b'\0')
            if not args or Path(args[0].decode()).name!='ros2_control_node':continue
            env=(p/'environ').read_bytes().split(b'\0')
            dirs=[s.split(b'=',1)[1].decode() for s in env if s.startswith(b'AI_WORKER_HX5_TRACE_DIR=')]
            maps=(p/'maps').read_text()
            if len(dirs)==1 and '/ai_worker_xr_hand_trace/' in maps:found.append((int(p.name),Path(dirs[0])))
        except (OSError,UnicodeError):continue
    if len(found)!=1:raise RuntimeError('Start robot session with --hand-trace --bringup first; no unique trace follower found')
    return found[0]


def prepare(manifest,seconds):
    from hardware.build_hand_trace import verify
    verify()
    if not 10<=seconds<=180:raise ValueError('Driver trace supports 10..180 seconds per capture')
    pid,root=running_trace()
    marker=root/'capture.request'
    if marker.exists():
        token=marker.read_text().split()[0]
        if not (root/f'trace_{token}.json').exists():raise RuntimeError('Another driver capture is active')
    token=str(time.time_ns())
    data=json.loads(manifest.read_text())
    data['driver_trace']={'directory':str(root),'token':token,'pid':pid,'clock':'steady nanoseconds + wall anchor',
                          'serial_boundary':'SDK host write call/return, not motor delivery acknowledgement'}
    manifest.write_text(json.dumps(data,indent=2)+'\n')
    temp=root/f'.capture.request.{os.getpid()}'
    temp.write_text(f'{token} {seconds+5}\n');temp.replace(marker)
    deadline=time.monotonic()+3
    while not (root/f'trace_{token}.bin').exists():
        if time.monotonic()>deadline:raise RuntimeError('Trace worker did not start; inspect driver process')
        time.sleep(.025)
    print('[HAND TRACE] Stage capture started; no live file inspection',flush=True)


def finish(manifest):
    data=json.loads(manifest.read_text());info=data.get('driver_trace')
    if not info:return
    root=Path(info['directory']);token=info['token'];marker=root/'capture.request'
    if marker.exists() and marker.read_text().split()[0]==token:marker.unlink()
    stats=root/f'trace_{token}.json';deadline=time.monotonic()+8
    while True:
        try:
            report=json.loads(stats.read_text());break
        except (OSError,ValueError):
            if time.monotonic()>deadline:raise RuntimeError('Trace did not finalize; preserve '+str(root))
            time.sleep(.05)
    target=manifest.parent/'driver_trace';target.mkdir(exist_ok=True)
    for path in (stats,root/f'trace_{token}.bin',root/'sources.tsv'):
        if path.exists():shutil.copyfile(path,target/path.name)
    for name in ('follower.log','session.log'):
        path=root.parent/name
        if path.is_file():shutil.copyfile(path,manifest.parent/name)
    print('[HAND TRACE SAVED] '+json.dumps(report,sort_keys=True),flush=True)
    if not report['io_ok']:raise RuntimeError('Trace file I/O failed; keep raw files')
    if report['dropped']:print('[HAND TRACE NOTE] Dropped observations are counted; analysis must check sequence gaps',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('prepare','finish'));parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--seconds',type=int,default=120);args=parser.parse_args()
    if args.action=='prepare':prepare(args.manifest,args.seconds)
    else:finish(args.manifest)

if __name__=='__main__':main()
