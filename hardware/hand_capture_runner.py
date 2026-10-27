"""Supervise passive bag capture; arm driver observation only after bag is ready."""
import argparse
import signal
import subprocess
import time
from pathlib import Path
import yaml
from hardware.hand_trace_capture import prepare,finish


def stop_recorder(process):
    if process.poll() is not None:return
    process.send_signal(signal.SIGINT)
    try:process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:process.kill();process.wait()


def wait_ready(process,log,timeout=20):
    deadline=time.monotonic()+timeout
    while True:
        if process.poll() is not None:
            raise RuntimeError('ROS recorder exited during startup; inspect '+str(log))
        if 'Recording...' in log.read_text(errors='replace'):return
        if time.monotonic()>=deadline:
            raise RuntimeError('ROS recorder did not become ready within20s; driver capture was not started. Inspect '+str(log))
        time.sleep(.1)


def capture(output,seconds,topics,driver_trace=False):
    if not 10<=seconds<=600 or (driver_trace and seconds>180):raise ValueError('Invalid capture duration')
    log=output/'recorder.log';manifest=output/'manifest.json'
    command=['ros2','bag','record','--storage','sqlite3','--output',str(output/'bag'),
             '--storage-config-file',str(output/'storage.yaml'),
             '--qos-profile-overrides-path',str(output/'qos.yaml'),'--topics',*topics]
    trace_armed=False
    with log.open('w') as stream:
        process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            wait_ready(process,log)
            if driver_trace:prepare(manifest,seconds);trace_armed=True
            print('[HAND LOG RECORDING] Start movements now; '+str(seconds)+' seconds',flush=True)
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                if process.poll() is not None:raise RuntimeError('ROS recorder exited before capture completed')
                time.sleep(.1)
        except KeyboardInterrupt:
            print('[HAND LOG] Stopping observation only',flush=True)
        finally:
            stop_recorder(process)
            if trace_armed:finish(manifest)
    metadata=output/'bag/metadata.yaml'
    if not metadata.exists():raise RuntimeError('Missing ROS bag metadata; preserve '+str(output))
    info=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
    if not info['message_count']:raise RuntimeError('ROS bag contains zero messages; preserve '+str(output))
    print('[HAND LOG COUNTS]',flush=True)
    for row in info['topics_with_message_count']:
        print(f"  {row['message_count']:7d}  {row['topic_metadata']['name']}",flush=True)
    print('[HAND LOG SAVED] '+str(output),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=int,default=120);p.add_argument('--driver-trace',action='store_true')
    p.add_argument('--topics',nargs='+',required=True);args=p.parse_args()
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    try:capture(args.output,args.seconds,args.topics,args.driver_trace)
    except Exception as error:
        print('[HAND LOG ERROR] '+str(error),flush=True);raise SystemExit(1)

if __name__=='__main__':main()
