"""Track only this session's child process groups; never kill unrelated ROS nodes."""
import os
import signal
import subprocess
import time
import uuid

OWNER_ENV = 'AI_WORKER_TELEOP_PROCESS_OWNER'


def owned_pids(owner):
    if not owner:return []
    marker=f'{OWNER_ENV}={owner}'.encode()
    result=[]
    for name in os.listdir('/proc'):
        if not name.isdigit():continue
        try:
            with open(f'/proc/{name}/environ','rb') as stream:
                if marker in stream.read().split(b'\0'):result.append(int(name))
        except (OSError,ProcessLookupError):pass
    return result


def start(cmd,*,env=None,output=None):
    child_env=dict(os.environ if env is None else env)
    owner=uuid.uuid4().hex
    child_env[OWNER_ENV]=owner
    process=subprocess.Popen(cmd,env=child_env,stdout=output,stderr=subprocess.STDOUT if output else None,
                             start_new_session=True,stdin=subprocess.DEVNULL)
    process._teleop_owner=owner
    return process


def stop(process):
    owner=getattr(process,'_teleop_owner',None)
    for sig,timeout in ((signal.SIGINT,8),(signal.SIGTERM,3),(signal.SIGKILL,2)):
        # ROS launch children may create separate process groups and may outlive
        # the launch parent. A per-child environment token identifies only this
        # launch tree, including reparented nodes and initialization executors.
        tagged=owned_pids(owner)
        if process.poll() is not None and not tagged:return
        if process.poll() is None:
            try:os.killpg(process.pid,sig)
            except ProcessLookupError:pass
        for pid in tagged:
            # Give a live ROS launch parent its normal SIGINT shutdown first.
            # Later escalation also catches children in independent groups.
            if sig==signal.SIGINT and process.poll() is None:continue
            # Recheck the token before signaling a possibly reused PID.
            try:
                with open(f'/proc/{pid}/environ','rb') as stream:
                    if f'{OWNER_ENV}={owner}'.encode() not in stream.read().split(b'\0'):continue
                if process.poll() is None and os.getpgid(pid)==process.pid:continue
                os.kill(pid,sig)
            except (OSError,ProcessLookupError):pass
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            process.poll()
            if process.returncode is not None and not owned_pids(owner):return
            time.sleep(.1)
    remaining=owned_pids(owner)
    if remaining:raise RuntimeError(f'Owned process cleanup incomplete: PIDs {remaining}')


def wait(processes,services=()):
    while True:
        for name,process in (*processes,*services):
            code=process.poll()
            if code is not None:
                raise RuntimeError(f'{name} exited ({code}); see its terminal/log')
        time.sleep(.2)
