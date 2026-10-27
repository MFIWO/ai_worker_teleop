"""Detect a closed host terminal even when docker exec survives its client."""
import os
from pathlib import Path
import select
import signal
import sys
import threading
import time


def watch_host(enabled,log):
    if not enabled:return
    def watch():
        last=time.monotonic()
        explicit_stop=False
        reason='stdin EOF (host terminal or SSH/docker client closed)'
        while True:
            readable,_,_=select.select([sys.stdin.fileno()],[],[],.25)
            if readable:
                data=os.read(sys.stdin.fileno(),4096)
                if not data:break
                if b'stop\n' in data:
                    explicit_stop=True
                    reason='explicit host stop request'
                    break
                if b'\n' in data:last=time.monotonic()
            if time.monotonic()-last>3.:
                reason=f'host heartbeat timeout: {time.monotonic()-last:.3f}s (limit 3s)'
                break
        # A disappeared SSH/terminal output must not interrupt final cleanup.
        try:
            if not explicit_stop:
                destination=Path(log)
                destination.parent.mkdir(parents=True,exist_ok=True)
                fd=os.open(destination,os.O_WRONLY|os.O_CREAT|os.O_APPEND,0o600)
                os.dup2(fd,1);os.dup2(fd,2);os.close(fd)
            print('[STOP] Host terminal/connection lease ended: '+reason,flush=True)
        finally:os.kill(os.getpid(),signal.SIGTERM)
    threading.Thread(target=watch,daemon=True,name='host-terminal-lease').start()
