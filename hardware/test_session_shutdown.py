"""Host Ctrl+C/EOF lifecycle tests with no ROS, SSH or hardware."""
from contextlib import redirect_stdout
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from hardware.robot_session import SessionLog,release_hands


class SessionShutdownTests(unittest.TestCase):
    def test_explicit_stop_and_eof_both_preserve_cleanup_log(self):
        script='''
import signal,sys,threading
from hardware.robot_session import SessionLog
from hardware.host_lease import watch_host
stop=threading.Event()
signal.signal(signal.SIGTERM,lambda *_:stop.set())
log=SessionLog(sys.argv[1])
watch_host(True,sys.argv[1],redirect_output=False)
assert stop.wait(5), 'host lease did not stop'
print('[CLEANUP] reached final cleanup',flush=True)
log.close()
'''
        with tempfile.TemporaryDirectory() as folder:
            for data in (b'alive\nstop\n',b''):
                path=Path(folder)/('explicit.log' if data else 'eof.log')
                result=subprocess.run([sys.executable,'-B','-c',script,str(path)],input=data,
                    stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=10)
                self.assertEqual(result.returncode,0,result.stdout.decode())
                text=path.read_text()
                self.assertIn('[CLEANUP] reached final cleanup',text)
                self.assertIn('Host terminal/connection lease ended',text)

    def test_broken_terminal_does_not_interrupt_persistent_cleanup_log(self):
        class Broken(io.StringIO):
            def write(self,text):raise BrokenPipeError
        with tempfile.TemporaryDirectory() as folder,redirect_stdout(Broken()):
            path=Path(folder)/'exit.log'
            log=SessionLog(path)
            try:print('cleanup continues')
            finally:log.close()
            self.assertIn('cleanup continues',path.read_text())

    def test_release_helper_failure_output_is_preserved(self):
        output=io.StringIO()
        error=subprocess.CalledProcessError(1,['helper'],output='motor verification failed\n')
        with redirect_stdout(output),patch('hardware.robot_session.subprocess.run',side_effect=error):
            with self.assertRaises(subprocess.CalledProcessError):release_hands(Path('description.urdf'))
        self.assertIn('motor verification failed',output.getvalue())


if __name__=='__main__':unittest.main()
