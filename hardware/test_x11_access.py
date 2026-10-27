"""Desktop authorization with a fake xhost; never changes the real X server."""
import contextlib
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from hardware import teleop_cli


class X11AccessTests(unittest.TestCase):
    def helper(self, display, status=0, required=False):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            fake=root/'xhost'
            fake.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$X11_TEST_ARGS"\nexit '+str(status)+'\n')
            fake.chmod(0o700)
            capture=root/'args'
            env=dict(os.environ,PATH=folder+':/usr/bin:/bin',DISPLAY=display,X11_TEST_ARGS=str(capture))
            script=Path(__file__).resolve().parents[1]/'docker/allow_x11.sh'
            result=subprocess.run(['bash',str(script),*(['--required'] if required else [])],
                                  env=env,capture_output=True,text=True)
            return result,capture.read_text() if capture.exists() else None

    def test_authorizes_only_local_root_and_propagates_xhost_failure(self):
        for status in (0,1):
            result,args=self.helper(':99',status=status)
            self.assertEqual(args,'+SI:localuser:root\n')
            self.assertEqual(result.returncode,status)
            if status:self.assertIn('logged-in desktop user',result.stderr)

    def test_no_display_is_allowed_without_keyboard_and_rejected_with_keyboard(self):
        for required in (False,True):
            result,args=self.helper('',required=required)
            self.assertIsNone(args)
            self.assertEqual(result.returncode,int(required))
            if required:self.assertIn('DISPLAY is missing',result.stderr)

    def test_dry_run_never_changes_desktop_or_starts_a_session(self):
        with patch.object(sys,'argv',['teleop.sh','pc','--pc-ip','192.0.2.1',
                                      '--robot-ip','robot','--loop','vr','--dry-run']),\
             patch.object(teleop_cli.subprocess,'run') as run,\
             patch.object(teleop_cli.subprocess,'Popen') as start,\
             contextlib.redirect_stdout(io.StringIO()):
            teleop_cli.main()
        run.assert_not_called()
        start.assert_not_called()

    def test_keyboard_authorization_failure_prevents_session_start(self):
        error=subprocess.CalledProcessError(1,['bash','allow_x11.sh'])
        with patch.object(sys,'argv',['teleop.sh','pc','--pc-ip','192.0.2.1',
                                      '--robot-ip','robot','--loop','vr','--input','hands']),\
             patch.object(teleop_cli.subprocess,'run',side_effect=error) as run,\
             patch.object(teleop_cli.subprocess,'Popen') as start:
            with self.assertRaises(subprocess.CalledProcessError):teleop_cli.main()
        self.assertEqual(run.call_args.args[0][-1],'--required')
        start.assert_not_called()


if __name__=='__main__':unittest.main()
