"""Launch console in the existing ROS container with this desktop's X11 cookie."""

import os
from pathlib import Path
import shlex
import subprocess
import sys


def main():
    display = os.environ.get('DISPLAY')
    authority = Path(os.environ.get('XAUTHORITY', str(Path.home() / '.Xauthority')))
    if not display or not authority.is_file():
        raise SystemExit('Run in the local desktop session with DISPLAY and XAUTHORITY set')
    container = os.environ.get('QUEST_PC_CONTAINER', 'robotis-applications')
    cookie = '/tmp/quest_operator.Xauthority'
    # Copy authentication without exposing the cookie or granting broad X server access.
    subprocess.run(['sg', 'docker', '-c', shlex.join(
        ['docker', 'cp', str(authority), f'{container}:{cookie}'])], check=True)
    command = ['docker', 'exec', '-e', f'DISPLAY={display}', '-e', f'XAUTHORITY={cookie}',
               '-e', f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "30")}']
    if '--check' not in sys.argv:
        command += ['-it']
    command += [container, 'bash',
                '/root/ros2_ws/src/robotis_applications/operator_console_pc.sh', *sys.argv[1:]]
    os.execvp('sg', ['sg', 'docker', '-c', shlex.join(command)])


if __name__ == '__main__':
    main()
