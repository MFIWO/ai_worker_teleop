"""[ADDED 2026-10-14] Local TLS and shell-order tests. No real ROS/motor calls.

Shell cases run an exact copy of the launcher with python3 replaced by a local
logging stub. They cannot start precheck, ready actions, Cyclo, or Vuer.
TLS cases generate ephemeral test keys in TemporaryDirectory, never production.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch

from hardware.vuer_tls import validate_vuer_tls


@unittest.skipUnless(shutil.which('openssl'), 'openssl is required')
class TLSFileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix='vuer_tls_test_')
        cls.root = Path(cls.tmp.name)
        for stem in ('server', 'other'):
            subprocess.run(
                ['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
                 '-keyout', str(cls.root / (stem + '.key')),
                 '-out', str(cls.root / (stem + '.crt')), '-days', '1',
                 '-subj', '/CN=vuer-test-only',
                 '-addext', 'subjectAltName=IP:192.168.5.101'],
                capture_output=True, check=True, timeout=15)
        cls.cert, cls.key = cls.root / 'server.crt', cls.root / 'server.key'

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_pc_ip_and_matching_key_succeed(self):
        self.assertEqual(validate_vuer_tls(self.cert, self.key, '192.168.5.101'),
                         'https://192.168.5.101:8012?ws=wss://192.168.5.101:8012')

    def test_wrong_ip_is_not_accepted(self):
        with self.assertRaisesRegex(ValueError, 'does not cover'):
            validate_vuer_tls(self.cert, self.key, '192.168.5.102')

    def test_mismatched_key_is_not_accepted(self):
        with self.assertRaisesRegex(ValueError, 'mismatched'):
            validate_vuer_tls(self.cert, self.root / 'other.key', '192.168.5.101')

    def test_missing_file_is_not_accepted(self):
        with self.assertRaisesRegex(ValueError, 'missing'):
            validate_vuer_tls(self.root / 'missing.crt', self.key, '192.168.5.101')

    def test_url_port_and_unreachable_addresses_are_rejected(self):
        for address in ('https://192.168.5.101', '192.168.5.101:8012',
                        'localhost', '127.0.0.1', '0.0.0.0', '224.0.0.1'):
            with self.subTest(address=address), self.assertRaises(ValueError):
                validate_vuer_tls(self.cert, self.key, address)

    def test_validity_window_is_checked(self):
        for value in (0, time.time() + 3 * 86400):
            with self.subTest(now=value), patch('hardware.vuer_tls.time.time', return_value=value):
                with self.assertRaisesRegex(ValueError, 'expired or not yet valid'):
                    validate_vuer_tls(self.cert, self.key, '192.168.5.101')


@unittest.skipUnless(Path('/opt/ros/jazzy/setup.bash').is_file()
                     and Path('/root/ros2_ws/install/setup.bash').is_file(),
                     'shell cases use the existing PC applications Docker setup')
class LauncherOrderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='vuer_launch_test_')
        self.root = Path(self.tmp.name)
        repo = Path(__file__).resolve().parents[1]
        self.launcher = self.root / 'run_xr_hardware_pc.sh'
        shutil.copy2(repo / self.launcher.name, self.launcher)
        mover = self.root / 'robotis_vuer/robotis_vuer/prepare_sg2_ready_pose.py'
        mover.parent.mkdir(parents=True)
        mover.write_text('# TEST ONLY: never executed; python3 is a logging stub.\n')
        precheck = self.root / 'hardware/ready_pose_precheck.py'
        precheck.parent.mkdir(parents=True)
        precheck.write_text('# TEST ONLY placeholder for the file-presence check.\n')
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        stub = self.bin / 'python3'
        stub.write_text('''#!/bin/bash
set -eu
printf '%s\\n' "$*" >> "$VUER_START_TEST_LOG"
case "$*" in
  *hardware.vuer_tls*) exit "${TEST_TLS_EXIT:-0}" ;;
  *hardware.ready_pose_precheck*) exit "${TEST_PRECHECK_EXIT:-0}" ;;
  *prepare_sg2_ready_pose.py*) exit "${TEST_MOVE_EXIT:-0}" ;;
  *robotis_vuer.vr_publisher_ai_worker_hardware*)
    printf 'ARM=%s POLICY=%s\\n' "$AI_WORKER_XR_ALLOW_ARM" "$AI_WORKER_XR_READINESS_POLICY" >> "$VUER_START_TEST_LOG"
    exit 0 ;;
  -c*) exit 0 ;;
  *) echo 'Unexpected test python invocation' >&2; exit 97 ;;
esac
''')
        stub.chmod(0o700)
        self.log = self.root / 'invocations.log'

    def tearDown(self):
        self.tmp.cleanup()

    def run_launcher(self, args, **overrides):
        env = dict(os.environ, PATH=str(self.bin) + ':' + os.environ['PATH'],
                   ROS_DOMAIN_ID='30', VUER_START_TEST_LOG=str(self.log))
        # [TEST ONLY] Domain30 is parsed by the shell but NO ROS process is run.
        env.pop('ZENOH_CONFIG_OVERRIDE', None)
        for name in ('TEST_TLS_EXIT', 'TEST_PRECHECK_EXIT', 'TEST_MOVE_EXIT'):
            env.pop(name, None)
        env.update(overrides)
        result = subprocess.run(['bash', str(self.launcher), *args], env=env,
                                capture_output=True, text=True, timeout=15)
        log = self.log.read_text() if self.log.exists() else ''
        return result, log

    def test_start_orders_tls_precheck_move_then_publisher(self):
        result, log = self.run_launcher(['sg2', '--start', '--server-ip', '192.168.5.101',
                                         '--robot-ip', '192.168.5.102'])
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        markers = ('hardware.vuer_tls', 'hardware.ready_pose_precheck',
                   'prepare_sg2_ready_pose.py', 'robotis_vuer.vr_publisher_ai_worker_hardware')
        positions = [log.index(marker) for marker in markers]
        self.assertEqual(positions, sorted(positions))
        self.assertIn('ARM=1 POLICY=driver_managed', log)
        self.assertIn('[READY POSE COMPLETE]', result.stdout)
        self.assertIn('https://192.168.5.101:8012?ws=wss://192.168.5.101:8012', result.stdout)

    def test_bad_tls_prevents_precheck_and_move(self):
        result, log = self.run_launcher(['sg2', '--start', '--server-ip', '192.168.5.101'],
                                        TEST_TLS_EXIT='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('hardware.ready_pose_precheck', log)
        self.assertNotIn('prepare_sg2_ready_pose.py', log)
        self.assertNotIn('vr_publisher_ai_worker_hardware', log)

    def test_precheck_failure_prevents_move_and_vuer(self):
        result, log = self.run_launcher(['sg2', '--start'], TEST_PRECHECK_EXIT='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('prepare_sg2_ready_pose.py', log)
        self.assertNotIn('vr_publisher_ai_worker_hardware', log)

    def test_failed_or_cancelled_move_prevents_vuer(self):
        for code in ('1', '130'):
            with self.subTest(exit_code=code):
                self.log.unlink(missing_ok=True)
                result, log = self.run_launcher(['sg2', '--start'], TEST_MOVE_EXIT=code)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('prepare_sg2_ready_pose.py', log)
                self.assertNotIn('vr_publisher_ai_worker_hardware', log)

    def test_old_arm_option_does_not_introduce_a_ready_move(self):
        result, log = self.run_launcher(['sg2', '--arm'])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('prepare_sg2_ready_pose.py', log)
        self.assertIn('ARM=1 POLICY=driver_managed', log)

    def test_ready_only_does_not_start_vuer(self):
        result, log = self.run_launcher(['sg2', '--ready-only'])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('prepare_sg2_ready_pose.py', log)
        self.assertNotIn('vr_publisher_ai_worker_hardware', log)

    def test_strict_option_is_preserved(self):
        result, log = self.run_launcher(['sg2', '--start', '--strict-commissioning'])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--readiness-policy strict_torque', log)
        self.assertIn('ARM=1 POLICY=strict_torque', log)

    def test_invalid_combinations_do_not_run_python(self):
        for args in (['sh5', '--start'], ['sg2', '--start', '--ready-only'],
                     ['sg2', '--server-ip'], ['sg2', '--robot-ip', '--start']):
            with self.subTest(args=args):
                result, log = self.run_launcher(args)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(log, '')

    def test_help_has_no_side_effects(self):
        result, log = self.run_launcher(['--help'])
        self.assertEqual(result.returncode, 0)
        self.assertIn('--start', result.stdout)
        self.assertEqual(log, '')


if __name__ == '__main__':
    unittest.main(verbosity=2)
