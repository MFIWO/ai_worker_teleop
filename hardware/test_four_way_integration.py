"""Offline regression for PR #1/#2/#3/#4 and explicit Docker install inventories.

No ROS, hardware ports, motor control, or simulation required.
"""
from pathlib import Path
import ast
import json
import unittest


ROOT = Path(__file__).resolve().parents[1]


class FourWayIntegrationTests(unittest.TestCase):
    def _source(self, relative):
        return (ROOT / relative).read_text()

    def test_robot_release_copies_every_new_runtime_module(self):
        installer = self._source('hardware/install_robot.sh')
        for name in ('loop_channels.py', 'ready_recovery.py'):
            with self.subTest(name=name):
                # Both the preflight and the copy manifest must contain this file.
                self.assertGreaterEqual(installer.count(name), 2)

    def test_pc_non_bind_mounted_release_copies_stick_filter(self):
        installer = self._source('hardware/install_pc.sh')
        self.assertIn('controller_stick_motion.py', installer)

    def test_mode2_pinches_and_trigger_rate_coexist(self):
        from hardware.controller_hand_modes import POSE_CODES
        from hardware.sh5_controller_hands import MODE2_TRIGGER_CLOSE_SPEED_SCALE
        poses = json.loads(self._source('hardware/hx5_controller_poses.json'))['poses']
        for side in ('left', 'right'):
            for pose in ('tripod_deep', 'ulnar_pinch'):
                with self.subTest(side=side, pose=pose):
                    self.assertIn(pose, poses[side])
                    self.assertIn(pose, POSE_CODES)
        self.assertEqual(MODE2_TRIGGER_CLOSE_SPEED_SCALE, 0.5)

    def test_pc_retains_mobile_stabilization_and_arm_clutch(self):
        source = self._source(
            'robotis_vuer/robotis_vuer/vr_publisher_sh5_hands_hardware.py')
        ast.parse(source)
        for fragment in (
            'MobileStickFilter()', 'self._mobile_filter.disarm',
            'self._clutch_reanchor_pending',
            'def _publish_synced_pose_frame_if_ready(self):',
            'def _ready_feedback(self, msg):',
            "'/xr_hardware/ready_request'",
        ):
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, source)

    def test_robot_supervisor_preserves_recording_channels_and_bounded_read(self):
        session = self._source('hardware/robot_session.py')
        ast.parse(session)
        for fragment in (
            'def mode2_ready_recovery_loop(', 'mode2_ready_recovery_loop(children, services, controller)',
            'args.loop_channels', 'args.loop_no_tactile',
            'from hardware.build_hand_write_rate import follower_environment',
        ):
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, session)
        overlay = self._source('hardware/build_hand_write_rate.py')
        self.assertIn('ai_worker_xr_hand_write_30hz_bounded_v1', overlay)


if __name__ == '__main__':
    unittest.main()
