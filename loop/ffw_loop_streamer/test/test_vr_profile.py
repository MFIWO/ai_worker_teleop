"""Verify VR arm actions stay independent of gripper presence and measured state."""

from pathlib import Path
import unittest

from ffw_loop_streamer.loop_streamer import build_groups, RobotStepMapper
import yaml


class VRProfileTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / 'config' / 'ffw_sg2_vr.yaml'
        self.config = yaml.safe_load(path.read_text())['/**']['ros__parameters']['ffw_sg2_vr']
        self.mapper = RobotStepMapper('ffw_sg2_vr', build_groups(self.config['joint_order']))
        self.state = {f'arm_{side}_joint{i}': .01 * i for side in ('l', 'r')
                      for i in range(1, 8)}
        self.command = {f'arm_{side}_joint{i}': .1 * i for side in ('l', 'r')
                        for i in range(7, 0, -1)}

    def payload(self, command):
        return self.mapper.build_payload(
            follower_positions=self.state,
            leader_positions={name: command for name in self.config['joint_list']})

    def test_hand_mode_without_gripper_keeps_both_seven_joint_actions(self):
        payload = self.payload(self.command)
        for side in ('left', 'right'):
            self.assertEqual(payload[f'ffw_sg2_vr.action.{side}_arm.joint_position'],
                             [.1 * i for i in range(1, 8)])
            self.assertIsNone(payload[f'ffw_sg2_vr.action.{side}_gripper.joint_position'])

    def test_controller_gripper_is_separate_and_does_not_change_arm_dimension(self):
        command = dict(self.command, gripper_l_joint1=.4, gripper_r_joint1=.7)
        payload = self.payload(command)
        self.assertEqual(payload['ffw_sg2_vr.action.left_gripper.joint_position'], [.4])
        self.assertEqual(payload['ffw_sg2_vr.action.right_gripper.joint_position'], [.7])
        self.assertEqual(len(payload['ffw_sg2_vr.action.left_arm.joint_position']), 7)

    def test_missing_actions_are_not_filled_from_measured_state(self):
        payload = self.mapper.build_payload(follower_positions=self.state)
        self.assertEqual(payload['ffw_sg2_vr.observation.left_arm.joint_position'],
                         [.01 * i for i in range(1, 8)])
        self.assertIsNone(payload['ffw_sg2_vr.action.left_arm.joint_position'])

    def test_legacy_and_vr_profiles_do_not_reuse_eight_joint_channel_names(self):
        self.assertNotIn('ffw_sg2_rev1.action.left.joint_position', self.mapper.channel_keys())
        self.assertEqual(len(self.mapper.channel_keys()), 8)
        topics = dict(s.split(':', 1) for s in self.config['joint_topic_list'])
        self.assertEqual(topics['leader_left_arm'],
                         '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory')


if __name__ == '__main__':
    unittest.main(verbosity=2)
