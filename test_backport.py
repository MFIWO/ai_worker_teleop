"""Check the SG2 arm backport without publishing robot commands.

Run in ai_worker with ROS_DOMAIN_ID=231 and the ROS workspace sourced.
"""

import importlib.util
import os
from pathlib import Path
import unittest

from geometry_msgs.msg import PoseStamped
from launch import LaunchContext
import numpy as np
import rclpy
from retargeting.robot_wrapper import RobotWrapper


ROOT = Path(os.environ.get('CYCLO_SOURCE', '/root/ros2_ws/src/cyclo_control'))


def load_module(name, path):
    import sys
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


arm = load_module('backport_arm', ROOT / 'cyclo_motion_controller_ros_py/scripts/arm_retargeting.py')


def pose(xyz, stamp=1):
    msg = PoseStamped()
    msg.header.frame_id = 'base_link'
    msg.header.stamp.sec = stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, xyz)
    msg.pose.orientation.w = 1.0
    return msg


SAMPLES = {
    'left': [[.1119726855, .2131234314, 1.1866575759],
             [.1698486728, .2083900060, .8456666055],
             [.3479461499, .1717197659, .5047557068]],
    'right': [[.0916023141, -.2109303233, 1.1736867113],
              [.1125693131, -.2087233343, .8286160046],
              [.3031660304, -.1652120717, .5073871588]],
}


class ArmBackportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = arm.ArmRetargetingTeleop()
        robot = RobotWrapper(cls.node.get_parameter('urdf_path').value)
        robot.compute_forward_kinematics(robot.q0)
        cls.link_positions = {
            f'arm_{side}_link{i}': robot.get_link_pose(
                robot.get_link_index(f'arm_{side}_link{i}'))[:3, 3].copy()
            for side in ('l', 'r') for i in (2, 4, 7)
        }
        cls.node._lookup_link_position = lambda link: cls.link_positions[link].copy()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        for side, points in SAMPLES.items():
            state = arm.ArmPoseState(*(pose(point) for point in points))
            setattr(self.node, side + '_pose_state', state)
        self.node._filtered_right_wrist_target = None
        self.node._filtered_left_wrist_target = None

    def test_recorded_startup_targets_fit_robot(self):
        targets = self.node._retarget_bimanual_pose_states()
        self.assertIsNotNone(targets)
        for side, elbow, wrist in (('right', *targets[:2]), ('left', *targets[2:])):
            short = side[0]
            geometry = getattr(self.node, side + '_geometry')
            e = self.node._pose_to_numpy(elbow)
            w = self.node._pose_to_numpy(wrist)
            actual = self.link_positions[f'arm_{short}_link7']
            shoulder = self.link_positions[f'arm_{short}_link2']
            self.assertTrue(np.isfinite(w).all())
            self.assertLess(np.linalg.norm(w - actual), .3)
            self.assertAlmostEqual(np.linalg.norm(e - shoulder), geometry.upper_arm_length)
            self.assertAlmostEqual(np.linalg.norm(w - e), geometry.forearm_length)
            self.assertEqual(wrist.header.frame_id, 'base_link')
            print(side, 'corrected_xyz=', np.round(w, 4),
                  'startup_error_m=', round(float(np.linalg.norm(w - actual)), 4))

    def test_global_position_offset_does_not_change_targets(self):
        original = self.node._retarget_bimanual_pose_states()
        for side, points in SAMPLES.items():
            shifted = [pose(np.array(point) + [2., -1., .6]) for point in points]
            setattr(self.node, side + '_pose_state', arm.ArmPoseState(*shifted))
        shifted = self.node._retarget_bimanual_pose_states()
        for before, after in zip(original, shifted):
            np.testing.assert_allclose(self.node._pose_to_numpy(before),
                                       self.node._pose_to_numpy(after), atol=1e-10)

    def test_mismatched_body_and_controller_frames_are_rejected(self):
        self.node.left_pose_state.wrist.header.stamp.sec = 2
        self.assertIsNone(self.node._retarget_bimanual_pose_states())

    def test_missing_tracking_is_rejected(self):
        self.node.right_pose_state.shoulder = None
        self.assertIsNone(self.node._retarget_bimanual_pose_states())

    def test_degenerate_arm_direction_is_rejected(self):
        self.node.right_pose_state.elbow = self.node.right_pose_state.shoulder
        self.assertIsNone(self.node._retarget_bimanual_pose_states())


class LaunchBackportTests(unittest.TestCase):
    def test_vr_adds_arm_only_and_preserves_leader_routes(self):
        module = load_module('backport_launch', ROOT /
                             'cyclo_motion_controller_ros/launch/ai_worker_controller.launch.py')
        calls = []
        real_node = module.Node

        def record_node(**kwargs):
            calls.append(kwargs)
            return real_node(**kwargs)

        module.Node = record_node
        module.generate_launch_description()
        nodes = {call['executable']: call for call in calls}
        ctx = LaunchContext()
        ctx.launch_configurations.update(controller_type='vr', hand='false')
        self.assertTrue(nodes['arm_retargeting_teleop']['condition'].evaluate(ctx))
        self.assertFalse(nodes['retargeting_teleop']['condition'].evaluate(ctx))
        routes = nodes['vr_controller_node']['remappings']
        self.assertEqual([target.perform(ctx) for _, target in routes],
                         ['/r_subgoal_pose', '/l_subgoal_pose'])
        ctx.launch_configurations['controller_type'] = 'leader'
        self.assertFalse(nodes['arm_retargeting_teleop']['condition'].evaluate(ctx))
        self.assertEqual([target.perform(ctx) for _, target in routes],
                         ['/r_elbow_pose', '/l_elbow_pose'])
        ctx.launch_configurations['controller_type'] = 'movel'
        self.assertFalse(nodes['arm_retargeting_teleop']['condition'].evaluate(ctx))


if __name__ == '__main__':
    unittest.main(verbosity=2)
