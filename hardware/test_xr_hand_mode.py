"""Hand-only mode selection, coordinate and latest-sample regressions; no motors."""
import unittest
from types import SimpleNamespace

import numpy as np

from hardware.xr_hand_input import xr_hand_points
from hardware.xr_hand_retarget_node import LatestHandSamples, hand_sample, make_retargeters
from hardware.teleop_cli import parser, command


def poses():
    result = np.tile(np.eye(4), (25, 1, 1))
    for i in range(1, 25):
        result[i, :3, 3] = [.002*i, .003*i, -.004*i]
    return result


def flatten(value):
    return value.transpose(0, 2, 1).reshape(-1).tolist()


class XRHandModeTests(unittest.TestCase):
    def test_cli_forwards_both_hand_modes_to_both_sessions(self):
        for side in ('pc', 'robot'):
            for hand_mode in ('1', '2'):
                args = parser().parse_args([side, '--pc-ip', '192.0.2.1', '--robot-ip', 'robot',
                                            '--input', 'hands', '--hand-mode', hand_mode])
                cmd = command(args, False)
                actual = cmd[-1] if side == 'robot' else ' '.join(cmd)
                self.assertIn('--input hands --hand-mode '+hand_mode, actual)

    def test_coordinates_match_xr_ai_axes_and_are_rigid_motion_invariant(self):
        raw = poses()
        points = xr_hand_points(flatten(raw))
        hx5_axes = np.array([[-1, 0, 0], [0, 0, -1], [0, -1, 0]])
        expected_hx5 = raw[:, :3, 3] @ np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]]).T
        np.testing.assert_allclose(points @ hx5_axes.T, expected_hx5)
        transform = np.array([[0., -1., 0., 2.], [1., 0., 0., -3.],
                              [0., 0., 1., 1.], [0., 0., 0., 1.]])
        np.testing.assert_allclose(xr_hand_points(flatten(transform @ raw)), points, atol=1e-12)

    def test_bad_tracking_is_rejected(self):
        for raw in ([0.]*400, [float('nan')]*400, [0.]*399):
            with self.assertRaises(ValueError):
                xr_hand_points(raw)
        msg = SimpleNamespace(header=SimpleNamespace(frame_id='wrong'), points=[])
        with self.assertRaises(ValueError):
            hand_sample(msg)

    def test_latest_samples_drop_backlog_stale_frames_and_hold_cache(self):
        cache = LatestHandSamples()
        cache.receive('left', 'old', 1.)
        cache.receive('left', 'latest', 1.02)
        cache.receive('right', 'stale', .5)
        self.assertEqual(cache.take(1.03), {'left': 'latest'})
        self.assertEqual(cache.take(1.04), {})
        cache.receive('left', 'before hold', 1.05)
        cache.clear()
        self.assertEqual(cache.take(1.06), {})

    def test_real_dexpilot_maps_to_twenty_finite_bounded_hand_joints(self):
        # Requires the installed robot model and numerical stack, but no ROS context.
        hands = make_retargeters()
        axes = np.array([[-1., 0, 0], [0, 0, -1.], [0, -1., 0]])
        for side, retargeter in hands.items():
            robot = retargeter.retargeting.optimizer.robot
            letter = 'l' if side=='left' else 'r'
            def reference(q):
                robot.compute_forward_kinematics(q)
                wrist = robot.get_link_pose(robot.get_link_index(f'hx5_d20_{side}_base'))[:3,3]
                points = np.zeros((25,3))
                for finger, ids in enumerate((range(1,5),range(5,10),range(10,15),range(15,20),range(20,25))):
                    tip = robot.get_link_pose(robot.get_link_index(f'finger_end_{letter}_link{finger+1}'))[:3,3]-wrist
                    for num, idx in enumerate(ids):
                        points[idx] = (tip @ axes) / 1.6 * (num+1)/len(ids)
                return points
            opened = retargeter.retarget(reference(np.zeros(robot.dof)))
            self.assertEqual(opened.shape, (20,))
            self.assertTrue(np.isfinite(opened).all())
            self.assertTrue(np.all(opened >= retargeter.lower))
            self.assertTrue(np.all(opened <= retargeter.upper))
            desired = np.zeros(robot.dof)
            for joint in range(5,20):
                if joint % 4:
                    desired[joint] = .7
            curled = retargeter.retarget(reference(desired))
            self.assertGreater(float(np.max(np.abs(opened-curled))), 1e-4)


if __name__ == '__main__':
    unittest.main()
