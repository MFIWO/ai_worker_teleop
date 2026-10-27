"""Independent thumb articulation, mirroring and real DexPilot constraints; no motors."""
import unittest
import numpy as np

from hardware.thumb_flexion import thumb_flexion, apply_thumb_flexion


def skeleton(mcp=0., ip=0., count=25):
    points = np.array([[.003*i, .004*i, .005*i] for i in range(count)])
    points[1] = [.03, .01, .02]
    for idx, angle in ((2, 0.), (3, mcp), (4, mcp+ip)):
        points[idx] = points[idx-1]+.025*np.array([np.cos(angle), np.sin(angle), 0.])
    return points


class ThumbFlexionTests(unittest.TestCase):
    def test_independent_mcp_and_ip_for_both_sides_and_layouts(self):
        for count in (21, 25):
            for side, sign in (('left', -1.), ('right', 1.)):
                for mcp, ip in ((0., 0.), (.7, 0.), (0., .9), (.4, .6)):
                    np.testing.assert_allclose(thumb_flexion(skeleton(mcp, ip, count), side),
                                               sign*np.array([mcp, ip]), atol=1e-7)

    def test_rigid_motion_scale_invariance_and_other_joints_preserved(self):
        points = skeleton(.3, .8)
        rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        transformed = 1.7*points@rotation.T+[10., -4., .5]
        np.testing.assert_allclose(thumb_flexion(transformed, 'left'), [-.3, -.8], atol=1e-7)
        q = np.arange(20)/20.
        corrected = apply_thumb_flexion(q, points, 'right')
        np.testing.assert_array_equal(corrected[np.r_[0:2, 4:20]], q[np.r_[0:2, 4:20]])
        np.testing.assert_array_equal(q, np.arange(20)/20.)

    def test_invalid_bones_rejected_and_physical_bounds_retained(self):
        p = skeleton();p[3] = p[2]
        with self.assertRaisesRegex(ValueError, 'Degenerate'):
            thumb_flexion(p, 'left')
        p = skeleton();p[4, 0] = np.nan
        with self.assertRaises(ValueError):thumb_flexion(p, 'right')
        np.testing.assert_allclose(thumb_flexion(skeleton(2., 2.), 'left'), [-1.57, -1.57])

    def test_real_dexpilot_uses_thumb_bends_inside_the_solve(self):
        from hardware.xr_hand_retarget_node import make_retargeters
        for side, hand in make_retargeters().items():
            self.assertEqual(hand.retargeting.optimizer.opt_dof, 18)
            for mcp, ip in ((0., .8), (.7, 0.), (.3, .6)):
                q = hand.retarget(skeleton(mcp, ip))
                expected = np.array([mcp, ip])*(-1. if side == 'left' else 1.)
                np.testing.assert_allclose(q[2:4], expected, atol=1e-6)
                self.assertTrue(np.all(q >= hand.lower) and np.all(q <= hand.upper))

    def test_real_robotis_tracks_independent_thumb_and_corrects_warm_start(self):
        from pathlib import Path
        from ament_index_python.packages import get_package_share_directory
        from retargeting.seq_retarget import ROBOTISHandRetargeter
        from hardware.robotis_hand_retarget_node import retarget_robotis
        root = Path(get_package_share_directory('cyclo_motion_controller_models'))/'models/hx5_d20'
        for side, sign in (('left', -1.), ('right', 1.)):
            hand = ROBOTISHandRetargeter(str(root/f'hx5_d20_{side}.urdf'), side)
            hand.filter.alpha = 1.
            hand.optimizer.opt.set_maxeval(14)
            hand.optimizer.opt.set_maxtime(.006)
            for mcp, ip in ((0., .8), (.7, 0.), (.3, .6)):
                q = retarget_robotis(hand, skeleton(mcp, ip, 21), side)
                np.testing.assert_allclose(q[2:4], sign*np.array([mcp, ip]), atol=1e-6)
                np.testing.assert_allclose(hand.last_qpos, q[hand.optimizer.idx_pin2target], atol=1e-6)


if __name__ == '__main__':unittest.main()
