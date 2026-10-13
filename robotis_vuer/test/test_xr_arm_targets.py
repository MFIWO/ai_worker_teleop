"""[ADDED] Numerical contract of the new XR mode; old algorithms untouched."""

import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R

MODULE = Path(__file__).resolve().parents[1] / 'robotis_vuer/xr_arm_targets.py'
spec = importlib.util.spec_from_file_location('xr_arm_targets_tested', MODULE)
m = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = m
spec.loader.exec_module(m)


def pose(p=(0., 0., 0.), angles=(0., 0., 0.)):
    t = np.eye(4)
    t[:3, 3] = p
    t[:3, :3] = R.from_euler('xyz', angles, degrees=True).as_matrix()
    return t


def fixture():
    head = np.eye(4)
    head[:3, :3] = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    head[:3, 3] = [0., 1.6, 0.]
    inputs = {'left': pose((-.20, 1.1, -.3), (5, 15, 30)),
              'right': pose((.20, 1.1, -.3), (-25, 5, -70))}
    robots = {'left': pose((.3, .2, 1.1), (50, -10, 30)),
              'right': pose((.3, -.2, 1.1), (-15, 20, -60))}
    return head, inputs, robots


class XRTargetTests(unittest.TestCase):
    def test_zero_start_is_measured_robot_pose(self):
        h, inputs, robots = fixture()
        mapper = m.CalibratedWristMapper(h, inputs, robots)
        for s in inputs:
            np.testing.assert_allclose(mapper.target(s, inputs[s]), robots[s], atol=1e-12)

    def test_forward_left_up_translation(self):
        h, inputs, robots = fixture()
        mapper = m.CalibratedWristMapper(h, inputs, robots)
        for xr, ros in (((0, 0, -.1), (.1, 0, 0)), ((-.1, 0, 0), (0, .1, 0)),
                        ((0, .1, 0), (0, 0, .1))):
            live = inputs['left'].copy()
            live[:3, 3] += xr
            np.testing.assert_allclose(mapper.target('left', live)[:3, 3] - robots['left'][:3, 3], ros)

    def test_world_origin_shift_invariant(self):
        h, inputs, robots = fixture()
        base = m.CalibratedWristMapper(h, inputs, robots)
        shift = np.array([8., 2., -9.])
        shifted = {s: t.copy() for s, t in inputs.items()}
        for t in shifted.values():
            t[:3, 3] += shift
        hh = h.copy()
        hh[:3, 3] += shift
        other = m.CalibratedWristMapper(hh, shifted, robots)
        live, live_shift = inputs['left'].copy(), shifted['left'].copy()
        live[:3, 3] += [.02, .01, -.04]
        live_shift[:3, 3] += [.02, .01, -.04]
        np.testing.assert_allclose(base.target('left', live), other.target('left', live_shift), atol=1e-12)

    def test_head_tilt_invariant_at_same_heading(self):
        h, inputs, robots = fixture()
        original = m.CalibratedWristMapper(h, inputs, robots)
        # Head's local Z is the right axis; pitching around it changes forward
        # elevation while preserving horizontal heading.
        tilted = h.copy()
        tilted[:3, :3] = h[:3, :3] @ R.from_euler('z', 45, degrees=True).as_matrix()
        comparison = m.CalibratedWristMapper(tilted, inputs, robots)
        np.testing.assert_allclose(original.basis, comparison.basis, atol=1e-12)

    def test_captured_head_is_not_live_reference(self):
        h, inputs, robots = fixture()
        mapper = m.CalibratedWristMapper(h, inputs, robots)
        h[:3, 3] += [0.5, 0.3, 0.4]
        h[:3, :3] = R.from_euler('y', 80, degrees=True).as_matrix()
        np.testing.assert_allclose(mapper.target('left', inputs['left']), robots['left'], atol=1e-12)

    def test_rotation_does_not_move_wrist_position(self):
        h, inputs, robots = fixture()
        mapper = m.CalibratedWristMapper(h, inputs, robots)
        live = inputs['left'].copy()
        live[:3, :3] = R.from_euler('y', 40, degrees=True).as_matrix() @ live[:3, :3]
        result = mapper.target('left', live)
        np.testing.assert_allclose(result[:3, 3], robots['left'][:3, 3])
        self.assertGreater(R.from_matrix(result[:3, :3] @ robots['left'][:3, :3].T).magnitude(), .5)

    def test_spatial_rotation_axes_match_both_sides(self):
        h, inputs, robots = fixture()
        mapper = m.CalibratedWristMapper(h, inputs, robots)
        delta = R.from_euler('y', 15, degrees=True).as_matrix()
        for side in inputs:
            live = inputs[side].copy()
            live[:3, :3] = delta @ live[:3, :3]
            expected = mapper.basis @ delta @ mapper.basis.T @ robots[side][:3, :3]
            np.testing.assert_allclose(mapper.target(side, live)[:3, :3], expected, atol=1e-12)

    def test_left_and_right_are_independent(self):
        h, inputs, robots = fixture()
        mapper = m.CalibratedWristMapper(h, inputs, robots)
        moved = inputs['left'].copy()
        moved[:3, 3] += [.1, .2, 0]
        mapper.target('left', moved)
        np.testing.assert_allclose(mapper.target('right', inputs['right']), robots['right'], atol=1e-12)

    def test_rigid_matrix_rejects_nan_reflection_and_scale(self):
        for value in (float('nan'), 2.0, -1.0):
            bad = np.eye(4)
            bad[0, 0] = value
            with self.assertRaises(ValueError):
                m.rigid_pose(bad)
        bad = np.eye(4)
        bad[3, 0] = .5
        with self.assertRaises(ValueError):
            m.rigid_pose(bad)

    def test_position_and_rotation_rate_bounds(self):
        a = pose()
        b = pose((1., 1., 1.), (80, 30, -20))
        for dt in (.001, .02, 2.):
            filtered = m.limited_pose(a, b, dt)
            self.assertLessEqual(np.linalg.norm(filtered[:3, 3]), .20 * min(dt, .05) + 1e-12)
            self.assertLessEqual(R.from_matrix(filtered[:3, :3]).magnitude(), min(dt, .05) + 1e-12)

    def test_rotation_wraparound_uses_short_arc(self):
        a, b = pose(angles=(0, 0, 179)), pose(angles=(0, 0, -179))
        c = m.limited_pose(a, b, .03)
        self.assertLess(R.from_matrix(c[:3, :3] @ a[:3, :3].T).magnitude(), np.deg2rad(2))

    def test_exact_half_turn_keeps_rotation(self):
        half_turn = pose(angles=(180, 0, 0))
        result = m.rigid_pose(half_turn)
        self.assertAlmostEqual(R.from_matrix(result[:3, :3]).magnitude(), np.pi)

    def test_elbow_preserves_start_and_segment_lengths(self):
        shoulder, elbow, wrist = np.array([0., .2, 1.4]), np.array([0., .2, 1.1]), np.array([.3, .2, 1.1])
        hint = m.ContinuousElbowHint(shoulder, elbow, wrist)
        np.testing.assert_allclose(hint.elbow(wrist), elbow, atol=1e-12)
        for x in np.linspace(.3, .42, 80):
            point, _ = hint.project_wrist([x, .2, 1.13])
            e = hint.elbow(point)
            self.assertAlmostEqual(np.linalg.norm(e - shoulder), .3, places=10)
            self.assertAlmostEqual(np.linalg.norm(point - e), .3, places=10)

    def test_workspace_projection_finite_and_bounded(self):
        hint = m.ContinuousElbowHint([0, 0, 1.4], [0, 0, 1.1], [.3, 0, 1.1])
        point, correction = hint.project_wrist([20., 0., 1.1])
        self.assertGreater(correction, 19.)
        self.assertLessEqual(np.linalg.norm(point - hint.shoulder), hint.maximum + 1e-12)
        self.assertTrue(np.isfinite(hint.elbow(point)).all())

    def test_degenerate_anchor_is_rejected_not_shifted(self):
        with self.assertRaises(ValueError):
            m.ContinuousElbowHint([0, 0, 0], [0, 0, .3], [.0001, 0, 0])
        with self.assertRaises(ValueError):
            m.ContinuousElbowHint([0, 0, 0], [.3, 0, 0], [.6, 0, 0])


if __name__ == '__main__':
    unittest.main()
