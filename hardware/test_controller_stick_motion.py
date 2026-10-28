"""[ADDED 2026-10-28] Offline controller regression tests; no ROS/motors."""
import math
import unittest

from hardware.controller_stick_motion import MobileStickFilter, head_targets


HEAD = {
    'head_joint1': {'lower': -0.2317, 'upper': 0.6951, 'velocity': 4.8},
    'head_joint2': {'lower': -0.35, 'upper': 0.35, 'velocity': 4.8},
}


class HeadJogTests(unittest.TestCase):
    def test_uses_physical_urdf_and_time_based_rate(self):
        result = head_targets((1., -1.), None, (0., 0.), HEAD, 1 / 30)
        self.assertAlmostEqual(result[0], 0.01, places=5)
        self.assertAlmostEqual(result[1], -0.01, places=5)
        self.assertLess(result[0], 0.06)  # old per-event jump

    def test_saturates_not_stops_at_both_joint_bounds(self):
        result = head_targets((1., -1.), (.694, -.349), (0., 0.), HEAD, .1)
        self.assertEqual(result, (.6951, -.35))
        self.assertEqual(head_targets((1., -1.), result, (0., 0.), HEAD, .033), result)

    def test_invalid_head_sample_and_missing_limits_fail(self):
        for stick in ((math.nan, 0.), (1.4, 0.)):
            with self.assertRaises(ValueError):
                head_targets(stick, None, (0., 0.), HEAD, .01)
        with self.assertRaises(ValueError):
            head_targets((1., 0.), None, (0., 0.), {}, .01)


class MobileTests(unittest.TestCase):
    def setUp(self):
        self.filter = MobileStickFilter()
        # Intentionally nonzero physical stick centers, including > old .05 DZ.
        self.neutral_left = (0.12, -0.09)
        self.neutral_right = (0.08, 0.11)

    def calibrate(self):
        f = self.filter
        self.assertEqual(f.update(self.neutral_left, self.neutral_right, pressed=True, now=0.), (0., 0., 0.))
        self.assertEqual(f.update(self.neutral_left, self.neutral_right, now=.02), (0., 0., 0.))
        self.assertFalse(f.ready)
        self.assertEqual(f.update(self.neutral_left, self.neutral_right, now=.36), (0., 0., 0.))
        self.assertTrue(f.ready)
        self.assertEqual(f.center, (*self.neutral_left, *self.neutral_right))

    def test_click_suppresses_motion_until_released_and_stationary(self):
        f = self.filter
        for t in (0., .1, .3):
            self.assertEqual(f.update((-1., .15), (0., .8), pressed=True, now=t), (0., 0., 0.))
        self.assertFalse(f.ready)
        self.assertEqual(f.update((-1., .15), (.0, .8), now=.35), (0., 0., 0.))
        self.assertFalse(f.ready)
        self.calibrate_after(.4)
        self.assertTrue(f.ready)

    def calibrate_after(self, t):
        f = self.filter
        f.update(self.neutral_left, self.neutral_right, now=t)
        f.update(self.neutral_left, self.neutral_right, now=t+.32)

    def test_zero_bias_no_creep_and_diagonal_suppressed(self):
        self.calibrate()
        f = self.filter
        for index in range(40):
            cmd = f.update(self.neutral_left, self.neutral_right, now=.4+index*.03)
            self.assertEqual(cmd, (0., 0., 0.))
        # Up, with a substantial horizontal stick error and yaw request.
        for index in range(18):
            cmd = f.update((-.85, .35), (.08, -.85), now=1.7+index*.03)
            self.assertGreaterEqual(cmd[0], 0.)
            self.assertEqual(cmd[1], 0.)
            self.assertEqual(cmd[2], 0.)
        self.assertGreater(cmd[0], .10)
        self.assertLessEqual(cmd[0], .2)

    def test_release_ramps_to_zero_without_backwards_kick(self):
        self.calibrate()
        f = self.filter
        for index in range(16):
            f.update((-.9, -.09), self.neutral_right, now=.4+index*.03)
        values = []
        for index in range(20):
            # Small zero drift around calibrated center must not reverse base.
            cmd = f.update((.12+(.015 if index % 2 else -.015), -.09),
                           self.neutral_right, now=.9+index*.03)
            self.assertEqual((cmd[1], cmd[2]), (0., 0.))
            values.append(cmd[0])
        self.assertTrue(all(a >= b >= -1e-9 for a, b in zip(values, values[1:])))
        self.assertAlmostEqual(values[-1], 0., places=6)

    def test_brief_reversal_bounce_and_axis_change_never_diagonal(self):
        self.calibrate()
        f = self.filter
        for index in range(20):
            f.update((-.9, -.09), self.neutral_right, now=.4+index*.03)
        # One-frame reverse spike on release is not an intentional reverse command.
        a = f.update((.9, -.09), self.neutral_right, now=1.01)
        self.assertGreaterEqual(a[0], 0.)
        # Change from forward to sideways without waiting for brake release.
        f.update(self.neutral_left, self.neutral_right, now=1.04)
        for index in range(22):
            cmd = f.update((.12, .9), self.neutral_right, now=1.07+index*.03)
            self.assertFalse(abs(cmd[0]) > 1e-7 and abs(cmd[1]) > 1e-7)
        self.assertGreater(cmd[1], 0.)

    def test_biased_deflected_input_cannot_be_auto_zero(self):
        f = self.filter
        for now in (.0, .4, 1.):
            self.assertEqual(f.update((-.8, 0.), (0., 0.), now=now), (0., 0., 0.))
        self.assertFalse(f.ready)
        self.assertIn('neutral', f.reason)

    def test_invalid_and_stale_input_disarms_and_zeros(self):
        self.calibrate()
        f = self.filter
        f.update((-.9, -.09), self.neutral_right, now=.4)
        self.assertGreater(f.output[0], 0.)
        with self.assertRaises(ValueError):
            f.update((math.nan, 0.), self.neutral_right, now=.45)
        self.assertEqual(f.output, (0., 0., 0.))
        self.assertFalse(f.ready)
        self.calibrate_after(.6)
        self.assertTrue(f.ready)
        self.assertEqual(f.update(self.neutral_left, self.neutral_right, now=.1), (0., 0., 0.))
        self.assertFalse(f.ready)


if __name__ == '__main__':
    unittest.main()
