"""[ADDED 2026-10-28] Offline controller regression tests; no ROS/motors."""
import math
import unittest

from hardware.controller_stick_motion import MobileStickFilter, head_targets, lift_target, BodyMotionPolicy


HEAD = {
    'head_joint1': {'lower': -0.2317, 'upper': 0.6951, 'velocity': 4.8},
    'head_joint2': {'lower': -0.35, 'upper': 0.35, 'velocity': 4.8},
}


class HeadJogTests(unittest.TestCase):
    def test_uses_physical_urdf_and_time_based_rate(self):
        result = head_targets((1., -1.), None, (0., 0.), HEAD, 1 / 30)
        self.assertAlmostEqual(result[0], 0.01, places=5)
        self.assertEqual(result[1], 0.)
        self.assertLess(result[0], 0.06)  # old per-event jump

    def test_saturates_not_stops_at_both_joint_bounds(self):
        result = head_targets((1., -1.), (.694, -.349), (0., 0.), HEAD, .1)
        self.assertEqual(result, (.6951, -.349))
        self.assertEqual(head_targets((1., -1.), result, (0., 0.), HEAD, .033), result)
        self.assertEqual(head_targets((0., -1.), result, (0.,0.), HEAD, .033),(.6951,-.35))

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

    def test_each_mobile_axis_reverses_without_neutral_in_both_directions(self):
        # Both controller hand modes share this filter. A held opposite input
        # must cross zero and move in reverse without an extra neutral sample.
        for axis in range(3):
            with self.subTest(axis=axis):
                self.setUp()
                self.calibrate()
                f = self.filter
                def sample(direction):
                    left, right = list(self.neutral_left), list(self.neutral_right)
                    if axis == 0: left[0] = -.9 * direction
                    elif axis == 1: left[1] = .9 * direction
                    else: right[1] = -.9 * direction
                    return left, right
                now = .4
                for direction in (1, -1, 1):
                    outputs = []
                    for _ in range(45):
                        outputs.append(f.update(*sample(direction), now=now))
                        now += .03
                    self.assertGreater(direction * outputs[-1][axis], .05)
                    for output in outputs:
                        self.assertTrue(all(value == 0. for index, value in enumerate(output)
                                            if index != axis))
                        self.assertLessEqual(abs(output[axis]), f.SPEED[axis])
                    self.assertTrue(f.ready)

    def test_jerk_limited_acceleration_and_release_no_sign_crossing(self):
        """[ADDED] d(speed)/dt changes smoothly in acceleration, release only brakes."""
        self.calibrate()
        f = self.filter
        velocity = []
        for index in range(9):
            velocity.append(f.update((-.95, -.09), self.neutral_right,
                                     now=.4 + .03 * index)[0])
        accelerations = [(b - a) / .03 for a, b in zip(velocity, velocity[1:])]
        for a, b in zip(accelerations, accelerations[1:]):
            self.assertLessEqual(abs(b - a), f.JERK[0] * .03 + 1e-8)

        brakes = [f.update(self.neutral_left, self.neutral_right,
                           now=.67 + .03 * index)[0] for index in range(20)]
        self.assertTrue(all(v >= 0. for v in brakes))
        self.assertTrue(all(old >= new for old, new in zip(brakes, brakes[1:])))
        self.assertAlmostEqual(brakes[-1], 0.)

    def test_yaw_to_forward_never_stacks_rotation_and_translation(self):
        """[ADDED] Mode changes cannot blend old yaw and new straight travel."""
        self.calibrate()
        f = self.filter
        for index in range(20):
            f.update(self.neutral_left, (.08, -.95), now=.4 + .03 * index)
        self.assertGreater(f.output[2], 0.)
        for index in range(35):
            v = f.update((-.95, -.09), (.08, -.95),
                         now=1.0 + index * .03)
            self.assertFalse(abs(v[2]) > 1e-7 and
                             (abs(v[0]) > 1e-7 or abs(v[1]) > 1e-7))
        self.assertGreater(f.output[0], 0.)
        self.assertEqual(f.output[2], 0.)

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



class RightDiagonalTests(unittest.TestCase):
    def test_diagonal_lift_and_yaw_with_original_signs(self):
        limits={'lift_joint':dict(lower=-.5,upper=0.,velocity=.3)}
        f=MobileStickFilter();f.update((0.,0.),(0.,0.),now=0.);f.update((0.,0.),(0.,0.),now=.31)
        target=-.25
        for i in range(20):
            target=lift_target(.707,target,-.25,limits,.03)
            base=f.update((0.,0.),(.707,.707),now=.34+i*.03)
        self.assertGreater(target,-.25)
        self.assertLess(base[2],0.)
        self.assertEqual(base[:2],(0.,0.))
        self.assertLess(lift_target(-.707,None,-.25,limits,.03),-.25)

    def test_lift_time_rate_bounds_and_bad_input(self):
        limits={'lift_joint':dict(lower=-.5,upper=0.,velocity=.3)}
        a=b=-.25
        for _ in range(30):a=lift_target(.707,a,-.25,limits,1/30)
        for _ in range(60):b=lift_target(.707,b,-.25,limits,1/60)
        self.assertAlmostEqual(a,b)
        self.assertEqual(lift_target(1.,-.001,0.,limits,.05),0.)
        with self.assertRaises(ValueError):lift_target(float('nan'),None,-.25,limits,.03)


class BodyMotionTests(unittest.TestCase):
    def test_motion_hysteresis_and_measured_lift(self):
        p=BodyMotionPolicy()
        self.assertFalse(p.update(0.,(0.,0.,0.),-.2,None))
        self.assertTrue(p.update(.1,(0.,0.,.1),-.2,None))
        self.assertTrue(p.update(.2,(0.,0.,0.),-.2,None))
        self.assertFalse(p.update(.4,(0.,0.,0.),-.2,None))
        self.assertTrue(p.update(.5,(0.,0.,0.),-.21,None))
        self.assertTrue(p.update(.6,(0.,0.,0.),-.21,-.25))
        self.assertFalse(p.update(1.,(0.,0.,0.),-.21,-.21))

if __name__ == '__main__':
    unittest.main()
