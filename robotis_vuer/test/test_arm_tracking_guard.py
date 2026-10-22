"""Tracking dropout and reacquisition tests, without a ROS graph."""

import unittest

import numpy as np
from robotis_vuer.arm_tracking_guard import ArmTrackingGuard
from scipy.spatial.transform import Rotation


def poses(x=0., angle=0.):
    return {'left': (np.array([x, 0., 0.]),
                     Rotation.from_euler('y', angle, degrees=True).as_quat()),
            'right': (np.zeros(3), np.array([0., 0., 0., 1.]))}


class TrackingGuardTests(unittest.TestCase):
    def setUp(self):
        self.guard = ArmTrackingGuard()
        self.assertFalse(self.guard.accept(poses(), 0.))
        self.assertTrue(self.guard.accept(poses(), .31))

    def test_normal_continuous_motion(self):
        self.assertTrue(self.guard.accept(poses(.01, 3.), .34))
        self.assertFalse(self.guard.holding)

    def test_logged_jump_holds_both_sides_without_advancing_reference(self):
        for jump in (.163, .133):
            self.assertFalse(self.guard.accept(poses(jump), 1.))
            self.assertFalse(self.guard.accept(poses(jump), 20.))
            np.testing.assert_array_equal(self.guard.last['left'][0], np.zeros(3))
            self.assertEqual(self.guard.reason, 'return_to_held_pose')

    def test_dropout_requires_stable_nearby_reacquisition(self):
        self.guard.hold('tracking_missing')
        self.assertFalse(self.guard.accept(poses(.01), 1.))
        self.assertFalse(self.guard.accept(poses(.01), 1.2))
        self.assertTrue(self.guard.accept(poses(.01), 1.31))

    def test_far_reacquisition_cannot_resume_just_by_waiting(self):
        self.guard.hold('tracking_missing')
        self.assertFalse(self.guard.accept(poses(.4), 1.))
        self.assertFalse(self.guard.accept(poses(.4), 100.))
        self.assertTrue(self.guard.holding)

    def test_orientation_jump_and_invalid_data_hold(self):
        self.assertFalse(self.guard.accept(poses(angle=40.), 1.))
        self.assertFalse(self.guard.accept(poses(float('nan')), 2.))
        self.assertEqual(self.guard.reason, 'invalid_pose')

    def test_unstable_return_restarts_stability_period(self):
        self.guard.hold('tracking_missing')
        self.assertFalse(self.guard.accept(poses(-.02), 1.))
        self.assertFalse(self.guard.accept(poses(.02), 1.2))
        self.assertFalse(self.guard.accept(poses(.02), 1.4))
        self.assertTrue(self.guard.accept(poses(.02), 1.51))

    def test_incomplete_pose_set_is_rejected(self):
        self.assertFalse(self.guard.accept({'left': poses()['left']}, 1.))


if __name__ == '__main__':
    unittest.main(verbosity=2)
