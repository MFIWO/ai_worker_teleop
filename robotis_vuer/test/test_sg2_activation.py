"""Exercise controller callbacks without ROS nodes, publishers, or robot commands."""

import asyncio
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from robotis_vuer.vr_publisher_sg2 import VRTrajectoryPublisher


class ControllerHarness:
    """Use production callbacks with in-memory publishers."""


for name in (
    '_arm_tracking_is_fresh', '_set_arm_control_enabled',
    '_check_arm_tracking_timeout', '_update_arm_activation',
    'can_publish_goal_pose', 'on_controller_move', 'is_valid_float',
    'calibrate_trigger',
):
    setattr(ControllerHarness, name, getattr(VRTrajectoryPublisher, name))


class ControllerActivationTests(unittest.TestCase):
    def setUp(self):
        self.node = n = ControllerHarness()
        n.arm_activation_lock = threading.RLock()
        n.arm_tracking_timeout_sec = 0.5
        n.arm_control_enabled = False
        n.arm_start_armed = False
        n.vr_publishing_enabled = True
        n.last_body_pose_sec = time.monotonic()
        n.last_controller_pose_sec = {'left': None, 'right': None}
        n.left_controller_state = {}
        n.right_controller_state = {}
        n.both_a_buttons_pressed_prev = False
        n.both_b_buttons_pressed_prev = False
        n.pending_body_pose_frame = False
        n.pending_controller_pose_frame = False
        n.controller_log_counter = 0
        n.log_every_n = 10000
        n.trigger_offsets = {'left': 0., 'right': 0.}
        n.trigger_scales = {'left': 1., 'right': 1.}
        n.left_gripper_max_position = n.right_gripper_max_position = 1.3
        for name in ('left_gripper_pub', 'right_gripper_pub', 'left_squeeze_pub',
                     'right_squeeze_pub', 'process_thumbstick',
                     '_publish_reactivate', '_publish_synced_pose_frame_if_ready'):
            setattr(n, name, Mock())
        n.get_logger = Mock(return_value=Mock())

    def event(self, start=False, stop=False, grip=0., trigger=0., missing=None,
              invalid=None):
        data = {}
        for side in ('left', 'right'):
            if side == missing:
                continue
            data[side + 'State'] = {
                'aButton': start, 'bButton': stop,
                'squeezeValue': grip, 'triggerValue': trigger,
            }
            data[side] = (np.zeros((4, 4)) if side == invalid else np.eye(4)).flatten(
                order='F').tolist()
        asyncio.run(self.node.on_controller_move(SimpleNamespace(value=data), None))
        self.node.get_logger().error.assert_not_called()

    def activate(self):
        self.event()
        self.event(start=True)
        self.assertTrue(self.node.can_publish_goal_pose())

    def test_start_latches_without_grip_and_button_release_preserves_it(self):
        self.activate()
        self.event(start=True)
        self.event(grip=1.)
        self.event(grip=0.)
        self.assertTrue(self.node.can_publish_goal_pose())
        self.node._publish_reactivate.assert_called_once_with(
            True, reason='X+A buttons', force_log=True)

    def test_stop_wins_over_start_and_requires_release_before_restart(self):
        self.activate()
        self.event()
        self.event(start=True, stop=True)
        self.assertFalse(self.node.can_publish_goal_pose())
        self.event(start=True)
        self.assertFalse(self.node.can_publish_goal_pose())
        self.event()
        self.event(start=True)
        self.assertTrue(self.node.can_publish_goal_pose())

    def test_timeout_stops_and_held_start_cannot_restart_on_reconnect(self):
        self.activate()
        self.node.last_body_pose_sec -= 1.
        self.node._check_arm_tracking_timeout()
        self.assertFalse(self.node.can_publish_goal_pose())
        self.node._publish_reactivate.assert_called_with(
            False, reason='tracking timeout', force_log=True)
        self.node.last_body_pose_sec = time.monotonic()
        self.event(start=True)
        self.assertFalse(self.node.can_publish_goal_pose())
        self.event()
        self.event(start=True)
        self.assertTrue(self.node.can_publish_goal_pose())

    def test_held_start_on_initial_connection_does_not_activate(self):
        self.event(start=True)
        self.assertFalse(self.node.can_publish_goal_pose())
        self.activate()

    def test_missing_controller_cannot_form_start_chord_from_cached_state(self):
        self.event()
        self.event(start=True, missing='right')
        self.event(start=True, missing='left')
        self.assertFalse(self.node.can_publish_goal_pose())
        self.node._publish_reactivate.assert_not_called()

    def test_invalid_controller_pose_does_not_refresh_tracking(self):
        self.activate()
        self.node.last_controller_pose_sec['left'] -= 1.
        self.event(invalid='left')
        self.assertFalse(self.node.can_publish_goal_pose())
        self.node._publish_reactivate.assert_called_with(
            False, reason='tracking unavailable', force_log=True)

    def test_trigger_still_controls_both_grippers_without_grip(self):
        self.activate()
        for trigger, expected in ((0., 0.), (.5, .65), (1., 1.3)):
            self.event(trigger=trigger)
            for side in ('left', 'right'):
                msg = getattr(self.node, side + '_gripper_pub').publish.call_args.args[0]
                self.assertEqual(msg.joint_names, [f'gripper_{side[0]}_joint1'])
                self.assertAlmostEqual(msg.points[0].positions[0], expected)


if __name__ == '__main__':
    unittest.main(verbosity=2)
