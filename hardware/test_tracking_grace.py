"""Exercise VR wait transitions without ROS, network, or actuator I/O."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

SOURCE = Path(__file__).resolve().parents[1] / 'robotis_vuer/robotis_vuer/vr_publisher_ai_worker_hardware.py'


def isolated_publisher():
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'HardwareXRPublisher')
    names = {'_tracking_input_state', '_feedback_input_state', '_arm_tracking_is_fresh', '_hold_for_tracking_gap'}
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    clock = SimpleNamespace(now=10.0)
    namespace = {'time': SimpleNamespace(monotonic=lambda: clock.now), 'SIDES': ('left', 'right')}
    isolated = ast.ClassDef(name='Publisher', bases=[], keywords=[], body=methods, decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[isolated], type_ignores=[])), str(SOURCE), 'exec'), namespace)
    node = namespace['Publisher']()
    node.TRACKING_GRACE_SECONDS = 5.0
    node.arm_tracking_timeout_sec = 0.5
    node.last_body_pose_sec = 9.0
    node.last_controller_pose_sec = {'left': 9.0, 'right': 9.0}
    node._joint_rx = 9.9
    node.arm_control_enabled = True
    node._phase = 'ACTIVE'
    node._tracking_waiting = False
    node._last_pub = 9.9
    node._command_rx = {'left': 9.9, 'right': 9.9}
    node._gate_state = None
    node._gate_rx = 9.9
    node._aux_base = [0.2, 0.0, 0.0]
    node._aux_joints = {'lift_joint': 0.1}
    node.goals = (1, 2, 3)
    node.outputs, node.events, node.stops = [], [], []
    node._publish_pair = lambda: node.outputs.append(node.goals)
    node._graph_check = lambda: None
    node._snapshot = lambda: None
    node._event = lambda kind, reason: node.events.append(kind)
    node._heartbeat = lambda: None
    node._tracking_timeout_reason = lambda now: 'tracking/feedback timeout'
    def stop(enabled, reason):
        node.arm_control_enabled = enabled
        node.stops.append(reason)
    node._set_arm_control_enabled = stop
    return node, clock


class TrackingGraceTests(unittest.TestCase):
    def test_gap_holds_goals_without_disarming_and_clears_auxiliary_motion(self):
        n, _ = isolated_publisher()
        self.assertTrue(n._hold_for_tracking_gap())
        self.assertTrue(n.arm_control_enabled)
        self.assertEqual(n.outputs, [n.goals])
        self.assertEqual(n._aux_base, [0., 0., 0.])
        self.assertEqual(n._aux_joints, {})
        self.assertEqual(n.events, ['TRACKING_WAIT'])

    def test_recovery_continues_without_arming_or_accumulated_time(self):
        n, clock = isolated_publisher()
        n._hold_for_tracking_gap()
        clock.now = 10.2
        n.last_body_pose_sec = 10.2
        n.last_controller_pose_sec = {'left': 10.2, 'right': 10.2}
        self.assertFalse(n._hold_for_tracking_gap())
        self.assertEqual(n.events, ['TRACKING_WAIT', 'TRACKING_RECOVERED'])
        self.assertEqual(n._last_pub, 10.2)
        self.assertTrue(n.arm_control_enabled)

    def test_five_seconds_is_maximum_age_not_five_extra_seconds(self):
        n, clock = isolated_publisher()
        clock.now = 14.0
        n._joint_rx = 14.0
        n._command_rx = {'left': 14.0, 'right': 14.0}
        self.assertTrue(n._arm_tracking_is_fresh())
        clock.now = 14.001
        self.assertFalse(n._arm_tracking_is_fresh())
        self.assertTrue(n._hold_for_tracking_gap())
        self.assertFalse(n.arm_control_enabled)

    def test_robot_feedback_gap_waits_without_disarming(self):
        n, _ = isolated_publisher()
        n._joint_rx = 9.0
        n._hold_for_tracking_gap()
        self.assertTrue(n.arm_control_enabled)
        self.assertEqual(n.outputs, [n.goals])

    def test_missing_cyclo_commands_wait_without_disarming(self):
        n, _ = isolated_publisher()
        n._command_rx['right'] = 9.0
        n._hold_for_tracking_gap()
        self.assertTrue(n.arm_control_enabled)
        self.assertEqual(n.outputs, [n.goals])

    def test_feedback_wait_does_not_use_stale_snapshot(self):
        n, _ = isolated_publisher()
        n.last_body_pose_sec = 9.9
        n.last_controller_pose_sec = {'left': 9.9, 'right': 9.9}
        n._joint_rx = 9.
        def reject():
            raise AssertionError('stale snapshot used during wait')
        n._snapshot = reject
        self.assertTrue(n._hold_for_tracking_gap())
        self.assertTrue(n.arm_control_enabled)

    def test_feedback_wait_expires_at_five_seconds(self):
        for field in ('joint', 'command'):
            n, _ = isolated_publisher()
            if field == 'joint':
                n._joint_rx = 4.999
            else:
                n._command_rx['right'] = 4.999
            self.assertTrue(n._hold_for_tracking_gap())
            self.assertFalse(n.arm_control_enabled)

    def test_gate_wait_is_observed_even_when_held_commands_keep_arriving(self):
        n, _ = isolated_publisher()
        n.last_body_pose_sec = n._joint_rx = 9.9
        n.last_controller_pose_sec = {'left': 9.9, 'right': 9.9}
        n._gate_state = {'feedback_waiting': True}
        self.assertTrue(n._hold_for_tracking_gap())
        self.assertTrue(n.arm_control_enabled)
        self.assertEqual(n.outputs, [n.goals])

    def test_gate_status_gap_waits_up_to_five_seconds(self):
        n, _ = isolated_publisher()
        n._gate_state = {'feedback_waiting': False}
        n._gate_rx = 9.
        self.assertTrue(n._hold_for_tracking_gap())
        self.assertTrue(n.arm_control_enabled)
        n._gate_rx = 4.999
        self.assertTrue(n._hold_for_tracking_gap())
        self.assertFalse(n.arm_control_enabled)

    def test_grace_does_not_allow_startup_with_stale_inputs(self):
        n, _ = isolated_publisher()
        n._phase = 'WARMUP'
        self.assertFalse(n._arm_tracking_is_fresh())
        n._phase = 'ACTIVE'
        n.arm_control_enabled = False
        self.assertFalse(n._arm_tracking_is_fresh())

    def test_missing_future_and_partial_inputs_are_rejected(self):
        for value in (None, 11.0):
            n, _ = isolated_publisher()
            n.last_controller_pose_sec['right'] = value
            self.assertFalse(n._arm_tracking_is_fresh())
        n, _ = isolated_publisher()
        n.last_controller_pose_sec = {'left': 9.9}
        self.assertFalse(n._arm_tracking_is_fresh())


if __name__ == '__main__':
    unittest.main()
