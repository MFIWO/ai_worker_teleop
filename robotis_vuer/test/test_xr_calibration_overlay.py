"""[ADDED] Read-only AR overlay checks. No simulator or joint commands."""
import asyncio
from copy import deepcopy
import json
import os
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from robotis_vuer.xr_arm_targets import CalibratedWristMapper
from robotis_vuer.xr_resume_session import FixedResumeSession
from robotis_vuer.xr_calibration_overlay import (
    controller_pose_for_robot, overlay_state, render_overlay, ROOT_KEY,
)
from test_xr_resume import fixture


class OverlayMathTests(unittest.TestCase):
    def setUp(self):
        self.head, self.inputs, self.robots, self.links = fixture()
        self.session = FixedResumeSession(self.head, self.inputs, self.robots, self.links, 1.)

    def test_inverse_with_nonidentity_rotations_scale_and_both_arms(self):
        rng = np.random.default_rng(231)
        for scale in (.1, .5, 1., 1.5):
            inputs, robots = deepcopy(self.inputs), deepcopy(self.robots)
            for side in inputs:
                inputs[side][:3, :3] = Rotation.random(random_state=rng).as_matrix()
                robots[side][:3, :3] = Rotation.random(random_state=rng).as_matrix()
            mapper = CalibratedWristMapper(self.head, inputs, robots, scale)
            for _ in range(20):
                for side in inputs:
                    live = inputs[side].copy()
                    live[:3, 3] += rng.uniform(-.1, .1, 3)
                    live[:3, :3] = Rotation.random(random_state=rng).as_matrix()
                    goal = mapper.target(side, live)
                    np.testing.assert_allclose(controller_pose_for_robot(mapper, side, goal),
                                               live, atol=1e-12)

    def test_target_does_not_follow_live_hand_or_head(self):
        a = overlay_state(self.session, self.inputs, self.robots, 'WAIT_START', True, self.head)
        changed = deepcopy(self.inputs)
        changed['left'][0, 3] += .06
        head = self.head.copy(); head[0, 3] += .3
        head[:3, :3] = Rotation.from_euler('y', 40, degrees=True).as_matrix() @ head[:3, :3]
        b = overlay_state(self.session, changed, self.robots, 'WAIT_START', True, head)
        np.testing.assert_array_equal(a['arms']['left']['target'], b['arms']['left']['target'])
        self.assertNotEqual(a['panel_position'], b['panel_position'])
        self.assertFalse(b['arms']['left']['matched'])

    def test_realign_target_is_current_robot_not_original_robot_pose(self):
        hand = deepcopy(self.inputs); hand['left'][2, 3] -= .04
        robots = {s: self.session.mapper.target(s, hand[s]) for s in hand}
        state = overlay_state(self.session, self.inputs, robots, 'ALIGN_REQUIRED', True)
        np.testing.assert_allclose(state['arms']['left']['target'], hand['left'], atol=1e-12)

    def test_orientation_must_match_even_when_position_matches(self):
        hand = deepcopy(self.inputs)
        hand['left'][:3, :3] = Rotation.from_euler('x', 11, degrees=True).as_matrix()
        state = overlay_state(self.session, hand, self.robots, 'WAIT_START', True)
        self.assertFalse(state['arms']['left']['matched'])
        self.assertAlmostEqual(state['arms']['left']['robot_error_mm'], 0.)
        self.assertAlmostEqual(state['arms']['left']['rotation_error_deg'], 11.)

    def test_two_centimetre_gate_not_visual_sphere_size_heuristic(self):
        for delta, expected in ((.0199, True), (.0201, False)):
            hand = deepcopy(self.inputs); hand['left'][0, 3] += delta
            state = overlay_state(self.session, hand, self.robots, 'WAIT_START', True)
            self.assertEqual(state['arms']['left']['matched'], expected)

    def test_held_and_measured_both_checked(self):
        held = deepcopy(self.robots); held['left'][0, 3] += .03
        state = overlay_state(self.session, self.inputs, self.robots, 'WARMUP', True, held=held)
        self.assertFalse(state['arms']['left']['matched'])
        returned = self.session.mapper.target('left', np.array(state['arms']['left']['target']))
        np.testing.assert_allclose(returned, held['left'], atol=1e-12)

    def test_scale_changes_display_sphere_but_not_robot_tolerance(self):
        self.session.mapper.scale = .5
        hand = deepcopy(self.inputs); hand['left'][0, 3] += .03
        state = overlay_state(self.session, hand, self.robots, 'WAIT_START', True)
        self.assertAlmostEqual(state['arms']['left']['tolerance_radius_m'], .04)
        self.assertTrue(state['arms']['left']['matched'])

    def test_stale_or_fault_hides_all_old_markers(self):
        for fresh, fault in ((False, False), (True, True)):
            state = overlay_state(self.session, self.inputs, self.robots, 'WAIT_START', fresh, fault=fault)
            self.assertEqual(state['arms'], {})
            text = json.dumps(render_overlay(state)._serialize(), allow_nan=False)
            self.assertNotIn('Sphere', text)
            self.assertNotIn('cal-left-target', text)

    def test_first_calibration_has_no_invented_target(self):
        state = overlay_state(None, self.inputs, self.robots, 'WAIT_CALIBRATION', True)
        self.assertIsNone(state['arms']['left']['target'])
        self.assertIn('FIRST CALIBRATION', state['message'])

    def test_no_session_mutation_after_repeated_rendering(self):
        before = deepcopy(self.session.__dict__)
        for _ in range(30):
            state = overlay_state(self.session, self.inputs, self.robots, 'ALIGN_REQUIRED', True)
            render_overlay(state)
        np.testing.assert_array_equal(before['mapper'].basis, self.session.mapper.basis)
        for side in self.inputs:
            np.testing.assert_array_equal(before['mapper'].anchors[side].input_pose,
                                          self.session.mapper.anchors[side].input_pose)
            np.testing.assert_array_equal(before['templates'][side].previous,
                                          self.session.templates[side].previous)
            self.assertEqual(before['templates'][side].maximum, self.session.templates[side].maximum)

    def test_native_3d_serialization_and_unique_keys(self):
        from vuer.events import Upsert
        state = overlay_state(self.session, self.inputs, self.robots, 'WAIT_START', True)
        packet = Upsert(render_overlay(state), strict=True)._serialize()
        json.dumps(packet, allow_nan=False)
        keys = []
        def visit(x):
            if isinstance(x, dict):
                if 'key' in x: keys.append(x['key'])
                for value in x.values(): visit(value)
            elif isinstance(x, (list, tuple)):
                for value in x: visit(value)
        visit(packet)
        self.assertIn(ROOT_KEY, keys)
        self.assertEqual(len(keys), len(set(keys)))
        self.assertNotIn('Html', json.dumps(packet))

    def test_invalid_inverse_input_rejected(self):
        bad = self.robots['left'].copy(); bad[0, 0] = float('nan')
        with self.assertRaises(ValueError):
            controller_pose_for_robot(self.session.mapper, 'left', bad)


class GuidedNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '232':
            raise unittest.SkipTest('read-only tests require isolated simulation domain232')
        import rclpy
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        import rclpy
        rclpy.shutdown()

    def setUp(self):
        from robotis_vuer.vr_publisher_sg2_xr_guided import XRGuidedPublisher
        self.node = XRGuidedPublisher(start_server=False)
        self.head, self.inputs, self.robots, self.links = fixture()
        self.node.head_transform_matrix = self.head.copy()
        self.node._session = FixedResumeSession(self.head, self.inputs, self.robots, self.links, 1.)
        self.node._phase = 'WAIT_START'
        self.node._goals = deepcopy(self.robots)
        self.node._snapshot = lambda: deepcopy(self.links)
        self.node._arm_tracking_is_fresh = lambda now=None: True
        for side in self.inputs:
            setattr(self.node, side + '_controller_matrix', self.inputs[side].copy())

    def tearDown(self):
        self.node.destroy_node()

    def test_view_does_not_publish_goals_or_change_phase(self):
        count = self.node.published_pairs
        self.node._publish_pair = lambda: self.fail('overlay must not publish goals')
        self.node._set_arm_control_enabled = lambda *args: self.fail('overlay must not change activation')
        a = self.node._overlay_snapshot()
        self.assertTrue(a['arms']['left']['matched'])
        self.assertEqual(self.node.published_pairs, count)
        self.assertEqual(self.node._phase, 'WAIT_START')

    def test_all_control_methods_inherited_without_changes(self):
        from robotis_vuer.vr_publisher_sg2_xr_extension import XRExtensionPublisher
        from robotis_vuer.vr_publisher_sg2_xr_guided import XRGuidedPublisher
        for method in ('_capture', '_set_arm_control_enabled', '_reference_fault',
                       '_publish_synced_pose_frame_if_ready', '_command_feedback',
                       'on_body_tracking_move', 'on_controller_move', '_graph_check'):
            self.assertIs(getattr(XRGuidedPublisher, method), getattr(XRExtensionPublisher, method))

    def test_real_scene_loop_keeps_stream_components_and_sends_upsert(self):
        from types import SimpleNamespace
        class Capture:
            def __init__(self): self.values = []
            def __matmul__(self, value): self.values.append(value)
            def __call__(self, value, **kwargs): self.values.append((value, kwargs))
        fake = SimpleNamespace(set=Capture(), upsert=Capture())
        async def run():
            task = asyncio.create_task(self.node.main_hand_tracking(fake))
            await asyncio.sleep(.13)
            task.cancel()
            try: await task
            except asyncio.CancelledError: pass
        asyncio.run(run())
        scene = fake.set.values[0]._serialize()
        text = json.dumps(scene)
        self.assertIn('MotionControllers', text)
        self.assertIn('body_tracking', text)
        self.assertNotIn('SceneBackground', text)
        self.assertGreaterEqual(len(fake.upsert.values), 1)
        for element, options in fake.upsert.values:
            self.assertTrue(options['strict'])
            self.assertEqual(element.key, ROOT_KEY)


if __name__ == '__main__':
    unittest.main()
