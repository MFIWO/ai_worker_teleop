"""[ADDED] Extension geometry/profile tests; no joint commands or simulator."""
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from copy import deepcopy

import numpy as np
from scipy.spatial.transform import Rotation as R

from robotis_vuer.xr_arm_targets import ContinuousElbowHint
from robotis_vuer.xr_extension_math import bend_degrees, extended_templates
from robotis_vuer.xr_resume_session import FixedResumeSession

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('make_extension_profile', ROOT/'sim/make_extension_profile.py')
profile = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profile)


class ExtensionTests(unittest.TestCase):
    def test_bend_angle_not_q4(self):
        u = np.array([.041004, 0, -.300])
        f = np.array([-.041004, 0, -.1489])
        q_straight = -(np.arctan2(.041004, .300)+np.arctan2(.041004, .1489))
        self.assertAlmostEqual(q_straight, -.4045572367, places=8)
        self.assertLess(bend_degrees(np.zeros(3), u, u+R.from_rotvec([0,q_straight,0]).apply(f)), 1e-8)
        self.assertGreater(bend_degrees(np.zeros(3), u, u+f), 20)
        beta = bend_degrees(np.zeros(3), u, u+R.from_rotvec([0,-.45,0]).apply(f))
        self.assertTrue(2 < beta < 3)

    def test_bad_geometry_rejected(self):
        with self.assertRaises(ValueError):
            bend_degrees([0,0,0], [0,0,0], [1,0,0])
        with self.assertRaises(ValueError):
            bend_degrees([0,0,float('nan')], [0,0,0], [1,0,0])

    def test_extended_shell_preserves_old(self):
        hint = ContinuousElbowHint(np.zeros(3), np.array([0,0,-.30]), np.array([.34,0,-.30]))
        templates = {'left': hint, 'right': deepcopy(hint)}
        maximum = hint.maximum
        result = extended_templates(templates)
        self.assertEqual(hint.maximum, maximum)
        self.assertAlmostEqual(result['left'].maximum, .995*(.3+.34))
        self.assertLess(result['left'].maximum, .3+.34)
        self.assertEqual(result['left'].minimum, hint.minimum)

    def test_resume_keeps_extension_and_mapping(self):
        head = np.eye(4); head[:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]
        poses = {s: np.eye(4) for s in ('left','right')}
        robots = {s: np.eye(4) for s in poses}
        for p in robots.values(): p[:3,3] = [.34,0,-.30]
        links = {s: {k: np.eye(4) for k in ('shoulder','elbow','wrist')} for s in poses}
        for s in links:
            links[s]['elbow'][:3,3] = [0,0,-.30]
            links[s]['wrist'] = robots[s].copy()
        session = FixedResumeSession(head, poses, robots, links, 1.0)
        session.templates = extended_templates(session.templates)
        initial = session.mapper.target('left', poses['left'])
        bound = session.templates['left'].maximum
        for _ in range(30):
            hints = session.fresh_hints(links)
            self.assertEqual(hints['left'].maximum, bound)
            np.testing.assert_array_equal(session.mapper.target('left', poses['left']), initial)
            session.require_alignment(poses, robots)

    def test_model_only_changes_two_uppers(self):
        text = '<robot name="test">' + ''.join(
            f'<joint name="arm_{s}_joint4" type="revolute"><axis xyz="0 1 0"/>'
            '<limit lower="-2.9361" upper="-0.7" effort="1000" velocity="4.8"/></joint>'
            for s in ('l','r')) + '</robot>'
        result = ET.fromstring(profile.make_urdf(text))
        for s in ('l','r'):
            limit = result.find(f"joint[@name='arm_{s}_joint4']/limit")
            self.assertEqual(limit.attrib, {'lower':'-2.9361','upper':'-0.45','effort':'1000','velocity':'4.8'})
        with self.assertRaises(ValueError): profile.make_urdf(text, 0.0)
        with self.assertRaises(ValueError): profile.make_urdf(text.replace('upper="-0.7"','upper="1.0"'))

    def test_all_other_config_preserved(self):
        original = {'vr_controller': {'ros__parameters': {'weight_elbow_position': 8.0, 'weight_position': 10.0, 'collision_safe_distance': .02}},
                    'reference_checker': {'ros__parameters': {'max_position_jump': .1}}}
        saved = deepcopy(original)
        result = profile.make_config(original)
        self.assertEqual(original, saved)
        result['vr_controller']['ros__parameters']['weight_elbow_position'] = 8.0
        result['vr_controller']['ros__parameters']['weight_position'] = 10.0
        self.assertEqual(result, original)


if __name__ == '__main__':
    unittest.main()
