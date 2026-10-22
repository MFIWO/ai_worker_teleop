"""[ADDED] Config profile tests: no ROS, motor commands or simulator required."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml

path = Path(__file__).resolve().parents[2] / 'sim/make_shoulder_free_config.py'
spec = importlib.util.spec_from_file_location('shoulder_profile', path)
profile = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profile)


class ShoulderFreeConfigTests(unittest.TestCase):
    def sample(self):
        return {'vr_controller': {'ros__parameters': {
            'weight_elbow_position': 8.0, 'weight_position': 10.0,
            'weight_orientation': 1.0, 'weight_damping': .1,
            'collision_safe_distance': .02, 'joint_state_timeout': .5}},
            'reference_checker': {'ros__parameters': {'position_threshold': .1}}}

    def test_exactly_one_parameter_changed(self):
        source = self.sample()
        new = profile.shoulder_free_config(source)
        self.assertEqual(source['vr_controller']['ros__parameters']['weight_elbow_position'], 8.)
        self.assertEqual(new['vr_controller']['ros__parameters']['weight_elbow_position'], 0.)
        new['vr_controller']['ros__parameters']['weight_elbow_position'] = 8.
        self.assertEqual(new, source)

    def test_original_readonly_and_idempotent_output(self):
        with tempfile.TemporaryDirectory() as folder:
            source, output = Path(folder)/'source.yaml', Path(folder)/'new.yaml'
            source.write_text(yaml.safe_dump(self.sample()))
            before = source.read_bytes()
            profile.create_profile(source, output)
            profile.create_profile(source, output)
            self.assertEqual(source.read_bytes(), before)
            with self.assertRaises(ValueError):
                profile.create_profile(source, source)

    def test_unrelated_output_refused(self):
        with tempfile.TemporaryDirectory() as folder:
            source, output = Path(folder)/'source.yaml', Path(folder)/'new.yaml'
            source.write_text(yaml.safe_dump(self.sample()))
            output.write_text('unrelated: original user config')
            with self.assertRaises(ValueError):
                profile.create_profile(source, output)

    def test_invalid_weights_refused(self):
        for value in (-1., float('nan'), float('inf'), '0'):
            source = self.sample()
            source['vr_controller']['ros__parameters']['weight_elbow_position'] = value
            with self.assertRaises(ValueError):
                profile.shoulder_free_config(source)


if __name__ == '__main__':
    unittest.main()
