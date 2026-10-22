"""[ADDED 2026-10-14] Numeric-only patch contract; no ROS or motor calls."""
import unittest

from hardware.build_qp_precision import ANCHOR, precision_header


class PrecisionPatchTests(unittest.TestCase):
    def test_only_termination_settings_are_added(self):
        original = '#pragma once\nvoid initialize() {\n' + ANCHOR + '\n}\n'
        actual = precision_header(original)
        self.assertIn('setAbsoluteTolerance(1e-5);', actual)
        self.assertIn('setRelativeTolerance(1e-5);', actual)
        remaining = [line for line in actual.splitlines()
                     if not line.strip().startswith('//') and 'Tolerance(' not in line]
        self.assertEqual('\n'.join(remaining) + '\n', original)

    def test_unknown_or_already_patched_source_is_refused(self):
        for source in ('', ANCHOR + '\n' + ANCHOR,
                       ANCHOR + '\nsolver_.settings()->setRelativeTolerance(.1);'):
            with self.subTest(source=source), self.assertRaises(ValueError):
                precision_header(source)


if __name__ == '__main__':
    unittest.main()
