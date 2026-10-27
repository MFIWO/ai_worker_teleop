"""Experimental bypass cannot be selected before physical qualification."""
import unittest
from unittest.mock import patch
from hardware.build_hand_direct import follower_environment, verify_running_follower, patch_source


class DirectDeploymentTests(unittest.TestCase):
    def test_failed_physical_qualification_prevents_operational_selection(self):
        with patch('hardware.build_hand_direct.verify'):
            with self.assertRaisesRegex(RuntimeError, 'disabled'):
                follower_environment(base={})
            with self.assertRaisesRegex(RuntimeError, 'not qualified'):
                verify_running_follower()

    def test_unknown_original_driver_fails_before_partial_patching(self):
        with self.assertRaises(ValueError):patch_source('unknown driver')


if __name__ == '__main__':unittest.main()
