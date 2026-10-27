"""Offline deployment checks. No ROS nodes, serial ports or motor commands."""
from pathlib import Path
import unittest
from unittest.mock import patch
from hardware.build_hand_write_rate import follower_environment,patch_source,replace_once


class HandWriteDeploymentTests(unittest.TestCase):
    def test_overlay_is_selected_only_in_the_follower_child_environment(self):
        original={'AMENT_PREFIX_PATH':'/original','LD_LIBRARY_PATH':'/original/lib','OTHER':'value'}
        with patch('hardware.build_hand_write_rate.verify') as verify:
            env=follower_environment(Path('/build'),base=original)
        verify.assert_called_once_with(Path('/build'))
        self.assertEqual(env['AI_WORKER_HX5_WRITE_HZ'],'30')
        self.assertEqual(env['AMENT_PREFIX_PATH'],'/build/install/dynamixel_hardware_interface:/original')
        self.assertEqual(env['LD_LIBRARY_PATH'],'/build/install/dynamixel_hardware_interface/lib:/original/lib')
        self.assertNotIn('AI_WORKER_HX5_WRITE_HZ',original)
        self.assertEqual(env['OTHER'],'value')

    def test_unverified_build_never_selects_a_driver(self):
        with patch('hardware.build_hand_write_rate.verify',side_effect=ValueError('bad manifest')):
            with self.assertRaisesRegex(ValueError,'bad manifest'):follower_environment(base={})

    def test_unknown_source_is_rejected_instead_of_a_partial_patch(self):
        with self.assertRaises(ValueError):patch_source('unrecognized driver version')
        for source in ('', 'anchor anchor'):
            with self.assertRaises(ValueError):replace_once(source,'anchor','patched')


if __name__=='__main__':unittest.main()
