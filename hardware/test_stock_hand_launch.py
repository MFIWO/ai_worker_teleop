"""Inspect the real launch description without launching ROS processes."""
import importlib.util
import os
from pathlib import Path
import unittest
from unittest.mock import patch


class StockHandLaunchTests(unittest.TestCase):
    def test_optical_hand_uses_original_cyclo_without_changing_arm_nodes(self):
        path=Path(__file__).with_name('hardware.launch.py')
        spec=importlib.util.spec_from_file_location('stock_hand_launch_fixture',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        actual_node=module.Node
        profile=dict(hand_control='robotis_native_measured_fingers',config='/tmp/config.yaml',
                     urdf='/tmp/model.urdf',srdf='/tmp/model.srdf')
        for mode in ('hands','controllers'):
            calls=[]
            processes=[]
            actual_process=module.ExecuteProcess
            def node(**kwargs):calls.append(kwargs);return actual_node(**kwargs)
            def process(**kwargs):processes.append(kwargs);return actual_process(**kwargs)
            with patch.dict(os.environ,AI_WORKER_XR_PROFILE='/tmp/profile.json',AI_WORKER_SH5_INPUT=mode,AI_WORKER_SH5_HAND_MODE='1'),\
                 patch.object(module,'load_profile',return_value=profile),patch.object(module,'Node',side_effect=node),\
                 patch.object(module,'ExecuteProcess',side_effect=process):
                description=module.generate_launch_description()
            retargeters=[n for n in processes if 'hardware.cyclo_hand_retarget_node' in n['cmd']]
            self.assertEqual(len(retargeters),1 if mode=='hands' else 0)
            self.assertEqual(len(calls),2)
            self.assertTrue(description.entities)

    def test_mode2_replaces_only_hand_retargeter(self):
        path=Path(__file__).with_name('hardware.launch.py')
        spec=importlib.util.spec_from_file_location('xr_hand_launch_fixture',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        profile=dict(hand_control='robotis_native_measured_fingers',config='/tmp/config.yaml',
                     urdf='/tmp/model.urdf',srdf='/tmp/model.srdf')
        results=[]
        for hand_mode in ('1','2'):
            calls=[]
            processes=[]
            actual=module.Node
            actual_process=module.ExecuteProcess
            def node(**kwargs):calls.append(kwargs);return actual(**kwargs)
            def process(**kwargs):processes.append(kwargs);return actual_process(**kwargs)
            with patch.dict(os.environ,AI_WORKER_XR_PROFILE='/tmp/profile.json',AI_WORKER_SH5_INPUT='hands',AI_WORKER_SH5_HAND_MODE=hand_mode),\
                 patch.object(module,'load_profile',return_value=profile),patch.object(module,'Node',side_effect=node),\
                 patch.object(module,'ExecuteProcess',side_effect=process):
                description=module.generate_launch_description()
            results.append(calls)
            if hand_mode=='2':
                self.assertEqual(len(calls),2)  # existing arm controller and checker only
                self.assertTrue(any('hardware.xr_hand_retarget_node' in p['cmd'] for p in processes))
        self.assertEqual(results[0][:2],results[1][:2])


if __name__=='__main__':unittest.main()
