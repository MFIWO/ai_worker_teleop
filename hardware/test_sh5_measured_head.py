"""Offline model restoration tests for optional keyboard head motion."""
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from hardware.sh5_hands_model import (add_measured_fingers, CONTROLLER_JOINTS,
    HEAD_JOINTS, HAND_CONTROL, finger_limits, verify_hand_profile)
from hardware.sh5_model import build_sh5_model
from hardware.test_sh5_model import _synthetic_models, _snapshot, _add_joint


class MeasuredHeadTests(unittest.TestCase):
    def test_head_can_move_without_replacing_arm_or_finger_kinematics(self):
        description,control,srdf=_synthetic_models()
        root=ET.fromstring(description)
        _add_joint(root,'head_joint2','head_link1','head_link2',axis='0 0 1')
        description=ET.tostring(root,encoding='unicode')
        snapshot=_snapshot(description)
        reduced,_,frozen,_=build_sh5_model(description,control,srdf,snapshot)
        legacy,_,legacy_frozen,_=add_measured_fingers(reduced,description,srdf,frozen)
        self.assertIn('head_joint1',legacy_frozen)
        self.assertEqual(len([j for j in ET.fromstring(legacy).findall('joint') if j.get('type')!='fixed']),55)
        live,_,live_frozen,_=add_measured_fingers(reduced,description,srdf,frozen,measured_head=True)
        generated=ET.fromstring(live)
        movable={j.get('name') for j in generated.findall('joint') if j.get('type')!='fixed'}
        self.assertEqual(movable,CONTROLLER_JOINTS|HEAD_JOINTS)
        self.assertFalse(HEAD_JOINTS&live_frozen.keys())
        for name in HEAD_JOINTS:
            self.assertEqual(ET.tostring(generated.find(f"joint[@name='{name}']")),
                             ET.tostring(root.find(f"joint[@name='{name}']")))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'model.urdf'
            path.write_text(live)
            profile=dict(model='sh5',hand_control=HAND_CONTROL,urdf=str(path),manual_auxiliary=True,
                         controller_joint_names=sorted(movable),frozen_joints=live_frozen,finger_limits=finger_limits(live))
            verify_hand_profile(profile)
            profile['manual_auxiliary']=False
            with self.assertRaises(ValueError):verify_hand_profile(profile)


if __name__=='__main__':unittest.main()
