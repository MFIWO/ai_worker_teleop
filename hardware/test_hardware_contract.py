"""[ADDED] Pure hardware profile/command tests; no ROS or actuation."""
from copy import deepcopy
import math
import unittest
import xml.etree.ElementTree as ET

from hardware.xr_contract import ARM_NAMES, ARM_DXL_IDS, arm_signature, verify_description, check_positions, check_step, motor_health
from hardware.make_profile import make_files


def fixture(model='sg2'):
    xml = f'<robot name="ffw_{model}_follower">'
    for short in ('l', 'r'):
        for i in range(1, 8):
            xml += (f'<joint name="arm_{short}_joint{i}" type="revolute">'
                    f'<parent link="arm_{short}_link{i-1}"/><child link="arm_{short}_link{i}"/>'
                    '<origin xyz="0 0 -0.1" rpy="0 0 0"/><axis xyz="0 1 0"/>'
                    f'<limit lower="-2.9" upper="{-0.7 if i == 4 else 2.9}" velocity="4.8" effort="10"/></joint>')
        for i in range(1, 21 if model == 'sh5' else 2):
            name = f'finger_{short}_joint{i}' if model == 'sh5' else f'gripper_{short}_joint{i}'
            xml += f'<joint name="{name}" type="fixed"/>'
    return xml+'</robot>'


class HardwareContractTests(unittest.TestCase):
    def setUp(self):
        self.limits = {k: [-2.9, -.7 if k.endswith('joint4') else 2.9]
                       for v in ARM_NAMES.values() for k in v}

    def test_model_and_geometry_must_match(self):
        xml = fixture()
        signature = arm_signature(ET.fromstring(xml))
        self.assertTrue(verify_description(xml, 'sg2', signature))
        with self.assertRaises(ValueError): verify_description(xml, 'sh5', signature)
        with self.assertRaises(ValueError): verify_description(xml.replace('0 0 -0.1', '0 0 -0.2', 1), 'sg2', signature)

    def test_sh5_names_not_gripper_alias(self):
        xml = fixture('sh5')
        self.assertTrue(verify_description(xml, 'sh5', arm_signature(ET.fromstring(xml))))
        with self.assertRaises(ValueError):
            verify_description(xml.replace('finger_l_joint20', 'unknown'), 'sh5', arm_signature(ET.fromstring(xml)))

    def test_seven_arm_only_and_extra_gripper_is_removed(self):
        names = list(ARM_NAMES['left'])+['gripper_l_joint1']
        values = [0, 0, 0, -1.57, 0, 0, 0, 1.2]
        result = check_positions('left', names, values, self.limits)
        self.assertEqual(list(result), list(ARM_NAMES['left']))
        self.assertNotIn('gripper_l_joint1', result)

    def test_wrong_joint_side_length_duplicate_nan_and_bound_rejected(self):
        names = list(ARM_NAMES['left']); values = [0,0,0,-1.57,0,0,0]
        cases = [(names[:-1],values), (names+['lift_joint'], values+[0]),
                 (names+[names[0]],values+[0]), (names, [math.nan]+values[1:]),
                 (list(ARM_NAMES['right']),values), (names, [0,0,0,-.45,0,0,0])]
        for ns, qs in cases:
            with self.subTest(ns=ns, qs=qs):
                with self.assertRaises(ValueError): check_positions('left',ns,qs,self.limits)

    def test_no_accumulated_jump_or_feedback_runaway(self):
        old = {'a': 0.}; measured = {'a': 0.}
        check_step({'a': .003}, measured, old, .02)
        with self.assertRaises(ValueError): check_step({'a': .02}, measured, old, .02)
        with self.assertRaises(ValueError): check_step({'a': .1}, measured, old, 100)
        with self.assertRaises(ValueError): check_step({'a': .001}, {}, old, .02)

    def test_profile_preserves_safety_and_position_limits(self):
        original={'vr_controller': {'ros__parameters': {'weight_position':10.,'weight_orientation':1.,
                  'weight_elbow_position':8.,'collision_buffer':.05,'collision_safe_distance':.02,'cbf_alpha':50.}},
                  'reference_checker': {'ros__parameters': {'ref_pos_jump_threshold':.1}}}
        before=deepcopy(original)
        output,cfg,limits,_=make_files('sg2',fixture(),fixture(),original)
        self.assertEqual(original,before)
        r=ET.fromstring(output)
        for names in ARM_NAMES.values():
            for k in names:
                b=r.find(f"joint[@name='{k}']/limit")
                self.assertEqual(float(b.get('velocity')),.35)
                self.assertEqual([float(b.get('lower')),float(b.get('upper'))],self.limits[k])
        self.assertEqual(cfg['reference_checker'],original['reference_checker'])
        self.assertEqual(cfg['vr_controller']['ros__parameters']['collision_buffer'],.05)
        self.assertEqual(cfg['vr_controller']['ros__parameters']['weight_elbow_position'],0.)
        self.assertIn('/xr_hardware/sg2/',cfg['vr_controller']['ros__parameters']['left_traj_topic'])

    def test_simulation_model_refused(self):
        cfg={'vr_controller': {'ros__parameters': {'weight_position':10.,'weight_orientation':1.}}}
        with self.assertRaisesRegex(ValueError,'expanded simulation|physical description'):
            make_files('sg2',fixture().replace('upper="-0.7"','upper="-0.45"'),fixture(),cfg)

    def test_motor_health_is_bound_to_known_arm_ids(self):
        good = ARM_DXL_IDS['left'] + ARM_DXL_IDS['right']
        self.assertTrue(motor_health(good, [True]*14, [0]*14, 0))
        self.assertFalse(motor_health(list(range(1,15)), [True]*14, [0]*14, 0))
        self.assertFalse(motor_health(good, [False]+[True]*13, [0]*14, 0))
        self.assertFalse(motor_health(good, [True]*14, [4]+[0]*13, 0))
        self.assertFalse(motor_health(good, [True]*14, [0]*14, -7))
        self.assertTrue(motor_health(good+[100], [True]*14+[False], [0]*15, 0))


if __name__ == '__main__': unittest.main()
