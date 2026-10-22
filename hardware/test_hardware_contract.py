"""[ADDED] Pure hardware profile/command tests; no ROS or actuation."""
from copy import deepcopy
import math
import unittest
import xml.etree.ElementTree as ET

from hardware.xr_contract import ARM_NAMES, ARM_DXL_IDS, POSITION_EPSILON_RAD, arm_signature, arm_limits, verify_description, check_positions, check_step, limit_step, motor_health
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

    def test_solver_roundoff_at_each_urdf_boundary_snaps_inside_limits(self):
        limits = dict(self.limits)
        limits['arm_l_joint2'] = [0., 3.14]
        limits['arm_r_joint2'] = [-3.14, 0.]
        for side, names in ARM_NAMES.items():
            for index, name in enumerate(names):
                lower, upper = limits[name]
                for edge, direction in ((lower, -1), (upper, 1)):
                    for noise in (1e-12, 1.5e-7, POSITION_EPSILON_RAD):
                        positions = [0., 0., 0., -1.57, 0., 0., 0.]
                        positions[index] = edge + direction * noise
                        result = check_positions(side, names, positions, limits)
                        self.assertEqual(result[name], edge)
                        self.assertTrue(all(limits[k][0] <= v <= limits[k][1] for k,v in result.items()))

    def test_real_boundary_violation_still_rejected_with_precise_value(self):
        names = ARM_NAMES['left']
        limits = dict(self.limits)
        limits['arm_l_joint2'] = [0., 3.14]
        for value in (-POSITION_EPSILON_RAD*1.01, -.001, 3.1401):
            positions = [0., value, 0., -1.57, 0., 0., 0.]
            with self.assertRaisesRegex(ValueError, 'violation=.*numerical_tolerance=1e-05rad'):
                check_positions('left', names, positions, limits)

    def test_no_accumulated_jump_or_feedback_runaway(self):
        old = {'a': 0.}; measured = {'a': 0.}
        speed = {'a': 4.8}
        check_step({'a': .003}, measured, old, .02, speed)
        check_step({'a': .02}, measured, old, .02, speed)
        with self.assertRaises(ValueError): check_step({'a': .02}, measured, old, .001, speed)
        with self.assertRaises(ValueError): check_step({'a': .49}, measured, old, 100, speed)
        with self.assertRaises(ValueError): check_step({'a': .001}, {}, old, .02, speed)

    def test_normal_path_slew_limits_benign_solver_step(self):
        old = {'a': 0.}; measured = {'a': 0.}
        out = limit_step({'a': .05}, measured, old, .001, {'a': 4.8})
        self.assertGreater(out['a'], 0.)
        self.assertLess(out['a'], .05)
        with self.assertRaisesRegex(ValueError, 'commanded/measured'):
            limit_step({'a': .481}, measured, old, .02, {'a': 4.8})

    def test_former_08rad_fault_accepts_urdf_valid_step(self):
        check_step({'a': .09}, {'a': 0.}, {'a': 0.}, .02, {'a': 4.8})
        self.assertAlmostEqual(limit_step({'a': .2}, {'a': 0.}, {'a': 0.}, .02,
                                          {'a': 4.8})['a'], .096)

    def test_joint_specific_velocities_and_tracking_budget(self):
        speeds = {'slow': .5, 'fast': 4.8}
        out = limit_step({'slow': .04, 'fast': .2}, {'slow': 0., 'fast': 0.},
                         {'slow': 0., 'fast': 0.}, .02, speeds)
        self.assertAlmostEqual(out['slow'], .01)
        self.assertAlmostEqual(out['fast'], .096)
        with self.assertRaisesRegex(ValueError, '0.050000rad'):
            limit_step({'slow': .051}, {'slow': 0.}, {'slow': 0.}, .02, speeds)
        with self.assertRaises(ValueError):
            limit_step({'slow': math.nan}, {'slow': 0.}, {'slow': 0.}, .02, speeds)

    def test_invalid_or_missing_urdf_velocity_rejected(self):
        _, speeds = arm_limits(fixture().replace('velocity="4.8"', 'velocity="0.5"', 1))
        self.assertEqual(speeds['arm_l_joint1'], .5)
        self.assertEqual(speeds['arm_l_joint2'], 4.8)
        for value in ('0', '-1', 'nan', 'inf'):
            with self.assertRaises(ValueError):
                arm_limits(fixture().replace('velocity="4.8"', f'velocity="{value}"', 1))

    def test_sg2_profile_uses_physical_urdf_position_and_velocity_limits(self):
        original={'vr_controller': {'ros__parameters': {'weight_position':10.,'weight_orientation':1.,
                  'weight_elbow_position':8.,'collision_buffer':.05,'collision_safe_distance':.02,'cbf_alpha':50.}},
                  'reference_checker': {'ros__parameters': {'ref_pos_jump_threshold':.1}}}
        before=deepcopy(original)
        physical = fixture().replace('upper="-0.7"', 'upper="1.0786"')
        output,cfg,limits,_=make_files('sg2',fixture(),physical,original)
        self.assertEqual(original,before)
        r=ET.fromstring(output)
        for names in ARM_NAMES.values():
            for k in names:
                b=r.find(f"joint[@name='{k}']/limit")
                self.assertEqual(float(b.get('velocity')),4.8)
                expected = [-2.9, 1.0786 if k.endswith('joint4') else 2.9]
                self.assertEqual([float(b.get('lower')),float(b.get('upper'))],expected)
                self.assertEqual(limits[k], expected)
        self.assertEqual(cfg['reference_checker'],original['reference_checker'])
        self.assertEqual(cfg['vr_controller']['ros__parameters']['collision_buffer'],.05)
        self.assertEqual(cfg['vr_controller']['ros__parameters']['weight_elbow_position'],0.)
        self.assertIn('/xr_hardware/sg2/',cfg['vr_controller']['ros__parameters']['left_traj_topic'])

    def test_sg2_controller_joint4_limit_is_replaced_by_physical_limit(self):
        cfg={'vr_controller': {'ros__parameters': {'weight_position':10.,'weight_orientation':1.}}}
        physical = fixture().replace('upper="-0.7"', 'upper="1.0786"')
        output,_,limits,_ = make_files(
            'sg2', fixture().replace('upper="-0.7"','upper="-0.45"'), physical, cfg)
        self.assertEqual(limits['arm_l_joint4'], [-2.9, 1.0786])
        self.assertEqual(
            float(ET.fromstring(output).find("joint[@name='arm_l_joint4']/limit").get('upper')),
            1.0786)

    def test_motor_health_is_bound_to_known_arm_ids(self):
        good = ARM_DXL_IDS['left'] + ARM_DXL_IDS['right']
        self.assertTrue(motor_health(good, [True]*14, [0]*14, 0))
        self.assertFalse(motor_health(list(range(1,15)), [True]*14, [0]*14, 0))
        self.assertFalse(motor_health(good, [False]+[True]*13, [0]*14, 0))
        self.assertFalse(motor_health(good, [True]*14, [4]+[0]*13, 0))
        self.assertFalse(motor_health(good, [True]*14, [0]*14, -7))
        self.assertTrue(motor_health(good+[100], [True]*14+[False], [0]*15, 0))


if __name__ == '__main__': unittest.main()
