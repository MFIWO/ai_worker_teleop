"""Validate restored SG2 controls without ROS or motors."""
import math
import unittest
from hardware.aux_controls import read_aux_limits, validate_aux, slew_aux


class AuxiliaryTests(unittest.TestCase):
    def setUp(self):
        self.limits=read_aux_limits('<robot>'+''.join(
            f'<joint name="{k}"><limit lower="{lo}" upper="{hi}" velocity="{vel}"/></joint>'
            for k,lo,hi,vel in [('head_joint1',-.2317,.6951,4.8),('head_joint2',-.35,.35,4.8),
                ('lift_joint',-.5,0.,4.8),('gripper_l_joint1',0.,1.1,6.5),
                ('gripper_r_joint1',0.,1.1,6.5)])+'</robot>')

    def test_real_gripper_range_and_per_joint_slew(self):
        payload=validate_aux({'joints':{'lift_joint':-.2,'gripper_l_joint1':1.1},
                              'base':[.2,0.,1/3]},self.limits)
        output=slew_aux(payload['joints'],{}, {'lift_joint':0.,'gripper_l_joint1':0.},self.limits,.02)
        self.assertAlmostEqual(output['lift_joint'],-.096)
        self.assertAlmostEqual(output['gripper_l_joint1'],.13)

    def test_unknown_nan_and_out_of_bounds_rejected(self):
        for payload in ({'joints':{'arm_l_joint1':0.}}, {'joints':{'lift_joint':.1}},
                        {'joints':{'gripper_l_joint1':1.3}}, {'base':[math.nan,0.,0.]},
                        {'base':[.3,0.,0.]}, {'unexpected':True}):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                validate_aux(payload,self.limits)

    def test_legacy_sg2_close_range_retains_other_limits_and_rejects_excess(self):
        xml = '<robot>' + ''.join(
            f'<joint name="{name}"><limit lower="{v["lower"]}" upper="{v["upper"]}" '
            f'velocity="{v["velocity"]}"/></joint>' for name, v in self.limits.items()) + '</robot>'
        restored = read_aux_limits(xml, sg2_legacy_gripper=True)
        for name in ('gripper_l_joint1', 'gripper_r_joint1'):
            self.assertEqual(restored[name]['upper'], 1.3)
            self.assertEqual(restored[name]['urdf_upper'], 1.1)
            payload = validate_aux({'joints': {name: 1.3}}, restored)
            previous = {name: 0.0}
            for _ in range(20):
                previous = slew_aux(payload['joints'], previous, {}, restored, .02)
            self.assertAlmostEqual(previous[name], 1.3)
            with self.assertRaises(ValueError):
                validate_aux({'joints': {name: 1.3001}}, restored)
        for name in ('head_joint1', 'head_joint2', 'lift_joint'):
            self.assertEqual(restored[name], self.limits[name])

    def test_missing_measured_state_cannot_start_joint(self):
        with self.assertRaises(ValueError):
            slew_aux({'head_joint1':.1},{},{},self.limits,.02)


if __name__ == '__main__':
    unittest.main()
