"""Offline controller-motion regressions; no ROS or device writes."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from hardware.controller_hand_modes import ControllerHandModes
from hardware.sh5_controller_hands import SH5HandOutput, targets
from hardware.sh5_hands_model import FINGER_NAMES

ROOT = Path(__file__).resolve().parents[1]
POSES = json.loads((ROOT/'hardware/hx5_controller_poses.json').read_text())['poses']


def controls(pose='angled_grip', trigger=0., grip=0.):
    return {s:dict(pose=pose, trigger=trigger, grip=grip) for s in FINGER_NAMES}


class HandMotionTests(unittest.TestCase):
    def test_c_shape_at_rest_and_proportional_close(self):
        for depth in (0., .01, .25, .5, 1.):
            result = targets(POSES, controls(trigger=depth))
            for side,names in FINGER_NAMES.items():
                for n,a,b in zip(names,POSES[side]['angled_prepare'],POSES[side]['angled_grip']):
                    self.assertAlmostEqual(result[n], a+depth*(b-a))

    def test_original_mode_and_grip_preserved(self):
        for pose in ('grasp','angled_grip'):
            result=targets(POSES,controls(pose,trigger=.8,grip=.4))
            for side,names in FINGER_NAMES.items():
                for n,a,b in zip(names,POSES[side]['open'],POSES[side]['pinch']):
                    self.assertAlmostEqual(result[n],a+.4*(b-a))
        for side,names in FINGER_NAMES.items():
            result=targets(POSES,controls('grasp',trigger=.5))
            for n,a,b in zip(names,POSES[side]['open'],POSES[side]['grasp']):
                self.assertAlmostEqual(result[n],(a+b)/2)

    def test_c_grip_adds_ring_little_flexion_without_their_roll(self):
        for side,poses in POSES.items():
            self.assertEqual(poses['angled_grip'][:12],poses['pinch'][:12])
            for offset in (12,16):
                self.assertEqual(poses['angled_grip'][offset],poses['angled_prepare'][offset])
                self.assertEqual(poses['angled_grip'][offset+1:offset+4],poses['pinch'][9:12])

    def test_selection_preserves_other_held_hand(self):
        modes=ControllerHandModes()
        modes.controls({'left':dict(trigger=1.,grip=0.),'right':dict(trigger=0.,grip=0.)})
        modes.buttons({'bButton':True},{})
        modes.buttons({},{})
        result=modes.controls({'left':dict(trigger=1.,grip=0.),'right':dict(trigger=0.,grip=0.)})
        self.assertEqual(result['left']['pose'],'full_grip')
        self.assertEqual(result['right']['pose'],'angled_grip')

    def test_continuous_c_motion_release_and_stale_hold(self):
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.mode,hand.hand_mode,hand.poses='controllers',2,POSES
        measured={n:a for s,names in FINGER_NAMES.items() for n,a in zip(names,POSES[s]['open'])}
        hand.limits={n:dict(lower=-4.,upper=4.,velocity=1.) for n in measured}
        hand.previous,hand.filtered,hand.preparing={},{},{}
        hand.active,hand.last_output=False,0.
        hand.node=SimpleNamespace(measured_rx=0.)
        hand.publish=Mock()
        last=dict(measured)
        for i in range(1,501):
            now=i*.01
            hand.controls=controls(trigger=0. if i<100 or i>=300 else (i-100)/200.)
            hand.rx=now
            hand.tick(measured,now,True)
            output=hand.publish.call_args.args[0]
            for n in output:
                self.assertLessEqual(abs(output[n]-last[n]),.01000001)
            measured,last=dict(output),dict(output)
        self.assertFalse(hand.preparing)  # no arrival gate / 3s timeout for C grip
        for s,names in FINGER_NAMES.items():
            for n,a in zip(names,POSES[s]['angled_prepare']):
                self.assertAlmostEqual(last[n],a,places=5)
        hand.node.measured_rx=5.4
        hand.tick(measured,5.4,True)
        self.assertFalse(hand.active)
        self.assertFalse(hand.filtered)

    def test_sh5_lift_only_in_head_mode(self):
        path=ROOT/'robotis_vuer/robotis_vuer/vr_publisher_sh5_hands_hardware.py'
        if not path.exists():self.skipTest('PC publisher source not installed in robot-only checkout')
        source=ast.parse(path.read_text())
        cls=next(n for n in source.body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='publish_right_joystick')
        class Base:
            def publish_right_joystick(self,value):self.output.append(value)
        isolated=ast.Module(body=[ast.ClassDef(name='Publisher',bases=[ast.Name(id='Base',ctx=ast.Load())],
            keywords=[],body=[method],decorator_list=[])],type_ignores=[])
        scope={'Base':Base}
        exec(compile(ast.fix_missing_locations(isolated),str(path),'exec'),scope)
        publisher=scope['Publisher']()
        publisher.output=[]
        publisher.joystick_mode=False
        publisher.publish_right_joystick(1.)
        self.assertEqual(publisher.output,[])
        publisher.joystick_mode=True
        publisher.publish_right_joystick(1.)
        self.assertEqual(publisher.output,[1.])


if __name__=='__main__':unittest.main()
