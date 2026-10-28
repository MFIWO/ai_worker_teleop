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
    def test_native_results_relay_immediately_per_side_and_stop_on_hold(self):
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.mode,hand.hand_mode='hands',1
        measured={n:0. for names in FINGER_NAMES.values() for n in names}
        hand.limits={n:dict(lower=-1.,upper=1.,velocity=.001) for n in measured}
        hand.previous,hand.preparing,hand.native={},{},{}
        hand.last_trigger={}
        hand.active,hand.last_output=False,0.
        hand.publish=Mock()
        hand.node=SimpleNamespace(measured_rx=10.,intent_enabled=True,fault=None,latch=Mock())
        names=FINGER_NAMES['left']
        msg=SimpleNamespace(joint_names=list(names),points=[SimpleNamespace(positions=[.8]*20)])
        hand.native_trajectory('left',msg)
        hand.publish.assert_not_called()  # global teleop still disarmed
        hand.update_authority(measured,10.,True)
        hand.publish.assert_not_called()  # no resampling/replay by timer
        hand.native_trajectory('left',msg)
        self.assertEqual(hand.publish.call_args.args[0],dict.fromkeys(names,.8))
        self.assertNotIn('right',hand.native)  # never wait for the opposite hand
        count=hand.publish.call_count
        hand.update_authority(measured,100.,True)  # no hand-specific age watchdog
        self.assertEqual(hand.publish.call_count,count)
        hand.node.measured_rx=100.
        hand.update_authority(measured,100.,False)
        self.assertFalse(hand.active)
        self.assertEqual(hand.publish.call_args.args[0],measured)
        count=hand.publish.call_count
        hand.native_trajectory('left',msg)
        self.assertEqual(hand.publish.call_count,count)
        hand.node.latch.assert_not_called()

    def test_optical_hand_timer_output_is_rejected(self):
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.mode='hands'
        hand.publish=Mock()
        with self.assertRaisesRegex(ValueError,'never on timer ticks'):
            hand.tick({},1.,True)
        hand.publish.assert_not_called()

    def test_mode2_trigger_closing_half_velocity_release_and_pinch_unchanged(self):
        # [CHANGED 2026-10-28] Only trigger CLOSING is rate-limited.
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.mode,hand.hand_mode,hand.poses='controllers',2,POSES
        measured={n:0. for names in FINGER_NAMES.values() for n in names}
        hand.limits={n:dict(lower=-4.,upper=4.,velocity=2.) for n in measured}
        hand.previous,hand.preparing,hand.last_trigger={},{},{}
        hand.active,hand.last_output=False,0.
        hand.node=SimpleNamespace(measured_rx=10.3)
        hand.publish=Mock()
        hand.controls=controls('full_grip',trigger=1.)
        hand.rx=0.  # Mode 2 does not add a separate .3s hand timeout.
        hand.tick(measured,10.,True)
        target=targets(POSES,hand.controls)
        first=dict(hand.publish.call_args.args[0])
        for name,goal in target.items():
            self.assertAlmostEqual(first[name],max(-.01,min(.01,goal)))

        hand.tick(measured,10.1,True)
        second=dict(hand.publish.call_args.args[0])
        for name in measured:
            self.assertLessEqual(abs(second[name]-first[name]),.1000001)
            if abs(target[name])>.11:
                self.assertNotAlmostEqual(second[name],target[name])

        # Releasing the trigger is NOT deliberately slowed.
        hand.controls=controls('full_grip',trigger=.25)
        hand.tick(measured,10.2,True)
        self.assertEqual(hand.publish.call_args.args[0],targets(POSES,hand.controls))
        # Grip-pinch remains directly proportional and bypasses the trigger limit.
        hand.controls=controls('full_grip',trigger=.25,grip=.4)
        hand.tick(measured,10.3,True)
        self.assertEqual(hand.publish.call_args.args[0],targets(POSES,hand.controls))
        hand.tick(measured,10.4,False)
        self.assertFalse(hand.active)
        self.assertFalse(hand.last_trigger)
        self.assertEqual(hand.publish.call_args.args[0],measured)

    def test_mode1_keeps_existing_velocity_limit_and_hand_timeout(self):
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.mode,hand.hand_mode,hand.poses='controllers',1,POSES
        measured={n:0. for names in FINGER_NAMES.values() for n in names}
        hand.limits={n:dict(lower=-4.,upper=4.,velocity=.1) for n in measured}
        hand.previous,hand.preparing={},{}
        hand.last_trigger={}
        hand.active,hand.last_output=False,0.
        hand.node=SimpleNamespace(measured_rx=1.)
        hand.publish=Mock()
        hand.controls={s:dict(trigger=1.,grip=0.) for s in FINGER_NAMES}
        hand.rx=1.
        hand.tick(measured,1.,True)
        self.assertTrue(all(abs(v)<=.00100001 for v in hand.publish.call_args.args[0].values()))
        hand.tick(measured,1.4,True)
        self.assertFalse(hand.active)

    def test_session_chords_and_partial_releases_never_select_pose(self):
        keys={'X':('left','aButton'),'Y':('left','bButton'),
              'A':('right','aButton'),'B':('right','bButton')}
        for pair in ('AX','BY'):
            for first in pair:
                second=next(b for b in pair if b!=first)
                for last in pair:
                    with self.subTest(pair=pair,first=first,last=last):
                        modes=ControllerHandModes()
                        modes.selected='point'
                        for i,pressed in enumerate((first,pair,last,'','','')):
                            states={'left':{},'right':{}}
                            for key in pressed:
                                side,name=keys[key]
                                states[side][name]=True
                            self.assertIsNone(modes.buttons(states['left'],states['right'],now=i*.1))
                        self.assertEqual(modes.selected,'point')

    def test_nonoverlapping_chord_events_cancel_pending_tap(self):
        modes=ControllerHandModes()
        modes.selected='point'
        modes.buttons({'aButton':True},{},now=0.)
        modes.buttons({},{},now=.01)  # transient all-up before opposite event
        modes.buttons({},{'aButton':True},now=.08)
        modes.buttons({},{},now=.1)
        self.assertIsNone(modes.buttons({},{},now=.4))
        self.assertEqual(modes.selected,'point')

    def test_single_tap_waits_for_quiet_interval_and_reset_cancels(self):
        modes=ControllerHandModes()
        modes.buttons({'bButton':True},{},now=0.)
        self.assertIsNone(modes.buttons({},{},now=.1))
        self.assertIsNone(modes.buttons({},{},now=.2))
        self.assertEqual(modes.buttons({},{},now=.3),('Y','angled_grip'))
        modes.buttons({},{'bButton':True},now=.4)
        modes.buttons({},{},now=.5)
        modes.reset_buttons()
        self.assertIsNone(modes.buttons({},{},now=1.))
        self.assertEqual(modes.selected,'angled_grip')

    def test_late_single_button_tail_after_chord_is_ignored(self):
        modes=ControllerHandModes()
        modes.buttons({'bButton':True},{'bButton':True},now=0.)
        modes.buttons({},{},now=.1)
        modes.buttons({'bButton':True},{},now=.12)
        modes.buttons({},{},now=.14)
        self.assertIsNone(modes.buttons({},{},now=.5))
        self.assertEqual(modes.selected,'full_grip')

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
        modes.buttons({'bButton':True},{},now=0.)
        modes.buttons({},{},now=.01)
        modes.buttons({},{},now=.2)
        result=modes.controls({'left':dict(trigger=1.,grip=0.),'right':dict(trigger=0.,grip=0.)})
        self.assertEqual(result['left']['pose'],'full_grip')
        self.assertEqual(result['right']['pose'],'angled_grip')

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
