"""Offline controller-motion regressions; no ROS or device writes."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from hardware.controller_hand_modes import ControllerHandModes
from hardware.sh5_controller_hands import SH5HandOutput, targets, pointing_currents
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

    def test_mode2_trigger_closing_quarter_velocity_and_bounded_release(self):
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
            self.assertAlmostEqual(first[name],max(-.005,min(.005,goal)))

        hand.tick(measured,10.1,True)
        second=dict(hand.publish.call_args.args[0])
        for name in measured:
            self.assertLessEqual(abs(second[name]-first[name]),.0500001)
            if abs(target[name])>.11:
                self.assertNotAlmostEqual(second[name],target[name])

        # A trigger reduction must not bypass the remaining close ramp.
        hand.controls=controls('full_grip',trigger=.25)
        hand.tick(measured,10.2,True)
        third=dict(hand.publish.call_args.args[0])
        for name in measured:
            self.assertLessEqual(abs(third[name]-second[name]),.2000001)
        # Grip-pinch remains directly proportional and bypasses the trigger limit.
        hand.controls=controls('full_grip',trigger=.25,grip=.4)
        hand.tick(measured,10.3,True)
        self.assertEqual(hand.publish.call_args.args[0],targets(POSES,hand.controls))
        held = dict(hand.publish.call_args.args[0])
        hand.tick(measured,10.4,False)
        self.assertFalse(hand.active)
        self.assertFalse(hand.last_trigger)
        self.assertEqual(hand.publish.call_args.args[0],held)
        hand.tick(measured,10.5,False)
        self.assertEqual(hand.publish.call_args.args[0],held)

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
                        modes.selected='thumb_index_pinch'
                        for i,pressed in enumerate((first,pair,last,'','','')):
                            states={'left':{},'right':{}}
                            for key in pressed:
                                side,name=keys[key]
                                states[side][name]=True
                            self.assertIsNone(modes.buttons(states['left'],states['right'],now=i*.1))
                        self.assertEqual(modes.selected,'thumb_index_pinch')

    def test_nonoverlapping_chord_events_cancel_pending_tap(self):
        modes=ControllerHandModes()
        modes.selected='thumb_index_pinch'
        modes.buttons({'aButton':True},{},now=0.)
        modes.buttons({},{},now=.01)  # transient all-up before opposite event
        modes.buttons({},{'aButton':True},now=.08)
        modes.buttons({},{},now=.1)
        self.assertIsNone(modes.buttons({},{},now=.4))
        self.assertEqual(modes.selected,'thumb_index_pinch')

    def test_single_tap_waits_for_quiet_interval_and_reset_cancels(self):
        modes=ControllerHandModes()
        modes.buttons({'bButton':True},{},now=0.)
        self.assertIsNone(modes.buttons({},{},now=.1))
        self.assertIsNone(modes.buttons({},{},now=.2))
        self.assertEqual(modes.buttons({},{},now=.3),('Y','thumb_index_pinch'))
        modes.buttons({},{'bButton':True},now=.4)
        modes.buttons({},{},now=.5)
        modes.reset_buttons()
        self.assertIsNone(modes.buttons({},{},now=1.))
        self.assertEqual(modes.selected,'thumb_index_pinch')

    def test_late_single_button_tail_after_chord_is_ignored(self):
        modes=ControllerHandModes()
        modes.buttons({'bButton':True},{'bButton':True},now=0.)
        modes.buttons({},{},now=.1)
        modes.buttons({'bButton':True},{},now=.12)
        modes.buttons({},{},now=.14)
        self.assertIsNone(modes.buttons({},{},now=.5))
        self.assertEqual(modes.selected,'full_grip')

    def test_A_X_Y_select_requested_finger_poses(self):
        for button,expected in (('A','thumb_out_grip'),('X','index_pinch'),('Y','thumb_index_pinch')):  # [CHANGED 2026-10-29]
            modes=ControllerHandModes()
            left,right=({'aButton':True}, {}) if button=='X' else (({'bButton':True}, {}) if button=='Y' else ({}, {'aButton':True}))
            modes.buttons(left,right,now=0.)
            modes.buttons({},{},now=.01)
            self.assertEqual(modes.buttons({},{},now=.20),(button,expected))
            self.assertEqual(modes.selected,expected)
            commanded=targets(POSES,controls(pose=expected,trigger=1.,grip=0.))
            for side,names in FINGER_NAMES.items():
                for joint,goal in zip(names,POSES[side][expected]):
                    self.assertAlmostEqual(commanded[joint],goal)

    def test_A_restores_historical_thumb_first_four_finger_targets(self):
        # [ADDED 2026-10-29] Exact A endpoints from main e321ba8, no invented motor goals.
        for side,names in FINGER_NAMES.items():
            a = POSES[side]['thumb_out_grip']
            full = POSES[side]['full_grip']
            self.assertEqual(a[:4], [-.15,.55,0.,0.] if side=='left'
                             else [.297,-.55,0.,0.])
            self.assertEqual(a[4:],full[4:])  # All four fingers participate.
            for depth in (0.,.25,.5,1.):
                reached = targets(POSES,controls('thumb_out_grip',trigger=depth))
                for name,opened,closed in zip(names,POSES[side]['open'],a):
                    self.assertAlmostEqual(reached[name],opened+depth*(closed-opened))

    def test_A_thumb_first_preparation_holds_four_fingers_until_measured_arrival(self):
        # [ADDED 2026-10-29] No finger closure before measured thumb opposition.
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.poses,hand.preparing=POSES,{}
        hand.controls={'left':dict(pose='thumb_out_grip',trigger=1.,grip=0.),
                       'right':dict(pose='full_grip',trigger=0.,grip=0.)}
        measured={n:v for side,names in FINGER_NAMES.items()
                  for n,v in zip(names,POSES[side]['open'])}
        left=FINGER_NAMES['left']
        target=targets(POSES,hand.controls)
        result=hand.prepare_targets(dict(target),measured,1.)
        for name,want in zip(left[:4],POSES['left']['thumb_out_grip'][:4]):
            self.assertAlmostEqual(result[name],want)
        for name in left[4:]:
            self.assertAlmostEqual(result[name],measured[name])
        self.assertFalse(hand.preparing['left']['done'])
        for name,want in zip(left[:4],POSES['left']['thumb_out_grip'][:4]):
            measured[name]=want
        second=hand.prepare_targets(dict(target),measured,1.2)
        self.assertTrue(hand.preparing['left']['done'])
        for name in left[4:]:
            self.assertAlmostEqual(second[name],target[name])
        hand.controls['left']['trigger']=0.
        hand.prepare_targets(dict(targets(POSES,hand.controls)),measured,1.3)
        self.assertNotIn('left',hand.preparing)

    def test_A_missing_thumb_feedback_times_out_instead_of_forcing_finger_closure(self):
        # [ADDED 2026-10-29] Unreachable thumb target must fail closed.
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.poses,hand.preparing=POSES,{}
        hand.controls={'left':dict(pose='thumb_out_grip',trigger=.8,grip=0.),
                       'right':dict(pose='full_grip',trigger=0.,grip=0.)}
        measured={n:v for side,names in FINGER_NAMES.items()
                  for n,v in zip(names,POSES[side]['open'])}
        hand.prepare_targets(targets(POSES,hand.controls),measured,0.)
        with self.assertRaisesRegex(ValueError,'left: thumb_out_grip preparation not reached'):
            hand.prepare_targets(targets(POSES,hand.controls),measured,8.1)

    def test_deep_thumb_exceeds_old_angled_and_ulnar_fingers_are_independent(self):
        for side,poses in POSES.items():
            # [CHANGED] Thumb joint-3 and joint-4 close more than former Y.
            for i in (2,3):
                self.assertGreater(abs(poses['tripod_deep'][i]),abs(poses['angled_grip'][i]))
                self.assertGreater(abs(poses['ulnar_pinch'][i]),abs(poses['angled_grip'][i]))
            # Index and middle are open for B; ring and little are open for A.
            self.assertEqual(poses['ulnar_pinch'][4:12],poses['open'][4:12])
            self.assertEqual(poses['tripod_deep'][12:20],poses['open'][12:20])
            self.assertNotEqual(poses['ulnar_pinch'][13:16],poses['open'][13:16])
            self.assertNotEqual(poses['ulnar_pinch'][17:20],poses['open'][17:20])

    def test_mode2_grip_does_not_override_selected_trigger_pose(self):
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.hand_mode=2
        hand.controls=None
        payload=controls(pose='ulnar_pinch',trigger=.7,grip=1.)
        hand.receive(payload,now=1.)
        self.assertEqual(hand.controls['left']['grip'],0.)
        self.assertEqual(hand.controls['right']['grip'],0.)
        selected=targets(POSES,hand.controls)
        for side,names in FINGER_NAMES.items():
            self.assertAlmostEqual(selected[names[13]],.7*POSES[side]['ulnar_pinch'][13]
                                   +.3*POSES[side]['ulnar_prepare'][13])

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
        self.assertEqual(result['right']['pose'],'thumb_index_pinch')

    def test_sh5_lift_available_in_both_modes(self):
        from hardware.controller_stick_motion import lift_target
        path=ROOT/'robotis_vuer/robotis_vuer/vr_publisher_sh5_hands_hardware.py'
        cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='publish_right_joystick')
        isolated=ast.Module(body=[ast.ClassDef(name='Publisher',bases=[],keywords=[],body=[method],decorator_list=[])],type_ignores=[])
        scope={'time':SimpleNamespace(monotonic=lambda:10.),'lift_target':lift_target}
        exec(compile(ast.fix_missing_locations(isolated),str(path),'exec'),scope)
        n=scope['Publisher']();n.control_min_period=1/30;n.last_lift_command=None
        n.lift_joint_current_position=-.25;n._aux_joints={}
        n._hardware_manifest={'auxiliary_limits':{'lift_joint':dict(lower=-.5,upper=0.,velocity=.3)}}
        n.get_logger=lambda:SimpleNamespace(warning=Mock())
        for mode in (False,True):
            n.joystick_mode=mode;n._lift_tick=None;n.last_lift_command=None
            n.publish_right_joystick(.707)
            self.assertGreater(n._aux_joints['lift_joint'],-.25)

    def test_B_ready_action_preserves_selected_grasp(self):
        modes=ControllerHandModes()
        modes.selected='index_pinch'
        modes.buttons({},{'bButton':True},now=0.)
        modes.buttons({},{},now=.01)
        self.assertEqual(modes.buttons({},{},now=.2),('B','ready_pose'))
        self.assertEqual(modes.selected,'index_pinch')

    def test_pointing_holds_configured_angle_without_tactile_or_measured_chasing(self):
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.poses=POSES;hand.controls=controls('pointing',trigger=1.)
        names=FINGER_NAMES['right'][4:8]
        hand.previous=dict.fromkeys(names,.8)
        output=targets(POSES,hand.controls)
        hand.lock_pointing_contact(output,1.)
        expected=dict(zip(names,POSES['right']['pointing'][4:8]))
        self.assertEqual({n:output[n] for n in names},expected)
        hand.previous=dict.fromkeys(names,1.)
        hand.lock_pointing_contact(output,2.)
        self.assertEqual({n:output[n] for n in names},expected)
        hand.controls=controls('index_pinch',trigger=0.)
        hand.lock_pointing_contact(output,3.)
        self.assertFalse(hand.contact_goals)

    def test_pointing_entry_does_not_bypass_velocity_bound(self):
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.poses=POSES;hand.controls=controls('pointing',trigger=1.)
        output=targets(POSES,hand.controls)
        for side,names in FINGER_NAMES.items():
            for n in names[4:8]:output[n]=.2
        hand.lock_pointing_contact(output,1.)
        self.assertFalse(hand.contact_goals)
        self.assertEqual(output[FINGER_NAMES['right'][5]],.2)

    def test_pointing_trigger_current_ceiling_increases_but_never_exceeds_stock(self):
        previous=0.
        for depth in (0.,.1,.5,1.):
            values=pointing_currents(controls('pointing',trigger=depth))
            for row in values.values():
                self.assertEqual(row[:4],[300.]*4)
                self.assertEqual(row[8:],[300.]*12)
                self.assertEqual(row[4:8],[150.+150.*depth]*4)
                self.assertLessEqual(max(row),300.)
            self.assertGreaterEqual(values['left'][5],previous);previous=values['left'][5]
        self.assertEqual(pointing_currents(controls('index_pinch',trigger=1.))['left'],[300.]*20)

    def test_mode2_left_grip_reanchors_only_left_and_right_keeps_following(self):
        import threading
        from copy import deepcopy
        import numpy as np
        from robotis_vuer.xr_arm_targets import rigid_pose
        path=ROOT/'robotis_vuer/robotis_vuer/vr_publisher_sh5_hands_hardware.py'
        cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_publish_synced_pose_frame_if_ready')
        class Base:
            def _publish_synced_pose_frame_if_ready(self):self.motion_count+=1
        isolated=ast.Module(body=[ast.ClassDef(name='Publisher',bases=[ast.Name(id='Base',ctx=ast.Load())],keywords=[],body=[method],decorator_list=[])],type_ignores=[])
        scope={'Base':Base,'time':SimpleNamespace(monotonic=lambda:10.),'deepcopy':deepcopy,'rigid_pose':rigid_pose}
        exec(compile(ast.fix_missing_locations(isolated),str(path),'exec'),scope)
        n=scope['Publisher']()
        n._controller_mode,n._hand_mode,n.arm_control_enabled=True,2,True
        n._phase,n._xr_lock='ACTIVE',threading.RLock()
        n._last_pub,n._clutch_held,n._clutch_reanchor_pending=None,False,False
        n._mapper=SimpleNamespace(anchors={side:SimpleNamespace(input_pose=np.eye(4),robot_pose=np.eye(4)) for side in ('left','right')})
        n._session=SimpleNamespace(mapper=n._mapper)
        n._goals={'left':np.eye(4)};n._goals['left'][0,3]=.25
        n.left_controller_matrix=np.eye(4);n.left_controller_matrix[1,3]=.7
        n._last_input={'left':np.eye(4),'right':np.eye(4)}
        n.motion_count=0
        n._hold_for_tracking_gap=Mock(return_value=False);n._graph_check=Mock();n._snapshot=Mock()
        n._publish_synced_pose_frame_if_ready()
        self.assertEqual(n.motion_count,1)
        np.testing.assert_array_equal(n._mapper.anchors['left'].robot_pose,n._goals['left'])
        np.testing.assert_array_equal(n._mapper.anchors['right'].robot_pose,np.eye(4))
        self.assertIn('right',n._last_input)
        # Pressing after repositioning captures only the new left input.
        n._clutch_held,n._clutch_reanchor_pending=True,True
        n.left_controller_matrix[1,3]=1.2
        n._publish_synced_pose_frame_if_ready()
        np.testing.assert_array_equal(n._mapper.anchors['left'].input_pose,n.left_controller_matrix)
        anchor=n._mapper.anchors['left'].input_pose.copy()
        n.left_controller_matrix[1,3]=1.3
        n._publish_synced_pose_frame_if_ready()
        np.testing.assert_array_equal(n._mapper.anchors['left'].input_pose,anchor)
        self.assertEqual(n.motion_count,3)
        # No reanchor or inherited output with stale input.
        n._clutch_held=False;n._hold_for_tracking_gap.return_value=True
        n._publish_synced_pose_frame_if_ready()
        self.assertEqual(n.motion_count,3)


    def test_X_uses_original_grip_paths_with_preflexed_three_finger_start(self):
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.poses,hand.preparing=POSES,{}
        measured=dict.fromkeys((n for names in FINGER_NAMES.values() for n in names),0.)
        for depth in (0.,.01,.02,.021,.25,.5,.75,1.):
            hand.controls=controls('index_pinch',trigger=depth)
            desired=targets(POSES,hand.controls)
            before=desired.copy()
            hand.prepare_targets(desired,measured,1.)
            self.assertEqual(before,desired)
            for side,names in FINGER_NAMES.items():
                original=targets(POSES,controls('grasp',trigger=0.,grip=1.))
                for i,n in enumerate(names[:12]):
                    start_fraction=.25 if i<4 else .15
                    phase=start_fraction+(1-start_fraction)*depth
                    expected=POSES[side]['open'][i]+phase*(POSES[side]['pinch'][i]-POSES[side]['open'][i])
                    self.assertAlmostEqual(desired[n],expected)
                    if depth==1.:self.assertAlmostEqual(desired[n],original[n])
                self.assertEqual(POSES[side]['index_prepare'][12:],POSES[side]['index_pinch'][12:])
                for i in (13,14,15,17,18,19):self.assertGreater(desired[names[i]],0.)
        self.assertFalse(hand.preparing)

    def test_left_clutch_transports_lift_before_rebase(self):
        # Execute real wrapper with a snapshot that transports BOTH arms.
        import threading
        from copy import deepcopy
        import numpy as np
        from robotis_vuer.xr_arm_targets import rigid_pose
        path=ROOT/'robotis_vuer/robotis_vuer/vr_publisher_sh5_hands_hardware.py'
        cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_publish_synced_pose_frame_if_ready')
        class Base:
            def _publish_synced_pose_frame_if_ready(self):
                for side in ('left','right'):
                    np.testing.assert_allclose(self._mapper.anchors[side].robot_pose,self._goals[side])
        isolated=ast.Module(body=[ast.ClassDef(name='Publisher',bases=[ast.Name(id='Base',ctx=ast.Load())],keywords=[],body=[method],decorator_list=[])],type_ignores=[])
        scope={'Base':Base,'deepcopy':deepcopy,'rigid_pose':rigid_pose,'time':SimpleNamespace(monotonic=lambda:10.)}
        exec(compile(ast.fix_missing_locations(isolated),str(path),'exec'),scope)
        n=scope['Publisher']();n._controller_mode,n._hand_mode,n.arm_control_enabled=True,2,True
        n._phase,n._xr_lock,n._last_pub='ACTIVE',threading.RLock(),None
        n._clutch_held,n._clutch_reanchor_pending=False,False
        n.left_controller_matrix=np.eye(4);n._last_input={}
        n._mapper=SimpleNamespace(anchors={s:SimpleNamespace(input_pose=np.eye(4),robot_pose=np.eye(4)) for s in ('left','right')})
        n._session=SimpleNamespace(mapper=n._mapper);n._goals={s:np.eye(4) for s in ('left','right')}
        n._elbows={s:np.eye(4) for s in ('left','right')}
        n._hold_for_tracking_gap=Mock(return_value=False);n._graph_check=Mock()
        def snapshot():
            for side in ('left','right'):
                n._mapper.anchors[side].robot_pose[2,3]-=.1
                n._goals[side][2,3]-=.1
                n._elbows[side][2,3]-=.1
        n._snapshot=snapshot
        n._publish_synced_pose_frame_if_ready()
        for side in ('left','right'):
            self.assertAlmostEqual(n._goals[side][2,3],-.1)
            self.assertAlmostEqual(n._elbows[side][2,3],-.1)


    def test_small_trigger_reversal_does_not_bypass_remaining_close_ramp(self):
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.mode,hand.hand_mode,hand.poses='controllers',2,POSES
        measured={n:0. for names in FINGER_NAMES.values() for n in names}
        hand.limits={n:dict(lower=-4.,upper=4.,velocity=2.) for n in measured}
        hand.previous,hand.preparing,hand.last_trigger={},{},{}
        hand.active,hand.last_output=False,0.
        hand.node=SimpleNamespace(measured_rx=1.);hand.publish=Mock();hand.rx=1.
        hand.controls=controls('index_pinch',trigger=1.)
        hand.tick(measured,1.,True)
        first=hand.publish.call_args.args[0].copy()
        hand.controls=controls('index_pinch',trigger=.999)
        hand.tick(measured,1.01,True)
        second=hand.publish.call_args.args[0]
        for side,names in FINGER_NAMES.items():
            for i in (5,6,9,10):
                if POSES[side]['index_pinch'][i]>0.01:
                    self.assertGreater(second[names[i]],first[names[i]])
                else:
                    self.assertAlmostEqual(second[names[i]],first[names[i]])
                self.assertLessEqual(second[names[i]]-first[names[i]],.00500001)


    def test_Y_continuous_simultaneous_thumb_and_ulnar_fingers(self):
        hand=SH5HandOutput.__new__(SH5HandOutput);hand.poses=POSES;hand.preparing={}
        measured=dict.fromkeys((n for names in FINGER_NAMES.values() for n in names),0.)
        hand.controls=controls('ulnar_pinch',trigger=.1)
        desired=targets(POSES,hand.controls);before=desired.copy()
        self.assertEqual(hand.prepare_targets(desired,measured,1.),before)
        for side,names in FINGER_NAMES.items():
            start,end=POSES[side]['ulnar_prepare'],POSES[side]['ulnar_pinch']
            for i in (2,3,13,14,17,18):
                self.assertAlmostEqual(desired[names[i]],start[i]+.1*(end[i]-start[i]))
                self.assertNotEqual(desired[names[i]],end[i])
        self.assertFalse(hand.preparing)

    def test_default_grip_has_opposed_thumb_C_rest(self):
        for side,poses in POSES.items():
            start,end=poses['full_prepare'],poses['full_grip']
            self.assertEqual(start[:2],end[:2])
            for i in (5,9,13,17):self.assertGreater(start[i],.5)
            for i in (2,3):self.assertGreater(abs(start[i]),0.)


    def test_real_overhead_loop_holds_left_elbow_during_lift(self):
        import threading
        from copy import deepcopy
        import numpy as np
        from scipy.spatial.transform import Rotation
        from robotis_vuer.xr_arm_targets import rigid_pose
        path=ROOT/'robotis_vuer/robotis_vuer/vr_publisher_sg2_xr_overhead.py'
        cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_publish_synced_pose_frame_if_ready')
        isolated=ast.Module(body=[ast.ClassDef(name='Publisher',bases=[],keywords=[],body=[method],decorator_list=[])],type_ignores=[])
        scope={'time':SimpleNamespace(monotonic=lambda:10.),'np':np,'Rotation':Rotation,'deepcopy':deepcopy,'rigid_pose':rigid_pose,'SIDES':('left','right'),
               'limited_pose':lambda old,new,*args:new.copy()}
        exec(compile(ast.fix_missing_locations(isolated),str(path),'exec'),scope)
        n=scope['Publisher']();n._path_ready=True;n.arm_control_enabled=True;n._phase='ACTIVE'
        n._last_pub=None;n._xr_lock=threading.RLock();n._controller_fault_latched=False
        n._arm_tracking_is_fresh=lambda now:True;n._graph_check=Mock();n._check_shoulder_stationarity=Mock()
        n._command_rx=dict.fromkeys(('left','right'),10.);n.FEEDBACK_MAX_AGE=5.
        n.TRACKING_POSITION_LIMIT_M=.25;n.TRACKING_ROTATION_LIMIT_RAD=1.57
        n.goal_speed,n.angular_speed=.2,1.
        n._goals={side:np.eye(4) for side in ('left','right')}
        n._elbows={side:np.eye(4) for side in ('left','right')}
        n.left_controller_matrix=n.right_controller_matrix=np.eye(4)
        n._mapper=SimpleNamespace(anchors={side:SimpleNamespace(robot_pose=np.eye(4)) for side in ('left','right')})
        n._session=SimpleNamespace(inspect=Mock(return_value={}))
        n._hints={side:SimpleNamespace(path_anchor=np.zeros(3),
            project_wrist=lambda pos:(pos.copy(),0.),limit_filtered_step=lambda old,new:(new.copy(),1.),
            elbow=Mock(return_value=np.array([0.,0.,-.1]))) for side in ('left','right')}
        n._arm_input_enabled=lambda side:side!='left'
        def snapshot():
            for side in ('left','right'):
                n._goals[side][2,3]-=.1;n._elbows[side][2,3]-=.1
            return {side:{'wrist':n._goals[side].copy()} for side in ('left','right')}
        n._snapshot=snapshot
        n._mapper.target=lambda side,pose:n._goals[side].copy()
        n._publish_pair=Mock();n._set_arm_control_enabled=Mock()
        n._publish_synced_pose_frame_if_ready()
        n._set_arm_control_enabled.assert_not_called();n._publish_pair.assert_called_once()
        self.assertTrue(n._path_frame['left']['clutch_hold'])
        n._hints['left'].elbow.assert_not_called()
        for side in ('left','right'):
            self.assertAlmostEqual(n._goals[side][2,3],-.1)
            self.assertAlmostEqual(n._elbows[side][2,3],-.1)
        # A held arm still fails the inherited measured tracking guard.
        n._last_pub=None
        n._snapshot=lambda:{side:{'wrist':np.eye(4)} for side in ('left','right')}
        n._goals['left'][0,3]=1.
        n._publish_synced_pose_frame_if_ready()
        n._set_arm_control_enabled.assert_called_once()
        self.assertIn('tracking error',n._set_arm_control_enabled.call_args.args[1])


    def test_body_motion_snapshot_relaxes_elbows_then_reanchors(self):
        from copy import deepcopy
        import numpy as np
        from robotis_vuer.xr_arm_targets import rigid_pose
        from hardware.controller_stick_motion import BodyMotionPolicy
        path=ROOT/'robotis_vuer/robotis_vuer/vr_publisher_sh5_hands_hardware.py'
        cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_snapshot')
        links={s:{'elbow':np.eye(4)} for s in ('left','right')}
        for link in links.values():link['elbow'][2,3]=-.2
        class Base:
            def _snapshot(self):return deepcopy(links)
        clock=[1.]
        isolated=ast.Module(body=[ast.ClassDef(name='Publisher',bases=[ast.Name(id='Base',ctx=ast.Load())],keywords=[],body=[method],decorator_list=[])],type_ignores=[])
        scope={'Base':Base,'deepcopy':deepcopy,'rigid_pose':rigid_pose,'time':SimpleNamespace(monotonic=lambda:clock[0]),'SIDES':('left','right'),'Bool':lambda **kw:SimpleNamespace(**kw)}
        exec(compile(ast.fix_missing_locations(isolated),str(path),'exec'),scope)
        n=scope['Publisher']();n._controller_mode,n._hand_mode=True,2
        n._body_elbow_pub=Mock();n._body_motion=False;n._body_policy=BodyMotionPolicy()
        n._aux_base=[0.,0.,.1];n._aux_joints={};n.lift_joint_current_position=-.2
        n._mapper=SimpleNamespace(anchors={s:SimpleNamespace(input_pose=np.eye(4),robot_pose=np.eye(4)) for s in ('left','right')})
        def hints():return {s:SimpleNamespace(previous=np.zeros(3),last_bend=np.ones(3)) for s in ('left','right')}
        n._hints=hints();n._session=SimpleNamespace(mapper=n._mapper,templates=hints())
        n._elbows={s:np.eye(4) for s in ('left','right')};n._goals={s:np.eye(4) for s in ('left','right')}
        n.left_controller_matrix=np.eye(4);n.left_controller_matrix[1,3]=.8
        n.right_controller_matrix=np.eye(4);n.right_controller_matrix[1,3]=-.8
        n._last_input={};n.get_logger=lambda:SimpleNamespace(info=Mock())
        n._snapshot();self.assertTrue(n._body_motion)
        for side in ('left','right'):self.assertAlmostEqual(n._elbows[side][2,3],-.2)
        self.assertTrue(n._body_elbow_pub.publish.call_args.args[0].data)
        n._aux_base=[0.,0.,0.];clock[0]=1.3;n._snapshot()
        self.assertFalse(n._body_motion)
        self.assertFalse(n._body_elbow_pub.publish.call_args.args[0].data)
        np.testing.assert_array_equal(n._mapper.anchors['left'].input_pose,n.left_controller_matrix)
        np.testing.assert_array_equal(n._mapper.anchors['right'].input_pose,n.right_controller_matrix)

if __name__=='__main__':unittest.main()
