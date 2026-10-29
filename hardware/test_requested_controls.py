"""Requested two-finger Y pinch, symmetric clutches and spring-back release pacing."""
import ast
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import unittest

from hardware.test_controller_hand_motion import POSES, controls
from hardware.sh5_controller_hands import SH5HandOutput, targets, MODE2_TRIGGER_OPEN_SPEED_SCALE
from hardware.sh5_hands_model import FINGER_NAMES


class RequestedControlsTests(unittest.TestCase):
    def test_right_clutch_reanchors_only_right_and_repress_does_not_jump(self):
        import threading
        import numpy as np
        from copy import deepcopy
        from robotis_vuer.xr_arm_targets import rigid_pose
        path=Path(__file__).resolve().parents[1]/'robotis_vuer/robotis_vuer/vr_publisher_sh5_hands_hardware.py'
        cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_publish_synced_pose_frame_if_ready')
        class Base:
            def _publish_synced_pose_frame_if_ready(self):self.calls+=1
        tree=ast.Module(body=[ast.ClassDef(name='Publisher',bases=[ast.Name(id='Base',ctx=ast.Load())],keywords=[],body=[method],decorator_list=[])],type_ignores=[])
        scope=dict(Base=Base,time=S(monotonic=lambda:1.),deepcopy=deepcopy,rigid_pose=rigid_pose)
        exec(compile(ast.fix_missing_locations(tree),str(path),'exec'),scope)
        n=scope['Publisher']();n._controller_mode=True;n._hand_mode=2;n.arm_control_enabled=True
        n._phase='ACTIVE';n._mapper=S(anchors={s:S(input_pose=np.eye(4),robot_pose=np.eye(4)) for s in ('left','right')})
        n._session=S(mapper=n._mapper);n._xr_lock=threading.RLock();n._last_pub=None
        n._clutch_held=True;n._clutch_reanchor_pending=False
        n._right_grip_held=False;n._right_clutch_reanchor_pending=False
        n._goals={s:np.eye(4) for s in ('left','right')};n._goals['right'][0,3]=.2
        n.left_controller_matrix=np.eye(4);n.right_controller_matrix=np.eye(4)
        n.right_controller_matrix[1,3]=.5;n._last_input=dict(left=1,right=2);n.calls=0
        n._snapshot=Mock();n._hold_for_tracking_gap=Mock(return_value=False);n._graph_check=Mock()
        n._publish_synced_pose_frame_if_ready()
        np.testing.assert_array_equal(n._mapper.anchors['right'].robot_pose,n._goals['right'])
        np.testing.assert_array_equal(n._mapper.anchors['left'].input_pose,np.eye(4))
        self.assertIn('left',n._last_input);self.assertNotIn('right',n._last_input)
        n._right_grip_held=True;n._right_clutch_reanchor_pending=True;n.right_controller_matrix[1,3]=.8
        n._publish_synced_pose_frame_if_ready()
        anchor=n._mapper.anchors['right'].input_pose.copy()
        n.right_controller_matrix[1,3]=.9;n._publish_synced_pose_frame_if_ready()
        np.testing.assert_array_equal(n._mapper.anchors['right'].input_pose,anchor)
        self.assertFalse(n._right_clutch_reanchor_pending)
        self.assertEqual(n.calls,3)

    def test_Y_only_moves_thumb_and_index(self):
        from hardware.controller_hand_modes import POSE_BUTTONS
        self.assertEqual(POSE_BUTTONS['Y'],'thumb_index_pinch')
        for depth in (0.,.2,.5,1.):
            goal=targets(POSES,controls('thumb_index_pinch',trigger=depth))
            for side,names in FINGER_NAMES.items():
                for i in range(8,20):self.assertEqual(goal[names[i]],POSES[side]['open'][i])
        for side in FINGER_NAMES:
            self.assertNotEqual(POSES[side]['thumb_index_prepare'][:8],POSES[side]['thumb_index_pinch'][:8])

    def test_abrupt_release_is_paced_until_open_even_with_negative_thumb_joints(self):
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.mode,hand.hand_mode,hand.poses='controllers',2,POSES
        names=[n for group in FINGER_NAMES.values() for n in group]
        measured=dict.fromkeys(names,0.)
        hand.limits={n:dict(lower=-4.,upper=4.,velocity=2.) for n in names}
        hand.previous=targets(POSES,controls('full_grip',trigger=1.))
        hand.preparing,hand.last_trigger={},{}
        hand.active,hand.last_output,hand.rx=True,1.,1.
        hand.node=S(measured_rx=1.)
        hand.publish=Mock();hand.controls=controls('full_grip',trigger=0.)
        expected=targets(POSES,hand.controls)
        initial=dict(hand.previous)
        for tick in range(1,501):
            before=dict(hand.previous)
            hand.tick(measured,1.+tick*.01,True)
            for n in names:
                self.assertLessEqual(abs(hand.previous[n]-before[n]),2.*MODE2_TRIGGER_OPEN_SPEED_SCALE*.01+1e-8)
                self.assertLessEqual(abs(hand.previous[n]-expected[n]),abs(before[n]-expected[n])+1e-8)
            if tick==1:self.assertNotEqual(hand.previous,expected)
        self.assertEqual(hand.previous,expected)
        self.assertNotEqual(initial,expected)

    def test_grips_are_independent_and_motion_does_not_disable_tracking(self):
        path=Path(__file__).resolve().parents[1]/'robotis_vuer/robotis_vuer/vr_publisher_sh5_hands_hardware.py'
        cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_arm_input_enabled')
        module=ast.Module(body=[ast.ClassDef(name='Publisher',bases=[],keywords=[],body=[method],decorator_list=[])],type_ignores=[])
        scope={};exec(compile(ast.fix_missing_locations(module),str(path),'exec'),scope)
        n=scope['Publisher']();n._controller_mode=True;n._hand_mode=2
        for moving in (False,True):
            n._body_motion=moving
            for left,right in ((False,False),(True,False),(False,True),(True,True)):
                n._clutch_held,n._right_grip_held=left,right
                self.assertEqual(n._arm_input_enabled('left'),left)
                self.assertEqual(n._arm_input_enabled('right'),right)
