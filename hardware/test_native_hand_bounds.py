"""Native optimizer margin regressions without ROS or device access."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from hardware.sh5_controller_hands import SH5HandOutput
from hardware.sh5_hands_model import FINGER_NAMES


class NativeHandBoundsTests(unittest.TestCase):
    def setup_hand(self, side):
        hand=SH5HandOutput.__new__(SH5HandOutput)
        hand.native={}
        hand.limits={n:dict(lower=-1.,upper=1.) for n in FINGER_NAMES[side]}
        hand.node=SimpleNamespace(intent_enabled=True,latch=Mock())
        msg=SimpleNamespace(joint_names=list(FINGER_NAMES[side]),
                            points=[SimpleNamespace(positions=[0.]*20)])
        return hand,msg

    def test_solver_boundary_margin_clamps_both_hands_without_fault(self):
        for side in FINGER_NAMES:
            for value,expected in ((-1.001,-1.),(1.001,1.),(.25,.25)):
                hand,msg=self.setup_hand(side)
                msg.points[0].positions=[value]*20
                hand.native_trajectory(side,msg)
                hand.node.latch.assert_not_called()
                self.assertEqual(set(hand.native[side][0].values()),{expected})

    def test_large_overshoot_and_nonfinite_still_latch_with_joint_details(self):
        for value in (-1.002,1.002,math.nan,math.inf):
            hand,msg=self.setup_hand('left')
            hand.native['left']=({},0.)
            msg.points[0].positions[0]=value
            hand.native_trajectory('left',msg)
            hand.node.latch.assert_called_once()
            self.assertIn('finger_l_joint1',hand.node.latch.call_args.args[0])
            self.assertNotIn('left',hand.native)

    def test_wrong_joint_mapping_still_rejected(self):
        hand,msg=self.setup_hand('left')
        msg.joint_names[-1]=msg.joint_names[0]
        hand.native_trajectory('left',msg)
        hand.node.latch.assert_called_once()
        self.assertNotIn('left',hand.native)


if __name__=='__main__':unittest.main()
