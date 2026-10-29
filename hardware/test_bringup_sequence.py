"""Ready action ordering and lift trajectory limits; no ROS context or motors."""
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace as S

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from hardware.ready_pose import ReadyPoseMover, ARM_NAMES, READY_Q, make_trajectory


class BringupTests(unittest.TestCase):
    def mover(self, events):
        n=ReadyPoseMover.__new__(ReadyPoseMover)
        n.description_ok=n.lift_first=True
        n.state=({name:q for names in ARM_NAMES.values() for name,q in zip(names,READY_Q)},
                 {name:0. for names in ARM_NAMES.values() for name in names})
        n.fresh=lambda:True;n.check_other_commanders=Mock();n.get_logger=lambda:Mock()
        n.raise_lift=Mock(side_effect=lambda:events.append('lift top verified'))
        n.wait_for=lambda fn,*args: self.assertTrue(fn())
        n.requests=[];n.handles=[];n.cancel_requests=[]
        n.action_clients={}
        for side in ('l','r'):
            response=S(status=GoalStatus.STATUS_SUCCEEDED,result=S(error_code=FollowJointTrajectory.Result.SUCCESSFUL))
            future=S(done=lambda:True,result=lambda r=response:r)
            handle=S(accepted=True,get_result_async=lambda f=future:f)
            request=S(done=lambda:True,result=lambda h=handle:h,add_done_callback=lambda cb:None)
            n.action_clients[side]=S(server_is_ready=lambda:True,
                send_goal_async=lambda goal,s=side,r=request:(events.append('arm '+s),r)[1])
        return n

    def test_lift_completion_precedes_both_arm_goals(self):
        events=[];n=self.mover(events)
        with patch('hardware.ready_pose.time.monotonic',side_effect=range(100)),patch('hardware.ready_pose.rclpy.spin_once'):
            n.move(5.)
        self.assertEqual(events,['lift top verified','arm l','arm r'])

    def test_lift_failure_never_sends_arm_goals(self):
        events=[];n=self.mover(events);n.raise_lift.side_effect=RuntimeError('lift not reached')
        with patch('hardware.ready_pose.time.monotonic',side_effect=range(100)),patch('hardware.ready_pose.rclpy.spin_once'):
            with self.assertRaisesRegex(RuntimeError,'lift not reached'):n.move(5.)
        self.assertEqual(events,[])

    def test_lift_quintic_respects_conservative_speed_and_endpoint(self):
        trajectory,duration=make_trajectory(['lift_joint'],[-.5],[0.],3.,max_speed=.12)
        self.assertGreaterEqual(duration,1.875*.5/.12)
        self.assertEqual(list(trajectory.points[-1].positions),[0.])
        self.assertLessEqual(max(abs(p.velocities[0]) for p in trajectory.points),.120001)
