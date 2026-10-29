"""Move selected hardware arms to the xr_tele ready pose before starting the VR controller.

Ready angles and quintic interpolation match xr_tele's robotis_ai_worker.py.
Optional SH5 bringup first raises lift to its measured URDF upper bound.
"""

import argparse
import json
import math
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


READY_Q = [0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]
ARM_NAMES = {
    side: [f'arm_{side}_joint{i}' for i in range(1, 8)] for side in ('l', 'r')
}


def make_trajectory(names, start, target, duration, num_points=100, *, max_speed=3.):
    """Build the same zero-endpoint-velocity quintic used by xr_tele."""
    if (len(names) != len(start) or len(start) != len(target)
            or not start or num_points < 2
            or not all(math.isfinite(v) for v in [*start, *target, duration])
            or duration <= 0 or not math.isfinite(max_speed) or max_speed<=0):
        raise ValueError('Invalid ready-pose trajectory input')
    # Peak derivative of the unit quintic is 1.875; xr_tele's cap is 3 rad/s.
    duration = max(duration, 1.875 * max(abs(b - a) for a, b in zip(start, target)) / max_speed)
    msg = JointTrajectory(joint_names=names)
    for i in range(num_points):
        u = i / (num_points - 1)
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        ds = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration
        dds = (60 * u - 180 * u**2 + 120 * u**3) / duration**2
        delta = [b - a for a, b in zip(start, target)]
        point = JointTrajectoryPoint(
            positions=[a + s * d for a, d in zip(start, delta)],
            velocities=[ds * d for d in delta],
            accelerations=[dds * d for d in delta],
        )
        nanoseconds = round(duration * u * 1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 10**9)
        msg.points.append(point)
    return msg, duration


class ReadyPoseMover(Node):
    def __init__(self,model='sg2',session_recovery=False,lift_first=False):
        super().__init__('hardware_arm_ready_pose')
        self.model = model
        self.lift_first = bool(lift_first)
        self.lift_limit = None
        if self.lift_first and model!='sh5':
            raise ValueError('lift-first bringup supports only SH5')
        self.session_recovery = bool(session_recovery)  # [CHANGED] only launcher-owned Cyclo stop/restart sequence
        if self.session_recovery and model != 'sh5':
            raise ValueError('session ready recovery supports only SH5')
        self.description_ok = False
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile,DurabilityPolicy
        self.create_subscription(String,'/robot_description',self.on_description,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state = None
        self.state_time = 0.
        self.handles = []
        self.requests = []
        self.cancel_requests = []
        self.aborting = False
        self.create_subscription(JointState, '/joint_states', self.on_state,
                                 qos_profile_sensor_data)
        self.action_clients = {
            side: ActionClient(self, FollowJointTrajectory,
                               f'/arm_{side}_controller/follow_joint_trajectory')
            for side in ARM_NAMES
        }
        self.lift_client = (ActionClient(self, FollowJointTrajectory,
                            '/lift_controller/follow_joint_trajectory') if self.lift_first else None)

    def on_description(self,msg):
        import xml.etree.ElementTree as ET
        try:
            root = ET.fromstring(msg.data)
            if root.get('name') != f'ffw_{self.model}_follower':raise ValueError('wrong follower model')
            if 'mock_components/GenericSystem' in msg.data or 'gazebo_ros2_control' in msg.data:
                raise ValueError('mock/simulation description rejected')
            for names in ARM_NAMES.values():
                for name,target in zip(names,READY_Q):
                    bound = root.find(f"joint[@name='{name}']/limit")
                    if bound is None or not float(bound.get('lower'))<=target<=float(bound.get('upper')):
                        raise ValueError('ready target outside physical description: '+name)
            if self.lift_first:
                joint=root.find("joint[@name='lift_joint']")
                bound=joint.find('limit') if joint is not None else None
                axis=joint.find('axis') if joint is not None else None
                if bound is None or axis is None or joint.get('type')!='prismatic':
                    raise ValueError('physical prismatic lift with limits required')
                direction=[float(v) for v in axis.get('xyz','').split()]
                lower,upper,speed=[float(bound.get(k)) for k in ('lower','upper','velocity')]
                if direction!=[0.,0.,1.] or not all(map(math.isfinite,(lower,upper,speed))) or lower>=upper or speed<=0:
                    raise ValueError('unexpected lift axis/limits')
                self.lift_limit=dict(lower=lower,upper=upper,velocity=min(.12,speed))
            self.description_ok = True
        except (ValueError,TypeError,ET.ParseError):self.description_ok = False

    def on_state(self, msg):
        if len(msg.name) != len(msg.position) or len(msg.name) != len(msg.velocity):
            return
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        names = ARM_NAMES['l'] + ARM_NAMES['r']
        if self.lift_first:names=names+['lift_joint']
        if not all(name in positions and math.isfinite(positions[name])
                   and math.isfinite(velocities[name]) for name in names):
            return
        self.state = (positions, velocities)
        self.state_time = time.monotonic()

    def wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Timed out: {description}')
            rclpy.spin_once(self, timeout_sec=.05)

    def fresh(self):
        return self.state is not None and time.monotonic() - self.state_time < .5

    def check_other_commanders(self):
        # An already-running cyclo VR node retains its pre-move q_desired.
        # It must be started AFTER this move, not merely paused with /reactivate.
        blocked = {'vr_controller', 'leader_controller', 'ai_worker_movej_controller',
                   'vr_publisher_sg2', 'vr_publisher_sh5'}
        if not self.session_recovery:
            blocked.add('vr_trajectory_publisher')
        # [CHANGED] Only the launcher-managed recovery exempts the DISARMED PC
        # tracking node; it MUST NOT exempt a running Cyclo controller.
        found = blocked.intersection(self.get_node_names())
        for side in ('left', 'right'):
            topic = f'/leader/joint_trajectory_command_broadcaster_{side}/joint_trajectory'
            found.update(info.node_name for info in self.get_publishers_info_by_topic(topic))
        if found:
            raise RuntimeError(
                'Stop the existing PC VR server and robot VR/leader controller first '
                '(keep follower bringup running). Found: ' + ', '.join(sorted(found)))
        # Old SH5 startup executors can own action goals without publishing to
        # the streaming topics. Never overlap them with this arms-only move.
        if any('joint_trajectory_executor' in name for name in self.get_node_names()):
            raise RuntimeError('Wait for or stop the existing follower initialization executors before moving arms')

    def remember_handle(self, future):
        handle = future.result()
        if handle.accepted:
            self.handles.append(handle)
            if self.aborting:
                self.cancel_requests.append(handle.cancel_goal_async())

    def move(self, duration):
        self.wait_for(lambda: self.description_ok and self.fresh() and all(
            client.server_is_ready() for client in self.action_clients.values()),
            12., 'fresh 14-arm-joint feedback and both follower action servers')
        # Let graph discovery catch up before looking for competing controllers.
        deadline = time.monotonic() + 1.
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
        self.check_other_commanders()
        if not self.fresh():
            raise RuntimeError('Joint feedback stopped before the ready-pose move')
        positions, velocities = self.state
        if any(abs(velocities[n]) > .1 for names in ARM_NAMES.values() for n in names):
            raise RuntimeError('Arms are still moving; wait for follower initialization to finish')
        if self.lift_first:
            self.raise_lift()
            self.check_other_commanders()
            if not self.fresh():raise RuntimeError('Feedback stopped after lift raise')
            positions, velocities = self.state
        trajectories = {}
        # Use one shared duration even when the initial poses are asymmetric.
        delta_max = max(abs(positions[n] - q) for names in ARM_NAMES.values()
                        for n, q in zip(names, READY_Q))
        duration = max(duration, 1.875 * delta_max / 3.)
        for side, names in ARM_NAMES.items():
            trajectories[side], _ = make_trajectory(
                names, [positions[n] for n in names], READY_Q, duration)
        self.get_logger().info(f'Moving both arms to xr_tele ready pose over {duration:.1f}s')
        requests = self.requests
        for side, client in self.action_clients.items():
            goal = FollowJointTrajectory.Goal(trajectory=trajectories[side])
            request = client.send_goal_async(goal)
            request.add_done_callback(self.remember_handle)
            requests.append(request)
        self.wait_for(lambda: all(f.done() for f in requests), 5., 'goal acceptance')
        if not all(f.result().accepted for f in requests):
            raise RuntimeError('A follower controller rejected the ready pose')
        results = [f.result().get_result_async() for f in requests]
        self.wait_for(lambda: all(f.done() for f in results), duration + 8.,
                      'ready-pose completion')
        for future in results:
            result = future.result()
            if (result.status != GoalStatus.STATUS_SUCCEEDED
                    or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                raise RuntimeError(f'Ready-pose action failed: {result.result.error_string}')

        def settled():
            if not self.fresh():
                return False
            q, dq = self.state
            return all(abs(q[n] - v) <= .05 and abs(dq[n]) <= .1
                       for names in ARM_NAMES.values() for n, v in zip(names, READY_Q))

        self.wait_for(settled, 2., 'measured ready pose (position and velocity)')
        self.get_logger().info(
            'READY: both arms reached the pose; starting the VR publisher')

    def raise_lift(self):
        """Complete and verify lift motion before sending either arm goal."""
        self.wait_for(lambda:self.lift_client.server_is_ready(),8.,'lift action server')
        positions,velocities=self.state
        bound=self.lift_limit
        current=positions['lift_joint']
        if not bound or not bound['lower']-.003<=current<=bound['upper']+.003 or abs(velocities['lift_joint'])>.02:
            raise RuntimeError('Lift feedback outside physical limits or lift still moving')
        target=bound['upper']
        if abs(current-target)>.003:
            trajectory,duration=make_trajectory(['lift_joint'],[current],[target],3.,max_speed=bound['velocity'])
            self.get_logger().info(f'BRINGUP: lift first to URDF top {target:.3f}m over {duration:.1f}s; arms held')
            request=self.lift_client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory))
            request.add_done_callback(self.remember_handle)
            self.requests.append(request)
            self.wait_for(request.done,5.,'lift goal acceptance')
            if not request.result().accepted:raise RuntimeError('Follower rejected lift raise')
            result=request.result().get_result_async()
            self.wait_for(result.done,duration+8.,'lift reaching top')
            response=result.result()
            if response.status!=GoalStatus.STATUS_SUCCEEDED or response.result.error_code!=FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError('Lift action failed: '+response.result.error_string)
        self.wait_for(lambda:self.fresh() and abs(self.state[0]['lift_joint']-target)<=.003
                      and abs(self.state[1]['lift_joint'])<=.01,3.,'measured lift top and stop')
        self.get_logger().info('BRINGUP: measured lift top verified; starting arm ready trajectory')

    def cancel_moves(self):
        self.aborting = True
        self.cancel_requests.extend(handle.cancel_goal_async() for handle in self.handles)
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if all(f.done() for f in self.requests + self.cancel_requests):
                break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',choices=('sg2','sh5'),default='sg2')
    parser.add_argument('--duration', type=float, default=5.)
    parser.add_argument('--lift-first', action='store_true', help='SH5 bringup: lift to physical URDF top before arm goals')
    parser.add_argument('--session-recovery', action='store_true',
                        help='SH5 only: Cyclo already stopped by robot launcher; disarmed VR publisher may remain')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print target only; no ROS node or motion')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
    if args.lift_first and (args.model!='sh5' or args.session_recovery):
        parser.error('--lift-first applies only to SH5 startup, not ready recovery')
    if args.dry_run:
        print(json.dumps({
            'joint_names': ARM_NAMES, 'target_per_arm': READY_Q,
            'minimum_duration_sec': args.duration, 'robot_commands_published': False,
            'lift_first':args.lift_first,
        }))
        return 0
    rclpy.init(args=[])
    node = ReadyPoseMover(args.model, session_recovery=args.session_recovery,lift_first=args.lift_first)
    try:
        node.move(args.duration)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        node.get_logger().error(f'Ready pose aborted; VR will not start. {exc}')
        node.cancel_moves()
        return 1
    finally:
        node.destroy_node()
        # [CHANGED] ROS signal handlers may have already shut the context down
        # on Ctrl+C; avoid an unrelated secondary RCLError traceback.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
