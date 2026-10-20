#!/usr/bin/env python3
"""Clock-step regression against a native node in isolated localhost/domain232.

Publishes synthetic joint feedback and /clock only. No follower or gate is run.
Never changes the host clock. Run against both old and new binaries for comparison.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    required = {'ROS_DOMAIN_ID': '232', 'ROS_LOCALHOST_ONLY': '1',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}
    if any(os.environ.get(k) != v for k, v in required.items()):
        raise SystemExit('Requires isolated localhost/domain232/FastDDS')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-steady', action='store_true')
    parser.add_argument('--feedback-timeout', type=float, default=.5)
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool
    from trajectory_msgs.msg import JointTrajectory
    import yaml

    profile = json.loads(args.profile.read_text())
    params = yaml.safe_load(Path(profile['config']).read_text())['vr_controller']['ros__parameters']
    joints = json.loads((args.profile.parent / 'snapshot/joints.json').read_text())
    rclpy.init()
    node = Node('clock_step_fixture')
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    feedback = node.create_publisher(JointState, params['joint_states_topic'], 10)
    activate = node.create_publisher(Bool, params.get('reactivate_topic', '/reactivate'), 10)
    goals = {side: node.create_publisher(PoseStamped, params[side + '_goal_pose_topic'], 10)
             for side in ('r', 'l')}
    poses = {}
    commands = []
    subscriptions = []
    for side in ('r', 'l'):
        subscriptions.append(node.create_subscription(PoseStamped,
            params[side + '_gripper_pose_topic'],
            lambda msg, s=side: poses.__setitem__(s, msg), 10))
    for side in ('right', 'left'):
        subscriptions.append(node.create_subscription(JointTrajectory,
            params[side + '_traj_topic'], lambda msg: commands.append(time.monotonic()), 10))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log_path = args.output.with_suffix('.log')
    clock_seconds = 100

    def tick(duration, feed=True):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            clock = Clock()
            clock.clock.sec = clock_seconds
            clock_pub.publish(clock)
            if feed:
                msg = JointState()
                msg.header.stamp = clock.clock
                msg.name = list(joints)
                msg.position = list(joints.values())
                msg.velocity = [0.] * len(joints)
                feedback.publish(msg)
            for side, msg in poses.items():
                goals[side].publish(msg)
            for _ in range(16):
                rclpy.spin_once(node, timeout_sec=0.)
            time.sleep(.01)

    def timeouts():
        return log_path.read_text().count('Joint states timed out.')

    result = {'physical_hardware_test': False, 'host_clock_changed': False}
    with log_path.open('w') as log:
        proc = subprocess.Popen([str(args.binary), '--ros-args', '--params-file', profile['config'],
            '-p', 'urdf_path:=' + profile['urdf'], '-p', 'srdf_path:=' + profile['srdf'],
            '-p', 'use_sim_time:=true', '-p', 'activation_delay_seconds:=0.15',
            '-p', 'joint_state_timeout:=' + str(args.feedback_timeout),
            '-p', 'startup_ramp_seconds:=0.15'], stdout=log, stderr=subprocess.STDOUT)
        try:
            tick(2.)
            if len(poses) != 2 or proc.poll() is not None:
                raise RuntimeError('Native node did not receive synthetic joints')
            activate.publish(Bool(data=True))
            tick(.8)
            result['commands_with_frozen_ros_clock'] = len(commands)
            before = timeouts()
            clock_seconds = 1792403305
            tick(.15, feed=False)  # A normal sub-timeout gap across a forward clock jump.
            result['forward_step_false_timeouts'] = timeouts() - before
            tick(.4)
            before = timeouts()
            clock_seconds = 10
            tick(.15, feed=False)
            result['backward_step_false_timeouts'] = timeouts() - before
            tick(.4)
            before = timeouts()
            tick(.8, feed=False)  # A real feedback loss while ROS clock is frozen.
            result['short_gap_timeouts'] = timeouts() - before
            if args.feedback_timeout > .5:
                n = len(commands)
                tick(.3, feed=False)
                result['commands_during_short_gap_hold'] = len(commands) - n
                activations = log_path.read_text().count('Startup check passed.')
                tick(.5)
                result['short_gap_resumed_without_rearming'] = (len(commands) > n and
                    log_path.read_text().count('Startup check passed.') == activations)
                before = timeouts()
                tick(args.feedback_timeout + .3, feed=False)
            result['real_feedback_loss_timeouts'] = timeouts() - before
            n = len(commands)
            tick(.3, feed=False)
            result['commands_during_timeout_hold'] = len(commands) - n
            tick(.8)
            result['commands_after_feedback_recovery'] = len(commands) - n
            if proc.poll() is not None:
                raise RuntimeError('Native node exited unexpectedly')
            result['passed'] = (result['forward_step_false_timeouts'] == 0
                and result['backward_step_false_timeouts'] == 0
                and result['real_feedback_loss_timeouts'] == 1
                and result['commands_with_frozen_ros_clock'] > 0
                and result['commands_during_timeout_hold'] == 0
                and result['commands_after_feedback_recovery'] > 0)
            if args.feedback_timeout > .5:
                result['passed'] &= (result['short_gap_timeouts'] == 0
                    and result['commands_during_short_gap_hold'] == 0
                    and result['short_gap_resumed_without_rearming'])
        finally:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            node.destroy_node()
            rclpy.shutdown()
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if args.require_steady and not result['passed']:
        raise SystemExit('Steady clock regression failed')


if __name__ == '__main__':
    main()
