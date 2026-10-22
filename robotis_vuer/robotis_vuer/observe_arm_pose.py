"""Record VR poses and robot feedback on the PC without sending robot commands."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60.)
    parser.add_argument('--output', help='JSON path; default: /workspace/teleop_logs/<time>.json')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    started = datetime.now(timezone.utc)
    output = Path(args.output) if args.output else Path('/workspace/teleop_logs') / (
        started.strftime('vr_arm_%Y%m%dT%H%M%S_%fZ.json'))
    output.parent.mkdir(parents=True, exist_ok=True)
    # Check the output location before waiting through the capture; never overwrite a log.
    with output.open('x') as stream:
        rclpy.init(args=[])
        node = Node('quest_arm_pose_observer')
        latest, counts, frames = {}, Counter(), []

        def save(topic, **values):
            counts[topic] += 1
            latest[topic] = dict(received_monotonic=time.monotonic(), **values)

        def joints(topic, msg):
            names = msg.name if isinstance(msg, JointState) else msg.joint_names
            positions = msg.position if isinstance(msg, JointState) else (
                msg.points[0].positions if msg.points else [])
            save(topic, stamp=[msg.header.stamp.sec, msg.header.stamp.nanosec],
                 positions=dict(zip(names, positions)),
                 degrees={n: math.degrees(v) for n, v in zip(names, positions)
                          if n.startswith(('arm_l_joint', 'arm_r_joint'))})

        def pose(topic, msg):
            p, q = msg.pose.position, msg.pose.orientation
            save(topic, frame=msg.header.frame_id,
                 stamp=[msg.header.stamp.sec, msg.header.stamp.nanosec],
                 xyz=[p.x, p.y, p.z], xyzw=[q.x, q.y, q.z, q.w])

        def subscribe(msg_type, topic, callback):
            node.create_subscription(msg_type, topic,
                                     lambda msg: callback(topic, msg), qos_profile_sensor_data)

        subscribe(JointState, '/joint_states', joints)
        subscribe(Bool, '/reactivate', lambda topic, msg: save(topic, value=msg.data))
        subscribe(String, '/vr/diagnostics/arm_tracking_status',
                  lambda topic, msg: save(topic, value=msg.data))
        for topic in ('/vr/diagnostics/head_pose', '/vr/diagnostics/arm_reference_pose'):
            subscribe(PoseStamped, topic, pose)
        for side, short in (('left', 'l'), ('right', 'r')):
            subscribe(JointTrajectory,
                      f'/leader/joint_trajectory_command_broadcaster_{side}/joint_trajectory',
                      joints)
            for part in ('shoulder', 'elbow', 'wrist', 'subgoal', 'goal', 'gripper'):
                subscribe(PoseStamped, f'/{short}_{part}_pose', pose)

        print(f'Read-only capture for {args.seconds:g}s. Ctrl+C also saves the log.', flush=True)
        print(f'Output: {output}', flush=True)
        print('Keep the PC VR server and robot controller running. '
              'Original hand poses are sent only while hand teleop is enabled.', flush=True)
        start = last_frame = last_print = time.monotonic()
        interrupted = False
        try:
            while rclpy.ok() and time.monotonic() - start < args.seconds:
                rclpy.spin_once(node, timeout_sec=.02)
                now = time.monotonic()
                if now - last_frame >= .1:
                    frame = {topic: dict(age_sec=now - value['received_monotonic'], **value)
                             for topic, value in latest.items()}
                    frames.append({'elapsed_sec': now - start, 'topics': frame})
                    last_frame = now
                if now - last_print >= 1.:
                    def summary(topic):
                        value = latest.get(topic)
                        if value is None:
                            return 'NO DATA'
                        age = now - value['received_monotonic']
                        if age > .5:
                            return f'STALE ({age:.1f}s)'
                        if 'xyz' in value:
                            return [round(v, 3) for v in value['xyz']]
                        return 'OK'

                    print(json.dumps({
                        'seconds': round(now - start, 1),
                        'joints': summary('/joint_states'),
                        'left_shoulder_xyz': summary('/l_shoulder_pose'),
                        'right_shoulder_xyz': summary('/r_shoulder_pose'),
                        'left_goal': summary('/l_goal_pose'),
                        'right_goal': summary('/r_goal_pose'),
                        'tracking': latest.get('/vr/diagnostics/arm_tracking_status', {}).get(
                            'value', 'unavailable'),
                    }), flush=True)
                    last_print = now
        except (KeyboardInterrupt, ExternalShutdownException):
            interrupted = True
        finally:
            json.dump({
                'started_utc': started.isoformat(), 'elapsed_sec': time.monotonic() - start,
                'domain_id': os.environ.get('ROS_DOMAIN_ID'), 'interrupted': interrupted,
                'sample_hz': 10, 'sampling': 'latest received values; not time synchronized',
                'counts': dict(counts), 'samples': frames, 'robot_commands_published': False,
            }, stream, indent=2)
            stream.write('\n')
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        required = ('/joint_states', '/l_goal_pose', '/r_goal_pose',
                    '/l_shoulder_pose', '/r_shoulder_pose', '/l_elbow_pose', '/r_elbow_pose')
        missing = [topic for topic in required if not counts[topic]]
        print(f'Saved: {output}\nTopic counts: {dict(counts)}', flush=True)
        if missing:
            print('Missing topics: ' + ', '.join(missing), flush=True)


if __name__ == '__main__':
    main()
