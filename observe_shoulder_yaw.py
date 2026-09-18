"""Record Quest goals and actual/commanded shoulder joints; publish no commands."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--output', default='/tmp/quest_shoulder_yaw.json')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Node('quest_shoulder_yaw_observer')
    latest, counts, frames = {}, Counter(), []

    def save(topic, **values):
        counts[topic] += 1
        latest[topic] = dict(received_monotonic=time.monotonic(), **values)

    def joints(topic, msg):
        names = msg.name if isinstance(msg, JointState) else msg.joint_names
        positions = msg.position if isinstance(msg, JointState) else (
            msg.points[0].positions if msg.points else [])
        save(topic, degrees={name: math.degrees(value)
                            for name, value in zip(names, positions)
                            if name.startswith(('arm_l_joint', 'arm_r_joint'))})

    def pose(topic, msg):
        p, q = msg.pose.position, msg.pose.orientation
        save(topic, frame=msg.header.frame_id,
             stamp=[msg.header.stamp.sec, msg.header.stamp.nanosec],
             xyz=[p.x, p.y, p.z], xyzw=[q.x, q.y, q.z, q.w])

    def subscribe(msg_type, topic, callback):
        node.create_subscription(msg_type, topic,
                                 lambda msg: callback(topic, msg), qos_profile_sensor_data)

    subscribe(JointState, '/joint_states', joints)
    for side, short in (('left', 'l'), ('right', 'r')):
        subscribe(JointTrajectory,
                  f'/leader/joint_trajectory_command_broadcaster_{side}/joint_trajectory',
                  joints)
        for part in ('shoulder', 'elbow', 'wrist', 'subgoal', 'goal', 'gripper'):
            subscribe(PoseStamped, f'/{short}_{part}_pose', pose)

    print('Read-only observer ready. No activation or motor commands are published.', flush=True)
    print('joint1=pitch, joint2=roll, joint3=yaw; all joint values are degrees.', flush=True)
    start = last_frame = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if now - last_frame >= 1.:
                frame = {topic: dict(age_sec=now - value['received_monotonic'], **value)
                         for topic, value in latest.items()}
                frames.append(dict(elapsed_sec=now - start, topics=frame))
                actual = frame.get('/joint_states', {})
                angles = actual.get('degrees', {})
                print(json.dumps(dict(
                    elapsed_sec=round(now - start, 1),
                    actual_age_sec=actual.get('age_sec'),
                    shoulder_yaw_deg={k: v for k, v in angles.items()
                                      if k.endswith('joint3')})), flush=True)
                last_frame = now
    finally:
        Path(args.output).write_text(json.dumps(
            dict(counts=dict(counts), samples=frames, robot_commands_published=False),
            indent=2) + '\n')
        print(f'Saved {args.output}; topic counts: {dict(counts)}', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
