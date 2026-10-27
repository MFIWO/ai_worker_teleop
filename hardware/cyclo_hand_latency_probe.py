"""Compare original Cyclo and mode 1 in isolated ROS; never open hardware.

Synthetic landmarks measure input-to-trajectory processing only. They cannot
establish physical motion latency or tracking quality.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    required = {'ROS_DOMAIN_ID': '232', 'ROS_LOCALHOST_ONLY': '1',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}
    if any(os.environ.get(k) != v for k, v in required.items()):
        raise SystemExit('Requires isolated domain232 / localhost / FastDDS')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    import numpy as np
    import rclpy
    from geometry_msgs.msg import Point32
    from robotis_interfaces.msg import HandJoints
    from trajectory_msgs.msg import JointTrajectory
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from hardware.test_thumb_flexion import skeleton
    from hardware.sh5_controller_hands import HAND_TOPICS

    rclpy.init()
    node = rclpy.create_node('isolated_cyclo_latency_probe')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    received = {}
    def callback(msg, side):
        received[side] = (time.monotonic(), msg)
    subscriptions = [node.create_subscription(JointTrajectory, topic,
        lambda msg, side=side: callback(msg, side), qos)
        for side, topic in HAND_TOPICS.items()]
    pubs = {side: node.create_publisher(HandJoints, f'/{side}_hand/hand_joint_pos', qos)
            for side in HAND_TOPICS}
    report = {'hardware_opened': False, 'synthetic_input': True,
              'domain': 232, 'phases': {}}
    try:
        for label, module in [('cyclo_original', 'scripts.teleop_retargeting'),
                              ('current_mode1', 'hardware.cyclo_hand_retarget_node')]:
            # The original module may only define main(), so invoke it explicitly.
            command = [sys.executable, '-B', '-c',
                       f'from {module} import main; main()']
            with tempfile.TemporaryFile(mode='w+') as log:
                proc = subprocess.Popen(command, stdout=log, stderr=log)
                samples = {s: [] for s in pubs}
                try:
                    deadline = time.monotonic()+20
                    while not (all(p.get_subscription_count() == 1 for p in pubs.values())
                               and all(s.get_publisher_count() == 1 for s in subscriptions)):
                        if proc.poll() is not None or time.monotonic() > deadline:
                            raise RuntimeError('Node startup/discovery failed')
                        rclpy.spin_once(node, timeout_sec=.05)
                    # DDS discovery counts can precede usable best-effort links.
                    # Establish both streams before measuring one outstanding pair.
                    startup = HandJoints(joints=[Point32(x=float(p[0]), y=float(p[1]),
                        z=float(p[2])) for p in skeleton(.2, .7, 21)])
                    received.clear()
                    deadline = time.monotonic()+15
                    while len(received) < 2:
                        if time.monotonic() > deadline:
                            raise RuntimeError('Input/output streams did not become ready')
                        for pub in pubs.values():
                            pub.publish(startup)
                        until = time.monotonic()+.1
                        while time.monotonic() < until:
                            rclpy.spin_once(node, timeout_sec=.01)
                    until = time.monotonic()+.5
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=.01)
                    for index in range(80):
                        # One outstanding pair prevents ambiguous sample matching.
                        received.clear()
                        points = skeleton(.2+.2*(index % 2), .7-.3*(index % 2), 21)
                        msg = HandJoints(joints=[Point32(x=float(p[0]), y=float(p[1]),
                                                       z=float(p[2])) for p in points])
                        started = time.monotonic()
                        for pub in pubs.values():
                            pub.publish(msg)
                        while len(received) < 2 and time.monotonic()-started < 1:
                            rclpy.spin_once(node, timeout_sec=.005)
                        if len(received) != 2:
                            raise RuntimeError(f'Missing output at pair {index}')
                        for side, (stamp, trajectory) in received.items():
                            values = trajectory.points[0].positions
                            if len(values) != 20 or not np.isfinite(values).all():
                                raise RuntimeError('Invalid trajectory')
                            if index >= 20:
                                samples[side].append((stamp-started)*1000)
                        # Approximately 30 input pairs/s unless processing overruns.
                        remaining = started+1/30-time.monotonic()
                        if remaining > 0:
                            time.sleep(remaining)
                    report['phases'][label] = {s: {
                        'count': len(v), 'median_ms': float(np.median(v)),
                        'p95_ms': float(np.percentile(v, 95)), 'max_ms': max(v),
                        'over_33ms': sum(x > 1000/30 for x in v),
                        'samples_ms': v} for s, v in samples.items()}
                except Exception:
                    log.seek(0)
                    print(log.read()[-4000:], file=sys.stderr)
                    raise
                finally:
                    proc.terminate()
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
            # Allow old ROS endpoints to disappear before the next phase.
            deadline = time.monotonic()+5
            while any(s.get_publisher_count() for s in subscriptions):
                if time.monotonic() > deadline:
                    raise RuntimeError('Old publisher still discovered')
                rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: {s: {a: b for a, b in v.items() if a != 'samples_ms'}
                         for s, v in phase.items()}
                      for k, phase in report['phases'].items()}, indent=2))


if __name__ == '__main__':
    main()
