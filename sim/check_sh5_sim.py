"""Read-only graph/status checks for the SH5 simulation."""
import argparse
import json
import time
import rclpy
from std_msgs.msg import String
from sim.prepare_sh5_sim import require_sim


def main():
    require_sim()
    parser = argparse.ArgumentParser()
    parser.add_argument('--preflight', action='store_true')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node('sh5_sim_check')
    status = {}
    for topic in ('/sh5_sim/input_status', '/sh5_sim/bridge_status'):
        node.create_subscription(String, topic, lambda m, t=topic: status.update({t: json.loads(m.data)}), 10)
    end = time.monotonic() + 3
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    nodes = node.get_node_names()
    problems = []
    if args.preflight:
        for name in ('retargeting_teleop', 'vr_controller', 'reference_checker', 'sh5_sim_bridge'):
            if name in nodes:
                problems.append(f'{name} already running in domain232; stop previous terminal first')
        for side in ('left', 'right'):
            for suffix in (side, side + '_hand'):
                topic = f'/leader/joint_trajectory_command_broadcaster_{suffix}/joint_trajectory'
                if node.count_publishers(topic):
                    problems.append('Existing trajectory writer: ' + topic)
    else:
        print(json.dumps(status, indent=2, ensure_ascii=False))
        for name in ('retargeting_teleop', 'vr_controller', 'reference_checker', 'sh5_sim_bridge'):
            if nodes.count(name) != 1:
                problems.append(f'Expected exactly one {name}; found {nodes.count(name)}')
        bridge = status.get('/sh5_sim/bridge_status', {})
        if not bridge.get('feedback_fresh') or bridge.get('fault'):
            problems.append('Simulator feedback missing or bridge fault: ' + str(bridge.get('fault')))
        if not status.get('/sh5_sim/input_status', {}).get('input_fresh'):
            problems.append('Quest head/both hands or measured feedback not fresh yet')
    node.destroy_node()
    rclpy.shutdown()
    for error in problems:
        print('[SH5 SIM CHECK]', error)
    if problems:
        raise SystemExit(2)
    print('[SH5 SIM CHECK] PASS')


if __name__ == '__main__':
    main()
