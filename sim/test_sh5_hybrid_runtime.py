"""Synthetic HAND_MOVE -> real publisher/retarget/Cyclo -> measured PhysX smoke.

Requires an already-running simulation controller. Never a Quest or hardware
certification. Writes measured metrics and exits nonzero if either arm/hand
does not move or tracking loss does not stop the publisher.
"""
import asyncio
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from trajectory_msgs.msg import JointTrajectory
from robotis_vuer.vr_publisher_sh5_sim import SH5SimulationPublisher
from robotis_vuer.sh5_optical_hand import SH5_HAND_FRAMES, VR_HAND_TO_SH5
from sim.prepare_sh5_sim import require_sim
from sim.sh5_native_hand_retarget_probe import landmarks


def hand_frame(side, shift=0., curl=0.):
    wrist = np.array([-.2 if side == 'left' else .2, 1.1, -.3 - shift])
    matrices = np.tile(np.eye(4), (25, 1, 1))
    matrices[:, :3, 3] = wrist
    points = landmarks(1. + .8 * curl, curl)
    for index, p in zip(SH5_HAND_FRAMES, points):
        matrices[index, :3, 3] += VR_HAND_TO_SH5.T @ np.array([p.x, p.y, p.z])
    return np.concatenate([m.flatten(order='F') for m in matrices]).tolist()


def main():
    require_sim()
    rclpy.init(args=[])
    node = SH5SimulationPublisher(start_server=False)
    loop = asyncio.new_event_loop()
    raw_hands = {s: [] for s in ('left', 'right')}
    for side in raw_hands:
        node.create_subscription(JointTrajectory, f'/sh5_sim/raw/{side}_hand',
            lambda m, s=side: raw_hands[s].append(list(m.points[-1].positions)) if m.points else None,
            node.vr_stream_qos)
    head = np.eye(4)
    head[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    head[:3, 3] = [0, 1.6, 0]
    body = np.tile(np.eye(4).flatten(order='F'), 7)
    body[96:112] = head.flatten(order='F')
    def step(shift=0., curl=0., hands=True):
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.)
        loop.run_until_complete(node.on_body_tracking_move(SimpleNamespace(value={'body': body}), None))
        if hands:
            value = {s: hand_frame(s, shift, curl) for s in ('left', 'right')}
            loop.run_until_complete(node.on_hand_move(SimpleNamespace(value=value), None))
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.)
        time.sleep(.02)
    def measured():
        return dict(zip(node.current_joint_states.name, node.current_joint_states.position))
    result = {'synthetic_input': True, 'physical_robot': False, 'backend': 'PhysX', 'passed': False}
    try:
        end = time.monotonic() + 15
        while time.monotonic() < end:
            step()
            try:
                node._graph_check()
                node._snapshot()
                break
            except ValueError:
                pass
        node._graph_check()
        for _ in range(30):
            step()
        node._set_arm_control_enabled(True, 'synthetic optical integration test')
        end = time.monotonic() + 15
        while time.monotonic() < end and node._phase != 'ACTIVE':
            step()
            if node._phase not in ('SETTLING', 'WARMUP', 'WAIT_START', 'ACTIVE'):
                raise RuntimeError(f'Start refused: {node._last_reason}')
        if node._phase != 'ACTIVE':
            raise RuntimeError('Activation timed out: ' + str(node._last_reason))
        for _ in range(50):
            step()
        baseline, pose_before = measured(), node._snapshot()
        start = time.monotonic()
        while time.monotonic() - start < 6.:
            u = min(1., (time.monotonic() - start) / 4.)
            u = u*u*(3 - 2*u)
            step(.025*u, .45*u)
            if node._phase != 'ACTIVE':
                raise RuntimeError('Motion stopped: ' + str(node._last_reason))
        final, pose_after = measured(), node._snapshot()
        result['arm_motion_rad'] = {s: max(abs(final[f'arm_{s}_joint{i}'] - baseline[f'arm_{s}_joint{i}'])
            for i in range(1, 8)) for s in ('l', 'r')}
        result['finger_motion_rad'] = {s: max(abs(final[f'finger_{s}_joint{i}'] - baseline[f'finger_{s}_joint{i}'])
            for i in range(1, 21)) for s in ('l', 'r')}
        result['wrist_motion_m'] = {s: float(np.linalg.norm(pose_after[s]['wrist'][:3, 3] -
            pose_before[s]['wrist'][:3, 3])) for s in ('left', 'right')}
        result['wrist_error_m'] = {s: float(np.linalg.norm(node._goals[s][:3, 3] -
            pose_after[s]['wrist'][:3, 3])) for s in ('left', 'right')}
        end = time.monotonic() + node.OPTICAL_MAX_AGE + 1.
        while time.monotonic() < end:
            step(hands=False)
        result['tracking_loss_stops'] = not node.arm_control_enabled
        held = measured()
        end = time.monotonic() + .7
        while time.monotonic() < end:
            step(hands=False)
        stopped = measured()
        result['post_stop_drift_rad'] = max(abs(stopped[k] - held[k]) for k in held
            if k.startswith(('arm_', 'finger_')) and k in stopped)
        result['raw_hand_messages'] = {s: len(values) for s, values in raw_hands.items()}
        result['raw_hand_span_rad'] = {s: float(np.max(np.ptp(values, axis=0))) if values else 0.
            for s, values in raw_hands.items()}
        result['passed'] = (all(v > .003 for v in result['arm_motion_rad'].values()) and
            all(v > .001 for v in result['finger_motion_rad'].values()) and
            all(v < .025 for v in result['wrist_error_m'].values()) and
            all(v > .012 for v in result['wrist_motion_m'].values()) and
            result['tracking_loss_stops'] and result['post_stop_drift_rad'] < .015)
    except Exception as error:
        result['error'] = repr(error)
        result['phase'] = node._phase
        result['bridge'] = node._bridge_status
    finally:
        if rclpy.ok():
            node._set_arm_control_enabled(False, 'integration test finished')
            node._publish_sim_status()
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        rclpy.try_shutdown()
        loop.close()
    Path('/tmp/sh5_hybrid_runtime_result.json').write_text(json.dumps(result, indent=2))
    print('SH5_HYBRID_RUNTIME_RESULT', json.dumps(result), flush=True)
    raise SystemExit(0 if result['passed'] else 2)


if __name__ == '__main__':
    main()
