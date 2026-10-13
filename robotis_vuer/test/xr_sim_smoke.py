"""[ADDED] Actual ROS + Cyclo + PhysX smoke with SYNTHETIC WebXR events.

Requires the SG2 simulator and unchanged Cyclo controller (arm:=false) in
domain 232. It never connects to physical domain 30 or a Quest browser.
"""
import argparse
import asyncio
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from scipy.spatial.transform import Rotation as R

from robotis_vuer.vr_publisher_sg2_xr import XRDeltaPublisher


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '232':
        raise SystemExit('This synthetic command test requires isolated simulation domain 232')
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = XRDeltaPublisher(start_server=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    loop = asyncio.new_event_loop()
    head = np.eye(4)
    head[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    head[:3, 3] = [0, 1.6, 0]
    inputs = {s: np.eye(4) for s in ('left', 'right')}
    inputs['left'][:3, 3] = [-.20, 1.1, -.3]
    inputs['right'][:3, 3] = [.20, 1.1, -.3]
    rows = []
    failures = []
    wall_start = time.monotonic()
    started = None
    stages = set()
    try:
        while time.monotonic() - wall_start < 48:
            for _ in range(60):
                executor.spin_once(timeout_sec=0.)
            now = time.monotonic()
            if started is None:
                try:
                    node._snapshot()
                    if node._joint_rx is not None and len(node._robot_poses) == 2:
                        started = now
                        print('[TEST] Sim TF, joint feedback and Cyclo poses received', flush=True)
                except Exception:
                    pass
                if now - wall_start > 12:
                    failures.append('feedback unavailable for 12 seconds')
                    break
            elapsed = 0. if started is None else now - started
            if elapsed > 31:
                break
            h = head.copy()
            left, right = inputs['left'].copy(), inputs['right'].copy()
            # Test head-only movement after calibration: it must not move goals.
            if 7 < elapsed < 11:
                h[:3, :3] = R.from_euler('y', 50, degrees=True).as_matrix() @ h[:3, :3]
                h[:3, 3] += [.1, -.15, .08]
            left[:3, 3] += [0, 0, -.05 * np.clip((elapsed - 14) / 4, 0, 1)]
            if elapsed > 21:
                angle = 15 * np.clip((elapsed - 21) / 2, 0, 1)
                left[:3, :3] = R.from_euler('y', angle, degrees=True).as_matrix()
            a_button = ((2.0 < elapsed < 2.15) or (12.0 < elapsed < 12.15)
                        or (27.0 < elapsed < 27.15))
            b_button = 25.0 < elapsed < 25.15
            body = np.tile(np.eye(4).flatten(order='F'), 7)
            body[6 * 16:7 * 16] = h.flatten(order='F')
            states = {'aButton': bool(a_button), 'bButton': bool(b_button),
                      'triggerValue': 1.0, 'thumbstickValue': [1.0, 1.0]}
            if 29 < elapsed < 29.2:
                left[0, 0] = np.nan  # Active invalid input must send False.
            loop.run_until_complete(node.on_body_tracking_move(SimpleNamespace(value={'body': body}), None))
            loop.run_until_complete(node.on_controller_move(SimpleNamespace(value={
                'left': left.flatten(order='F').tolist(), 'right': right.flatten(order='F').tolist(),
                'leftState': states, 'rightState': states}), None))
            for _ in range(60):
                executor.spin_once(timeout_sec=0.)
            if node._goals:
                try:
                    actual = node._snapshot()
                    rows.append({'t': elapsed, 'active': node.arm_control_enabled,
                                 'mapper_id': id(node._mapper) if node._mapper is not None else None,
                                 'goal_l': node._goals['left'].tolist(), 'goal_r': node._goals['right'].tolist(),
                                 'actual_l': actual['left']['wrist'].tolist(),
                                 'actual_r': actual['right']['wrist'].tolist(),
                                 'joints': dict(zip(node.current_joint_states.name, node.current_joint_states.position))})
                except Exception:
                    pass
            stage = int(elapsed // 5) * 5
            if stage not in stages:
                stages.add(stage)
                print(f'[TEST] t={elapsed:.1f}s active={node.arm_control_enabled} pairs={node.published_pairs}', flush=True)
            time.sleep(.02)
    except Exception as error:
        failures.append(repr(error))
    finally:
        node._set_arm_control_enabled(False, 'synthetic smoke completed')
        for _ in range(10):
            executor.spin_once(timeout_sec=0.)
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        loop.close()

    def at(lo, hi):
        return [x for x in rows if lo <= x['t'] <= hi]

    metrics = {}
    try:
        start = at(5, 6)[-1]
        settled_start = at(13.3, 13.8)[-1]
        head_only = at(7.2, 10.5)
        end = at(19.5, 20.5)[-1]
        rotated = at(24, 24.8)[-1]
        metrics['head_only_goal_drift_m'] = max(float(np.linalg.norm(
            np.array(x['goal_l'])[:3, 3] - np.array(start['goal_l'])[:3, 3])) for x in head_only)
        metrics['left_actual_forward_m'] = float(np.array(end['actual_l'])[0, 3] - np.array(start['actual_l'])[0, 3])
        metrics['right_actual_drift_m'] = float(np.linalg.norm(np.array(end['actual_r'])[:3, 3] - np.array(start['actual_r'])[:3, 3]))
        # The original t=5 baseline is still inside Cyclo's 3s delay + 8s
        # slow-start ramp. Preserve it above; separately measure the operator
        # movement relative to a settled baseline just before the t=14 motion.
        metrics['settled_left_forward_m'] = float(np.array(end['actual_l'])[0, 3] - np.array(settled_start['actual_l'])[0, 3])
        metrics['settled_right_drift_m'] = float(np.linalg.norm(np.array(end['actual_r'])[:3, 3] - np.array(settled_start['actual_r'])[:3, 3]))
        metrics['left_position_error_m'] = float(np.linalg.norm(np.array(end['actual_l'])[:3, 3] - np.array(end['goal_l'])[:3, 3]))
        metrics['rotation_only_goal_position_drift_m'] = float(np.linalg.norm(np.array(rotated['goal_l'])[:3, 3] - np.array(end['goal_l'])[:3, 3]))
        metrics['left_rotation_error_deg'] = float(np.rad2deg(R.from_matrix(
            np.array(rotated['goal_l'])[:3, :3] @ np.array(rotated['actual_l'])[:3, :3].T).magnitude()))
        checks = {
            'head_invariance': metrics['head_only_goal_drift_m'] < 1e-5,
            'left_moved': metrics['left_actual_forward_m'] > .025,
            'right_stays': metrics['right_actual_drift_m'] < .015,
            'wrist_tracking': metrics['left_position_error_m'] < .03,
            'pure_rotation_no_translation': metrics['rotation_only_goal_position_drift_m'] < .003,
            'wrist_rotation_tracking': metrics['left_rotation_error_deg'] < 15,
            'active_XA_does_not_recapture': bool(at(11, 13)) and len({x['mapper_id'] for x in at(11, 13)}) == 1,
            'YB_stops': bool(at(25.3, 26)) and not any(x['active'] for x in at(25.3, 26)),
            'invalid_input_stops': bool(at(29.3, 30.5)) and not any(x['active'] for x in at(29.3, 30.5)),
        }
        failures.extend(key for key, value in checks.items() if not value)
    except Exception as error:
        checks = {}
        failures.append(f'insufficient active trajectory: {error}')
    result = {'passed': not failures, 'failures': failures, 'metrics': metrics,
              'checks': checks, 'synthetic_input': True, 'domain': 232, 'rows': rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print('[RESULT]', json.dumps({k: v for k, v in result.items() if k != 'rows'}), flush=True)
    raise SystemExit(0 if result['passed'] else 2)


if __name__ == '__main__':
    main()
