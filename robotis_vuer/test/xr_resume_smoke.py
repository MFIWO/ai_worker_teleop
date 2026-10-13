"""[ADDED] Synthetic XR -> real Cyclo -> PhysX, fixed-reference restart test.

Requires dedicated domain232 simulator/controller, never physical domain30.
This is a simulation test, not a recorded Quest replay or hardware validation.
"""
import argparse
import asyncio
import hashlib
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from scipy.spatial.transform import Rotation

from robotis_vuer.vr_publisher_sg2_xr_resume import XRResumePublisher


def signature(session):
    data = {'basis': session.mapper.basis.tolist(),
            'anchors': {s: [a.input_pose.tolist(), a.robot_pose.tolist()]
                        for s, a in session.mapper.anchors.items()},
            'bounds': {s: [h.minimum, h.maximum, h.upper, h.forearm]
                       for s, h in session.templates.items()}}
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '232':
        raise SystemExit('Simulation test requires ROS_DOMAIN_ID232')
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = XRResumePublisher(start_server=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    loop = asyncio.new_event_loop()
    head = np.eye(4)
    head[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    head[:3, 3] = [0, 1.6, 0]
    inputs = {s: np.eye(4) for s in ('left', 'right')}
    inputs['left'][:3, 3], inputs['right'][:3, 3] = [-.2, 1.1, -.3], [.2, 1.1, -.3]
    neutral = {s: p.copy() for s, p in inputs.items()}
    rows, checks, metrics = [], {}, {}
    begun = time.monotonic()

    def step(a=False, b=False):
        for _ in range(50):
            executor.spin_once(timeout_sec=0.)
        body = np.tile(np.eye(4).flatten(order='F'), 7)
        body[6*16:7*16] = head.flatten(order='F')
        loop.run_until_complete(node.on_body_tracking_move(SimpleNamespace(value={'body': body}), None))
        state = {'aButton': a, 'bButton': b}
        loop.run_until_complete(node.on_controller_move(SimpleNamespace(value={
            'left': inputs['left'].flatten(order='F').tolist(),
            'right': inputs['right'].flatten(order='F').tolist(),
            'leftState': state, 'rightState': state}), None))
        for _ in range(50):
            executor.spin_once(timeout_sec=0.)
        row = {'t': time.monotonic()-begun, 'phase': node._phase,
               'active': node.arm_control_enabled, 'resume_count': node._resume_count,
               'published_pairs': node.published_pairs, 'reason': node._last_reason}
        if node._session is not None:
            row['signature'] = signature(node._session)
            row['raw_targets'] = {s: node._session.mapper.target(s, inputs[s]).tolist() for s in inputs}
        try:
            links = node._snapshot()
            row['actual'] = {s: links[s]['wrist'].tolist() for s in inputs}
            row['goals'] = {s: p.tolist() for s, p in node._goals.items()}
        except Exception:
            pass
        rows.append(row)
        time.sleep(.02)

    def wait(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            step()

    def press(a=False, b=False):
        step()
        step(a=a, b=b)
        step()

    def until_ready():
        limit = time.monotonic()+18
        while time.monotonic() < limit and node._phase != 'ACTIVE':
            step()
            if node._phase not in ('SETTLING', 'WARMUP', 'ACTIVE'):
                raise RuntimeError('Activation failed: '+str(node._last_reason))
        if node._phase != 'ACTIVE':
            raise RuntimeError('XR never became READY')

    def align_to_measured():
        for s in inputs:
            actual = node._snapshot()[s]['wrist']
            mapper = node._session.mapper
            anchor = mapper.anchors[s]
            basis = mapper.basis
            inputs[s][:3, 3] = anchor.input_pose[:3, 3] + basis.T @ (
                actual[:3, 3] - anchor.robot_pose[:3, 3]) / mapper.scale
            inputs[s][:3, :3] = (basis.T @ actual[:3, :3] @ anchor.robot_pose[:3, :3].T
                                 @ basis @ anchor.input_pose[:3, :3])

    error = None
    try:
        timeout = time.monotonic()+12
        while time.monotonic() < timeout:
            step()
            if node._joint_rx is not None and len(node._robot_poses) == 2:
                break
        wait(1)
        press(a=True)
        until_ready()
        sig = signature(node._session)
        print('[TEST] First calibration READY', flush=True)
        initial_goals = {s: p.copy() for s, p in node._goals.items()}
        head[:3, :3] = Rotation.from_euler('y', 40, degrees=True).as_matrix() @ head[:3, :3]
        wait(.5)
        checks['head_does_not_redefine_basis'] = signature(node._session) == sig
        start = time.monotonic()
        while time.monotonic()-start < 2:
            inputs['left'][2, 3] = neutral['left'][2, 3]-.04*min(1,(time.monotonic()-start)/1.5)
            step()
        wait(1)
        links = node._snapshot()
        metrics['extension_error_mm'] = 1000*float(np.linalg.norm(
            node._goals['left'][:3, 3]-links['left']['wrist'][:3, 3]))
        checks['initial_wrist_tracking'] = metrics['extension_error_mm'] < 20
        # Gradual synthetic range violation, not a discontinuous >15cm packet.
        start = time.monotonic()
        while node.arm_control_enabled and time.monotonic()-start < 1:
            inputs['left'][2, 3] = neutral['left'][2, 3]-(.04+.54*min(1,(time.monotonic()-start)/.4))
            step()
        checks['range_fault_stops'] = (not node.arm_control_enabled and '0.5m clutch' in node._last_reason)
        if not checks['range_fault_stops']:
            raise RuntimeError('Expected clutch range stop: '+str(node._last_reason))
        print('[TEST] 0.5m clutch stop reproduced', flush=True)
        wait(1)
        for s in inputs:
            inputs[s] = neutral[s].copy()
        wait(.7)
        before = node.published_pairs
        press(a=True)
        checks['wrong_hand_XA_refused'] = not node.arm_control_enabled and 'ALIGN_REQUIRED' in node._last_reason
        checks['refusal_publishes_no_new_goals'] = node.published_pairs == before
        checks['reference_not_reanchored_on_failure'] = signature(node._session) == sig
        print('[TEST] Misaligned X+A refused:', node._last_reason, flush=True)
        wait(.8)
        align_to_measured()
        wait(.7)
        press(a=True)
        until_ready()
        checks['aligned_resume_succeeds'] = node._phase == 'ACTIVE'
        checks['first_resume_reference_unchanged'] = signature(node._session) == sig
        print('[TEST] Aligned restart READY with original reference', flush=True)
        # A second ordinary stop/start, with a different HMD heading.
        press(b=True)
        wait(.8)
        head[:3, :3] = Rotation.from_euler('y', -70, degrees=True).as_matrix() @ head[:3, :3]
        align_to_measured()
        wait(.7)
        press(a=True)
        until_ready()
        checks['second_resume_reference_unchanged'] = signature(node._session) == sig
        same_hand_goals = {s: node._session.mapper.target(s, neutral[s]) for s in inputs}
        metrics['same_hand_reference_drift_m'] = max(float(np.linalg.norm(
            same_hand_goals[s][:3, 3]-initial_goals[s][:3, 3])) for s in inputs)
        checks['zero_additive_reference_drift'] = metrics['same_hand_reference_drift_m'] < 1e-12
        press(b=True)
        checks['YB_still_stops'] = not node.arm_control_enabled
    except Exception as exc:
        error = repr(exc)
    finally:
        node._set_arm_control_enabled(False, 'fixed-reference smoke cleanup')
        for _ in range(10):
            executor.spin_once(timeout_sec=0.)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        loop.close()
    passed = error is None and all(checks.values()) and len(checks) >= 10
    result = {'passed': passed, 'error': error, 'checks': checks, 'metrics': metrics,
              'synthetic_input': True, 'domain': 232, 'rows': rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print('[RESULT]', json.dumps({k:v for k,v in result.items() if k!='rows'}), flush=True)
    raise SystemExit(0 if passed else 2)


if __name__ == '__main__':
    main()
