"""[ADDED] Synthetic XR -> real Cyclo -> PhysX shoulder participation test.

Only simulation domain 232 is permitted. A small SG2 URDF FK fixture generates
reachable wrist references; it NEVER publishes joint angles or runs another IK.
The unchanged Cyclo QP chooses all fourteen arm joints. Old/new profiles receive
the same pitch/roll/yaw fixture. Required: sim, VR controller arm:=false.
"""
import argparse
import asyncio
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from scipy.spatial.transform import Rotation as R
from rcl_interfaces.srv import GetParameters

from robotis_vuer.vr_publisher_sg2_xr import XRDeltaPublisher
from robotis_vuer.xr_arm_targets import level_basis_from_body_head


class FixtureFK:
    """Test-only FK from the exact installed model, not an alternate controller."""
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.joints = {x.get('name'): x for x in root.findall('joint')}

    def arm(self, side, q):
        result = np.eye(4)
        for number, angle in enumerate(q, 1):
            joint = self.joints[f'arm_{side}_joint{number}']
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            lower = float(joint.find('limit').get('lower'))
            upper = float(joint.find('limit').get('upper'))
            if not lower <= angle <= upper:
                raise ValueError(f'Fixture violates actual joint limits: {side}/{number}')
            transform = np.eye(4)
            transform[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
            transform[:3, 3] = xyz
            result = result @ transform
            spin = np.eye(4)
            spin[:3, :3] = R.from_rotvec(angle * axis).as_matrix()
            result = result @ spin
        return result


def smooth_fraction(elapsed, start, duration=6.):
    u = np.clip((elapsed - start) / duration, 0., 1.)
    return float(u * u * (3. - 2. * u))


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '232':
        raise SystemExit('Requires isolated test domain 232; no real robot.')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--elbow-weight', type=float, required=True)
    args = parser.parse_args()
    fk = FixtureFK(args.urdf)
    rclpy.init()
    node = XRDeltaPublisher(start_server=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    client = node.create_client(GetParameters, '/vr_controller/get_parameters')
    if not client.wait_for_service(timeout_sec=5.):
        raise RuntimeError('Missing real Cyclo parameter service')
    request = GetParameters.Request(names=['weight_elbow_position'])
    future = client.call_async(request)
    executor.spin_until_future_complete(future, timeout_sec=5.)
    if not future.done() or future.result() is None:
        raise RuntimeError('Cannot read actual Cyclo elbow weight')
    actual_weight = future.result().values[0].double_value
    if abs(actual_weight - args.elbow_weight) > 1e-9:
        raise RuntimeError(f'Wrong controller profile {actual_weight} != {args.elbow_weight}')
    print('[PROFILE] actual Cyclo elbow weight=', actual_weight, flush=True)

    loop = asyncio.new_event_loop()
    head = np.eye(4)
    head[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    head[:3, 3] = [0., 1.6, 0.]
    basis = level_basis_from_body_head(head)
    inputs = {s: np.eye(4) for s in ('left', 'right')}
    inputs['left'][:3, 3] = [-.2, 1.1, -.3]
    inputs['right'][:3, 3] = [.2, 1.1, -.3]
    q0 = np.array([0., 0., 0., -1.57, 0., 0., 0.])
    references = {s: fk.arm(s[0], q0) for s in inputs}
    rows, failures = [], []
    started = None
    wall = time.monotonic()
    last_status = -1
    try:
        while time.monotonic() - wall < 70:
            for _ in range(60):
                executor.spin_once(timeout_sec=0.)
            now = time.monotonic()
            if started is None:
                try:
                    node._snapshot()
                    if node._joint_rx is not None and len(node._robot_poses) == 2:
                        started = now
                except Exception:
                    pass
                if now - wall > 15:
                    raise RuntimeError('No fresh controller/PhysX feedback')
            elapsed = 0. if started is None else now - started
            if elapsed > 49:
                break
            poses = {}
            for side, short in (('left', 'l'), ('right', 'r')):
                sign = 1 if side == 'left' else -1
                q = q0.copy()
                q[0] = -.40 * smooth_fraction(elapsed, 14.)
                q[1] = sign * .35 * smooth_fraction(elapsed, 24.)
                q[2] = sign * .60 * smooth_fraction(elapsed, 34.)
                fixture = fk.arm(short, q)
                base = references[side]
                pose = inputs[side].copy()
                pose[:3, 3] += basis.T @ (fixture[:3, 3] - base[:3, 3])
                pose[:3, :3] = basis.T @ (fixture[:3, :3] @ base[:3, :3].T) @ basis
                poses[side] = pose
            states = {'aButton': bool(2. < elapsed < 2.2),
                      'bButton': bool(46. < elapsed < 46.2)}
            body = np.tile(np.eye(4).flatten(order='F'), 7)
            body[96:112] = head.flatten(order='F')
            loop.run_until_complete(node.on_body_tracking_move(SimpleNamespace(value={'body': body}), None))
            loop.run_until_complete(node.on_controller_move(SimpleNamespace(value={
                'left': poses['left'].flatten(order='F').tolist(),
                'right': poses['right'].flatten(order='F').tolist(),
                'leftState': states, 'rightState': states}), None))
            for _ in range(60):
                executor.spin_once(timeout_sec=0.)
            if node._goals:
                links = node._snapshot()
                rows.append({'t': elapsed, 'active': node.arm_control_enabled,
                             'joints': dict(zip(node.current_joint_states.name, node.current_joint_states.position)),
                             'goal_l': node._goals['left'].tolist(),
                             'actual_l': links['left']['wrist'].tolist(),
                             'goal_r': node._goals['right'].tolist(),
                             'actual_r': links['right']['wrist'].tolist(),
                             'elbow_l': links['left']['elbow'].tolist(),
                             'elbow_r': links['right']['elbow'].tolist()})
            status = int(elapsed // 5)
            if status != last_status:
                last_status = status
                print(f'[TEST] t={elapsed:.1f} active={node.arm_control_enabled}', flush=True)
            time.sleep(.02)
    except Exception as error:
        failures.append(repr(error))
    finally:
        node._set_arm_control_enabled(False, 'shoulder synthetic test finished')
        for _ in range(10):
            executor.spin_once(timeout_sec=0.)
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        loop.close()
    metrics = {}
    try:
        baseline = [x for x in rows if 13 < x['t'] < 13.8][-1]
        active_rows = [x for x in rows if 14 < x['t'] < 45.5]
        for side in ('l', 'r'):
            for number, axis in ((1, 'pitch'), (2, 'roll'), (3, 'yaw')):
                key = f'arm_{side}_joint{number}'
                values = [baseline['joints'][key]] + [x['joints'][key] for x in active_rows]
                metrics[f'{side}_{axis}_range_deg'] = float(np.rad2deg(max(values) - min(values)))
            end = [x for x in rows if 44 < x['t'] < 45.5][-1]
            actual, goal = np.array(end['actual_'+side]), np.array(end['goal_'+side])
            metrics[f'{side}_wrist_error_m'] = float(np.linalg.norm(actual[:3, 3]-goal[:3, 3]))
            metrics[f'{side}_wrist_error_deg'] = float(np.rad2deg(R.from_matrix(goal[:3, :3] @ actual[:3, :3].T).magnitude()))
        if not active_rows or not all(x['active'] for x in active_rows):
            failures.append('unexpected inactive state during shoulder sweep')
        if args.elbow_weight == 0.:
            for side in ('l','r'):
                for axis in ('pitch','roll','yaw'):
                    if metrics[f'{side}_{axis}_range_deg'] < 3.:
                        failures.append(f'{side}_{axis} moved less than 3 degrees')
                if metrics[f'{side}_wrist_error_m'] > .03 or metrics[f'{side}_wrist_error_deg'] > 15.:
                    failures.append(f'{side} wrist tracking poor')
    except Exception as error:
        failures.append('insufficient data: '+repr(error))
    result = {'passed': not failures, 'failures': failures, 'metrics': metrics,
              'synthetic_input': True, 'domain': 232, 'elbow_weight': actual_weight,
              'rows': rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print('[RESULT]', json.dumps({k:v for k,v in result.items() if k!='rows'}), flush=True)
    raise SystemExit(0 if result['passed'] else 2)


if __name__ == '__main__':
    main()
