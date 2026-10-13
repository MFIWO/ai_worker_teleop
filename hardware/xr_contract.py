"""[ADDED] SG2/SH5 deployment and command boundary checks, without ROS effects."""
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

SIDES = {'left': 'l', 'right': 'r'}
ARM_NAMES = {s: tuple(f'arm_{v}_joint{i}' for i in range(1, 8)) for s, v in SIDES.items()}
FOLLOWER_TOPICS = {s: f'/leader/joint_trajectory_command_broadcaster_{s}/joint_trajectory' for s in SIDES}
GATE_NAME = 'xr_hardware_gate'
PC_NAME = 'ai_worker_xr_hardware'
MAX_JOINT_SPEED = 0.35
GOAL_SPEED = 0.06
ANGULAR_SPEED = 0.35
LEASE_SECONDS = 0.5
ARM_DXL_IDS = {'right': list(range(1, 8)), 'left': list(range(31, 38))}


def configured_arm_ids(description_text, model):
    root = ET.fromstring(description_text)
    controls = root.findall(f".//ros2_control[@name='ffw_{model}_follower']")
    if len(controls) != 1:
        raise ValueError('expanded physical ros2_control block missing/ambiguous; use live robot_description')
    mapping = {}
    for gpio in controls[0].findall('gpio'):
        typ, identity = gpio.find("param[@name='type']"), gpio.find("param[@name='ID']")
        if typ is None or (typ.text or '').strip() != 'dxl' or identity is None:
            continue
        value = int((identity.text or '').strip())
        if value in mapping:
            raise ValueError('duplicate physical Dynamixel ID in description')
        mapping[value] = gpio.get('name')
    for ids in ARM_DXL_IDS.values():
        for value in ids:
            if mapping.get(value) != f'dxl{value}':
                raise ValueError(f'expected arm Dynamixel dxl{value} missing from expanded description')
    return {side: list(ids) for side, ids in ARM_DXL_IDS.items()}


def motor_read_health(ids, errors, comm_state, expected=ARM_DXL_IDS):
    """[ADDED] Check fresh motor bus IDs, error codes and communication without torque claims.

    A positive Torque Enable register read (TorqueEvidence) is additionally
    required before this can authorize a physical motion.
    """
    ids, errors = list(ids), list(errors)
    if (comm_state != 0 or len(ids) != len(set(ids)) or len(ids) != len(errors)):
        return False
    reported = dict(zip(ids, errors))
    return all(identity in reported and reported[identity] == 0
               for group in expected.values() for identity in group)


def motor_health(ids, torque, errors, comm_state, expected=ARM_DXL_IDS):
    ids, torque, errors = list(ids), list(torque), list(errors)
    if (comm_state != 0 or len(ids) != len(set(ids)) or len(ids) != len(torque)
            or len(ids) != len(errors)):
        return False
    indexed = {identity: (bool(t), e) for identity, t, e in zip(ids, torque, errors)}
    return all(identity in indexed and indexed[identity] == (True, 0)
               for values in expected.values() for identity in values)


def private_topic(model, side):
    if model not in ('sg2', 'sh5') or side not in SIDES:
        raise ValueError('expected sg2/sh5 and left/right')
    return f'/xr_hardware/{model}/{side}/joint_trajectory'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def arm_signature(root):
    """Kinematic structure only; no claim about the attached hand collision model."""
    result = {}
    for names in ARM_NAMES.values():
        for name in names:
            joint = root.find(f"joint[@name='{name}']")
            if joint is None or joint.get('type') != 'revolute':
                raise ValueError(f'missing revolute joint {name}')
            origin = joint.find('origin')
            axis = joint.find('axis')
            result[name] = {
                'parent': joint.find('parent').get('link'),
                'child': joint.find('child').get('link'),
                'xyz': [float(x) for x in origin.get('xyz', '0 0 0').split()],
                'rpy': [float(x) for x in origin.get('rpy', '0 0 0').split()],
                'axis': [float(x) for x in axis.get('xyz').split()],
            }
    return result


def verify_description(text, model, expected_signature):
    """Check follower identity and arm geometry, not just a requested model label."""
    root = ET.fromstring(text)
    if root.get('name') != f'ffw_{model}_follower':
        raise ValueError(f'wrong /robot_description: {root.get("name")}, expected ffw_{model}_follower')
    if arm_signature(root) != expected_signature:
        raise ValueError('robot_description arm geometry differs from selected controller model')
    joints = {j.get('name') for j in root.findall('joint')}
    required = ({f'finger_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 21)}
                if model == 'sh5' else {f'gripper_{s}_joint1' for s in ('l', 'r')})
    if not required.issubset(joints):
        raise ValueError(f'{model} end tooling is missing from robot_description')
    return True


def check_positions(side, names, positions, limits):
    """Select seven arm joints. A legacy gripper entry is stripped, never forwarded."""
    names, positions = list(names), list(positions)
    if len(names) != len(positions) or len(names) != len(set(names)):
        raise ValueError('duplicate joint names or position length mismatch')
    allowed = set(ARM_NAMES[side]) | {f'gripper_{SIDES[side]}_joint1'}
    if not set(ARM_NAMES[side]).issubset(names) or not set(names).issubset(allowed):
        raise ValueError('wrong-side, incomplete, or unintended joint trajectory')
    values = dict(zip(names, map(float, positions)))
    if not all(math.isfinite(v) for v in values.values()):
        raise ValueError('nonfinite joint target')
    for name in ARM_NAMES[side]:
        lower, upper = limits[name]
        if not lower <= values[name] <= upper:
            raise ValueError(f'{name}: target {values[name]:.6f} outside [{lower}, {upper}]')
    return {name: values[name] for name in ARM_NAMES[side]}


def check_step(target, measured, previous, elapsed, speed=MAX_JOINT_SPEED):
    """Reject, do not integrate or secretly clip, unexpected target leaps."""
    if not math.isfinite(elapsed) or elapsed <= 0:
        raise ValueError('non-positive command timing')
    step = speed * min(elapsed, .10) + 0.001
    for name, value in target.items():
        actual = measured.get(name)
        old = previous.get(name)
        if actual is None or old is None or not math.isfinite(actual) or not math.isfinite(old):
            raise ValueError('missing/nonfinite measured or held arm joint')
        if abs(value - actual) > .08:
            raise ValueError(f'{name}: commanded/measured error exceeds .08rad')
        if abs(value - old) > step:
            raise ValueError(f'{name}: output step exceeds commissioning speed bound')


def load_profile(path):
    profile = json.loads(Path(path).read_text())
    if (profile.get('schema') != 1 or profile.get('mode') != 'hardware_commissioning'
            or profile.get('model') not in ('sg2', 'sh5')):
        raise ValueError('invalid hardware manifest')
    if profile.get('validation_fixture') and os.environ.get('AI_WORKER_XR_EXECUTION') != 'validation':
        raise ValueError('synthetic validation profile cannot be used for physical hardware')
    for key in ('urdf', 'srdf', 'config'):
        if sha256(profile[key]) != profile[key + '_sha256']:
            raise ValueError(f'{key} changed after profile generation')
    if profile.get('max_joint_speed') != MAX_JOINT_SPEED or profile.get('goal_speed') != GOAL_SPEED:
        raise ValueError('unknown commissioning limits')
    if profile.get('arm_dxl_ids') != ARM_DXL_IDS:
        raise ValueError('profile not bound to physical arm motor IDs')
    for short in ('l', 'r'):
        if profile['limits'][f'arm_{short}_joint4'][1] > -.7 + 1e-12:
            raise ValueError('simulation elbow expansion must not enter hardware profile')
    return profile
