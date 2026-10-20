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
# Arm position and speed limits come from the verified URDF, per joint.
# Cartesian input filtering remains separate from actuator joint limits.
GOAL_SPEED = 0.30
ANGULAR_SPEED = 1.50
LEASE_SECONDS = 0.5
# Existing maximum relay command interval. URDF has no following-error field;
# the tracking budget is the distance each joint can travel in this interval.
COMMAND_WINDOW_SECONDS = 0.10
# Match the selected QP core's 1e-5 numerical tolerance. This is a snap-to-
# boundary allowance, not an expansion of the physical command interval.
POSITION_EPSILON_RAD = 1e-5
ARM_DXL_IDS = {'right': list(range(1, 8)), 'left': list(range(31, 38))}
# [CHANGED 2026-10-14] Normal operation delegates torque management to the
# existing follower driver. An independent torque-bit check is opt-in, not a
# prerequisite for using a driver whose torque telemetry is unsupported.
READINESS_POLICIES = ('driver_managed', 'strict_torque')


def validate_readiness_policy(policy):
    if policy not in READINESS_POLICIES:
        raise ValueError('unknown motor readiness policy: ' + repr(policy))
    return policy


def motor_readiness(ids, torque, errors, comm_state, policy, expected=ARM_DXL_IDS):
    """[CHANGED] Separate valid motor reads from reported torque status.

    driver_managed never asserts that torque is ON. The native driver owns
    enable/disable and hardware faults; joint tracking and watchdogs remain
    mandatory in callers. strict_torque preserves the older extra predicate.
    Neither mode polls registers, writes torque, or fabricates telemetry.
    """
    validate_readiness_policy(policy)
    ids, torque, errors = list(ids), list(torque), list(errors)
    if len(torque) != len(ids) or not motor_read_health(ids, errors, comm_state, expected):
        return False
    return policy == 'driver_managed' or motor_health(ids, torque, errors, comm_state, expected)


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

    [CHANGED] This is motor READ health, not proof of physical Torque Enable.
    The selected readiness policy decides whether reported torque is required.
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
        value = values[name]
        if not lower - POSITION_EPSILON_RAD <= value <= upper + POSITION_EPSILON_RAD:
            violation = max(lower - value, value - upper)
            raise ValueError(f'{name}: target {value:.17g} outside [{lower}, {upper}] '
                             f'(violation={violation:.17g}rad, numerical_tolerance={POSITION_EPSILON_RAD:g}rad)')
        values[name] = min(upper, max(lower, value))
    return {name: values[name] for name in ARM_NAMES[side]}


def arm_limits(text):
    """Read finite position and positive velocity limits from the selected URDF."""
    root = ET.fromstring(text)
    positions, velocities = {}, {}
    for names in ARM_NAMES.values():
        for name in names:
            joints = root.findall(f"joint[@name='{name}']")
            if len(joints) != 1 or joints[0].get('type') != 'revolute':
                raise ValueError(f'{name}: missing/ambiguous revolute URDF joint')
            bound = joints[0].find('limit')
            if bound is None:
                raise ValueError(f'{name}: URDF limit missing')
            lower, upper, velocity = (float(bound.get(k)) for k in ('lower', 'upper', 'velocity'))
            if not all(math.isfinite(v) for v in (lower, upper, velocity)) or not lower < upper or velocity <= 0:
                raise ValueError(f'{name}: invalid URDF limits')
            positions[name], velocities[name] = [lower, upper], velocity
    return positions, velocities


def step_bounds(name, value, actual, old, elapsed, velocities, enforce_tracking=True):
    speed = velocities[name]
    if not all(v is not None and math.isfinite(v) for v in (value, actual, old, speed)) or speed <= 0:
        raise ValueError(f'{name}: missing/nonfinite command, feedback or URDF speed')
    tracking_bound = speed * COMMAND_WINDOW_SECONDS
    if enforce_tracking and abs(value - actual) > tracking_bound:
        raise ValueError(f'{name}: commanded/measured error exceeds URDF velocity tracking budget '
                         f'{tracking_bound:.6f}rad (velocity={speed:g}rad/s, '
                         f'window={COMMAND_WINDOW_SECONDS:g}s, target={value:+.6f}, '
                         f'measured={actual:+.6f}, error={value-actual:+.6f}, previous={old:+.6f})')
    return speed * min(elapsed, COMMAND_WINDOW_SECONDS)


def check_step(target, measured, previous, elapsed, velocities):
    """Reject steps exceeding each joint's URDF velocity and tracking budget."""
    if not math.isfinite(elapsed) or elapsed <= 0:
        raise ValueError('non-positive command timing')
    for name, value in target.items():
        actual = measured.get(name)
        old = previous.get(name)
        step = step_bounds(name, value, actual, old, elapsed, velocities)
        if abs(value - old) > step:
            raise ValueError(f'{name}: output step exceeds URDF speed bound ({velocities[name]:g}rad/s)')


def limit_step(target, measured, previous, elapsed, velocities, enforce_tracking=True):
    """Slew-limit each joint at URDF speed, retaining its derived tracking guard."""
    if not math.isfinite(elapsed) or elapsed <= 0:
        raise ValueError('non-positive command timing')
    result = {}
    for name, value in target.items():
        actual = measured.get(name)
        old = previous.get(name)
        step = step_bounds(name, value, actual, old, elapsed, velocities, enforce_tracking)
        delta = value - old
        if abs(delta) > step:
            value = old + math.copysign(step, delta)
        result[name] = value
    return result


def load_profile(path):
    profile = json.loads(Path(path).read_text())
    if (profile.get('schema') != 1 or profile.get('mode') != 'hardware_commissioning'
            or profile.get('model') not in ('sg2', 'sh5')):
        raise ValueError('invalid hardware manifest')
    # [CHANGED] Old manifests retain their strict meaning; no implicit upgrade.
    validate_readiness_policy(profile.get('readiness_policy', 'strict_torque'))
    if profile.get('validation_fixture') and os.environ.get('AI_WORKER_XR_EXECUTION') != 'validation':
        raise ValueError('synthetic validation profile cannot be used for physical hardware')
    for key in ('urdf', 'srdf', 'config'):
        if sha256(profile[key]) != profile[key + '_sha256']:
            raise ValueError(f'{key} changed after profile generation')
    positions, velocities = arm_limits(Path(profile['urdf']).read_text())
    if profile['limits'] != positions:
        raise ValueError('profile position limits differ from verified URDF')
    if 'joint_velocity_limits' in profile and profile['joint_velocity_limits'] != velocities:
        raise ValueError('profile velocity limits differ from verified URDF')
    # Older manifests obtain per-joint limits from their already hashed URDF.
    profile['joint_velocity_limits'] = velocities
    profile['max_joint_speed'] = max(velocities.values())
    profile['tracking_window_seconds'] = COMMAND_WINDOW_SECONDS
    profile['tracking_error_limits'] = {k: v * COMMAND_WINDOW_SECONDS for k, v in velocities.items()}
    profile['tracking_policy'] = ('urdf_bounds_and_slew' if profile.get('readiness_policy') == 'driver_managed'
                                  else 'strict_following_error')
    profile['time_policy'] = 'local_monotonic_receipt'
    if profile.get('goal_speed') != GOAL_SPEED:
        raise ValueError('unknown Cartesian goal speed')
    if profile['model'] == 'sg2':
        from hardware.aux_controls import read_aux_limits
        # Controller meshes/models may simplify head and tooling. Auxiliary
        # limits are taken from the hashed physical description instead.
        description = Path(profile['urdf']).parent / 'snapshot/robot_description.urdf'
        profile['auxiliary_limits'] = {}
        if description.is_file():
            if sha256(description) != profile['description_sha256']:
                raise ValueError('physical auxiliary description hash mismatch')
            profile['auxiliary_limits'] = read_aux_limits(description.read_text(), sg2_legacy_gripper=True)
            profile['gripper_command_policy'] = 'original_sg2_vr_1.3rad'
    else:
        profile['auxiliary_limits'] = {}
    if profile.get('arm_dxl_ids') != ARM_DXL_IDS:
        raise ValueError('profile not bound to physical arm motor IDs')
    # SH5 still uses its separately-reviewed reduced model. SG2, however, is
    # explicitly bound to the live physical URDF limits in make_profile.py.
    if profile['model'] == 'sh5':
        for short in ('l', 'r'):
            if profile['limits'][f'arm_{short}_joint4'][1] > -.7 + 1e-12:
                raise ValueError('unexpected SH5 arm limit expansion')
    return profile
