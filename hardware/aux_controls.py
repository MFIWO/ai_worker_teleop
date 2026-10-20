"""SG2 auxiliary command validation; gate remains the only follower writer."""
import math
import xml.etree.ElementTree as ET

JOINT_GROUPS = {'head': ('head_joint1', 'head_joint2'), 'lift': ('lift_joint',)}
GRIPPERS = {'left': 'gripper_l_joint1', 'right': 'gripper_r_joint1'}
AUX_TOPICS = {'head': '/leader/joystick_controller_left/joint_trajectory',
              'lift': '/leader/joystick_controller_right/joint_trajectory'}
# Original SG2 stick scales: 1/5 m/s, 1/3 rad/s.
BASE_LIMITS = (.2, .2, 1 / 3)
SG2_LEGACY_GRIPPER_MAX_POSITION = 1.3


def read_aux_limits(text, *, sg2_legacy_gripper=False):
    """Read URDF limits; SG2 may explicitly restore the original VR command range."""
    root = ET.fromstring(text)
    result = {}
    for name in (*JOINT_GROUPS['head'], *JOINT_GROUPS['lift'], *GRIPPERS.values()):
        joints = root.findall(f"joint[@name='{name}']")
        if len(joints) != 1 or joints[0].find('limit') is None:
            raise ValueError(f'{name}: auxiliary physical URDF limit missing')
        limit = joints[0].find('limit')
        values = {k: float(limit.get(k)) for k in ('lower', 'upper', 'velocity')}
        if (not all(math.isfinite(v) for v in values.values()) or
                values['lower'] >= values['upper'] or values['velocity'] <= 0):
            raise ValueError(f'{name}: invalid auxiliary URDF limit')
        if sg2_legacy_gripper and name in GRIPPERS.values():
            # The original SG2 VR path uses trigger * 1.3. Keep the URDF
            # value visible while restoring only this gripper command scale.
            values['urdf_upper'] = values['upper']
            values['upper'] = SG2_LEGACY_GRIPPER_MAX_POSITION
        result[name] = values
    return result


def validate_aux(payload, limits):
    if not isinstance(payload, dict) or set(payload) - {'joints', 'base'}:
        raise ValueError('invalid auxiliary payload')
    joints = payload.get('joints', {})
    base = payload.get('base', [0., 0., 0.])
    if (not isinstance(joints, dict) or set(joints) - set(limits) or
            not isinstance(base, (list, tuple)) or len(base) != 3):
        raise ValueError('unknown auxiliary joint or invalid base command')
    cleaned = {}
    for name, value in joints.items():
        value = float(value)
        bound = limits[name]
        if not math.isfinite(value) or not bound['lower'] <= value <= bound['upper']:
            raise ValueError(f'{name}: auxiliary command outside configured range')
        cleaned[name] = value
    base = [float(v) for v in base]
    if any(not math.isfinite(v) or abs(v) > b + 1e-9 for v, b in zip(base, BASE_LIMITS)):
        raise ValueError('base command exceeds original SG2 stick scale')
    return {'joints': cleaned, 'base': base}


def slew_aux(targets, previous, measured, limits, elapsed):
    result = {}
    for name, target in targets.items():
        old = previous.get(name, measured.get(name))
        if old is None or not math.isfinite(old):
            raise ValueError(f'{name}: no fresh measured auxiliary joint')
        step = limits[name]['velocity'] * min(max(elapsed, .001), .1)
        bound = limits[name]
        result[name] = min(bound['upper'], max(bound['lower'],
                          old + min(step, max(-step, target - old))))
    return result
