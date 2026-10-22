"""Add measured finger kinematics to the existing SH5 arm model."""
from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from hardware.sh5_model import RETAINED_JOINTS, _model, _collision_srdf

HAND_CONTROL = 'robotis_native_measured_fingers'
FINGER_NAMES = {side: tuple(f'finger_{short}_joint{i}' for i in range(1, 21))
                for side, short in (('left', 'l'), ('right', 'r'))}
FINGERS = frozenset().union(*FINGER_NAMES.values())
CONTROLLER_JOINTS = frozenset(RETAINED_JOINTS) | FINGERS
HEAD_JOINTS = frozenset(('head_joint1', 'head_joint2'))


def add_measured_fingers(reduced_urdf, description, source_srdf, frozen, *, measured_head=False):
    """Restore only the 40 original finger joints; leave arm joints untouched.

    The matching native executable fixes their QP velocities at zero and
    refreshes their positions from measured feedback each cycle. Hand output
    is produced separately by the original ROBOTIS retargeter.
    """
    original = ET.fromstring(description)
    tree = ET.fromstring(reduced_urdf)
    restored = FINGERS | HEAD_JOINTS if measured_head else FINGERS
    for name in sorted(restored):
        old = tree.find(f"joint[@name='{name}']")
        new = original.find(f"joint[@name='{name}']")
        if old is None or new is None or old.get('type') != 'fixed' or new.get('type') != 'revolute':
            raise ValueError(f'{name}: expected original revolute and reduced fixed joint')
        tree.remove(old)
        tree.append(deepcopy(new))
    text = ET.tostring(tree, encoding='unicode')
    root, links, joints = _model(text, 'SH5 measured fingers')
    # Rebuild exclusions from the original SRDF, not the frozen-hand SRDF.
    # In particular, fingers are no longer one fixed cluster with the wrist.
    srdf, report = _collision_srdf(root, links, joints, source_srdf)
    return text, ET.tostring(srdf, encoding='unicode'), {
        name: value for name, value in frozen.items() if name not in restored}, report


def finger_limits(text):
    root = ET.fromstring(text)
    result = {}
    for name in sorted(FINGERS):
        joint = root.find(f"joint[@name='{name}']")
        if joint is None or joint.get('type') != 'revolute':
            raise ValueError(f'{name}: measured finger missing from controller model')
        bound = joint.find('limit')
        if bound is None:
            raise ValueError(f'{name}: missing finger limits')
        values = {k: float(bound.get(k)) for k in ('lower', 'upper', 'velocity')}
        if (not all(map(math.isfinite, values.values())) or
                values['lower'] >= values['upper'] or values['velocity'] <= 0):
            raise ValueError(f'{name}: invalid finger limits')
        result[name] = values
    return result


def verify_hand_profile(profile):
    if profile.get('model') != 'sh5' or profile.get('hand_control') != HAND_CONTROL:
        raise ValueError('Expected SH5 measured-finger profile')
    tree = ET.parse(profile['urdf']).getroot()
    movable = {j.get('name') for j in tree.findall('joint') if j.get('type') != 'fixed'}
    expected = CONTROLLER_JOINTS | HEAD_JOINTS if profile.get('manual_auxiliary') else CONTROLLER_JOINTS
    if movable != expected or set(profile['controller_joint_names']) != movable:
        raise ValueError('Unexpected joints in SH5 measured-finger/head model')
    moving = FINGERS | (HEAD_JOINTS if profile.get('manual_auxiliary') else frozenset())
    if moving.intersection(profile.get('frozen_joints', {})):
        raise ValueError('Measured fingers/head cannot use frozen-posture collision geometry')
    limits = finger_limits(Path(profile['urdf']).read_text())
    if profile.get('finger_limits') != limits:
        raise ValueError('Finger bounds differ from hashed controller URDF')
    return limits
