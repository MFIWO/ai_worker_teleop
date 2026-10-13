"""[ADDED] Build an SH5 control model at a measured, monitored passive posture.

All SH5 links, meshes, collision elements and inertials are retained. Only the
14 arm joints and lift remain movable in the model. The caller must monitor the
returned frozen joint positions while using it, resolve its mesh resources, and
keep Cyclo's lift velocity bound at zero. This module does not access ROS or disk.
"""
from collections import defaultdict
import hashlib
from itertools import combinations
import math
from typing import Mapping
import xml.etree.ElementTree as ET


ARM_NAMES = tuple(f'arm_{side}_joint{i}' for side in ('l', 'r') for i in range(1, 8))
RETAINED_JOINTS = ARM_NAMES + ('lift_joint',)
MAX_ARM_SPEED = 0.35
_IDENTITY = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))


def _number(value, label):
    if isinstance(value, bool):
        raise ValueError(f'{label}: expected a finite number, not bool')
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f'{label}: expected a finite number') from error
    if not math.isfinite(result):
        raise ValueError(f'{label}: expected a finite number')
    return result


def _vector(value, label):
    parts = value.split()
    if len(parts) != 3:
        raise ValueError(f'{label}: expected three coordinates')
    return tuple(_number(x, label) for x in parts)


def _product(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))


def _rpy_rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # URDF fixed-axis RPY: Rz(yaw) Ry(pitch) Rx(roll).
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr))


def _origin(joint):
    element = joint.find('origin')
    if element is None:
        return (0., 0., 0.), _IDENTITY
    xyz = _vector(element.get('xyz', '0 0 0'), joint.get('name') + ' origin xyz')
    rpy = _vector(element.get('rpy', '0 0 0'), joint.get('name') + ' origin rpy')
    return xyz, _rpy_rotation(rpy)


def _axis(joint):
    element = joint.find('axis')
    values = _vector(element.get('xyz', '1 0 0') if element is not None else '1 0 0',
                     joint.get('name') + ' axis')
    norm = math.hypot(*values)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError(joint.get('name') + ': invalid joint axis')
    return tuple(x / norm for x in values)


def _axis_rotation(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1. - c
    return ((c+x*x*t, x*y*t-z*s, x*z*t+y*s),
            (y*x*t+z*s, c+y*y*t, y*z*t-x*s),
            (z*x*t-y*s, z*y*t+x*s, c+z*z*t))


def _rotation_rpy(rotation):
    cp = math.hypot(rotation[0][0], rotation[1][0])
    pitch = math.atan2(-rotation[2][0], cp)
    if cp > 1e-12:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        # At either gimbal pole, choose yaw=0 and preserve the complete matrix.
        roll = math.atan2(-rotation[1][2], rotation[1][1])
        yaw = 0.
    return roll, pitch, yaw


def _text_vector(values):
    return ' '.join(format(_number(v, 'composed joint origin'), '.17g') for v in values)


def _model(text, label):
    if not isinstance(text, str):
        raise ValueError(f'{label}: expanded XML text is required')
    if '<!DOCTYPE' in text or '<!ENTITY' in text:
        raise ValueError(f'{label}: XML entity declarations are not supported')
    try:
        root = ET.fromstring(text)
    except ET.ParseError as error:
        raise ValueError(f'{label}: invalid XML: {error}') from error
    if root.tag != 'robot':
        raise ValueError(f'{label}: expected a robot root')
    if any('xacro' in e.tag for e in root.iter()):
        raise ValueError(f'{label}: expand xacro before building the model')
    links, joints = {}, {}
    for tag, index in (('link', links), ('joint', joints)):
        for element in root.findall(tag):
            name = element.get('name')
            if not name or name in index:
                raise ValueError(f'{label}: duplicate or missing {tag} name: {name}')
            index[name] = element
    if not links or not joints:
        raise ValueError(f'{label}: robot links and joints are required')
    parent_of, children = {}, defaultdict(list)
    for name, joint in joints.items():
        parent, child = joint.find('parent'), joint.find('child')
        if parent is None or child is None:
            raise ValueError(f'{label}: {name} has no parent/child')
        p, c = parent.get('link'), child.get('link')
        if p not in links or c not in links or p == c or c in parent_of:
            raise ValueError(f'{label}: invalid tree connection at {name}')
        parent_of[c] = name
        children[p].append(c)
        if joint.get('type') not in ('fixed', 'revolute', 'continuous', 'prismatic'):
            raise ValueError(f'{label}: unsupported joint type at {name}')
        _origin(joint)
        if joint.get('type') != 'fixed':
            _axis(joint)
    roots = set(links) - set(parent_of)
    if len(roots) != 1:
        raise ValueError(f'{label}: expected one connected kinematic root')
    visited, pending = set(), list(roots)
    while pending:
        link = pending.pop()
        if link in visited:
            raise ValueError(f'{label}: cyclic kinematic graph')
        visited.add(link)
        pending.extend(children[link])
    if visited != set(links):
        raise ValueError(f'{label}: disconnected/cyclic kinematic graph')
    return root, links, joints


def _same_joint_geometry(actual, reference, name, expected_type):
    if actual.get('type') != expected_type or reference.get('type') != expected_type:
        raise ValueError(f'arm-chain mismatch: {name} must be {expected_type}')
    if actual.find('mimic') is not None or reference.find('mimic') is not None:
        raise ValueError(f'arm-chain mismatch: retained {name} cannot mimic another joint')
    for tag in ('parent', 'child'):
        if actual.find(tag).get('link') != reference.find(tag).get('link'):
            raise ValueError(f'arm-chain mismatch: {name} {tag}')
    ax, ar = _origin(actual)
    bx, br = _origin(reference)
    left = ax + tuple(x for row in ar for x in row) + _axis(actual)
    right = bx + tuple(x for row in br for x in row) + _axis(reference)
    if any(abs(a-b) > 1e-9 for a, b in zip(left, right)):
        raise ValueError(f'arm-chain mismatch: {name} origin/axis')


def _limits(joint):
    element = joint.find('limit')
    if element is None:
        raise ValueError(joint.get('name') + ': missing joint limits')
    result = tuple(_number(element.get(k), joint.get('name') + ' ' + k)
                   for k in ('lower', 'upper', 'velocity'))
    if result[0] >= result[1] or result[2] <= 0.:
        raise ValueError(joint.get('name') + ': invalid position/velocity limits')
    return result


def _freeze_joint(joint, position):
    xyz, rotation = _origin(joint)
    axis = _axis(joint)
    if joint.get('type') in ('revolute', 'continuous'):
        rotation = _product(rotation, _axis_rotation(axis, position))
    else:  # The input graph validator permits only 1-DOF joint types here.
        translation = tuple(sum(rotation[i][k] * axis[k] for k in range(3))
                            for i in range(3))
        xyz = tuple(xyz[i] + position * translation[i] for i in range(3))
    origin = joint.find('origin')
    if origin is None:
        origin = ET.SubElement(joint, 'origin')
    # [CHANGED] T_fixed = T_original_origin * T_motion(q_measured).
    origin.set('xyz', _text_vector(xyz))
    origin.set('rpy', _text_vector(_rotation_rpy(rotation)))
    joint.set('type', 'fixed')
    for tag in ('axis', 'limit', 'mimic', 'dynamics', 'safety_controller', 'calibration'):
        for element in list(joint.findall(tag)):
            joint.remove(element)


def _collision_srdf(description, links, joints, source_text):
    if not isinstance(source_text, str) or '<!DOCTYPE' in source_text or '<!ENTITY' in source_text:
        raise ValueError('source SRDF must be plain XML text without entity declarations')
    try:
        original = ET.fromstring(source_text)
    except ET.ParseError as error:
        raise ValueError(f'invalid source SRDF: {error}') from error
    if original.tag != 'robot':
        raise ValueError('source SRDF must have a robot root')
    result = ET.Element('robot', {'name': description.get('name')})
    pairs, omitted = {}, []
    kept = 0
    for element in original.findall('disable_collisions'):
        a, b = element.get('link1'), element.get('link2')
        if not a or not b or a == b:
            raise ValueError('invalid original SRDF collision pair')
        missing = sorted({a, b} - set(links))
        if missing:
            omitted.append({'link1': a, 'link2': b, 'reason': element.get('reason', ''),
                            'missing_links': missing})
            continue
        key = tuple(sorted((a, b)))
        if key not in pairs:
            pairs[key] = element.get('reason', 'Original')
            kept += 1
    # [ADDED] Only original parent-child adjacencies and rigidly connected
    # clusters supplement the compatible input exclusions. No hand/arm sweep.
    adjacent_added = 0
    for joint in joints.values():
        pair = tuple(sorted((joint.find('parent').get('link'), joint.find('child').get('link'))))
        if pair not in pairs:
            pairs[pair] = 'Adjacent'
            adjacent_added += 1
    representatives = {name: name for name in links}

    def representative(name):
        while representatives[name] != name:
            representatives[name] = representatives[representatives[name]]
            name = representatives[name]
        return name

    for joint in joints.values():
        if joint.get('type') == 'fixed':
            a = representative(joint.find('parent').get('link'))
            b = representative(joint.find('child').get('link'))
            representatives[b] = a
    clusters = defaultdict(list)
    for link in links:
        clusters[representative(link)].append(link)
    fixed_added = 0
    for cluster in clusters.values():
        for pair in combinations(sorted(cluster), 2):
            if pair not in pairs:
                pairs[pair] = 'FixedCluster'
                fixed_added += 1
    for (a, b), reason in sorted(pairs.items()):
        ET.SubElement(result, 'disable_collisions', {'link1': a, 'link2': b, 'reason': reason})
    return result, {'kept_original_disabled_pair_count': kept,
                    'omitted_original_disabled_pairs': omitted,
                    'added_original_adjacent_pair_count': adjacent_added,
                    'added_fixed_cluster_pair_count': fixed_added,
                    'disabled_pair_count': len(pairs),
                    'srdf_semantic_tags_not_copied': sorted({e.tag for e in original
                                                            if e.tag != 'disable_collisions'})}


def build_sh5_model(description_text: str, control_text: str, srdf_text: str,
                    snapshot: Mapping[str, float]):
    """Return ``(urdf_xml, srdf_xml, frozen_positions, report)`` without side effects.

    ``description_text`` must be the actual expanded SH5 description. ``snapshot``
    supplies a finite scalar position for EVERY movable description joint,
    including wheels/head/fingers. It must come from a coherent measured sample;
    timestamps and hardware provenance are the caller's responsibility. The
    returned frozen positions are a required runtime gate contract, not guesses.
    """
    description, links, joints = _model(description_text, 'SH5 description')
    _, _, control_joints = _model(control_text, 'Cyclo control model')
    if description.get('name') != 'ffw_sh5_follower':
        raise ValueError('expected the expanded ffw_sh5_follower description')
    expected_fingers = {f'finger_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 21)}
    if not expected_fingers.issubset(joints):
        raise ValueError('SH5 description must include all 40 finger joints')
    if any(joints[name].get('type') != 'revolute' for name in expected_fingers):
        raise ValueError('SH5 fingers must be measured revolute joints before reduction')
    for name in RETAINED_JOINTS:
        if name not in joints or name not in control_joints:
            raise ValueError(f'arm-chain mismatch: missing {name}')
        _same_joint_geometry(joints[name], control_joints[name], name,
                             'prismatic' if name == 'lift_joint' else 'revolute')
    # Validate explicit expected parents, not just two identically wrong inputs.
    for side in ('l', 'r'):
        for i in range(1, 8):
            joint = joints[f'arm_{side}_joint{i}']
            expected_parent = 'arm_base_link' if i == 1 else f'arm_{side}_link{i-1}'
            if (joint.find('parent').get('link') != expected_parent or
                    joint.find('child').get('link') != f'arm_{side}_link{i}'):
                raise ValueError('arm-chain mismatch: unexpected SH5 arm topology')
    if (joints['lift_joint'].find('parent').get('link') != 'base_link' or
            joints['lift_joint'].find('child').get('link') != 'arm_base_link'):
        raise ValueError('arm-chain mismatch: unexpected SH5 lift topology')
    if not isinstance(snapshot, Mapping):
        raise ValueError('a measured joint-position snapshot mapping is required')
    movable = {name for name, joint in joints.items() if joint.get('type') != 'fixed'}
    missing = sorted(movable - set(snapshot))
    if missing:
        raise ValueError('missing measured snapshot joints: ' + ', '.join(missing))
    measured = {name: _number(snapshot[name], 'snapshot ' + name) for name in sorted(movable)}
    arm_limits, velocities = {}, {}
    for name in ARM_NAMES:
        actual, reference = _limits(joints[name]), _limits(control_joints[name])
        if name.endswith('joint4') and reference[1] > -.7 + 1e-12:
            raise ValueError('refuse simulation-expanded joint4 limit in Cyclo control model')
        lower, upper = max(actual[0], reference[0]), min(actual[1], reference[1])
        if lower >= upper:
            raise ValueError(name + ': empty conservative position-limit intersection')
        velocity = min(actual[2], reference[2], MAX_ARM_SPEED)
        # [CHANGED] Intersect, never widen either source's arm limits.
        bound = joints[name].find('limit')
        for key, value in (('lower', lower), ('upper', upper), ('velocity', velocity)):
            bound.set(key, format(value, '.17g'))
        arm_limits[name], velocities[name] = [lower, upper], velocity
    frozen = {name: measured[name] for name in sorted(movable - set(RETAINED_JOINTS))}
    for name, position in frozen.items():
        _freeze_joint(joints[name], position)
    srdf, pair_report = _collision_srdf(description, links, joints, srdf_text)
    report = {
        'schema': 1, 'model': 'sh5', 'description_robot_name': description.get('name'),
        'description_sha256': hashlib.sha256(description_text.encode()).hexdigest(),
        'control_source_sha256': hashlib.sha256(control_text.encode()).hexdigest(),
        'srdf_source_sha256': hashlib.sha256(srdf_text.encode()).hexdigest(),
        'link_count': len(links),
        'collision_element_count': sum(len(link.findall('collision')) for link in links.values()),
        'retained_movable_joints': list(RETAINED_JOINTS), 'movable_dof': 15,
        'frozen_joint_count': len(frozen), 'frozen_posture_must_be_monitored': True,
        'arm_limits': arm_limits, 'arm_velocity_limits': velocities,
        'required_lift_velocity_bound': 0.0,
        'tool_collision_coverage': 'SH5 description geometry at measured frozen hand posture',
        'mesh_resources_verified': False, 'physical_motion_verified': False,
        **pair_report,
    }
    return ET.tostring(description, encoding='unicode'), ET.tostring(srdf, encoding='unicode'), frozen, report
