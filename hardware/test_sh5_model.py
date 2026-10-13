"""[ADDED] Offline SH5 reduction checks; no ROS, processes or robot commands.

Standalone synthetic tests always run. To include the actual installation, set
SH5_DESCRIPTION_URDF, SH5_CONTROL_URDF and SH5_CONTROL_SRDF to existing XML files.
The independent FK oracle uses quaternions, not the builder's RPY/Rodrigues code.
"""
from copy import deepcopy
import math
import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from hardware.sh5_model import ARM_NAMES, RETAINED_JOINTS, build_sh5_model


def _xml(root):
    return ET.tostring(root, encoding='unicode')


def _add_link(root, name):
    element = ET.SubElement(root, 'link', name=name)
    for kind in ('visual', 'collision'):
        shape = ET.SubElement(element, kind)
        ET.SubElement(shape, 'origin', xyz='.001 .002 .003', rpy='.1 -.2 .3')
        ET.SubElement(ET.SubElement(shape, 'geometry'), 'box', size='.01 .02 .03')
    return element


def _add_joint(root, name, parent, child, kind='revolute', xyz='0 0 0',
               rpy='0 0 0', axis='0 0 1', lower=-3.14, upper=3.14, velocity=4.8):
    _add_link(root, child)
    joint = ET.SubElement(root, 'joint', name=name, type=kind)
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'origin', xyz=xyz, rpy=rpy)
    if kind != 'fixed':
        ET.SubElement(joint, 'axis', xyz=axis)
        limit = ET.SubElement(joint, 'limit', velocity=str(velocity), effort='1000')
        if kind != 'continuous':
            limit.set('lower', str(lower))
            limit.set('upper', str(upper))
    return joint


def _synthetic_models():
    description = ET.Element('robot', name='ffw_sh5_follower')
    _add_link(description, 'base_link')
    _add_joint(description, 'lift_joint', 'base_link', 'arm_base_link', 'prismatic',
               xyz='-.0199 0 1.4316', lower=-.5, upper=0.)
    for side, sign in (('l', 1), ('r', -1)):
        origins = [f'0 {sign*.1045} 0', f'0 {sign*.123} 0', '0 0 -.165',
                   '.041004 0 -.135', '-.041004 0 -.1489', '0 0 -.1041', '0 0 -.0885']
        axes = ['0 1 0', '1 0 0', '0 0 1', '0 1 0', '0 0 1', '0 1 0', '1 0 0']
        for i, (xyz, axis) in enumerate(zip(origins, axes), 1):
            parent = 'arm_base_link' if i == 1 else f'arm_{side}_link{i-1}'
            lower, upper = (-2.9361, 1.0786) if i == 4 else (-3.14, 3.14)
            _add_joint(description, f'arm_{side}_joint{i}', parent, f'arm_{side}_link{i}',
                       xyz=xyz, axis=axis, lower=lower, upper=upper)
        base = 'hx5_d20_' + ('left' if side == 'l' else 'right') + '_base'
        _add_joint(description, base+'_joint', f'arm_{side}_link7', base, 'fixed',
                   xyz='0 0 -.078', rpy=f'0 {math.pi} {sign*math.pi/2}')
        for i in range(1, 21):
            parent = base if (i-1) % 4 == 0 else f'finger_{side}_link{i-1}'
            _add_joint(description, f'finger_{side}_joint{i}', parent, f'finger_{side}_link{i}',
                       xyz=f'.012 {i*.001} .013', rpy='.13 -.21 .32', axis='1 2 -1',
                       lower=-1.57, upper=1.57)
    _add_joint(description, 'head_joint1', 'arm_base_link', 'head_link1',
               xyz='.02 -.03 .25', rpy='.2 -.4 .6', axis='0 1 0')
    _add_joint(description, 'wheel_drive', 'base_link', 'wheel_link', 'continuous',
               xyz='.1 .2 .3', rpy='-.2 .1 .5', axis='0 1 0')
    _add_joint(description, 'aux_slider', 'base_link', 'aux_link', 'prismatic',
               xyz='.1 -.2 .3', rpy='-.2 .6 .3', axis='1 2 -1', lower=-.5, upper=.5)
    control = deepcopy(description)
    control.set('name', 'ffw_sg2_follower')
    for side in ('l', 'r'):
        control.find(f"joint[@name='arm_{side}_joint4']/limit").set('upper', '-0.7')
        control.find(f"joint[@name='arm_{side}_joint1']/limit").set('upper', '1.5707')
    srdf = ET.Element('robot', name='ffw_sg2_follower')
    ET.SubElement(srdf, 'group', name='old_semantic_group')
    ET.SubElement(srdf, 'disable_collisions', link1='arm_l_link1', link2='arm_r_link1', reason='Never')
    ET.SubElement(srdf, 'disable_collisions', link1='camera_l_link', link2='arm_l_link7', reason='Adjacent')
    return _xml(description), _xml(control), _xml(srdf)


def _snapshot(text, fraction=.37):
    result = {}
    for index, joint in enumerate(ET.fromstring(text).findall('joint')):
        if joint.get('type') == 'fixed':
            continue
        bound = joint.find('limit')
        if joint.get('type') == 'continuous':
            value = .41 + .03*index
        else:
            lower, upper = float(bound.get('lower')), float(bound.get('upper'))
            value = lower + fraction*(upper-lower)
        result[joint.get('name')] = value
    return result


def _quaternion_product(a, b):
    return np.r_[a[0]*b[0] - np.dot(a[1:], b[1:]),
                 a[0]*b[1:] + b[0]*a[1:] + np.cross(a[1:], b[1:])]


def _quaternion_matrix(quat):
    w, x, y, z = quat / np.linalg.norm(quat)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _joint_transform(joint, measured):
    origin = joint.find('origin')
    xyz = np.zeros(3) if origin is None else np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    rpy = np.zeros(3) if origin is None else np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
    quat = np.array([1., 0., 0., 0.])
    for basis, angle in zip(np.eye(3), rpy):
        delta = np.r_[math.cos(angle/2), basis*math.sin(angle/2)]
        quat = _quaternion_product(delta, quat)
    matrix = np.eye(4)
    matrix[:3, :3] = _quaternion_matrix(quat)
    matrix[:3, 3] = xyz
    if joint.get('type') != 'fixed':
        axis_tag = joint.find('axis')
        axis = np.fromstring(axis_tag.get('xyz', '1 0 0') if axis_tag is not None else '1 0 0', sep=' ')
        axis /= np.linalg.norm(axis)
        q = measured[joint.get('name')]
        delta = np.eye(4)
        if joint.get('type') == 'prismatic':
            delta[:3, 3] = axis*q
        else:
            delta[:3, :3] = _quaternion_matrix(np.r_[math.cos(q/2), axis*math.sin(q/2)])
        matrix = matrix @ delta
    return matrix


def _fk(text, measured):
    root = ET.fromstring(text)
    joints = root.findall('joint')
    children = {joint.find('child').get('link') for joint in joints}
    roots = {link.get('name') for link in root.findall('link')} - children
    poses = {next(iter(roots)): np.eye(4)}
    pending = list(joints)
    while pending:
        count = len(pending)
        for joint in list(pending):
            parent = joint.find('parent').get('link')
            if parent in poses:
                poses[joint.find('child').get('link')] = poses[parent] @ _joint_transform(joint, measured)
                pending.remove(joint)
        if len(pending) == count:
            raise AssertionError('independent FK found an invalid tree')
    return poses


class _ModelAssertions:
    def assert_geometry_preserved(self, original, generated):
        a = {l.get('name'): _xml(l) for l in ET.fromstring(original).findall('link')}
        b = {l.get('name'): _xml(l) for l in ET.fromstring(generated).findall('link')}
        self.assertEqual(a, b)

    def assert_frozen_fk(self, original, generated, snapshot):
        # Vary the still-actuated arms/lift AFTER reduction as well as the hand
        # pose used to build it, verifying whole-subtree transform composition.
        for phase in (0., .07, -.11):
            positions = dict(snapshot)
            for name in RETAINED_JOINTS:
                positions[name] += phase
            expected, actual = _fk(original, positions), _fk(generated, positions)
            self.assertEqual(set(expected), set(actual))
            for name in expected:
                np.testing.assert_allclose(actual[name], expected[name], atol=3e-12, rtol=0,
                                           err_msg=f'link FK changed: {name}, phase={phase}')

    def assert_limits(self, description, control, generated):
        d, c, g = map(ET.fromstring, (description, control, generated))
        movable = {j.get('name'): j.get('type') for j in g.findall('joint') if j.get('type') != 'fixed'}
        self.assertEqual(movable, {name: 'prismatic' if name == 'lift_joint' else 'revolute'
                                   for name in RETAINED_JOINTS})
        for name in ARM_NAMES:
            old, ref, new = [t.find(f"joint[@name='{name}']/limit") for t in (d, c, g)]
            self.assertEqual(float(new.get('lower')), max(float(old.get('lower')), float(ref.get('lower'))))
            self.assertEqual(float(new.get('upper')), min(float(old.get('upper')), float(ref.get('upper'))))
            self.assertEqual(float(new.get('velocity')), min(.35, float(old.get('velocity')), float(ref.get('velocity'))))


class SH5ModelTests(_ModelAssertions, unittest.TestCase):
    def setUp(self):
        self.description, self.control, self.srdf = _synthetic_models()
        self.snapshot = _snapshot(self.description)

    def build(self, description=None, control=None, srdf=None, snapshot=None):
        return build_sh5_model(description or self.description, control or self.control,
                               srdf or self.srdf, self.snapshot if snapshot is None else snapshot)

    def test_all_geometry_and_frozen_fk_at_multiple_nonzero_hand_postures(self):
        for fraction in (.18, .43, .71):
            with self.subTest(fraction=fraction):
                snapshot = _snapshot(self.description, fraction)
                xml, srdf, frozen, report = self.build(snapshot=snapshot)
                self.assert_geometry_preserved(self.description, xml)
                self.assert_frozen_fk(self.description, xml, snapshot)
                self.assert_limits(self.description, self.control, xml)
                self.assertEqual(set(frozen), set(snapshot)-set(RETAINED_JOINTS))
                self.assertEqual(report['movable_dof'], 15)
                self.assertTrue(report['frozen_posture_must_be_monitored'])
                self.assertFalse(report['physical_motion_verified'])

    def test_passive_prismatic_translation_respects_rotated_axis(self):
        snapshot = dict(self.snapshot, aux_slider=.21)
        xml, _, frozen, _ = self.build(snapshot=snapshot)
        self.assertEqual(frozen['aux_slider'], .21)
        expected = _joint_transform(ET.fromstring(self.description).find("joint[@name='aux_slider']"), snapshot)
        actual = _joint_transform(ET.fromstring(xml).find("joint[@name='aux_slider']"), {})
        np.testing.assert_allclose(actual, expected, atol=2e-15, rtol=0)

    def test_gimbal_poles_preserve_rotations(self):
        tree = ET.fromstring(self.description)
        joint = tree.find("joint[@name='head_joint1']")
        joint.find('origin').set('rpy', '0 0 .3')
        for angle in (-math.pi/2, math.pi/2):
            snapshot = dict(self.snapshot, head_joint1=angle)
            xml, _, _, _ = self.build(description=_xml(tree), snapshot=snapshot)
            self.assert_frozen_fk(_xml(tree), xml, snapshot)

    def test_narrower_description_and_velocity_limits_are_not_widened(self):
        tree = ET.fromstring(self.description)
        bound = tree.find("joint[@name='arm_l_joint1']/limit")
        bound.set('lower', '-1.2')
        bound.set('upper', '.9')
        bound.set('velocity', '.17')
        xml, _, _, report = self.build(description=_xml(tree))
        self.assert_limits(_xml(tree), self.control, xml)
        self.assertEqual(report['arm_limits']['arm_l_joint1'], [-1.2, .9])
        self.assertEqual(report['arm_velocity_limits']['arm_l_joint1'], .17)

    def test_srdf_additions_only_original_adjacency_or_fixed_cluster(self):
        xml, srdf, _, report = self.build()
        tree, semantic = ET.fromstring(xml), ET.fromstring(srdf)
        pairs = {tuple(sorted((e.get('link1'), e.get('link2')))) for e in semantic.findall('disable_collisions')}
        original = {('arm_l_link1', 'arm_r_link1')}
        adjacency = {tuple(sorted((j.find('parent').get('link'), j.find('child').get('link'))))
                     for j in tree.findall('joint')}
        fixed_graph = {l.get('name'): set() for l in tree.findall('link')}
        for j in tree.findall('joint'):
            if j.get('type') == 'fixed':
                a, b = j.find('parent').get('link'), j.find('child').get('link')
                fixed_graph[a].add(b)
                fixed_graph[b].add(a)
        for a, b in pairs - original - adjacency:
            visited, pending = set(), [a]
            while pending:
                here = pending.pop()
                if here not in visited:
                    visited.add(here)
                    pending.extend(fixed_graph[here] - visited)
            self.assertIn(b, visited)
        self.assertIn(tuple(sorted(('finger_l_link1', 'finger_l_link20'))), pairs)
        self.assertNotIn(tuple(sorted(('finger_l_link20', 'arm_r_link4'))), pairs)
        self.assertNotIn(('arm_l_link7', 'arm_r_link7'), pairs)
        self.assertEqual(report['omitted_original_disabled_pairs'][0]['missing_links'], ['camera_l_link'])
        self.assertEqual(semantic.get('name'), 'ffw_sh5_follower')
        self.assertEqual(report['srdf_semantic_tags_not_copied'], ['group'])

    def test_every_missing_measured_joint_rejected(self):
        for name in ('finger_l_joint7', 'wheel_drive', 'head_joint1', 'lift_joint', 'arm_l_joint1'):
            snapshot = dict(self.snapshot)
            del snapshot[name]
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, 'missing measured snapshot'):
                self.build(snapshot=snapshot)

    def test_invalid_measured_values_rejected(self):
        for value in (float('nan'), float('inf'), True, None, 'invalid'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'finite number'):
                self.build(snapshot=dict(self.snapshot, finger_r_joint12=value))

    def test_simulation_elbow_bound_rejected(self):
        tree = ET.fromstring(self.control)
        tree.find("joint[@name='arm_l_joint4']/limit").set('upper', '-.45')
        with self.assertRaisesRegex(ValueError, 'simulation-expanded'):
            self.build(control=_xml(tree))

    def test_empty_arm_limit_intersection_rejected(self):
        tree = ET.fromstring(self.description)
        tree.find("joint[@name='arm_l_joint4']/limit").set('lower', '0')
        with self.assertRaisesRegex(ValueError, 'empty conservative'):
            self.build(description=_xml(tree))

    def test_arm_chain_origin_axis_mismatch_rejected(self):
        for tag, key, value in (('axis', 'xyz', '1 0 0'), ('origin', 'xyz', '.1 0 -.165')):
            tree = ET.fromstring(self.description)
            tree.find(f"joint[@name='arm_l_joint3']/{tag}").set(key, value)
            with self.subTest(tag=tag), self.assertRaisesRegex(ValueError, 'arm-chain mismatch'):
                self.build(description=_xml(tree))

    def test_passive_floating_joint_rejected(self):
        tree = ET.fromstring(self.description)
        tree.find("joint[@name='wheel_drive']").set('type', 'floating')
        with self.assertRaisesRegex(ValueError, 'unsupported joint type'):
            self.build(description=_xml(tree))

    def test_prefrozen_fingers_rejected_without_measured_contract(self):
        tree = ET.fromstring(self.description)
        tree.find("joint[@name='finger_l_joint2']").set('type', 'fixed')
        with self.assertRaisesRegex(ValueError, 'measured revolute'):
            self.build(description=_xml(tree))

    def test_duplicate_joint_rejected(self):
        tree = ET.fromstring(self.description)
        tree.append(deepcopy(tree.find("joint[@name='arm_l_joint2']")))
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            self.build(description=_xml(tree))

    def test_deterministic_and_inputs_unchanged(self):
        before = dict(self.snapshot)
        first = self.build()
        second = self.build()
        self.assertEqual(first, second)
        self.assertEqual(self.snapshot, before)


class ActualSH5ModelTests(_ModelAssertions, unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        names = ('SH5_DESCRIPTION_URDF', 'SH5_CONTROL_URDF', 'SH5_CONTROL_SRDF')
        paths = [os.environ.get(name) for name in names]
        if not all(paths):
            raise unittest.SkipTest('Set SH5_DESCRIPTION_URDF/SH5_CONTROL_URDF/SH5_CONTROL_SRDF for installation checks')
        cls.description, cls.control, cls.srdf = [Path(path).read_text() for path in paths]

    def test_actual_nonzero_postures_preserve_all_link_fk_and_geometry(self):
        for fraction in (.23, .47, .68):
            with self.subTest(fraction=fraction):
                snapshot = _snapshot(self.description, fraction)
                xml, _, frozen, report = build_sh5_model(self.description, self.control, self.srdf, snapshot)
                self.assert_geometry_preserved(self.description, xml)
                self.assert_frozen_fk(self.description, xml, snapshot)
                self.assert_limits(self.description, self.control, xml)
                self.assertEqual(len(frozen), len(snapshot)-15)
                self.assertTrue(all(name in frozen for name in
                                    [f'finger_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 21)]))
                self.assertEqual(report['movable_dof'], 15)

    def test_actual_missing_finger_measurement_rejected(self):
        snapshot = _snapshot(self.description)
        del snapshot['finger_r_joint20']
        with self.assertRaisesRegex(ValueError, 'finger_r_joint20'):
            build_sh5_model(self.description, self.control, self.srdf, snapshot)

    def test_actual_cross_hand_pairs_kept_and_omissions_reported(self):
        snapshot = _snapshot(self.description)
        _, srdf, _, report = build_sh5_model(self.description, self.control, self.srdf, snapshot)
        pairs = {tuple(sorted((e.get('link1'), e.get('link2'))))
                 for e in ET.fromstring(srdf).findall('disable_collisions')}
        self.assertNotIn(('finger_l_link20', 'finger_r_link20'), pairs)
        self.assertNotIn(tuple(sorted(('finger_l_link20', 'arm_r_link4'))), pairs)
        self.assertEqual(report['link_count'], len(ET.fromstring(self.description).findall('link')))
        self.assertTrue(report['omitted_original_disabled_pairs'])


if __name__ == '__main__':
    unittest.main()
