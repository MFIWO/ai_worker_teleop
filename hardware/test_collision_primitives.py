"""[ADDED] Offline containment tests using actual coal and Pinocchio bindings.

Run with the robot's ROS-sourced Python. No ROS imports, commands, or nodes.
XR_COLLISION_TEST_URDF may point to a fully resolved SH5 profile for the actual
mesh regression. The standard local validation profile is used when present.
"""
from copy import deepcopy
import math
import os
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import coal
import pinocchio as pin

from hardware.collision_primitives import enclose_collision_meshes


VERTICES = np.array([[10., -20., 30.], [40., -20., 30.],
                     [10., 50., 30.], [10., -20., 90.]])
FACES = ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))


def _obj(path):
    lines = ['v ' + ' '.join(map(str, row)) for row in VERTICES]
    lines += ['f ' + ' '.join(str(i+1) for i in face) for face in FACES]
    path.write_text('\n'.join(lines) + '\n')


def _stl(path, vertices=VERTICES, faces=FACES):
    with path.open('wb') as file:
        file.write(b'offline enclosing box test'.ljust(80, b'\x00'))
        file.write(struct.pack('<I', len(faces)))
        for face in faces:
            file.write(struct.pack('<12fH', 0., 0., 0.,
                                   *vertices[np.array(face)].reshape(-1), 0))


def _xml(path, scale='.001 .002 .003', origin=True):
    root = ET.Element('robot', name='asymmetric_test')
    link = ET.SubElement(root, 'link', name='base_link')
    visual = ET.SubElement(link, 'visual', name='untouched_visual')
    vg = ET.SubElement(visual, 'geometry')
    ET.SubElement(vg, 'mesh', filename='package://unresolved_visual/keep.stl', scale='2 3 4')
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass', value='2.4')
    ET.SubElement(inertial, 'inertia', ixx='1', iyy='1', izz='1', ixy='0', ixz='0', iyz='0')
    collision = ET.SubElement(link, 'collision', name='source_mesh')
    if origin:
        ET.SubElement(collision, 'origin', xyz='.15 -.2 .35', rpy='.4 -.2 .7')
    geometry = ET.SubElement(collision, 'geometry')
    attrs = {'filename': str(path)}
    if scale is not None:
        attrs['scale'] = scale
    ET.SubElement(geometry, 'mesh', attrs)
    sphere = ET.SubElement(link, 'collision', name='untouched_sphere')
    ET.SubElement(sphere, 'origin', xyz='1 2 3', rpy='.1 .2 .3')
    ET.SubElement(ET.SubElement(sphere, 'geometry'), 'sphere', radius='.02')
    ET.SubElement(root, 'link', name='child_link')
    joint = ET.SubElement(root, 'joint', name='arm_joint', type='revolute')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link='child_link')
    ET.SubElement(joint, 'origin', xyz='.01 .02 .03', rpy='.1 .2 .3')
    ET.SubElement(joint, 'axis', xyz='0 1 0')
    ET.SubElement(joint, 'limit', lower='-1', upper='1', velocity='.35', effort='10')
    return ET.tostring(root, encoding='unicode')


def _signature(element):
    return element.tag, sorted(element.attrib.items()), (element.text or '').strip(), tuple(
        _signature(child) for child in element)


def _pin_geometry(xml):
    model = pin.buildModelFromXML(xml)
    return model, pin.buildGeomFromUrdfString(model, xml, pin.GeometryType.COLLISION)


class CollisionPrimitiveTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='xr_collision_primitives_')
        self.addCleanup(self.tmp.cleanup)
        self.mesh = Path(self.tmp.name) / 'asymmetric mesh.obj'
        _obj(self.mesh)

    def assert_model_containment(self, original, replacement):
        """Independently parse both URDFs and transform all mesh vertices via Pin."""
        model_before, before = _pin_geometry(original)
        model_after, after = _pin_geometry(replacement)
        self.assertEqual(model_before.nq, model_after.nq)
        self.assertEqual(model_before.nv, model_after.nv)
        self.assertEqual(before.ngeoms, after.ngeoms)
        checked = 0
        for old, new in zip(before.geometryObjects, after.geometryObjects):
            self.assertEqual(old.name, new.name)
            self.assertEqual(old.parentJoint, new.parentJoint)
            if not hasattr(old.geometry, 'vertices'):
                continue
            self.assertIsInstance(new.geometry, coal.Box)
            points = np.asarray(old.geometry.vertices())
            parent_points = points @ old.placement.rotation.T + old.placement.translation
            box_points = (parent_points-new.placement.translation) @ new.placement.rotation
            half = np.asarray(new.geometry.halfSide)
            self.assertTrue(np.all(box_points <= half + 5e-13), old.name)
            self.assertTrue(np.all(box_points >= -half - 5e-13), old.name)
            checked += 1
        return checked

    def test_nonuniform_scale_already_in_pin_geometry_and_coal_aabb(self):
        xml = _xml(self.mesh)
        output, report = enclose_collision_meshes(xml)
        self.assertEqual(self.assert_model_containment(xml, output), 1)
        record = report['meshes'][0]
        expected = VERTICES * [.001, .002, .003]
        np.testing.assert_allclose(record['local_min_m'], expected.min(axis=0), atol=1e-15)
        np.testing.assert_allclose(record['local_max_m'], expected.max(axis=0), atol=1e-15)
        np.testing.assert_allclose(record['extents_m'], np.ptp(expected, axis=0)+2e-6, atol=1e-15)
        _, source_geometry = _pin_geometry(xml)
        obj = source_geometry.geometryObjects[0]
        obj.geometry.computeLocalAABB()
        np.testing.assert_allclose(obj.meshScale, [.001, .002, .003])
        np.testing.assert_allclose(obj.geometry.aabb_local.max_, expected.max(axis=0))
        self.assertTrue(report['conservative'])
        self.assertFalse(report['exact_mesh_geometry'])
        self.assertTrue(record['scale_semantics_verified'])

    def test_rotated_origin_and_gimbal_poles_keep_rotation(self):
        for rpy in ('.4 -.2 .7', '0 0 1.5707963267948966',
                    f'.6 {math.pi/2} -.9', f'.6 {-math.pi/2} -.9'):
            with self.subTest(rpy=rpy):
                root = ET.fromstring(_xml(self.mesh))
                root.find('./link/collision/origin').set('rpy', rpy)
                source = ET.tostring(root, encoding='unicode')
                output, _ = enclose_collision_meshes(source)
                self.assert_model_containment(source, output)
                self.assertEqual(ET.fromstring(output).find('./link/collision/origin').get('rpy'), rpy)

    def test_default_scale_missing_origin_and_binary_stl(self):
        path = Path(self.tmp.name) / 'asymmetric.stl'
        _stl(path)
        source = _xml(path, scale=None, origin=False)
        output, report = enclose_collision_meshes(source)
        self.assert_model_containment(source, output)
        record = report['meshes'][0]
        np.testing.assert_allclose(record['mesh_scale'], [1, 1, 1])
        np.testing.assert_allclose(record['center_m'], (VERTICES.min(0)+VERTICES.max(0))/2)
        np.testing.assert_allclose(record['box_origin_xyz_m'], record['center_m'])

    def test_repeated_path_with_different_scales_does_not_alias(self):
        root = ET.fromstring(_xml(self.mesh))
        link = root.find('link')
        for scale in ('1 1 1', '.004 .002 .001', '.001 .002 .003'):
            collision = deepcopy(link.find('collision'))
            collision.set('name', 'copy_' + str(len(link)))
            collision.find('geometry/mesh').set('scale', scale)
            link.append(collision)
        source = ET.tostring(root, encoding='unicode')
        output, report = enclose_collision_meshes(source)
        self.assertEqual(self.assert_model_containment(source, output), 4)
        self.assertEqual(report['unique_scaled_mesh_count'], 3)

    def test_untouched_visual_inertial_joint_nonmesh_and_comments(self):
        source = _xml(self.mesh).replace('<robot name="asymmetric_test">',
                                        '<robot name="asymmetric_test"><!-- keep this -->')
        output, report = enclose_collision_meshes(source)
        before, after = ET.fromstring(source), ET.fromstring(output)
        for xpath in ('./link/visual', './link/inertial', './joint',
                      './link/collision[@name="untouched_sphere"]'):
            self.assertEqual(_signature(before.find(xpath)), _signature(after.find(xpath)))
        self.assertEqual([link.attrib for link in before.findall('link')],
                         [link.attrib for link in after.findall('link')])
        self.assertIn('<!-- keep this -->', output)
        self.assertFalse(report['collision_pairs_changed'])
        self.assertFalse(report['visuals_changed'])
        self.assertFalse(report['kinematics_changed'])

    def test_flat_mesh_gets_positive_conservative_thickness(self):
        path = Path(self.tmp.name) / 'flat.stl'
        _stl(path, np.array([[0., 0., 4.], [1., 0., 4.], [0., 1., 4.]]), ((0, 1, 2),))
        source = _xml(path)
        output, report = enclose_collision_meshes(source)
        self.assert_model_containment(source, output)
        self.assertGreater(report['meshes'][0]['extents_m'][2], 0.)

    def test_file_uri_with_spaces(self):
        source = _xml(self.mesh.as_uri())
        output, report = enclose_collision_meshes(source)
        self.assertEqual(report['meshes'][0]['resolved_mesh_path'], str(self.mesh))
        self.assert_model_containment(_xml(self.mesh), output)

    def test_invalid_mesh_scale_and_unresolved_files_fail(self):
        for scale in ('0 1 1', '-1 1 1', 'nan 1 1', 'inf 1 1', '1 2', 'x y z'):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                enclose_collision_meshes(_xml(self.mesh, scale=scale))
        for path in ('package://something/missing.stl', 'relative.stl',
                     'file://remote/tmp/foo.stl', str(Path(self.tmp.name)/'missing.obj')):
            with self.subTest(path=path), self.assertRaises(ValueError):
                enclose_collision_meshes(_xml(path))

    def test_invalid_padding_and_ambiguous_geometry_fail(self):
        for padding in (-1., float('nan'), float('inf'), True):
            with self.subTest(padding=padding), self.assertRaises(ValueError):
                enclose_collision_meshes(_xml(self.mesh), padding_m=padding)
        root = ET.fromstring(_xml(self.mesh))
        ET.SubElement(root.find('./link/collision/geometry'), 'box', size='1 1 1')
        with self.assertRaises(ValueError):
            enclose_collision_meshes(ET.tostring(root, encoding='unicode'))

    def test_no_mesh_round_trip_is_idempotent(self):
        source, _ = enclose_collision_meshes(_xml(self.mesh))
        again, report = enclose_collision_meshes(source)
        self.assertEqual(source, again)
        self.assertEqual(report['mesh_count'], 0)

    def test_actual_sh5_every_mesh_enclosed_and_frozen_kinematics_retained(self):
        value = os.environ.get('XR_COLLISION_TEST_URDF')
        path = Path(value or '/workspace/ai_worker_xr_validation/sh5_profile/sh5_hardware_arms.urdf')
        if not path.is_file():
            if value:
                self.fail('XR_COLLISION_TEST_URDF does not exist: ' + value)
            self.skipTest('no actual SH5 resolved profile supplied')
        source = path.read_text()
        output, report = enclose_collision_meshes(source)
        before, after = ET.fromstring(source), ET.fromstring(output)
        self.assertEqual(before.get('name'), 'ffw_sh5_follower')
        mesh_count = len(before.findall('./link/collision/geometry/mesh'))
        self.assertGreater(mesh_count, 40)
        self.assertEqual(report['mesh_count'], mesh_count)
        self.assertEqual(self.assert_model_containment(source, output), mesh_count)
        self.assertEqual([_signature(j) for j in before.findall('joint')],
                         [_signature(j) for j in after.findall('joint')])
        for old, new in zip(before.findall('link'), after.findall('link')):
            self.assertEqual(old.attrib, new.attrib)
            for tag in ('visual', 'inertial'):
                self.assertEqual([_signature(e) for e in old.findall(tag)],
                                 [_signature(e) for e in new.findall(tag)])
        model, _ = _pin_geometry(output)
        self.assertEqual((model.nq, model.nv), (15, 15))
        self.assertTrue(all(x['conservative'] and x['all_vertices_enclosed'] for x in report['meshes']))


if __name__ == '__main__':
    unittest.main()
