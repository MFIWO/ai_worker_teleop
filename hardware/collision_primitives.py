"""[ADDED] Enclose each collision mesh by a box; never change collision pairs.

Input collision mesh paths must already be resolved on the robot. This helper
reads mesh files but does not write files, launch nodes, or change the model's
links/joints, visual meshes, inertials, or non-mesh collision geometry.

The existing coal loader applies the URDF mesh scale to its vertices and local
AABB. Pinocchio's GeometryObject.meshScale is metadata, not a second scaling
operation. This is checked against a unit-scale load for every distinct mesh.
The boxes are conservative geometric approximations, not exact mesh geometry
or a guarantee about a controller's collision-distance solver/performance.
"""
import hashlib
import math
from pathlib import Path
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

import numpy as np


def _vector(text, label):
    try:
        result = np.array([float(x) for x in text.split()], dtype=float)
    except (TypeError, ValueError, AttributeError) as error:
        raise ValueError(f'{label}: expected three finite numbers') from error
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f'{label}: expected three finite numbers')
    return result


def _rotation(rpy):
    """URDF fixed-axis rotation: Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                     (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
                     (-sp, cp*sr, cp*cr)))


def _text(values):
    return ' '.join(format(float(x), '.17g') for x in values)


def _mesh_path(filename):
    if not isinstance(filename, str) or not filename:
        raise ValueError('collision mesh has no filename')
    if filename.startswith('file:'):
        uri = urlsplit(filename)
        if uri.scheme != 'file' or uri.netloc or uri.query or uri.fragment:
            raise ValueError(f'expected a local file URI: {filename}')
        path = Path(unquote(uri.path))
    else:
        path = Path(filename)
    if not path.is_absolute():
        raise ValueError(f'collision mesh must be a resolved absolute path: {filename}')
    try:
        path = path.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise ValueError(f'collision mesh does not exist: {filename}') from error
    if not path.is_file():
        raise ValueError(f'collision mesh is not a regular file: {filename}')
    return path


def _stamp(path):
    stat = path.stat()
    return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns


def _loaded_bounds(loader, path, scale):
    try:
        geometry = loader.load(str(path), scale)
        geometry.computeLocalAABB()
        lower = np.asarray(geometry.aabb_local.min_, dtype=float).copy()
        upper = np.asarray(geometry.aabb_local.max_, dtype=float).copy()
        vertices = np.asarray(geometry.vertices(), dtype=float)
    except Exception as error:
        raise ValueError(f'cannot obtain verified mesh bounds for {path}: {error}') from error
    if (lower.shape != (3,) or upper.shape != (3,)
            or not np.isfinite(lower).all() or not np.isfinite(upper).all()
            or np.any(upper < lower) or vertices.ndim != 2
            or vertices.shape[1] != 3 or not len(vertices)
            or not np.isfinite(vertices).all()):
        raise ValueError(f'invalid/empty mesh bounds or vertices: {path}')
    # [DEBUG] AABB must actually enclose every loaded vertex, not just a sample.
    vertex_min, vertex_max = vertices.min(axis=0), vertices.max(axis=0)
    if np.any(vertex_min < lower) or np.any(vertex_max > upper):
        raise ValueError(f'loader AABB does not enclose its vertices: {path}')
    return lower, upper, len(vertices)


def enclose_collision_meshes(urdf_text: str, *, padding_m: float = 1e-6):
    """Return ``(urdf_xml, report)`` with an enclosing box for EACH collision mesh.

    Box axes remain the mesh's original axes. The mesh scale is baked into the
    bounds exactly once, then the collision translation becomes ``t + R*c``;
    the original collision RPY text is retained. ``padding_m`` is outward padding
    on each face, supplemented by a floating-point rounding allowance. Positive,
    finite nonuniform scales are supported; unresolved, empty, malformed, and
    nonpositive-scale meshes fail instead of producing an unchecked model.

    Only numpy and the installed coal or hppfcl bindings are used. No trimesh/Assimp
    Python dependency or node is needed. Conservative boxes can introduce false
    collisions, so the caller must recheck initial clearance and controller timing.
    """
    if not isinstance(urdf_text, str):
        raise ValueError('expanded URDF XML text is required')
    if '<!DOCTYPE' in urdf_text or '<!ENTITY' in urdf_text:
        raise ValueError('XML entity declarations are not supported')
    if isinstance(padding_m, bool):
        raise ValueError('padding_m must be finite and nonnegative')
    try:
        padding_m = float(padding_m)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError('padding_m must be finite and nonnegative') from error
    if not math.isfinite(padding_m) or padding_m < 0.:
        raise ValueError('padding_m must be finite and nonnegative')
    try:
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True, insert_pis=True))
        root = ET.fromstring(urdf_text, parser=parser)
    except ET.ParseError as error:
        raise ValueError(f'invalid URDF XML: {error}') from error
    if root.tag != 'robot' or any(isinstance(e.tag, str) and 'xacro' in e.tag for e in root.iter()):
        raise ValueError('expected an expanded robot URDF')
    try:
        import coal
    except ModuleNotFoundError as error:
        if error.name != 'coal':
            raise
        try:
            # ROS Jazzy's HPP-FCL 2.x exposes the same verified mesh loader API.
            import hppfcl as coal
        except ImportError as error:
            raise RuntimeError('robot coal or hppfcl Python bindings are required; source its ROS environment') from error

    # Use a non-cached loader: identical paths with different scales must not
    # alias loader-cached geometry. Our own cache includes the complete scale.
    loader = coal.MeshLoader()
    unit_cache, scaled_cache, resources = {}, {}, {}
    entries = []
    mesh_count = len(root.findall('./link/collision/geometry/mesh'))
    for link in root.findall('link'):
        for index, collision in enumerate(link.findall('collision')):
            geometry = collision.find('geometry')
            if geometry is None or geometry.find('mesh') is None:
                continue
            if len(collision.findall('geometry')) != 1:
                raise ValueError(f'{link.get("name")}: collision has multiple geometry blocks')
            shapes = [e for e in geometry if isinstance(e.tag, str)]
            if len(shapes) != 1 or shapes[0].tag != 'mesh':
                raise ValueError(f'{link.get("name")}: ambiguous collision geometry')
            mesh = shapes[0]
            path = _mesh_path(mesh.get('filename'))
            scale = _vector(mesh.get('scale', '1 1 1'), 'mesh scale')
            if np.any(scale <= 0.):
                raise ValueError('mesh scale must be strictly positive; reflected/zero scales need separate validation')
            if path not in resources:
                before = _stamp(path)
                with path.open('rb') as file:
                    digest = hashlib.file_digest(file, 'sha256').hexdigest()
                resources[path] = (before, digest)
                unit_cache[path] = _loaded_bounds(loader, path, np.ones(3))
            key = (path, tuple(scale))
            if key not in scaled_cache:
                if np.array_equal(scale, np.ones(3)):
                    bounds = unit_cache[path]
                else:
                    bounds = _loaded_bounds(loader, path, scale)
                unit_lower, unit_upper, unit_count = unit_cache[path]
                lower, upper, count = bounds
                # [DEBUG] Runtime check for the installed loader's scale semantics.
                # Actual box dimensions still use the already-scaled loaded AABB.
                if (count != unit_count
                        or not np.allclose(lower, unit_lower*scale, rtol=1e-9, atol=1e-12)
                        or not np.allclose(upper, unit_upper*scale, rtol=1e-9, atol=1e-12)):
                    raise ValueError(f'coal mesh scaling differs from verified local-axis semantics: {path}')
                scaled_cache[key] = bounds
            lower, upper, count = scaled_cache[key]
            origin = collision.find('origin')
            if len(collision.findall('origin')) > 1:
                raise ValueError('collision has more than one origin')
            xyz = _vector(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', 'origin xyz')
            rpy_text = origin.get('rpy', '0 0 0') if origin is not None else '0 0 0'
            rotation = _rotation(_vector(rpy_text, 'origin rpy'))
            center = lower + .5*(upper-lower)
            # Outward rounding protects serialized/reloaded center and half-size.
            magnitude = max(1., float(np.max(np.abs(lower))), float(np.max(np.abs(upper))),
                            float(np.max(np.abs(xyz))))
            padding = max(padding_m, 64*np.finfo(float).eps*magnitude)
            half_size = np.nextafter(np.maximum(upper-center, center-lower) + padding, np.inf)
            size = 2.*half_size
            translated = xyz + rotation @ center
            if not np.isfinite(np.r_[center, size, translated]).all() or np.any(size <= 0.):
                raise ValueError('box dimensions/translation overflow')
            # Check the exact decimal round trip that will go into the URDF.
            size_text, xyz_text = _text(size), _text(translated)
            parsed_half = _vector(size_text, 'serialized box size')*.5
            parsed_center = rotation.T @ (_vector(xyz_text, 'serialized origin') - xyz)
            if np.any(parsed_center-parsed_half > lower) or np.any(parsed_center+parsed_half < upper):
                raise ValueError('serialized box would not enclose the loaded mesh')
            if origin is None:
                origin = ET.Element('origin')
                collision.insert(0, origin)
            # [CHANGED] Only collision origin translation and mesh geometry change.
            origin.set('xyz', xyz_text)
            slot = list(geometry).index(mesh)
            geometry.remove(mesh)
            geometry.insert(slot, ET.Element('box', {'size': size_text}))
            entries.append({
                'link': link.get('name'), 'collision_index': index,
                'collision_name': collision.get('name'),
                'mesh_filename': mesh.get('filename'), 'resolved_mesh_path': str(path),
                'mesh_sha256': resources[path][1], 'mesh_scale': scale.tolist(),
                'local_min_m': lower.tolist(), 'local_max_m': upper.tolist(),
                'center_m': center.tolist(), 'extents_m': size.tolist(),
                'original_origin_xyz_m': xyz.tolist(), 'origin_rpy': rpy_text,
                'box_origin_xyz_m': translated.tolist(), 'padding_per_face_m': padding,
                'vertex_count': int(count), 'all_vertices_enclosed': True,
                'scale_semantics_verified': True, 'conservative': True,
            })
    for path, (before, _) in resources.items():
        if _stamp(path) != before:
            raise ValueError(f'mesh resource changed during conversion: {path}')
    if len(entries) != mesh_count or root.findall('./link/collision/geometry/mesh'):
        raise ValueError('not every collision mesh was converted')
    report = {
        'schema': 1, 'approximation': 'enclosing_mesh_axis_aligned_boxes',
        'conservative': True, 'exact_mesh_geometry': False,
        'source_urdf_sha256': hashlib.sha256(urdf_text.encode()).hexdigest(),
        'coal_version': getattr(coal, '__version__', 'unknown'),
        'collision_backend': coal.__name__,
        'mesh_count': len(entries), 'unique_scaled_mesh_count': len(scaled_cache),
        'scale_semantics': 'coal local AABB already includes mesh scale; never multiply it twice',
        'collision_pairs_changed': False, 'kinematics_changed': False,
        'visuals_changed': False, 'inertials_changed': False,
        'clearance_recheck_required': bool(entries), 'physical_motion_verified': False,
        'meshes': entries,
    }
    return ET.tostring(root, encoding='unicode'), report
