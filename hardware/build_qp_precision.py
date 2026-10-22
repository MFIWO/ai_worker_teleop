#!/usr/bin/env python3
"""[ADDED 2026-10-14] Build a numeric-only Cyclo core copy; no ROS or motor I/O.

Reuses the earlier simulation convergence fix, NOT its model/joint expansion.
Only two OSQP termination tolerances change. Objective, bounds, collision CBF,
Solved-status requirement, and installed ROBOTIS sources remain unchanged.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess


SOURCE = Path('/root/ros2_ws/src/cyclo_control/cyclo_motion_controller_core')
ROOT = Path('/workspace/ai_worker_xr_qp_precision')
HEADER = Path('include/cyclo_motion_controller_core/optimization/qp_base.hpp')
ANCHOR = '    solver_.settings()->getSettings()->verbose = false;'
EPSILON = 1e-5


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def precision_header(original):
    """Exact two-line numerical patch. Never alter a QP constraint."""
    if original.count(ANCHOR) != 1:
        raise ValueError('Unknown Cyclo QPBase source: expected one solver initialization anchor')
    if 'setAbsoluteTolerance(' in original or 'setRelativeTolerance(' in original:
        raise ValueError('Source already specifies convergence tolerances; review before rebuilding')
    return original.replace(ANCHOR, ANCHOR + '''
    // [CHANGED 2026-10-14] Numerical convergence only, no constraint relaxation.
    // The 1000-valued slack cost makes default relative termination too loose
    // for low-velocity wrist hold. Apply after EVERY solver reconstruction.
    solver_.settings()->setAbsoluteTolerance(1e-5);
    solver_.settings()->setRelativeTolerance(1e-5);''')


def source_hashes(source):
    # [DEBUG] Build from the exact installed package; no models/configs are patched.
    return {str(p.relative_to(source)): sha(p)
            for p in sorted(source.rglob('*')) if p.is_file() and '.git' not in p.parts}


def build(source, root):
    source, root = source.resolve(), root.resolve()
    if not (source / HEADER).is_file():
        raise FileNotFoundError(f'Missing installed Cyclo source: {source / HEADER}')
    if root == source or source in root.parents or root in source.parents:
        raise ValueError('Build destination must be separate from the installed source')
    patched = precision_header((source / HEADER).read_text())
    before = source_hashes(source)
    manifest_file = root / 'precision_manifest.json'
    library = root / 'install/cyclo_motion_controller_core/lib/libcyclo_motion_controller_core.so'
    if manifest_file.is_file():
        manifest = json.loads(manifest_file.read_text())
        if (manifest.get('source_sha256') != before or
                manifest.get('patched_header_sha256') != hashlib.sha256(patched.encode()).hexdigest() or
                not library.is_file() or sha(library) != manifest.get('library_sha256')):
            raise ValueError('Precision build/source changed; preserve it and use a new build directory')
        print('[QP PRECISION] Existing verified numeric-only build: ' + str(library))
        return
    if root.exists() and any(root.iterdir()):
        raise FileExistsError(f'Unfinished/unrelated directory will not be overwritten: {root}')
    destination = root / 'src/cyclo_motion_controller_core'
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, destination, ignore=shutil.ignore_patterns('.git', '__pycache__', '*.pyc'))
    (destination / HEADER).write_text(patched)
    # [DEBUG] All copied compiled files must match; the sole exception is HEADER.
    for name, digest in before.items():
        if '__pycache__' in Path(name).parts or name.endswith('.pyc'):
            continue
        if name != str(HEADER) and sha(destination / name) != digest:
            raise RuntimeError('Unexpected copied-source change: ' + name)
    subprocess.run([
        'colcon', 'build', '--packages-select', 'cyclo_motion_controller_core',
        '--executor', 'sequential', '--cmake-args', '-DCMAKE_BUILD_TYPE=Release',
        '-DBUILD_TESTING=OFF',
    ], cwd=root, env={**os.environ, 'CMAKE_BUILD_PARALLEL_LEVEL': '2'},
       check=True, timeout=900)
    if before != source_hashes(source):
        raise RuntimeError('Installed source changed during build; do not select this library')
    if not library.is_file():
        raise RuntimeError('Build completed without the expected shared library')
    manifest = {
        'schema': 1, 'purpose': 'hardware_qp_numerical_convergence_only',
        'source': str(source), 'source_sha256': before,
        'patched_header_sha256': sha(destination / HEADER),
        'library': str(library), 'library_sha256': sha(library),
        'absolute_tolerance': EPSILON, 'relative_tolerance': EPSILON,
        'modified_source_files': [str(HEADER)],
        'constraints_changed': False, 'robot_commands_sent': False,
    }
    manifest_file.write_text(json.dumps(manifest, indent=2) + '\n')
    print('[QP PRECISION] Numeric-only core built: ' + str(library))


def verify(root):
    root = root.resolve()
    manifest = json.loads((root / 'precision_manifest.json').read_text())
    library = root / 'install/cyclo_motion_controller_core/lib/libcyclo_motion_controller_core.so'
    if (manifest.get('purpose') != 'hardware_qp_numerical_convergence_only' or
            manifest.get('absolute_tolerance') != EPSILON or
            manifest.get('relative_tolerance') != EPSILON or
            manifest.get('constraints_changed') is not False or
            str(library) != manifest.get('library') or sha(library) != manifest.get('library_sha256')):
        raise ValueError('Numeric-only library manifest/hash mismatch')
    source = Path(manifest['source'])
    if source_hashes(source) != manifest['source_sha256']:
        raise ValueError('Installed Cyclo sources changed after the precision build')
    print('[QP PRECISION] Verified 1e-5 convergence tolerances; original CBF/joint constraints retained')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, default=SOURCE)
    p.add_argument('--root', type=Path, default=ROOT)
    p.add_argument('--verify-only', action='store_true')
    a = p.parse_args()
    if a.verify_only:
        verify(a.root)
    else:
        build(a.source, a.root)


if __name__ == '__main__':
    main()
