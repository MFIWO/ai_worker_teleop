"""Build the unmodified ROBOTIS pressure broadcaster; never opens hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

PACKAGE = 'robotis_hand_pressure_broadcaster'
SOURCE = Path(__file__).resolve().parents[1] / 'vendor' / PACKAGE
ROOT = Path('/workspace/ai_worker_xr_hand_pressure')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_hashes(source=SOURCE):
    return {str(p.relative_to(source)): sha(p) for p in sorted(source.rglob('*'))
            if p.is_file() and '__pycache__' not in p.parts}


def library(root=ROOT):
    return Path(root) / 'install' / PACKAGE / 'lib' / ('lib' + PACKAGE + '.so')


def verify(root=ROOT, source=SOURCE):
    manifest = json.loads((Path(root) / 'pressure_manifest.json').read_text())
    if manifest['source_sha256'] != source_hashes(source):
        raise ValueError('Pressure broadcaster source changed; rebuild before bringup')
    if manifest['library_sha256'] != sha(library(root)):
        raise ValueError('Pressure broadcaster library changed; rebuild before bringup')
    return manifest


def follower_environment(base=None, root=ROOT):
    verify(root)
    env = dict(os.environ if base is None else base)
    prefix = str(Path(root) / 'install' / PACKAGE)
    for key, path in (('AMENT_PREFIX_PATH', prefix), ('CMAKE_PREFIX_PATH', prefix),
                      ('LD_LIBRARY_PATH', prefix + '/lib')):
        env[key] = path + (':' + env[key] if env.get(key) else '')
    return env


def build(root=ROOT, source=SOURCE):
    root, source = Path(root).resolve(), Path(source).resolve()
    if (root / 'pressure_manifest.json').exists():
        verify(root, source)
        print('[HX5 PRESSURE BUILD] Verified official read-only broadcaster', flush=True)
        return
    if not (source / 'package.xml').is_file():
        raise FileNotFoundError('Missing vendored ROBOTIS pressure broadcaster; rerun install_robot.sh')
    before = source_hashes(source)
    root.mkdir(parents=True, exist_ok=True)
    subprocess.run(['colcon', 'build', '--base-paths', str(source),
                    '--packages-select', PACKAGE, '--executor', 'sequential',
                    '--cmake-args', '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF'],
                   cwd=root, env={**os.environ, 'CMAKE_BUILD_PARALLEL_LEVEL': '2'},
                   check=True, timeout=900)
    if source_hashes(source) != before:
        raise RuntimeError('Pressure broadcaster source changed during build')
    manifest = {'source_sha256': before, 'library_sha256': sha(library(root)),
                'provenance': json.loads((source / 'SOURCE.json').read_text())}
    (root / 'pressure_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    verify(root, source)
    print('[HX5 PRESSURE BUILD] Built official broadcaster; no robot/controller started', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    if args.verify_only:
        verify()
    else:
        build()


if __name__ == '__main__':
    main()
