"""Build a separate hardware VR node with measured SH5 fingers; no robot I/O."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

from hardware.build_startup_timing import SOURCE, CPP, HEADER, patch_source, patch_header, sha

ROOT = Path('/workspace/ai_worker_xr_sh5_hands')


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Installed Cyclo source differs at: ' + old[:100])
    return text.replace(old, new)


def patched_source(text):
    text = patch_source(text)  # Exact same timing changes as the existing arm node.
    anchor = '  left_arm_joints_.clear();'
    text = replace_once(text, anchor, '''  int measured_fingers = 0;
  for (size_t i = 0; i < model_joint_names_.size(); ++i) {
    if (model_joint_names_[i].rfind("finger_", 0) == 0) {
      if (!kinematics_solver_->setJointVelocityBoundsByIndex(static_cast<int>(i), 0.0, 0.0)) {
        throw std::runtime_error("Cannot exclude SH5 fingers from arm QP outputs");
      }
      ++measured_fingers;
    }
  }
  if (measured_fingers != 40 || model_joint_names_.size() != 55) {
    throw std::runtime_error("SH5 hand executable requires 14 arms + lift + 40 measured fingers");
  }
''' + anchor)
    anchor = '    // If lift is commanded by another node, use measured lift state for'
    text = replace_once(text, anchor, '''    // Hand retargeting owns fingers. Use their actual positions for collision FK.
    for (size_t i = 0; i < model_joint_names_.size(); ++i) {
      if (model_joint_names_[i].rfind("finger_", 0) == 0) {
        q_feedback[static_cast<int>(i)] = q_[static_cast<int>(i)];
      }
    }
''' + anchor)
    return text


def verify():
    manifest = json.loads((ROOT / 'hands_manifest.json').read_text())
    destination = ROOT / 'src/cyclo_motion_controller_ros'
    binary = ROOT / 'build/cyclo_motion_controller_ros/vr_controller_node'
    expected = {
        'source_sha256': sha(SOURCE / CPP),
        'header_source_sha256': sha(SOURCE / HEADER),
        'patched_sha256': hashlib.sha256(patched_source((SOURCE / CPP).read_text()).encode()).hexdigest(),
        'header_patched_sha256': hashlib.sha256(patch_header((SOURCE / HEADER).read_text()).encode()).hexdigest(),
        'binary_sha256': sha(binary),
    }
    if (any(manifest.get(k) != v for k, v in expected.items()) or
            sha(destination / CPP) != expected['patched_sha256'] or
            sha(destination / HEADER) != expected['header_patched_sha256']):
        raise ValueError('SH5 hand build changed; rebuild with --rebuild before operation')
    print('[SH5 HANDS BUILD] Verified', binary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-only', action='store_true')
    parser.add_argument('--rebuild', action='store_true')
    args = parser.parse_args()
    if args.verify_only or ((ROOT / 'hands_manifest.json').exists() and not args.rebuild):
        verify()
        return
    source = patched_source((SOURCE / CPP).read_text())
    header = patch_header((SOURCE / HEADER).read_text())
    destination = ROOT / 'src/cyclo_motion_controller_ros'
    if not destination.exists():
        shutil.copytree(SOURCE, destination, ignore=shutil.ignore_patterns('.git', '__pycache__'))
    (destination / CPP).write_text(source)
    (destination / HEADER).write_text(header)
    subprocess.run(['colcon', 'build', '--packages-select', 'cyclo_motion_controller_ros',
                    '--cmake-target', 'vr_controller_node', '--executor', 'sequential',
                    '--cmake-args', '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF'],
                   cwd=ROOT, env={**os.environ, 'CMAKE_BUILD_PARALLEL_LEVEL': '2'}, check=True)
    binary = ROOT / 'build/cyclo_motion_controller_ros/vr_controller_node'
    (ROOT / 'hands_manifest.json').write_text(json.dumps({
        'source_sha256': sha(SOURCE / CPP), 'header_source_sha256': sha(SOURCE / HEADER),
        'patched_sha256': sha(destination / CPP), 'header_patched_sha256': sha(destination / HEADER),
        'binary_sha256': sha(binary), 'robot_commands_sent': False,
    }, indent=2) + '\n')
    verify()


if __name__ == '__main__':
    main()
