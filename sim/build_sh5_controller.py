"""Build a separate Cyclo executable for measured, externally driven SH5 fingers.

The arm QP and wrist objectives are unchanged. All 40 finger positions enter
collision FK from measured feedback; the arm QP cannot command those joints.
This executable refuses any environment except FastDDS simulation domain 232.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

from hardware.build_startup_timing import SOURCE, CPP, HEADER, patch_source, patch_header

ROOT = Path('/workspace/sh5_hybrid_sim_build')


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Installed Cyclo source differs at: ' + old[:100])
    return text.replace(old, new)


def patched_source(text):
    text = patch_source(text)
    text = replace_once(text, '#include <cmath>', '#include <cmath>\n#include <cstdlib>')
    text = replace_once(text, '  dt_ = time_step_;', '''  const char *domain = std::getenv("ROS_DOMAIN_ID");
  const char *mode = std::getenv("SH5_SIM_ONLY");
  const char *rmw = std::getenv("RMW_IMPLEMENTATION");
  if (!domain || std::string(domain) != "232" || !mode || std::string(mode) != "1" ||
      !rmw || std::string(rmw) != "rmw_fastrtps_cpp") {
    throw std::runtime_error("SH5 simulation executable: FastDDS/domain232/SH5_SIM_ONLY=1 required");
  }
  dt_ = time_step_;''')
    anchor = '  left_arm_joints_.clear();'
    text = replace_once(text, anchor, '''  int measured_fingers = 0;
  for (size_t i = 0; i < model_joint_names_.size(); ++i) {
    if (model_joint_names_[i].rfind("arm_", 0) != 0 && model_joint_names_[i] != "lift_joint") {
      if (!kinematics_solver_->setJointVelocityBoundsByIndex(static_cast<int>(i), 0.0, 0.0)) {
        throw std::runtime_error("Cannot lock SH5 fingers out of arm QP");
      }
      if (model_joint_names_[i].rfind("finger_", 0) == 0) { ++measured_fingers; }
    }
  }
  if (measured_fingers != 40) {
    throw std::runtime_error("SH5 simulation requires all 40 measured finger joints in model");
  }
''' + anchor)
    anchor = '    // If lift is commanded by another node, use measured lift state for'
    text = replace_once(text, anchor, '''    // SH5 fingers are actuated by ROBOTIS hand retargeting, never by this QP.
    // Refresh their collision geometry every cycle from actual /joint_states.
    for (size_t i = 0; i < model_joint_names_.size(); ++i) {
      if (model_joint_names_[i].rfind("arm_", 0) != 0) {
        q_feedback[static_cast<int>(i)] = q_[static_cast<int>(i)];
      }
    }
''' + anchor)
    return text


def main():
    if os.environ.get('SH5_SIM_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '232':
        raise SystemExit('SH5 simulation setup only')
    source = patched_source((SOURCE / CPP).read_text())
    header = patch_header((SOURCE / HEADER).read_text())
    digest = hashlib.sha256((source + header).encode()).hexdigest()
    binary = ROOT / 'build/cyclo_motion_controller_ros/vr_controller_node'
    manifest = ROOT / 'sh5_sim_build.json'
    if manifest.exists() and binary.exists():
        previous = json.loads(manifest.read_text())
        if (previous.get('source_sha256') == digest and previous.get('binary_sha256') ==
                hashlib.sha256(binary.read_bytes()).hexdigest()):
            print('[SH5 SIM BUILD] Verified existing executable', binary, flush=True)
            return
    destination = ROOT / 'src/cyclo_motion_controller_ros'
    if not destination.exists():
        shutil.copytree(SOURCE, destination, ignore=shutil.ignore_patterns('.git', '__pycache__'))
    (destination / CPP).write_text(source)
    (destination / HEADER).write_text(header)
    subprocess.run(['colcon', 'build', '--packages-select', 'cyclo_motion_controller_ros',
                    '--cmake-target', 'vr_controller_node', '--executor', 'sequential',
                    '--cmake-args', '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF'],
                   cwd=ROOT, env={**os.environ, 'CMAKE_BUILD_PARALLEL_LEVEL': '2'}, check=True)
    manifest.write_text(json.dumps({'source_sha256': digest,
        'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
        'simulation_only': True, 'hardware_collision_certification': False}, indent=2))
    print('[SH5 SIM BUILD] Complete', binary, flush=True)


if __name__ == '__main__':
    main()
