"""Build a separate hardware VR node with measured SH5 fingers; no robot I/O."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

from hardware.build_startup_timing import SOURCE, CPP, HEADER, patch_source, patch_header as timing_patch_header, sha

ROOT = Path('/workspace/ai_worker_xr_sh5_hands')
CORE = SOURCE.parent / 'cyclo_motion_controller_core'
CORE_CPP = Path('src/controllers/ai_worker/vr_controller.cpp')
SPARSE = Path(__file__).with_name('sh5_sparse_qp.cpp')
KIN_CPP = Path('src/kinematics/kinematics_solver.cpp')
KIN_HEADER = Path('include/cyclo_motion_controller_core/kinematics/kinematics_solver.hpp')


def write_changed(path, text):
    if not path.exists() or path.read_text() != text:
        path.write_text(text)


def sparse_core_source():
    text = (CORE / CORE_CPP).read_text()
    start = text.index('bool VRController::getOptJointVel(')
    end = text.index('void VRController::setWeight(', start)
    return text[:start] + SPARSE.read_text() + '\n\n' + text[end:]


def sparse_kinematics():
    header = (CORE / KIN_HEADER).read_text()
    header = '#include <limits>\n' + replace_once(header,
        '    const bool verbose);',
        '    const bool verbose,\n    const double gradient_distance_limit = std::numeric_limits<double>::infinity());')
    source = replace_once((CORE / KIN_CPP).read_text(),
        '  const bool verbose)\n{', '  const bool verbose,\n  const double gradient_distance_limit)\n{')
    source = replace_once(source,
        '      const auto & dist_res = geom_data_.distanceResults[idx];\n      const Eigen::Vector3d pA',
        '      const auto & dist_res = geom_data_.distanceResults[idx];\n'
        '      // All distances were computed above. Only bounded QP rows need a gradient.\n'
        '      if (dist_res.min_distance > gradient_distance_limit) {continue;}\n'
        '      const Eigen::Vector3d pA')
    return source, header


def prepare_sparse_core(destination):
    core = destination.parent / 'cyclo_motion_controller_core'
    if not core.exists():
        shutil.copytree(CORE, core, ignore=shutil.ignore_patterns('.git', '__pycache__'))
    write_changed(core / CORE_CPP, sparse_core_source())
    kin_source, kin_header = sparse_kinematics()
    write_changed(core / KIN_CPP, kin_source)
    write_changed(core / KIN_HEADER, kin_header)
    cmake = (SOURCE / 'CMakeLists.txt').read_text()
    cmake = cmake.replace('target_link_libraries(vr_controller_node PRIVATE cyclo_motion_controller_core::cyclo_motion_controller_core)',
                          'target_link_libraries(vr_controller_node PRIVATE sh5_vr_core)')
    cmake = cmake.replace('target_link_libraries(vr_controller_node PRIVATE ${CYCLO_MOTION_CONTROLLER_CORE_LIB})',
                          'target_link_libraries(vr_controller_node PRIVATE sh5_vr_core)')
    anchor = '# Link cyclo_motion_controller_core'
    cmake = replace_once(cmake, anchor, '''add_library(sh5_vr_core STATIC
  ../cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp
  ../cyclo_motion_controller_core/src/controllers/ai_worker/vr_controller.cpp)
target_include_directories(sh5_vr_core PUBLIC ../cyclo_motion_controller_core/include/cyclo_motion_controller_core)
target_link_libraries(sh5_vr_core PUBLIC Eigen3::Eigen pinocchio::pinocchio OsqpEigen::OsqpEigen)
''' + anchor)
    write_changed(destination / 'CMakeLists.txt', cmake)
    return core


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Installed Cyclo source differs at: ' + old[:100])
    return text.replace(old, new)


def patch_header(text):
    text = timing_patch_header(text)
    text = '#include <atomic>\n' + text
    return replace_once(text, '  double weight_elbow_position_;',
        '  double weight_elbow_position_;\n'
        '  std::atomic_bool body_elbows_relaxed_{false};\n'
        '  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr body_elbow_sub_;')


def patched_source(text):
    text = patch_source(text)  # Exact same timing changes as the existing arm node.
    anchor = '  left_arm_joints_.clear();'
    text = replace_once(text, anchor, '''  int measured_fingers = 0;
  int measured_head = 0;
  int measured_wheels = 0;
  for (size_t i = 0; i < model_joint_names_.size(); ++i) {
    const bool finger = model_joint_names_[i].rfind("finger_", 0) == 0;
    const bool head = model_joint_names_[i] == "head_joint1" || model_joint_names_[i] == "head_joint2";
    const bool wheel = model_joint_names_[i] == "left_wheel_drive" || model_joint_names_[i] == "right_wheel_drive" ||
      model_joint_names_[i] == "rear_wheel_drive" || model_joint_names_[i] == "left_wheel_steer" ||
      model_joint_names_[i] == "right_wheel_steer" || model_joint_names_[i] == "rear_wheel_steer";
    if (finger || head || wheel) {
      if (!kinematics_solver_->setJointVelocityBoundsByIndex(static_cast<int>(i), 0.0, 0.0)) {
        throw std::runtime_error("Cannot exclude SH5 fingers from arm QP outputs");
      }
      if (finger) ++measured_fingers;
      if (head) ++measured_head;
      if (wheel) ++measured_wheels;
    }
  }
  if (measured_fingers != 40 || (measured_head != 0 && measured_head != 2) ||
      (measured_wheels != 0 && measured_wheels != 6) || (measured_wheels && measured_head != 2) ||
      model_joint_names_.size() != static_cast<size_t>(55 + measured_head + measured_wheels)) {
    throw std::runtime_error("SH5 requires 14 arms + lift + 40 fingers, optional 2 head and 6 measured wheel joints");
  }
''' + anchor)
    anchor = '    // If lift is commanded by another node, use measured lift state for'
    text = replace_once(text, anchor, '''    // Hand retargeting owns fingers. Use their actual positions for collision FK.
    for (size_t i = 0; i < model_joint_names_.size(); ++i) {
      if (model_joint_names_[i].rfind("finger_", 0) == 0 ||
          model_joint_names_[i] == "head_joint1" || model_joint_names_[i] == "head_joint2" ||
          model_joint_names_[i].find("_wheel_") != std::string::npos) {
        q_feedback[static_cast<int>(i)] = q_[static_cast<int>(i)];
      }
    }
''' + anchor)
    text = replace_once(text, '  left_arm_joints_.clear();',
        '  body_elbow_sub_ = this->create_subscription<std_msgs::msg::Bool>(\n'
        '    "/xr_hardware/body_elbows_relaxed", 1,\n'
        '    [this](std_msgs::msg::Bool::SharedPtr msg) {\n'
        '      body_elbows_relaxed_.store(msg->data);\n'
        '    });\n'
        '  left_arm_joints_.clear();')
    for side in ('right','left'):
        text = replace_once(text,
            f'    weight_{side}_elbow.head(3).setConstant(weight_elbow_position_);',
            f'    weight_{side}_elbow.head(3).setConstant(\n'
            '      body_elbows_relaxed_.load() ? 0.05 : weight_elbow_position_);')
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
        'sparse_template_sha256': sha(SPARSE),
        'core_source_sha256': sha(CORE / CORE_CPP),
        'sparse_core_sha256': hashlib.sha256(sparse_core_source().encode()).hexdigest(),
    }
    for key, path, patched in zip(('kinematics', 'kinematics_header'), (KIN_CPP, KIN_HEADER), sparse_kinematics()):
        expected[key+'_source_sha256'] = sha(CORE / path)
        expected[key+'_patched_sha256'] = hashlib.sha256(patched.encode()).hexdigest()
        if sha(destination.parent / 'cyclo_motion_controller_core' / path) != expected[key+'_patched_sha256']:
            raise ValueError('SH5 kinematics build copy changed; rebuild before operation')
    if (any(manifest.get(k) != v for k, v in expected.items()) or
            sha(destination / CPP) != expected['patched_sha256'] or
            sha(destination / HEADER) != expected['header_patched_sha256']):
        raise ValueError('SH5 hand build changed; rebuild with --rebuild before operation')
    if sha(destination.parent / 'cyclo_motion_controller_core' / CORE_CPP) != expected['sparse_core_sha256']:
        raise ValueError('SH5 sparse QP build copy changed; rebuild before operation')
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
    write_changed(destination / CPP, source)
    write_changed(destination / HEADER, header)
    core = prepare_sparse_core(destination)
    # Robot wall time can lag existing object mtimes. Explicit rebuild must
    # compile the changed source even after a backward clock correction.
    build_dir = ROOT / 'build/cyclo_motion_controller_ros'
    if args.rebuild and (build_dir / 'CMakeCache.txt').exists():
        subprocess.run(['cmake','--build',str(build_dir),'--target','clean'],check=True)
    subprocess.run(['colcon', 'build', '--packages-select', 'cyclo_motion_controller_ros',
                    '--cmake-target', 'vr_controller_node', '--executor', 'sequential',
                    '--cmake-args', '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF'],
                   cwd=ROOT, env={**os.environ, 'CMAKE_BUILD_PARALLEL_LEVEL': '2'}, check=True)
    binary = ROOT / 'build/cyclo_motion_controller_ros/vr_controller_node'
    manifest = {
        'source_sha256': sha(SOURCE / CPP), 'header_source_sha256': sha(SOURCE / HEADER),
        'patched_sha256': sha(destination / CPP), 'header_patched_sha256': sha(destination / HEADER),
        'binary_sha256': sha(binary), 'robot_commands_sent': False,
        'sparse_template_sha256': sha(SPARSE), 'core_source_sha256': sha(CORE / CORE_CPP),
        'sparse_core_sha256': sha(core / CORE_CPP),
    }
    for key, path in zip(('kinematics', 'kinematics_header'), (KIN_CPP, KIN_HEADER)):
        manifest[key+'_source_sha256'] = sha(CORE / path)
        manifest[key+'_patched_sha256'] = sha(core / path)
    (ROOT / 'hands_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    verify()


if __name__ == '__main__':
    main()
