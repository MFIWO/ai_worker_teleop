#!/usr/bin/env python3
"""Build an isolated VR node with steady elapsed timing; no robot I/O."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

SOURCE = Path('/root/ros2_ws/src/cyclo_control/cyclo_motion_controller_ros')
ROOT = Path('/workspace/ai_worker_xr_startup_steady')
CPP = Path('src/nodes/ai_worker/vr_controller_node.cpp')
HEADER = Path('include/cyclo_motion_controller_ros/nodes/ai_worker/vr_controller_node.hpp')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def patch_source(text):
    replacements = {
        '  activate_start_ = this->get_clock()->now();':
        '  this->declare_parameter("activation_delay_seconds", 3.0);\n'
        '  this->declare_parameter("startup_ramp_seconds", 8.0);\n'
        '  activate_start_ = this->get_clock()->now();',
        'if (elapsed.seconds() >= 3.0)':
        'if (elapsed.seconds() >= this->get_parameter("activation_delay_seconds").as_double())',
        'double slow_start_duration = 8.0;':
        'double slow_start_duration = this->get_parameter("startup_ramp_seconds").as_double();',
        'activate_elapsed.seconds() - 3.0':
        'activate_elapsed.seconds() - this->get_parameter("activation_delay_seconds").as_double()',
    }
    for old, new in replacements.items():
        if old.startswith('  activate_start_'):
            if old not in text:
                raise ValueError('Missing constructor timing anchor')
            text = text.replace(old, new, 1)
        else:
            if text.count(old) != 1:
                raise ValueError('Unexpected Cyclo timing source: ' + old)
            text = text.replace(old, new)
    # Receipt ages and activation intervals must not follow ROS/system clock steps.
    # Keep this->now() for published message stamps.
    replacements = {
        'this->get_clock()->now()': 4,
        'this->now()': 9,
    }
    for expression, count in replacements.items():
        if text.count(expression) != count:
            raise ValueError('Unexpected Cyclo clock source: ' + expression)
    text = text.replace('this->get_clock()->now()', 'control_clock_.now()')
    for name, count in (('last_joint_state_time_', 2),
                        ('last_right_raw_traj_time_', 2),
                        ('last_left_raw_traj_time_', 2)):
        old = name + ' = this->now();'
        if text.count(old) != count:
            raise ValueError('Unexpected receipt clock assignment: ' + name)
        text = text.replace(old, name + ' = control_clock_.now();')
    text = text.replace('(this->now() - last_joint_state_time_)',
                        '(control_clock_.now() - last_joint_state_time_)')
    anchor = '        // Always compute & publish current gripper poses from measured state.'
    if text.count(anchor) != 1:
        raise ValueError('Unexpected Cyclo feedback hold anchor')
    text = text.replace(anchor, '''  // A short feedback gap pauses computation without disabling/re-arming.
  // The hardware gate holds the last output; timeout above remains authoritative.
  if ((control_clock_.now() - last_joint_state_time_).seconds() > 0.5) {
    syncCommandStateToFeedback();
    return;
  }

''' + anchor)
    return text


def patch_header(text):
    anchor = '  rclcpp::Time activate_start_;'
    if text.count(anchor) != 1:
        raise ValueError('Unexpected Cyclo activation member')
    return text.replace(anchor, '  rclcpp::Clock control_clock_{RCL_STEADY_TIME};\n' + anchor)


def verify(root):
    manifest = json.loads((root / 'timing_manifest.json').read_text())
    if manifest.get('elapsed_clock') != 'RCL_STEADY_TIME':
        raise ValueError('Rebuild the hardware node with steady elapsed timing')
    if manifest.get('feedback_hold_seconds') != .5:
        raise ValueError('Rebuild the hardware node with feedback wait support')
    binary = root / 'build/cyclo_motion_controller_ros/vr_controller_node'
    if sha(binary) != manifest['binary_sha256']:
        raise ValueError('Startup node binary hash mismatch')
    if sha(SOURCE / CPP) != manifest['source_sha256']:
        raise ValueError('Original VR source changed; rebuild timing overlay')
    if sha(root / 'src/cyclo_motion_controller_ros' / CPP) != manifest['patched_sha256']:
        raise ValueError('Timing overlay source changed')
    expected = hashlib.sha256(patch_source((SOURCE / CPP).read_text()).encode()).hexdigest()
    if manifest['patched_sha256'] != expected:
        raise ValueError('Timing patch changed; rebuild before operation')
    if sha(SOURCE / HEADER) != manifest['header_source_sha256']:
        raise ValueError('Original VR header changed; rebuild timing overlay')
    if sha(root / 'src/cyclo_motion_controller_ros' / HEADER) != manifest['header_patched_sha256']:
        raise ValueError('Steady clock header changed')
    print('[STARTUP TIMING] Verified configurable hardware VR node:', binary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-only', action='store_true')
    parser.add_argument('--rebuild', action='store_true')
    args = parser.parse_args()
    if args.verify_only or ((ROOT / 'timing_manifest.json').exists() and not args.rebuild):
        verify(ROOT)
        return
    patched = patch_source((SOURCE / CPP).read_text())
    patched_header = patch_header((SOURCE / HEADER).read_text())
    destination = ROOT / 'src/cyclo_motion_controller_ros'
    if not destination.exists():
        shutil.copytree(SOURCE, destination)
    (destination / CPP).write_text(patched)
    (destination / HEADER).write_text(patched_header)
    subprocess.run(['colcon', 'build', '--packages-select', 'cyclo_motion_controller_ros',
                    '--cmake-target', 'vr_controller_node', '--executor', 'sequential',
                    '--parallel-workers', '1', '--cmake-args', '-DCMAKE_BUILD_TYPE=Release',
                    '-DBUILD_TESTING=OFF'], cwd=ROOT, check=True)
    binary = ROOT / 'build/cyclo_motion_controller_ros/vr_controller_node'
    (ROOT / 'timing_manifest.json').write_text(json.dumps({
        'source_sha256': sha(SOURCE / CPP), 'patched_sha256': sha(destination / CPP),
        'binary_sha256': sha(binary), 'robot_commands_sent': False,
        'elapsed_clock': 'RCL_STEADY_TIME',
        'feedback_hold_seconds': .5,
        'header_source_sha256': sha(SOURCE / HEADER),
        'header_patched_sha256': sha(destination / HEADER),
    }, indent=2) + '\n')
    verify(ROOT)


if __name__ == '__main__':
    main()
