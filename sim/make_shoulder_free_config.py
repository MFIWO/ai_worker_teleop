#!/usr/bin/env python3
"""[ADDED] Create an opt-in Cyclo wrist-only task profile, not a new solver.

Only vr_controller.ros__parameters.weight_elbow_position changes to 0.0.
All gains, joint/model paths, collision and reference checks remain unchanged.
The installed source YAML is never overwritten. Restart Cyclo with config_file;
ros2 param set alone does not update the C++ constructor's cached weight.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import yaml


def shoulder_free_config(original):
    if not isinstance(original, dict):
        raise ValueError('Expected a ROS parameter mapping')
    changed = deepcopy(original)
    parameters = changed['vr_controller']['ros__parameters']
    old = parameters['weight_elbow_position']
    if not isinstance(old, (float, int)) or not math.isfinite(old) or old < 0:
        raise ValueError('Invalid original elbow task weight')
    # [CHANGED] Shoulder joints were NOT locked. Drop only the extra elbow
    # objective so the existing QP can use the full arm for wrist 6D tracking.
    parameters['weight_elbow_position'] = 0.0
    restored = deepcopy(changed)
    restored['vr_controller']['ros__parameters']['weight_elbow_position'] = old
    if restored != original:
        raise AssertionError('Unexpected change outside the elbow task weight')
    return changed


def create_profile(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output:
        raise ValueError('Refusing to overwrite installed controller settings')
    source_bytes = source.read_bytes()
    original = yaml.safe_load(source_bytes)
    modified = shoulder_free_config(original)
    header = ('# [ADDED] Opt-in SG2 XR shoulder-free profile, simulation only.\n'
              '# [CHANGED] ONLY weight_elbow_position -> 0.0; all safety settings preserved.\n'
              f'# Source: {source}\n'
              f'# Source SHA256: {hashlib.sha256(source_bytes).hexdigest()}\n')
    text = header + yaml.safe_dump(modified, sort_keys=False)
    if output.exists():
        if output.read_text() != text:
            raise ValueError(f'Refusing to replace a different existing profile: {output}')
    else:
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x') as handle:
            handle.write(text)
    if source.read_bytes() != source_bytes:
        raise RuntimeError('Installed config changed during profile generation')
    return {'output': str(output), 'source': str(source),
            'source_sha256': hashlib.sha256(source_bytes).hexdigest(),
            'changed_parameter': 'vr_controller.weight_elbow_position',
            'old': original['vr_controller']['ros__parameters']['weight_elbow_position'],
            'new': 0.0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.source is None:
        from ament_index_python.packages import get_package_share_directory
        args.source = Path(get_package_share_directory('cyclo_motion_controller_ros')) / 'config/ai_worker_config.yaml'
    print('[PROFILE]', json.dumps(create_profile(args.source, args.output)))


if __name__ == '__main__':
    main()
