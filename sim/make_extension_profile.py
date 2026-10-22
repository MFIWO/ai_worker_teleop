#!/usr/bin/env python3
"""[ADDED] Create a SIM-ONLY near-extension profile without editing installed files.

H1_2/H2 use wrist-pose IK; they do not command a human elbow angle directly.
This profile preserves Cyclo's IK and collision checks, sets elbow cost to zero,
and changes ONLY the two joint4 upper bounds in a separate controller URDF.
The preserved wrist-centric velocity QP is not identical to H1_2's nonlinear
IK. An optional separately built precision core tightens its numeric solve.
It is not a physical robot joint-limit recommendation.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml

Q4_UPPER = -0.45  # [EXPERIMENT] -25.78deg, before link2->4->5 collinearity at -.40456.
SHELL_FRACTION = 0.995  # [EXPERIMENT] was 0.94; keep a finite reach margin.


def make_urdf(text, upper=Q4_UPPER):
    if not math.isfinite(upper) or not -0.70 < upper <= Q4_UPPER:
        raise ValueError('Simulation experiment requires -0.70 < joint4 upper <= -0.45')
    before = ET.fromstring(text)
    for side in ('l', 'r'):
        name = f'arm_{side}_joint4'
        joint = before.find(f"joint[@name='{name}']")
        if joint is None or joint.attrib.get('type') != 'revolute':
            raise ValueError('Expected SG2 revolute ' + name)
        axis = joint.find('axis')
        limit = joint.find('limit')
        if axis is None or [float(x) for x in axis.attrib['xyz'].split()] != [0., 1., 0.]:
            raise ValueError('Unexpected SG2 elbow axis for ' + name)
        if limit is None or float(limit.attrib['upper']) != -0.7:
            raise ValueError('Expected ORIGINAL Cyclo upper=-0.7 for ' + name)
        pattern = r'(<joint\b[^>]*\bname="' + name + r'"[^>]*>)(.*?)(</joint>)'
        count = 0
        def replace_joint(match):
            nonlocal count
            count += 1
            body, n = re.subn(r'(<limit\b[^>]*\bupper=")[^"]+("[^>]*/>)',
                              lambda m: m[1] + str(upper) + m[2], match[2])
            if n != 1:
                raise ValueError('Expected one elbow limit tag')
            return match[1] + body + match[3]
        text = re.sub(pattern, replace_joint, text, flags=re.S)
        if count != 1:
            raise ValueError('Ambiguous/missing XML joint ' + name)
    after = ET.fromstring(text)
    expected = deepcopy(before)
    for side in ('l', 'r'):
        expected.find(f"joint[@name='arm_{side}_joint4']/limit").set('upper', str(upper))
    if ET.tostring(expected) != ET.tostring(after):
        raise ValueError('Unexpected non-elbow model change')
    return text


def make_config(original):
    result = deepcopy(original)
    p = result['vr_controller']['ros__parameters']
    if not isinstance(p['weight_elbow_position'], (int, float)):
        raise ValueError('Missing valid elbow weight')
    p['weight_elbow_position'] = 0.0
    if float(p['weight_position']) != 10.0:
        raise ValueError('Expected original wrist position weight10; inspect this installation')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument('--config', type=Path)
    args = parser.parse_args()
    if os.getenv('ROS_DOMAIN_ID') not in ('231', '232'):
        parser.error('Use simulation ROS_DOMAIN_ID=231 (232 for tests), not physical domain30')
    from ament_index_python.packages import get_package_share_directory
    urdf = (args.urdf or Path(get_package_share_directory('cyclo_motion_controller_models')) /
            'models/ai_worker/ffw_sg2_follower.urdf').resolve()
    config = (args.config or Path(get_package_share_directory('cyclo_motion_controller_ros')) /
              'config/ai_worker_config.yaml').resolve()
    source_bytes, config_bytes = urdf.read_bytes(), config.read_bytes()
    out = args.output_dir.resolve()
    dest_urdf, dest_yaml = out/'sg2_near_extension.urdf', out/'near_extension.yaml'
    if dest_urdf == urdf or dest_yaml == config:
        raise ValueError('Refusing to overwrite installed source')
    model_text = make_urdf(source_bytes.decode())
    config_text = yaml.safe_dump(make_config(yaml.safe_load(config_bytes)), sort_keys=False)
    manifest = {
        'simulation_only': True, 'source_urdf': str(urdf), 'source_config': str(config),
        'source_urdf_sha256': hashlib.sha256(source_bytes).hexdigest(),
        'source_config_sha256': hashlib.sha256(config_bytes).hexdigest(),
        'output_urdf_sha256': hashlib.sha256(model_text.encode()).hexdigest(),
        'joint4_upper_rad': Q4_UPPER, 'shell_fraction': SHELL_FRACTION,
        'precision_overlay_required': True,
        'changed_fields': ['arm_l_joint4.limit.upper', 'arm_r_joint4.limit.upper',
                           'vr_controller.ros__parameters.weight_elbow_position'],
    }
    payloads = {dest_urdf: model_text, dest_yaml: config_text,
                out/'near_extension_manifest.json': json.dumps(manifest, indent=2)+'\n'}
    # Preflight all outputs; preserve any independently modified experiment.
    for p, content in payloads.items():
        if p.exists() and p.read_text() != content:
            raise FileExistsError(f'Different existing output, use a new directory: {p}')
    out.mkdir(parents=True, exist_ok=True)
    for p, content in payloads.items():
        if not p.exists():
            with p.open('x') as file:
                file.write(content)
    if urdf.read_bytes() != source_bytes or config.read_bytes() != config_bytes:
        raise RuntimeError('Original source changed during profile creation')
    print('[PROFILE]', json.dumps(manifest), flush=True)
    print('[RUN] config_file:='+str(dest_yaml)+' follower_urdf_path:='+str(dest_urdf))


if __name__ == '__main__':
    main()
