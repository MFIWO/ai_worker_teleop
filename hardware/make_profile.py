#!/usr/bin/env python3
"""Generate a model-checked hardware copy bound to URDF limits. No robot commands."""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from hardware.xr_contract import (
    ARM_NAMES, SIDES, GOAL_SPEED, ANGULAR_SPEED,
    arm_signature, arm_limits, COMMAND_WINDOW_SECONDS, private_topic, sha256, verify_description, configured_arm_ids,
    READINESS_POLICIES,
)


def resolve_meshes(text):
    """[ADDED] Resolve resources on the ROBOT, not an absolute path from the PC."""
    from ament_index_python.packages import get_package_share_directory
    tree = ET.fromstring(text)
    # Cyclo builds the COLLISION model only. Installed legacy SG2 visual URIs
    # can reference an older mesh layout; they are not collision resources and
    # must neither be guessed nor substituted for real SH5 collision geometry.
    for mesh in tree.findall('.//collision/geometry/mesh'):
        uri = mesh.get('filename', '')
        if uri.startswith('package://'):
            package, relative = uri[len('package://'):].split('/', 1)
            path = Path(get_package_share_directory(package)) / relative
        elif uri.startswith('file://'):
            path = Path(uri[len('file://'):])
        else:
            path = Path(uri)
        if not path.is_absolute() or not path.is_file():
            raise ValueError(f'missing/unresolved robot mesh resource: {uri}')
        mesh.set('filename', str(path.resolve()))
    return ET.tostring(tree, encoding='unicode')


def make_files(model, controller_text, description_text, config):
    source = ET.fromstring(controller_text)
    physical = ET.fromstring(description_text)
    arm_limits(description_text)
    signature = arm_signature(source)
    verify_description(description_text, model, signature)
    tree = deepcopy(source)
    limits = {}
    for names in ARM_NAMES.values():
        for name in names:
            bound = tree.find(f"joint[@name='{name}']/limit")
            real_bound = physical.find(f"joint[@name='{name}']/limit")
            if real_bound is None:
                raise ValueError(f'{name}: physical URDF limit missing')
            # Both tool variants use the measured physical description's arm
            # position/velocity limits, not the legacy Cyclo trial restriction.
            lower, upper = float(real_bound.get('lower')), float(real_bound.get('upper'))
            velocity = float(real_bound.get('velocity'))
            if not lower < upper or velocity <= 0:
                raise ValueError(f'{name}: invalid physical URDF bounds')
            bound.set('lower', str(lower))
            bound.set('upper', str(upper))
            bound.set('velocity', str(velocity))
            limits[name] = [lower, upper]
    output = deepcopy(config)
    p = output['vr_controller']['ros__parameters']
    if p.get('lift_vel_bound', 0.0) != 0.0:
        raise ValueError('hardware arms-only commissioning requires the installed lift velocity bound0')
    # [CHANGED] Same wrist-centric retarget; untouched collision/CBF/startup parameters.
    p['weight_elbow_position'] = 0.0
    for s in SIDES:
        p[f'{s}_traj_topic'] = private_topic(model, s)
        p[f'{s}_raw_traj_topic'] = f'/xr_hardware/{model}/unused/{s}_gripper'
    p['lift_topic'] = f'/xr_hardware/{model}/unused/lift'
    p['joint_states_topic'] = f'/xr_hardware/{model}/joint_states'
    p['right_gripper_joint'] = '__xr_disabled_right_gripper__'
    p['left_gripper_joint'] = '__xr_disabled_left_gripper__'
    p['trajectory_time'] = 0.0
    if p.get('weight_position') != 10.0 or p.get('weight_orientation') != 1.0:
        raise ValueError('Unexpected Cyclo wrist weights; review the installed configuration')
    return ET.tostring(tree, encoding='unicode'), output, limits, signature


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True, choices=('sg2', 'sh5'))
    parser.add_argument('--hands', action='store_true', help='Enable native SH5 hands with measured finger FK')
    parser.add_argument('--manual-controls', action='store_true', help='SH5 hands: measured head FK and gated keyboard auxiliary motion')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--controller-urdf', type=Path)
    parser.add_argument('--srdf', type=Path)
    parser.add_argument('--description', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--snapshot', type=Path, required=True,
                        help='Passive measured joint snapshot. Required to freeze SH5 non-arm joints.')
    parser.add_argument('--readiness-policy', choices=READINESS_POLICIES,
                        default='strict_torque')  # [CHANGED] Normal wrapper selects driver_managed.
    a = parser.parse_args()
    if a.hands and a.model != "sh5":
        parser.error("--hands requires --model sh5")
    if a.manual_controls and not (a.hands and a.model == 'sh5'):
        parser.error('--manual-controls requires --model sh5 --hands')
    if any(p is None for p in (a.controller_urdf, a.srdf, a.description, a.config)):
        from ament_index_python.packages import get_package_share_directory as share
        models = Path(share('cyclo_motion_controller_models')) / 'models/ai_worker'
        a.controller_urdf = a.controller_urdf or models / 'ffw_sg2_follower.urdf'
        a.srdf = a.srdf or models / 'ffw_sg2_follower_default.srdf'
        a.description = a.description or Path(share('ffw_description')) / f'urdf/ffw_{a.model}_rev1_follower/ffw_{a.model}_follower.urdf'
        a.config = a.config or Path(share('cyclo_motion_controller_ros')) / 'config/ai_worker_config.yaml'
    description_text = a.description.read_text()
    motor_ids = configured_arm_ids(description_text, a.model)
    snapshot = json.loads(a.snapshot.read_text())
    urdf, cfg, limits, signature = make_files(a.model, a.controller_urdf.read_text(),
                                             description_text, yaml.safe_load(a.config.read_text()))
    frozen, model_report, srdf = {}, {}, a.srdf.read_text()
    if a.model == 'sh5':
        from hardware.sh5_model import build_sh5_model
        urdf, srdf, frozen, model_report = build_sh5_model(
            description_text, a.controller_urdf.read_text(), srdf, snapshot)
        limits = model_report['arm_limits']
        if a.hands:
            from hardware.sh5_hands_model import add_measured_fingers
            urdf, srdf, frozen, pairs = add_measured_fingers(
                urdf, description_text, a.srdf.read_text(), frozen, measured_head=a.manual_controls,
                measured_wheels=a.manual_controls)
            model_report.update(pairs, movable_dof=63 if a.manual_controls else 55, frozen_joint_count=len(frozen),
                tool_collision_coverage='SH5 geometry from measured finger positions',
                retained_movable_joints=[j.get('name') for j in ET.fromstring(urdf).findall('joint')
                                        if j.get('type') != 'fixed'])
    # [CHANGED] Diagnose wrong launch ORDER with the exact observed joint angle.
    # Do NOT loosen native limits or synthesize a corrected measured posture.
    for name, (lower, upper) in limits.items():
        if name not in snapshot:
            raise ValueError(f'{name}: no measured feedback in physical joint snapshot')
        measured = float(snapshot[name])
        if not lower <= measured <= upper:
            raise ValueError(
                f'{name}: measured {measured:+.4f} rad outside native Cyclo '
                f'[{lower:+.4f}, {upper:+.4f}] rad. '
                'SG2 must run its verified PC ready-pose trajectory FIRST '
                '(both arm joint4 near -1.57 rad), then run_controller.sh. '
                'No automatic motion or limit expansion has occurred.')
    urdf = resolve_meshes(urdf)
    manual_collision_urdf = urdf if a.manual_controls else None
    if model_report:
        model_report['mesh_resources_verified'] = True
    if a.model == 'sh5':
        # [ADDED] Exact SH5 meshes took ~2.7s per distance evaluation on the
        # checked PC. Enclosing boxes preserve coverage without a realtime hang.
        from hardware.collision_primitives import enclose_collision_meshes
        urdf, approximation = enclose_collision_meshes(urdf)
        model_report['collision_approximation'] = approximation
    paths = {'urdf': a.output / f'{a.model}_hardware_arms.urdf',
             'config': a.output / 'controller.yaml', 'srdf': a.output / 'controller.srdf'}
    contents = {'urdf': urdf, 'config': yaml.safe_dump(cfg, sort_keys=False), 'srdf': srdf}
    if a.manual_controls:
        paths['manual_collision_urdf'] = a.output / 'manual_exact_collision.urdf'
        contents['manual_collision_urdf'] = manual_collision_urdf
    # [DEBUG] Preflight every destination BEFORE any writes. Idempotent, never overwrite drift.
    for k, p in paths.items():
        if p.exists() and p.read_text() != contents[k]:
            raise ValueError(f'Unrelated/different generated file already exists: {p}')
    a.output.mkdir(parents=True, exist_ok=True)
    for k, p in paths.items():
        p.write_text(contents[k])
    from hardware.check_collision import check_initial_pose
    collision_report = check_initial_pose(paths['urdf'], paths['srdf'], snapshot,
        cfg['vr_controller']['ros__parameters']['collision_safe_distance'],
        enforce_margin=a.readiness_policy == 'strict_torque', measured_fingers=a.hands,
        measured_head=a.manual_controls, measured_wheels=a.manual_controls)
    if not collision_report['native_margin_met']:
        print('[COLLISION DIAGNOSTIC] Initial clearance is positive but below the native CBF margin. '
              'The native QP/CBF, startup tracking guard and reference checker remain unchanged; '
              'no automatic pose correction has been requested.')
    _, velocities = arm_limits(urdf)
    manifest = {'schema': 1, 'mode': 'hardware_commissioning', 'model': a.model,
                'readiness_policy': a.readiness_policy,  # [CHANGED] Explicit, carried to gate and PC.
                'validation_fixture': os.environ.get('AI_WORKER_XR_EXECUTION') == 'validation',
                'limits': limits, 'arm_signature': signature,
                'max_joint_speed': max(velocities.values()), 'goal_speed': GOAL_SPEED,
                'joint_velocity_limits': velocities,
                'tracking_window_seconds': COMMAND_WINDOW_SECONDS,
                'tracking_error_limits': {k: v * COMMAND_WINDOW_SECONDS for k, v in velocities.items()},
                'angular_speed': ANGULAR_SPEED, 'workspace_fraction': 1.0 if a.model == 'sh5' else .94,
                'arm_limit_policy': 'physical_description',
                'source_urdf': str(a.controller_urdf.resolve()),
                'source_urdf_sha256': sha256(a.controller_urdf),
                'description_sha256': sha256(a.description), 'arm_dxl_ids': motor_ids,
                'model_report': model_report,
                'initial_collision_report': collision_report,
                'tool_collision_coverage': ('SH5 collision meshes enclosed by conservative boxes; non-arm joints frozen at measured snapshot'
                                            if a.model == 'sh5' else 'installed SG2 Cyclo collision geometry'),
                'physical_motion_verified': False}
    generated = ET.fromstring(urdf)
    manifest['controller_joint_names'] = [j.get('name') for j in generated.findall('joint')
                                           if j.get('type') not in ('fixed',)]
    manifest['frozen_joints'] = frozen
    manifest['manual_auxiliary'] = bool(a.manual_controls)
    manifest['live_wheel_feedback'] = bool(a.manual_controls)
    if a.manual_controls:
        from hardware.sh5_hands_model import WHEEL_DRIVES
        manifest['continuous_sensor_joints'] = sorted(WHEEL_DRIVES)
    if a.hands:
        from hardware.sh5_hands_model import HAND_CONTROL, finger_limits
        manifest['hand_control'] = HAND_CONTROL
        manifest['finger_limits'] = finger_limits(urdf)
        manifest['tool_collision_coverage'] = 'SH5 geometry at live measured finger positions'
        manifest['hand_output'] = 'original ROBOTIS retargeting_teleop'

    for k, p in paths.items():
        manifest[k] = str(p.resolve())
        manifest[k+'_sha256'] = sha256(p)
    (a.output/'profile.json').write_text(json.dumps(manifest, indent=2))
    print('[PROFILE]', a.output/'profile.json')
    print('[PRESERVED] physical URDF position/velocity bounds, collision geometry, CBF and original source files')
    print('[READINESS POLICY]', a.readiness_policy)
    print(f'[HARDWARE] Arm velocity uses verified URDF (max {max(velocities.values()):g}rad/s); '
          f'Cartesian speed caps are non-binding ({GOAL_SPEED:g}m/s, {ANGULAR_SPEED:g}rad/s); '
          'arm QP does not command fingers/base/lift')
    if a.hands:
        print('[SH5 HANDS] Native hand retarget enabled; arm collision FK uses measured fingers.')
    elif a.model == 'sh5':
        print('[SH5] Collision meshes conservatively enclosed by boxes; visual meshes retained; non-arm joints frozen.')
        print('[LIMITATION] Not a moving-finger controller or a physical collision certification.')


if __name__ == '__main__':
    main()
