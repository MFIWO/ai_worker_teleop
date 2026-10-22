"""Capture PhysX feedback and generate a measured-joint SH5 simulation control model."""
from copy import deepcopy
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory as share
import yaml

from hardware.sh5_model import build_sh5_model, _collision_srdf, _model
from hardware.make_profile import resolve_meshes
from hardware.collision_primitives import enclose_collision_meshes

PROFILE = Path('/workspace/sh5_hybrid_sim_profile')
FINGERS = {f'finger_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 21)}


def measured_names(names, positions):
    """Explicit USD -> expanded-URDF aliases for the six swerve joints."""
    result = dict(zip(names, positions))
    for side in ('left', 'right', 'rear'):
        for kind in ('steer', 'drive'):
            target = f'{side}_wheel_{kind}'
            source = target + '_joint'
            if target not in result and source in result:
                result[target] = result[source]
    return result


def require_sim():
    if any(os.environ.get(k) != v for k, v in {
            'ROS_DOMAIN_ID': '232', 'SH5_SIM_ONLY': '1',
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}.items()):
        raise RuntimeError('Requires SH5_SIM_ONLY=1 and FastDDS domain232')


def make_model(description, control, srdf, snapshot):
    reduced, _, frozen, report = build_sh5_model(description, control, srdf, snapshot)
    original = ET.fromstring(description)
    tree = ET.fromstring(reduced)
    # Restore external-joint kinematics. Unlike the hardware arms-only profile,
    # fingers/head/wheels stay movable in FK and receive live measured positions.
    for joint in list(tree.findall('joint')):
        name = joint.get('name')
        if name in frozen:
            restored = deepcopy(original.find(f"joint[@name='{name}']"))
            # Pinocchio's scalar QP requires nq==nv. Represent unwrapped wheel
            # angle as a measured scalar, locked out of the arm optimizer.
            if restored.get('type') == 'continuous':
                restored.set('type', 'revolute')
                limit = restored.find('limit')
                if limit is None:
                    limit = ET.SubElement(restored, 'limit')
                limit.set('lower', '-1000000')
                limit.set('upper', '1000000')
                limit.set('velocity', limit.get('velocity', '1'))
                limit.set('effort', limit.get('effort', '1'))
            tree.remove(joint)
            tree.append(restored)
            del frozen[name]
    text = ET.tostring(tree, encoding='unicode')
    root, links, joints = _model(text, 'SH5 simulation')
    collision_pairs, _ = _collision_srdf(root, links, joints, srdf)
    text, approximation = enclose_collision_meshes(resolve_meshes(text))
    return text, ET.tostring(collision_pairs, encoding='unicode'), frozen, report, approximation


def main():
    require_sim()
    desc_path = Path(share('ffw_description')) / 'urdf/ffw_sh5_rev1_follower/ffw_sh5_follower.urdf'
    description = desc_path.read_text()
    names = {j.get('name') for j in ET.fromstring(description).findall('joint') if j.get('type') != 'fixed'}
    rclpy.init(args=[])
    node = rclpy.create_node('sh5_sim_capture')
    sample = {}
    def receive(msg):
        values = measured_names(msg.name, msg.position)
        if (len(msg.name) == len(msg.position) and len(set(msg.name)) == len(msg.name)
                and names.issubset(values) and np.isfinite(msg.position).all()):
            sample.clear()
            sample.update(values)
    node.create_subscription(JointState, '/joint_states', receive,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    end = time.monotonic() + 20
    while not sample and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    node.destroy_node()
    rclpy.shutdown()
    if not sample:
        raise RuntimeError('No complete measured SH5 /joint_states in domain232; start simulator first')
    models = Path(share('cyclo_motion_controller_models')) / 'models/ai_worker'
    text, srdf, frozen, report, approximation = make_model(description,
        (models / 'ffw_sg2_follower.urdf').read_text(),
        (models / 'ffw_sg2_follower_default.srdf').read_text(), sample)
    for name, (lower, upper) in report['arm_limits'].items():
        if not lower <= sample[name] <= upper:
            raise ValueError(f'Simulator {name}={sample[name]:.4f} outside original arm limits [{lower}, {upper}]')
    PROFILE.mkdir(parents=True, exist_ok=True)
    urdf_path, srdf_path = PROFILE / 'sh5_sim_dynamic.urdf', PROFILE / 'sh5_sim_dynamic.srdf'
    urdf_path.write_text(text)
    srdf_path.write_text(srdf)
    config = yaml.safe_load((Path(share('cyclo_motion_controller_ros')) / 'config/ai_worker_config.yaml').read_text())
    p = config['vr_controller']['ros__parameters']
    p.update(urdf_path=str(urdf_path), srdf_path=str(srdf_path), weight_elbow_position=0.0,
             joint_states_topic='/sh5_sim/controller_joint_states',
             right_gripper_joint='__unused_right_gripper__', left_gripper_joint='__unused_left_gripper__',
             right_raw_traj_topic='/sh5_sim/unused/right_raw', left_raw_traj_topic='/sh5_sim/unused/left_raw',
             lift_topic='/sh5_sim/unused/lift', activation_delay_seconds=.1, startup_ramp_seconds=.1,
             joint_state_timeout=5.0)
    # Private output permits a single simulation relay to hold both arms and hands
    # on tracking/feedback loss, without touching the real-robot command gate.
    for side in ('left', 'right'):
        p[f'{side}_traj_topic'] = f'/sh5_sim/raw/{side}_arm'
    (PROFILE / 'controller.yaml').write_text(yaml.safe_dump({
        k: config[k] for k in ('vr_controller', 'reference_checker')}, sort_keys=False))
    (PROFILE / 'snapshot.json').write_text(json.dumps(sample, indent=2))
    (PROFILE / 'profile.json').write_text(json.dumps({
        'simulation_only': True, 'urdf': str(urdf_path), 'srdf': str(srdf_path),
        'config': str(PROFILE / 'controller.yaml'), 'frozen_joints': frozen,
        'measured_finger_joints': sorted(FINGERS), 'arm_limits': report['arm_limits'],
        'arm_velocity_limits': report['arm_velocity_limits'],
        'arm_limit_policy': report['arm_limit_policy'],
        'collision_geometry': approximation['approximation'],
        'dynamic_finger_collision_verified': False,
        'note': 'Measured finger FK in native CBF; no predictive finger collision or hardware certification.'
    }, indent=2))
    print('[SH5 SIM PROFILE] Physical URDF arm limits; old 0.35 rad/s cap removed.', flush=True)
    print('[SH5 SIM ARM LIMITS] ' + json.dumps({
        'position': report['arm_limits'], 'velocity': report['arm_velocity_limits']}, sort_keys=True), flush=True)


if __name__ == '__main__':
    main()
