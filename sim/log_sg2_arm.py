#!/usr/bin/env python3
"""[ADDED] Passive SG2 simulation arm logger; NEVER publishes robot commands.

Run in ai_worker Docker to read its actual installed Cyclo URDF limits.
Records named measured/commanded arm joints, PhysX TF vs wrist goals, and
activation/fault events. Does not infer XR clipping from post-clipping goals.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory


def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def chord_bend_deg(shoulder, elbow, point):
    """[ADDED] 0 means collinear S->E and E->P, not joint4 zero."""
    u, f = np.asarray(elbow)-np.asarray(shoulder), np.asarray(point)-np.asarray(elbow)
    if (not np.isfinite(u).all() or not np.isfinite(f).all()
            or min(np.linalg.norm(u), np.linalg.norm(f)) < 1e-5):
        raise ValueError('invalid geometric elbow segments')
    return float(np.degrees(np.arctan2(np.linalg.norm(np.cross(u, f)), np.dot(u, f))))


def pose_parts(pose):
    p, q = pose.position, pose.orientation
    xyz = np.array([p.x, p.y, p.z], dtype=float)
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if (not np.isfinite(xyz).all() or not np.isfinite(quat).all()
            or abs(np.linalg.norm(quat) - 1.0) > 0.02):
        raise ValueError('invalid pose/quaternion')
    return xyz, Rotation.from_quat(quat)


class ArmLogger(Node):
    def __init__(self, output, urdf, hz, hardware_mode=False):
        super().__init__('sg2_passive_arm_logger', enable_rosout=False,
                         start_parameter_services=False)
        self.started = time.monotonic()
        self.next_print = 0.0
        self.output = output
        self.cache = {}
        self.active = None
        self.diverged = None
        self.js_count = 0
        self.tf_valid_count = 0
        self.last_tf_error = {}
        # [ADDED] Optional fixed-resume telemetry; old XR remains supported.
        self.xr_diagnostic = None
        self.path_diagnostic = None  # [ADDED] raw/workspace/published/actual path stages.
        self.limits = {}
        tree = ET.parse(urdf)
        for side in ('l', 'r'):
            for i in range(1, 8):
                name = f'arm_{side}_joint{i}'
                entry = tree.find(f"joint[@name='{name}']/limit")
                if entry is None:
                    raise ValueError(f'Controller URDF has no limits for {name}')
                self.limits[name] = (float(entry.attrib['lower']), float(entry.attrib['upper']))
        output.mkdir(parents=True, exist_ok=False)
        metadata = {
            'urdf': str(urdf), 'urdf_sha256': hashlib.sha256(urdf.read_bytes()).hexdigest(),
            'domain': os.environ.get('ROS_DOMAIN_ID'), 'sample_hz': hz,
            'position_limits_rad': self.limits,
            'measured_source': '/joint_states and base_link -> arm_[lr]_link7 TF',
            'controller_gripper_pose_is_NOT_used_as_measured': True,
            'sampling': 'latest samples; EE errors require source stamp skew <=0.10s',
            'near_upper_diagnostic_tolerance_deg': 2.0,
            'XR_pre_projection_request_available': 'only when fresh /sg2_xr/diagnostics is received',
            'XR_clipping_cannot_be_confirmed_from_post_projection_goals_alone': True,
            'XR_optional_diagnostic_topic': '/sg2_xr/diagnostics',
            'XR_projection_columns_valid_only_with_fresh_diagnostic': True,
            'elbow_proximal_bend_definition': 'angle(link2->link4, link4->link5); 0=collinear',
            'elbow_wrist_bend_definition': 'angle(link2->link4, link4->link7); depends on wrist joints',
            'geometric_angle_is_NOT_joint4_angle': True,
            'hardware_mode': bool(hardware_mode),
            'TF_is_independent_PhysX_measurement': not hardware_mode,
            'path_diagnostics_topic': '/sg2_xr/path_diagnostics',
            'path_rows_require_fresh_active_frame': True,
        }
        if hardware_mode:
            metadata['measured_source'] = '/joint_states motor feedback + robot_state_publisher FK (not independent physical EE measurement)'
        (output / 'metadata.json').write_text(json.dumps(metadata, indent=2))
        self.csv_file = (output / 'arms.csv').open('x', newline='', buffering=1)
        self.events = (output / 'events.jsonl').open('x', buffering=1)
        fields = ['elapsed_s', 'ros_time_s', 'side', 'active', 'reference_diverged',
                  'js_source_s', 'cmd_source_s', 'goal_source_s', 'tf_source_s',
                  'js_age_s', 'cmd_age_s', 'goal_age_s', 'tf_age_s', 'goal_tf_skew_s',
                  'q4_lower_deg', 'q4_upper_deg', 'q4_margin_deg', 'cmd_q4_margin_deg',
                  'q4_cmd_minus_measured_deg', 'ee_pos_error_mm', 'ee_ori_error_deg',
                  'goal_shoulder_distance_m', 'measured_shoulder_distance_m',
                  'goal_x_m', 'goal_y_m', 'goal_z_m',
                  'actual_x_m', 'actual_y_m', 'actual_z_m', 'flags']
        fields += [f'q{i}_{kind}_deg' for i in range(1, 8) for kind in ('measured', 'cmd')]
        fields += ['xr_mode', 'xr_phase', 'xr_session_id', 'xr_diagnostic_age_s',
                   'xr_projection_mm', 'xr_raw_radius_m', 'xr_shell_max_m',
                   'xr_align_position_mm', 'xr_align_rotation_deg']
        fields += ['elbow_proximal_bend_deg', 'elbow_wrist_bend_deg', 'elbow_geometry_tf_skew_s']
        fields += ['path_mode', 'path_ros_time_s', 'path_age_s', 'path_goal_skew_s',
                   'path_anchor_displacement_m', 'path_workspace_clipped_mm',
                   'path_input_rotation_from_anchor_deg']
        fields += [f'path_{stage}_{axis}_m' for stage in ('raw','workspace','published','actual')
                   for axis in 'xyz']
        fields += [f'path_{stage}_q{axis}' for stage in ('raw','published','actual') for axis in 'xyzw']
        self.writer = csv.DictWriter(self.csv_file, fieldnames=fields)
        self.writer.writeheader()
        self.fields = fields
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.subscriptions_keep = [self.create_subscription(
            JointState, '/joint_states', lambda m: self.store('js', m), qos)]
        for side, full in (('l', 'left'), ('r', 'right')):
            self.subscriptions_keep.append(self.create_subscription(
                PoseStamped, f'/{side}_goal_pose', lambda m, s=side: self.store(s+'goal', m), qos))
            self.subscriptions_keep.append(self.create_subscription(
                JointTrajectory, f'/leader/joint_trajectory_command_broadcaster_{full}/joint_trajectory',
                lambda m, s=side: self.store(s+'cmd', m), qos))
        for msg_type, topic in ((Bool, '/reactivate'), (Bool, '/reference_diverged'),
                                (String, '/sg2_xr/status'), (String, '/sg2_xr/events'),
                                (String, '/sg2_xr/diagnostics'), (String, '/sg2_xr/path_diagnostics')):
            self.subscriptions_keep.append(self.create_subscription(
                msg_type, topic, lambda m, t=topic: self.event(t, m.data), 10))
        if hardware_mode:
            self.subscriptions_keep.append(self.create_subscription(
                String, '/xr_hardware/gate_status', lambda m:self.event('/xr_hardware/gate_status',m.data),10))
        self.timer = self.create_timer(1.0 / hz, self.sample)
        print(f'[LOG] {output}\n[URDF] {urdf}', flush=True)
        print('[NOTE] PASSIVE: no activation, no joint commands, no controller edits.', flush=True)
        for side in ('l', 'r'):
            lo, hi = self.limits[f'arm_{side}_joint4']
            print(f'[LIMIT] {side.upper()} q4=[{math.degrees(lo):.3f}, {math.degrees(hi):.3f}] deg', flush=True)
        print('[NOTE] Legacy XR has no raw clamp telemetry; fixed-resume optionally fills xr_projection_mm.', flush=True)

    def store(self, key, message):
        self.cache[key] = (message, time.monotonic())
        if key == 'js':
            self.js_count += 1

    def event(self, topic, value):
        if topic == '/sg2_xr/path_diagnostics':
            try:
                self.path_diagnostic = (json.loads(value), time.monotonic())
            except (TypeError, ValueError):
                self.path_diagnostic = None
        if topic == '/sg2_xr/diagnostics':
            try:
                self.xr_diagnostic = (json.loads(value), time.monotonic())
            except (TypeError, ValueError):
                self.xr_diagnostic = None
        if topic == '/reactivate':
            self.active = bool(value)
        elif topic == '/reference_diverged':
            self.diverged = bool(value)
        elif topic == '/sg2_xr/status':
            try:
                self.active = bool(json.loads(value)['active'])
            except (ValueError, KeyError, TypeError):
                pass
        self.events.write(json.dumps({'elapsed_s': time.monotonic() - self.started,
                                      'topic': topic, 'data': value}) + '\n')

    def latest(self, key, now, ros_now, row, label, flags):
        msg, received = self.cache.get(key, (None, -math.inf))
        if msg is None:
            flags.append('NO_' + label.upper())
            return None
        source = stamp_sec(msg)
        row[label + '_source_s'] = source
        row[label + '_age_s'] = now - received
        # JointTrajectory commands in this stack may legitimately have stamp 0.
        stale_source = label != 'cmd' and not -0.1 <= ros_now - source <= 0.5
        if now - received > 0.5 or stale_source:
            flags.append('STALE_' + label.upper())
            return None
        return msg

    def measured_pose(self, link, ros_now):
        transform = self.tf.lookup_transform('base_link', link, Time())
        source = stamp_sec(transform)
        if not -0.1 <= ros_now - source <= 0.5:
            raise ValueError('stale TF')
        pose = PoseStamped().pose
        p = transform.transform.translation
        pose.position.x, pose.position.y, pose.position.z = p.x, p.y, p.z
        pose.orientation = transform.transform.rotation
        xyz, rot = pose_parts(pose)
        return xyz, rot, source

    def sample(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        print_now = now >= self.next_print
        for side in ('l', 'r'):
            row = dict.fromkeys(self.fields, math.nan)
            row.update(elapsed_s=now-self.started, ros_time_s=ros_now, side=side,
                       active=self.active, reference_diverged=self.diverged)
            flags = []
            state = self.latest('js', now, ros_now, row, 'js', flags)
            cmd = self.latest(side+'cmd', now, ros_now, row, 'cmd', flags)
            goal = self.latest(side+'goal', now, ros_now, row, 'goal', flags)
            measured = {}
            targets = {}
            if state is not None:
                if (len(state.name) == len(state.position) and len(set(state.name)) == len(state.name)
                        and np.isfinite(state.position).all()):
                    measured = dict(zip(state.name, state.position))
                else:
                    flags.append('INVALID_JS')
            if cmd is not None and cmd.points:
                p = cmd.points[-1]
                if len(cmd.joint_names) == len(p.positions) and np.isfinite(p.positions).all():
                    targets = dict(zip(cmd.joint_names, p.positions))
            for i in range(1, 8):
                name = f'arm_{side}_joint{i}'
                row[f'q{i}_measured_deg'] = math.degrees(measured.get(name, math.nan))
                row[f'q{i}_cmd_deg'] = math.degrees(targets.get(name, math.nan))
            lo, hi = map(math.degrees, self.limits[f'arm_{side}_joint4'])
            row['q4_lower_deg'], row['q4_upper_deg'] = lo, hi
            row['q4_margin_deg'] = hi - row['q4_measured_deg']
            row['cmd_q4_margin_deg'] = hi - row['q4_cmd_deg']
            row['q4_cmd_minus_measured_deg'] = row['q4_cmd_deg'] - row['q4_measured_deg']
            for key, prefix in (('q4_margin_deg', 'Q4'), ('cmd_q4_margin_deg', 'CMD_Q4')):
                margin = row[key]
                if math.isfinite(margin):
                    if margin < -0.3:
                        flags.append(prefix + '_OVER_UPPER')
                    elif margin <= 2.0:
                        flags.append(prefix + '_NEAR_UPPER')
            try:
                xyz, rot, tf_stamp = self.measured_pose(f'arm_{side}_link7', ros_now)
                row['tf_source_s'], row['tf_age_s'] = tf_stamp, ros_now-tf_stamp
                for axis, value in zip('xyz', xyz):
                    row[f'actual_{axis}_m'] = float(value)
                self.tf_valid_count += 1
                shoulder, _, shoulder_stamp = self.measured_pose(f'arm_{side}_link2', ros_now)
                row['measured_shoulder_distance_m'] = float(np.linalg.norm(xyz-shoulder))
                # [ADDED] Measure geometry from actual PhysX TF, not commanded
                # FK or the raw q4 coordinate. Keep both endpoint definitions.
                try:
                    elbow, _, elbow_stamp = self.measured_pose(f'arm_{side}_link4', ros_now)
                    proximal, _, proximal_stamp = self.measured_pose(f'arm_{side}_link5', ros_now)
                    stamps = (tf_stamp, shoulder_stamp, elbow_stamp, proximal_stamp)
                    skew = max(stamps)-min(stamps)
                    row['elbow_geometry_tf_skew_s'] = skew
                    if skew <= 0.10:
                        row['elbow_proximal_bend_deg'] = chord_bend_deg(shoulder, elbow, proximal)
                        row['elbow_wrist_bend_deg'] = chord_bend_deg(shoulder, elbow, xyz)
                    else:
                        flags.append('ELBOW_GEOMETRY_UNSYNC')
                except Exception:
                    flags.append('ELBOW_GEOMETRY_UNAVAILABLE')
                if goal is not None and goal.header.frame_id == 'base_link':
                    gxyz, grot = pose_parts(goal.pose)
                    for axis, value in zip('xyz', gxyz):
                        row[f'goal_{axis}_m'] = float(value)
                    row['goal_shoulder_distance_m'] = float(np.linalg.norm(gxyz-shoulder))
                    skew = abs(stamp_sec(goal)-tf_stamp)
                    row['goal_tf_skew_s'] = skew
                    if skew <= 0.1:
                        row['ee_pos_error_mm'] = float(np.linalg.norm(gxyz-xyz) * 1000)
                        row['ee_ori_error_deg'] = float((grot * rot.inv()).magnitude() * 180 / np.pi)
                    else:
                        flags.append('GOAL_TF_UNSYNC')
                elif goal is not None:
                    flags.append('GOAL_FRAME_NOT_BASE_LINK')
            except Exception as exc:
                flags.append('TF_OR_POSE_UNAVAILABLE')
                error = f'{type(exc).__name__}: {exc}'
                if self.last_tf_error.get(side) != error:
                    self.last_tf_error[side] = error
                    self.event(f'logger/{side}/tf_error', error)
            row['flags'] = '|'.join(flags) or 'OK'
            if self.xr_diagnostic is not None:
                diagnostic, received = self.xr_diagnostic
                row['xr_diagnostic_age_s'] = now - received
                if 0 <= now - received <= 0.3:
                    arm = diagnostic.get('arms', {}).get({'l': 'left', 'r': 'right'}[side], {})
                    row.update(xr_mode=diagnostic.get('mode', ''), xr_phase=diagnostic.get('phase', ''),
                               xr_session_id=diagnostic.get('session_id', ''),
                               xr_projection_mm=1000*arm.get('projection_m', math.nan),
                               xr_raw_radius_m=arm.get('raw_radius_m', math.nan),
                               xr_shell_max_m=arm.get('shell_max_m', math.nan),
                               xr_align_position_mm=1000*arm.get('align_position_m', math.nan),
                               xr_align_rotation_deg=arm.get('align_rotation_deg', math.nan))
            # [ADDED] Keep source age/skew explicit; these are sampled stages,
            # not a hardware-synchronised pose trace. Raw rotations are logged
            # so a straight hand translation with turning wrist is distinguishable.
            if self.path_diagnostic is not None:
                path, rx = self.path_diagnostic
                age = ros_now - path.get('ros_time_s', -math.inf)
                frame_age = path.get('frame_age_s', math.inf)
                if path.get('fresh') and 0 <= now-rx <= .3 and -.1 <= age <= .3 and frame_age <= .3:
                    data = path.get('arms',{}).get({'l':'left','r':'right'}[side],{})
                    row.update(path_mode=path.get('mode',''), path_ros_time_s=path['ros_time_s'],
                               path_age_s=age+frame_age,
                               path_goal_skew_s=abs(path['ros_time_s']-row.get('goal_source_s',math.nan)),
                               path_anchor_displacement_m=data.get('anchor_displacement_m',math.nan),
                               path_workspace_clipped_mm=1000*data.get('workspace_clipped_m',math.nan),
                               path_input_rotation_from_anchor_deg=data.get('raw_orientation_from_anchor_deg',math.nan))
                    for stage in ('raw','workspace','published','actual'):
                        matrix=np.asarray(data.get(stage+'_pose_base',[]),dtype=float)
                        if matrix.shape==(4,4) and np.isfinite(matrix).all():
                            for axis,value in zip('xyz',matrix[:3,3]):
                                row[f'path_{stage}_{axis}_m']=float(value)
                            if stage!='workspace':
                                for axis,value in zip('xyzw',Rotation.from_matrix(matrix[:3,:3]).as_quat()):
                                    row[f'path_{stage}_q{axis}']=float(value)
            self.writer.writerow(row)
            if print_now:
                print(f"[{now-self.started:6.1f}s {side.upper()}] active={self.active} "
                      f"q4={row['q4_measured_deg']:7.2f}deg cmd={row['q4_cmd_deg']:7.2f}deg "
                      f"upper={hi:.2f}deg margin={row['q4_margin_deg']:.2f}deg "
                      f"bend={row['elbow_proximal_bend_deg']:.1f}deg "
                      f"EE={row['ee_pos_error_mm']:.1f}mm/{row['ee_ori_error_deg']:.1f}deg "
                      f"r_goal={row['goal_shoulder_distance_m']:.3f}m {row['flags']}", flush=True)
        if print_now:
            self.next_print = now + 1.0
        self.csv_file.flush()

    def finish(self):
        self.csv_file.close()
        self.events.close()
        print(f'[SAVED] {self.output}; joint_messages={self.js_count}, valid_TF_samples={self.tf_valid_count}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument('--hz', type=float, default=10.0)
    parser.add_argument('--duration', type=float, default=0, help='seconds; 0=until Ctrl+C')
    parser.add_argument('--hardware', action='store_true', help='Explicit passive real-robot log on domain30; no commands')
    args = parser.parse_args()
    allowed = ('30',) if args.hardware else ('231', '232')
    if os.environ.get('ROS_DOMAIN_ID') not in allowed:
        parser.error('Use --hardware only on real domain30, otherwise sim domain231/232')
    if args.hardware and args.urdf is None:
        parser.error('Hardware logs require explicit generated --urdf (SG2 and SH5 differ)')
    if not 1 <= args.hz <= 30 or not math.isfinite(args.duration) or args.duration < 0:
        parser.error('require hz in [1,30], duration >= 0')
    urdf = args.urdf or Path(get_package_share_directory('cyclo_motion_controller_models')) / 'models/ai_worker/ffw_sg2_follower.urdf'
    rclpy.init(args=[])
    node = None
    try:
        node = ArmLogger(args.output, urdf.resolve(), args.hz, hardware_mode=args.hardware)
        while rclpy.ok() and (args.duration == 0 or time.monotonic()-node.started < args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.finish()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
