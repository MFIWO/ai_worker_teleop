"""[ADDED] SG2/SH5 real-arm commissioning using the tested wrist delta + AR UI.

No expanded simulation elbow model/shell, no simulator precision overlay,
no hand/trigger/base/lift output. Hardware command gate must be present on robot.
Default is observe-only; --arm in the wrapper opts in, then X+A still required.
"""
from copy import deepcopy
import json
import os
import signal
import time

import numpy as np
import rclpy
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, String

from hardware.xr_contract import (
    ARM_NAMES, SIDES, FOLLOWER_TOPICS, GATE_NAME, PC_NAME,
    GOAL_SPEED, ANGULAR_SPEED, MAX_JOINT_SPEED, private_topic, check_positions,
)
from robotis_vuer.vr_publisher_sg2_xr import XRDeltaPublisher
from robotis_vuer.vr_publisher_sg2_xr_resume import XRResumePublisher
from robotis_vuer.vr_publisher_sg2_xr_overhead import XROverheadPublisher
from robotis_vuer.xr_path_workspace import AnchorRayHint


class HardwareXRPublisher(XROverheadPublisher):
    DEFAULT_GOAL_SPEED = GOAL_SPEED
    DEFAULT_ANGULAR_SPEED = ANGULAR_SPEED

    def _validate_runtime_domain(self):
        mode, domain = os.environ.get('AI_WORKER_XR_EXECUTION'), os.environ.get('ROS_DOMAIN_ID')
        if mode == 'validation' and domain == '232':
            return
        if mode != 'hardware' or domain != '30':
            raise RuntimeError('Hardware retarget requires explicit hardware/domain30; tests only validation/domain232')

    def __init__(self, start_server=True):
        self.hardware_model = os.environ.get('AI_WORKER_XR_MODEL', '')
        if self.hardware_model not in ('sg2', 'sh5'):
            raise ValueError('Select sg2 or sh5 explicitly; never infer from available topics')
        self.hardware_allow_arm = os.environ.get('AI_WORKER_XR_ALLOW_ARM') == '1'
        self._hardware_manifest = None
        self._gate_state = None
        self._gate_rx = -1e9
        self._gate_profile_rx = -1e9
        self._gate_future = None
        self._controller_values = None
        self._intent_sequence = 0
        self._last_hardware_readiness = None
        super().__init__(start_server=False)
        if (self.goal_speed > GOAL_SPEED or self.angular_speed > ANGULAR_SPEED
                or self.translation_scale > 1.0):
            raise ValueError('Hardware commissioning caps: .06m/s, .35rad/s, translation scale<=1')
        self._gate_client = AsyncParameterClient(self, '/' + GATE_NAME)
        self._gate_sub = self.create_subscription(String, '/xr_hardware/gate_status', self._gate_status, 10)
        self._gate_fault_sub = self.create_subscription(Bool, '/xr_hardware/fault', self._reference_fault, 10)
        self._intent_pub = self.create_publisher(String, '/xr_hardware/intent', 10)
        self._intent_timer = self.create_timer(.10, self._heartbeat)
        self.get_logger().info('[HARDWARE] ' + ('X+A may arm after checks' if self.hardware_allow_arm else
                                             'OBSERVE ONLY: X+A cannot enable; rerun wrapper with --arm after preflight'))
        if start_server:
            self.start_vuer_server()

    def _log_extension_mode(self):
        self.get_logger().info(f'[HARDWARE] {self.hardware_model.upper()} original joint bounds; '
                               'no -.45rad simulation expansion; measured .94 workspace; no fingers.')

    def _log_path_mode(self):
        self.get_logger().info('[HARDWARE PATH] Fixed calibration + anchor-ray path + AR guides; '
                               '.06m/s wrist goal; external robot-side watchdog required.')

    def _gate_status(self, msg):
        try:
            data = json.loads(msg.data)
            age = self.get_clock().now().nanoseconds*1e-9 - float(data['ros_time_s'])
            if data['model'] != self.hardware_model or not -.1 <= age <= .5:
                raise ValueError('wrong/stale hardware gate status')
            self._gate_state, self._gate_rx = data, time.monotonic()
            readiness = (bool(data.get('description_ok') and data.get('measured_fresh') and
                              data.get('motor_healthy') and self._profile_verified and not data.get('fault')),
                         str(data.get('fault') or ''))
            if readiness != self._last_hardware_readiness:
                self._last_hardware_readiness = readiness
                self.get_logger().info('[HARDWARE PRECHECK] ' + ('MODEL / MOTOR FEEDBACK READY' if readiness[0]
                    else 'NOT READY: ' + str(data.get('fault') or data.get('reason'))))
            if data.get('fault') and not self._controller_fault_latched:
                self._reference_fault(Bool(data=True))
        except (ValueError, TypeError, KeyError):
            self._gate_rx = -1e9

    def _reference_fault(self, msg):
        # [FIX] Status repeats a latched fault; do not erase transient histories
        # or flood STOP events on every heartbeat after the first real stop.
        if msg.data and getattr(self, '_controller_fault_latched', False):
            return
        super()._reference_fault(msg)

    def _heartbeat(self):
        self._intent_sequence += 1
        self._intent_pub.publish(String(data=json.dumps({
            'model': self.hardware_model,
            'enabled': bool(self.hardware_allow_arm and self.arm_control_enabled and not self._controller_fault_latched),
            'session_id': self._session_id, 'sequence': self._intent_sequence,
            'phase': self._phase, 'ros_time_s': self.get_clock().now().nanoseconds*1e-9,
        })))

    def _query_profile(self):
        if not hasattr(self, '_gate_client'):
            return
        if (self._gate_future is None or self._gate_future.done()) and self._gate_client.services_are_ready():
            self._gate_future = self._gate_client.get_parameters(['profile_json'])
            self._gate_future.add_done_callback(self._gate_profile)
        if (self._profile_future is None or self._profile_future.done()) and self._profile_client.services_are_ready():
            self._profile_future = self._profile_client.get_parameters([
                'urdf_path', 'srdf_path', 'weight_elbow_position', 'weight_position',
                'weight_orientation', 'r_gripper_name', 'l_gripper_name',
                'right_traj_topic', 'left_traj_topic'])
            self._profile_future.add_done_callback(self._profile_result)

    def _gate_profile(self, future):
        try:
            p = json.loads(future.result().values[0].string_value)
            if (p['schema'] != 1 or p['mode'] != 'hardware_commissioning' or p['model'] != self.hardware_model
                    or p['max_joint_speed'] != MAX_JOINT_SPEED or p['workspace_fraction'] != .94
                    or p['goal_speed'] != GOAL_SPEED):
                raise ValueError('unsupported hardware profile')
            for short in ('l', 'r'):
                if p['limits'][f'arm_{short}_joint4'][1] > -.7 + 1e-12:
                    raise ValueError('simulation elbow model rejected on real hardware')
            self._hardware_manifest = p
            self._gate_profile_rx = time.monotonic()
        except Exception:
            self._hardware_manifest = None
            self._profile_verified = False

    def _profile_result(self, future):
        try:
            v = future.result().values
            p = self._hardware_manifest
            self._profile_verified = bool(p and
                v[0].string_value == p['urdf'] and v[1].string_value == p['srdf'] and
                v[2].type == 3 and v[2].double_value == 0.0 and
                v[3].type == 3 and v[3].double_value == 10.0 and
                v[4].type == 3 and v[4].double_value == 1.0 and
                v[5].string_value == 'arm_r_link7' and v[6].string_value == 'arm_l_link7' and
                v[7].string_value == private_topic(self.hardware_model, 'right') and
                v[8].string_value == private_topic(self.hardware_model, 'left'))
            self._profile_rx = time.monotonic()
        except Exception:
            self._profile_verified = False

    def _graph_check(self):
        # [CHANGED] Physical follower outputs must be gated; legacy writers are refused.
        XRDeltaPublisher._graph_check(self)
        now = time.monotonic()
        if (not self._profile_verified or now-self._profile_rx > 4.
                or now-self._gate_profile_rx > 4.):
            raise ValueError('HARDWARE_PROFILE_REQUIRED: run matching hardware/controller launcher, not sim launcher')
        if (self._gate_state is None or now-self._gate_rx > .5
                or not self._gate_state.get('description_ok') or not self._gate_state.get('measured_fresh')
                or not self._gate_state.get('motor_healthy')):
            raise ValueError('fresh physical robot_description, joint_states and hardware gate required')
        if self._gate_state.get('fault'):
            raise ValueError('hardware gate fault latched')
        nodes = [n for n, _ in self.get_node_names_and_namespaces()]
        if nodes.count('reference_checker') != 1:
            raise ValueError('exactly one original reference_checker is required')
        for side in SIDES:
            for topic, expected in ((FOLLOWER_TOPICS[side], GATE_NAME),
                                    (private_topic(self.hardware_model, side), 'vr_controller')):
                writers = self.get_publishers_info_by_topic(topic)
                if len(writers) != 1 or writers[0].node_name != expected:
                    raise ValueError(f'{topic}: require exactly one {expected}, stop old controller first')

    def _capture(self):
        # [CHANGED] Bypass SIM-only extended_templates; preserve the original .94 reach margin.
        first = self._session is None
        XRResumePublisher._capture(self)
        if first:
            self._session.templates = {s: AnchorRayHint.from_hint(h, self._session.mapper.anchors[s].robot_pose[:3, 3])
                                       for s, h in self._session.templates.items()}
            self._hints = deepcopy(self._session.templates)
        self._path_frame = None

    def _set_arm_control_enabled(self, enabled, reason):
        if enabled and not self.hardware_allow_arm:
            if getattr(self, '_resume_ready', False):
                self._event('OBSERVE_ONLY', 'restart hardware PC wrapper with --arm for manual X+A activation')
            return
        super()._set_arm_control_enabled(enabled, reason)
        if hasattr(self, '_intent_pub'):
            self._heartbeat()

    def _command_feedback(self, msg, side):
        # [CHANGED] Use real selected bounds, not extension's -.45 experiment constant.
        if self._hardware_manifest is not None:
            try:
                if not msg.points:
                    raise ValueError('empty gated command')
                check_positions(side, msg.joint_names, msg.points[-1].positions,
                                self._hardware_manifest['limits'])
            except (ValueError, TypeError) as error:
                if self.arm_control_enabled:
                    self._set_arm_control_enabled(False, 'hardware command guard: ' + str(error))
                return
        XRResumePublisher._command_feedback(self, msg, side)

    def _publish_synced_pose_frame_if_ready(self):
        if getattr(self, '_path_ready', False) and self.arm_control_enabled and self._phase == 'ACTIVE':
            try:
                links = self._snapshot()
                for side in SIDES:
                    actual, goal = links[side]['wrist'], self._goals[side]
                    if (np.linalg.norm(goal[:3, 3]-actual[:3, 3]) > .05 or
                            Rotation.from_matrix(goal[:3, :3] @ actual[:3, :3].T).magnitude() > np.deg2rad(20)):
                        raise ValueError(f'{side}: hardware commissioning tracking error >50mm/20deg')
            except Exception as error:
                self._set_arm_control_enabled(False, 'hardware tracking guard: ' + str(error))
                return
        super()._publish_synced_pose_frame_if_ready()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = HardwareXRPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._set_arm_control_enabled(False, 'hardware PC publisher shutting down')
            rclpy.spin_once(node, timeout_sec=.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
