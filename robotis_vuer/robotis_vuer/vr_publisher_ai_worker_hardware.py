"""[ADDED] SG2/SH5 real-arm commissioning using the tested wrist delta + AR UI.

No expanded simulation elbow model/shell, no simulator precision overlay,
SG2 sticks and triggers route through the robot hardware command gate.
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
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from hardware.xr_contract import (
    ARM_NAMES, SIDES, FOLLOWER_TOPICS, GATE_NAME, PC_NAME,
    GOAL_SPEED, ANGULAR_SPEED, COMMAND_WINDOW_SECONDS, private_topic, check_positions, arm_position_tolerance,
    validate_readiness_policy,
)
from hardware.aux_controls import GRIPPERS, BASE_LIMITS
from robotis_vuer.vr_publisher_sg2 import VRTrajectoryPublisher
from robotis_vuer.vr_publisher_sg2_xr import XRDeltaPublisher
from robotis_vuer.vr_publisher_sg2_xr_resume import XRResumePublisher
from robotis_vuer.vr_publisher_sg2_xr_overhead import XROverheadPublisher
from robotis_vuer.xr_path_workspace import AnchorRayHint
from robotis_vuer.xr_arm_targets import limited_pose, rigid_pose
from robotis_vuer.tact_trigger import TactTrigger, TACT_TRIGGER_TOPIC  # [ADDED 2026-10-16] leader buttons


class AuxiliarySink:
    """Reuse original SG2 stick mapping without creating a second follower writer."""
    def __init__(self, node, base=False):
        self.node, self.base = node, base

    def publish(self, msg):
        if self.base:
            self.node._aux_base = [min(b, max(-b, v)) for v, b in
                                   zip((msg.linear.x, msg.linear.y, msg.angular.z), BASE_LIMITS)]
        else:
            for name, value in zip(msg.joint_names, msg.points[-1].positions):
                bound = (self.node._hardware_manifest or {}).get('auxiliary_limits', {}).get(name)
                if bound:
                    value = min(bound['upper'], max(bound['lower'], float(value)))
                    self.node._aux_joints[name] = value
                    if name == 'lift_joint':
                        self.node.last_lift_command = value
            if msg.joint_names == ['head_joint1', 'head_joint2']:
                # Parent writes last_head_command after publish; corrected below.
                self.node._head_target_clamped = tuple(self.node._aux_joints.get(k) for k in msg.joint_names)


class HardwareXRPublisher(XROverheadPublisher):
    TRACKING_GRACE_SECONDS = 50.0
    FEEDBACK_MAX_AGE = 5.0
    ARM_TRACKING_TIMEOUT = 5.0
    RECEIPT_TIME_ONLY = True
    REANCHOR_ON_RESUME = True
    REZERO_ON_READY = True
    MANUAL_FAULT_RESET = True
    DEFAULT_GOAL_SPEED = GOAL_SPEED
    DEFAULT_ANGULAR_SPEED = ANGULAR_SPEED
    MAX_GOAL_SPEED = GOAL_SPEED
    MAX_ANGULAR_SPEED = ANGULAR_SPEED
    # Defaults until the verified Cyclo profile supplies its actual timing.
    SETTLING_SECONDS = 0.10
    CONTROLLER_ACTIVATION_DELAY = 0.10
    WARMUP_HOLD_SECONDS = 0.10
    STOP_SETTLE_SECONDS = 0.20
    STILL_WINDOW_SECONDS = 0.15
    ALIGN_POSITION_M = .03
    ALIGN_ROTATION_DEG = 15.
    TRACKING_POSITION_LIMIT_M = float('inf')
    TRACKING_ROTATION_LIMIT_RAD = float('inf')

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
        self.WORKSPACE_FRACTION = 1.0 if self.hardware_model == 'sh5' else .94
        self.hardware_allow_arm = os.environ.get('AI_WORKER_XR_ALLOW_ARM') == '1'
        # [CHANGED] Main PC wrapper selects normal mode. Older callers remain
        # strict, and a mismatched PC/robot policy cannot silently activate.
        self.hardware_readiness_policy = validate_readiness_policy(
            os.environ.get('AI_WORKER_XR_READINESS_POLICY', 'strict_torque'))
        self._hardware_manifest = None
        self._gate_state = None
        self._gate_rx = -1e9
        self._gate_profile_rx = -1e9
        self._gate_future = None
        self._profile_request_started = {}
        self._controller_values = None
        self._startup_timing = None
        self._intent_sequence = 0
        self._last_hardware_readiness = None
        self._gate_fault_reason = None  # [CHANGED] Gate and reference-checker faults are different sources.
        self._gate_status_sequence = -1
        self._gate_instance = None
        self._retired_gate_instances = set()
        self._resume_bridge_targets = None
        self._resume_bridge_rx = None
        self._tf_receipts = {}
        self._fault_reset_future = None
        self._fault_reset_wait = None
        self._fault_status_only = False
        self._fault_reset_started = None
        self._aux_joints = {}
        self._aux_base = [0., 0., 0.]
        self._head_target_clamped = None
        self._lift_follow_position = None  # [CHANGED] SH5 measured lift baseline for arm-frame following.
        self._tracking_waiting = False
        super().__init__(start_server=False)
        self.head_joint_pub = AuxiliarySink(self)
        self.lift_joint_pub = AuxiliarySink(self)
        self.cmd_vel_pub = AuxiliarySink(self, base=True)
        if (self.goal_speed > GOAL_SPEED or self.angular_speed > ANGULAR_SPEED
                or self.translation_scale > 1.0):
            raise ValueError('Hardware Cartesian filter parameters exceed selected defaults')
        self._gate_client = AsyncParameterClient(self, '/' + GATE_NAME)
        self._fault_reset_client = self.create_client(Trigger, '/xr_hardware/reset_fault')
        self._gate_sub = self.create_subscription(String, '/xr_hardware/gate_status', self._gate_status, 10)
        self._gate_fault_sub = self.create_subscription(Bool, '/xr_hardware/fault', self._hardware_gate_fault, 10)
        self._intent_pub = self.create_publisher(String, '/xr_hardware/intent', 10)
        # [ADDED 2026-10-16] Leader tact-switch emulation for Cyclo recording control:
        # single left stick click = 'left' (cancel), single right stick click = 'right' (start/save),
        # 2s hold = '*_long_time', both clicks = existing mode toggle only (no trigger).
        self._tact = TactTrigger(long_press_duration=2.0)
        self._tact_pub = self.create_publisher(String, TACT_TRIGGER_TOPIC, 10)
        self._intent_timer = self.create_timer(.10, self._heartbeat)
        self._tf_receipt_sub = self.create_subscription(
            TFMessage, '/tf', self._tf_received,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info('[HARDWARE] ' + ('X+A may arm after checks' if self.hardware_allow_arm else
                                             'OBSERVE ONLY: X+A cannot enable; rerun wrapper with --arm after preflight'))
        self.get_logger().info(
            f'[HARDWARE STARTUP] settle={self.SETTLING_SECONDS:g}s; '
            'activation/ramp timing is read from Cyclo. X+A rezeros hands after preparation.')
        self.get_logger().info('[HARDWARE CONTROLS] SG2: left=head, right=lift; BOTH stick clicks '
                               'toggle left=base XY/right=yaw+lift; triggers=grippers. '
                               'Auxiliary motion enabled only in ACTIVE; Y+B stops all. '
                               'X+A resumes with measured anchors; startup stillness=30mm/15deg. '
                               'Freshness=local monotonic receipt; remote epoch ignored.')
        if start_server:
            self.start_vuer_server()

    def _log_extension_mode(self):
        self.get_logger().info(f'[HARDWARE] {self.hardware_model.upper()} original joint bounds; '
                               f'measured workspace fraction={self.WORKSPACE_FRACTION:g}; SG2 sticks/triggers through hardware gate.')

    def _tf_received(self, msg):
        now = time.monotonic()
        with self._xr_lock:
            for transform in msg.transforms:
                name = transform.child_frame_id
                stamp = transform.header.stamp.sec + transform.header.stamp.nanosec*1e-9
                previous, _ = self._tf_receipts.get(name, (-float('inf'), -float('inf')))
                if stamp > previous:
                    self._tf_receipts[name] = (stamp, now)

    def _log_path_mode(self):
        self.get_logger().info('[HARDWARE PATH] Measured resume + anchor-ray path + AR guides; '
                               f'non-binding Cartesian caps {GOAL_SPEED:g}m/s/{ANGULAR_SPEED:g}rad/s; '
                               'physical URDF/QP joint limits remain authoritative.')

    def _gate_status(self, msg):
        try:
            data = json.loads(msg.data)
            if (data['model'] != self.hardware_model
                    or data.get('readiness_policy', 'strict_torque') != self.hardware_readiness_policy):
                raise ValueError('wrong/stale hardware gate status')
            instance, sequence = str(data['gate_instance']), int(data['status_sequence'])
            if instance in self._retired_gate_instances:
                return
            if instance == self._gate_instance and sequence <= self._gate_status_sequence:
                return
            changed_instance = instance != self._gate_instance
            if self._gate_instance is not None and changed_instance:
                self._retired_gate_instances.add(self._gate_instance)
            self._gate_instance, self._gate_status_sequence = instance, sequence
            if changed_instance:
                self._invalidate_profile_requests()
            self._gate_state, self._gate_rx = data, time.monotonic()
            readiness = (bool(data.get('description_ok') and data.get('measured_fresh') and
                              data.get('motor_healthy') and self._profile_verified and not data.get('fault')),
                         str(data.get('fault') or ''))
            if readiness != self._last_hardware_readiness:
                self._last_hardware_readiness = readiness
                self.get_logger().info('[HARDWARE PRECHECK] ' + ('MODEL / MOTOR FEEDBACK READY' if readiness[0]
                    else 'NOT READY: ' + str(data.get('fault') or data.get('reason'))))
            if (self._fault_reset_wait is not None and instance == self._fault_reset_wait[0]
                    and sequence > self._fault_reset_wait[1]):
                self._fault_reset_wait = None
                self._fault_reset_started = None
                if not data.get('fault'):
                    self._controller_fault_latched = False
                    self._gate_fault_reason = None
                    self._phase = 'ALIGN_REQUIRED' if self._session else 'WAIT_CALIBRATION'
                    self._event('FAULT_CLEARED', 'manual X+A reset confirmed; preparing measured restart')
                    self._set_arm_control_enabled(True, 'manual X+A after fault reset')
            if data.get('fault') and self._fault_reset_future is None:
                self._latch_hardware_gate_fault(str(data['fault']))
        except (ValueError, TypeError, KeyError):
            self._gate_rx = -1e9

    def _hardware_gate_fault(self, msg):
        # [CHANGED 2026-10-14] The immediate Bool may precede the detailed status.
        # Stop immediately, but never mislabel a joint/lease fault as Cyclo divergence.
        if msg.data and self._fault_reset_future is None and not self._fault_status_only:
            reason = (self._gate_state or {}).get('fault')
            self._latch_hardware_gate_fault(str(reason or 'robot-side command gate fault; awaiting gate_status detail'))

    def _latch_hardware_gate_fault(self, reason):
        if reason == self._gate_fault_reason:
            return
        first = self._gate_fault_reason is None
        self._gate_fault_reason = reason
        self._controller_fault_latched = True
        if first or self.arm_control_enabled:
            self._set_arm_control_enabled(False, 'hardware gate fault: ' + reason)
        else:
            # [CHANGED] Enrich the original error once, without repeated stops/recalibration.
            self._event('GATE_FAULT_DETAIL', 'hardware gate fault: ' + reason)

    def _request_fault_reset(self):
        if self._fault_reset_future is not None or self._fault_reset_wait is not None:
            return
        if not self._fault_reset_client.service_is_ready():
            self._event('RESET_REFUSED', 'robot fault reset service unavailable')
            return
        self._heartbeat()  # Explicit disarmed lease before the reset request.
        self._fault_reset_future = self._fault_reset_client.call_async(Trigger.Request())
        self._fault_reset_started = time.monotonic()
        self._fault_reset_future.add_done_callback(self._fault_reset_result)
        self._event('RESET_REQUESTED', 'manual X+A; verifying robot feedback before resetting fault')

    def _fault_reset_result(self, future):
        self._fault_reset_future = None
        try:
            response = future.result()
            if not response.success:
                raise ValueError(response.message)
            barrier = json.loads(response.message)
            self._fault_reset_wait = (str(barrier['gate_instance']), int(barrier['status_sequence']))
            self._fault_status_only = True
            self._event('RESET_ACCEPTED', 'waiting for ordered post-reset gate status')
        except Exception as error:
            self._fault_reset_started = None
            self._event('RESET_REFUSED', str(error))

    def _reference_fault(self, msg):
        # [FIX] Status repeats a latched fault; do not erase transient histories
        # or flood STOP events on every heartbeat after the first real stop.
        if msg.data and getattr(self, '_controller_fault_latched', False):
            return
        if msg.data:
            self._controller_fault_latched = True
            self._set_arm_control_enabled(False, 'Cyclo reference fault; X+A requests checked reset')

    def _controller_aux_selected(self):
        return self.hardware_model == 'sg2'

    def _controller_aux_ready(self, now=None):
        now = time.monotonic() if now is None else now
        return (self._controller_aux_selected() and self.hardware_allow_arm and self.arm_control_enabled
                and self._phase == 'ACTIVE' and not self._controller_fault_latched
                and bool((self._hardware_manifest or {}).get('auxiliary_limits'))
                and self._arm_tracking_is_fresh(now)
                and self._feedback_input_state(now) == 'fresh'
                and self._tracking_input_state(now) == 'fresh')

    def _heartbeat(self):
        self._intent_sequence += 1
        aux_enabled = self._controller_aux_ready()
        self._intent_pub.publish(String(data=json.dumps({
            'model': self.hardware_model,
            'enabled': bool(self.hardware_allow_arm and self.arm_control_enabled and
                            not self._controller_fault_latched and self._resume_bridge_targets is None),
            'session_id': self._session_id, 'sequence': self._intent_sequence,
            'phase': self._phase, 'ros_time_s': self.get_clock().now().nanoseconds*1e-9,
            'tracking_fresh': self._tracking_input_state() == 'fresh',
            'robot_feedback_fresh': self._feedback_input_state(time.monotonic()) == 'fresh',
            'aux': {'joints': self._aux_joints, 'base': self._aux_base} if aux_enabled else {},
            **self._extra_intent(time.monotonic()),
        })))

    def _extra_intent(self, now):
        return {}

    def _invalidate_profile_requests(self):
        # Old servers can disappear without completing their service futures.
        # Retire requests before cancellation callbacks can touch the new state.
        for name in ('_gate_future','_profile_future'):
            future = getattr(self,name,None)
            setattr(self,name,None)
            if future is not None and not future.done():future.cancel()
        self._profile_request_started.clear()
        self._hardware_manifest = None
        self._profile_verified = False
        self._gate_profile_rx = self._profile_rx = -1e9

    def _query_profile(self):
        if not hasattr(self, '_gate_client'):
            return
        requests = (
            ('_gate_future',self._gate_client,['profile_json'],self._gate_profile),
            ('_profile_future',self._profile_client,[
                'urdf_path', 'srdf_path', 'weight_elbow_position', 'weight_position',
                'weight_orientation', 'r_gripper_name', 'l_gripper_name',
                'right_traj_topic', 'left_traj_topic',
                'activation_delay_seconds', 'startup_ramp_seconds'],self._profile_result))
        now = time.monotonic()
        for name,client,parameters,handler in requests:
            future = getattr(self,name,None)
            if future is not None and not future.done():
                if now-self._profile_request_started.get(name,now) < 3.:continue
                setattr(self,name,None)
                future.cancel()
                self.get_logger().warning('[PROFILE QUERY RETRY] No reply within 3s; querying current controller again')
            if not client.services_are_ready():continue
            future = client.get_parameters(parameters)
            setattr(self,name,future)
            self._profile_request_started[name] = now
            instance = self._gate_instance
            def received(result, name=name, instance=instance, handler=handler):
                if result is getattr(self,name,None) and instance == self._gate_instance and not result.cancelled():
                    handler(result)
            future.add_done_callback(received)

    def _gate_profile(self, future):
        try:
            p = json.loads(future.result().values[0].string_value)
            if (p['schema'] != 1 or p['mode'] != 'hardware_commissioning' or p['model'] != self.hardware_model
                    or p['workspace_fraction'] != self.WORKSPACE_FRACTION
                    or p['goal_speed'] != GOAL_SPEED
                    or p.get('time_policy') != 'local_monotonic_receipt'
                    or p.get('readiness_policy', 'strict_torque') != self.hardware_readiness_policy):
                raise ValueError('unsupported hardware profile')
            velocities = p['joint_velocity_limits']
            required = set().union(*ARM_NAMES.values())
            if (set(velocities) != required or
                    any(not np.isfinite(v) or v <= 0 for v in velocities.values()) or
                    p['max_joint_speed'] != max(velocities.values()) or
                    p['tracking_window_seconds'] != COMMAND_WINDOW_SECONDS or
                    p['tracking_error_limits'] != {k: v * COMMAND_WINDOW_SECONDS for k, v in velocities.items()}):
                raise ValueError('invalid per-joint URDF gate limits')
            if (self.hardware_model == 'sh5' and
                    p.get('arm_limit_policy') not in ('physical_description','sh5_bent_elbow_v1')):
                raise ValueError('Update robot launcher and regenerate the SH5 physical-limit profile')
            self._hardware_manifest = p
            self._gate_profile_rx = time.monotonic()
        except Exception:
            self._hardware_manifest = None
            self._profile_verified = False

    def _profile_result(self, future):
        try:
            v = future.result().values
            p = self._hardware_manifest
            timing = tuple(x.double_value for x in v[9:11])
            if (len(timing) != 2 or any(x.type != 3 for x in v[9:11])
                    or not all(np.isfinite(t) and t >= 0. for t in timing)):
                raise ValueError('controller startup timing is unavailable')
            self._profile_verified = bool(p and
                v[0].string_value == p['urdf'] and v[1].string_value == p['srdf'] and
                v[2].type == 3 and v[2].double_value == 0.0 and
                v[3].type == 3 and v[3].double_value == 10.0 and
                v[4].type == 3 and v[4].double_value == 1.0 and
                v[5].string_value == 'arm_r_link7' and v[6].string_value == 'arm_l_link7' and
                v[7].string_value == private_topic(self.hardware_model, 'right') and
                v[8].string_value == private_topic(self.hardware_model, 'left'))
            if self._profile_verified:
                if timing != self._startup_timing:
                    self.get_logger().info(f'[HARDWARE STARTUP ACTUAL] activation={timing[0]:g}s '
                                           f'ramp={timing[1]:g}s; zero hands after startup completes')
                self._startup_timing = timing
                self.CONTROLLER_ACTIVATION_DELAY, self.WARMUP_HOLD_SECONDS = timing
            self._profile_rx = time.monotonic()
        except Exception:
            self._profile_verified = False

    def _rezero_start_inputs(self, inputs, actual, reason):
        """Rebase both human wrists atomically, preserving held goals and robot basis."""
        mapper = deepcopy(self._mapper)
        for side in SIDES:
            mapper.anchors[side].input_pose = rigid_pose(inputs[side])
            mapper.anchors[side].robot_pose = rigid_pose(self._goals[side])
        # Validate against the same measured-pose entrance contract before commit.
        session = deepcopy(self._session)
        session.mapper = mapper
        session.require_alignment(inputs, actual)
        self._session.mapper = self._mapper = mapper
        self._last_input = deepcopy(inputs)
        self._last_pub = time.monotonic()
        self._diagnostics = self._session.inspect(inputs, actual)
        self._event('INPUT_ZERO', reason + '; current hands -> held wrist targets; '
                    'robot targets, heading and elbow/workspace references preserved')

    def _graph_check(self):
        # [CHANGED] Physical follower outputs must be gated; legacy writers are refused.
        XRDeltaPublisher._graph_check(self)
        now = time.monotonic()
        active = self.arm_control_enabled and self._phase == 'ACTIVE'
        profile_age_limit = self.TRACKING_GRACE_SECONDS + 20. if active else 40.
        if (not self._profile_verified or now-self._profile_rx > profile_age_limit
                or now-self._gate_profile_rx > profile_age_limit):
            raise ValueError('HARDWARE_PROFILE_REQUIRED: run matching hardware/controller launcher, not sim launcher')
        feedback_wait = (self.arm_control_enabled and self._phase == 'ACTIVE'
                         and self._feedback_input_state(now) == 'waiting')
        gate_age_limit = self.TRACKING_GRACE_SECONDS if active else self.FEEDBACK_MAX_AGE
        if (self._gate_state is None or now-self._gate_rx > gate_age_limit
                or not self._gate_state.get('description_ok')
                or (not feedback_wait and (not self._gate_state.get('measured_fresh')
                                          or not self._gate_state.get('motor_healthy')))):
            raise ValueError('fresh physical robot_description, joint_states and hardware gate required')
        if self._gate_state.get('fault'):
            raise ValueError('hardware gate fault latched: ' + str(self._gate_state['fault']))
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
        # Like xr_tele's U release: anchor the current hands to measured wrists.
        previous = deepcopy(self._last_published)
        XRResumePublisher._capture(self)
        self._session.templates = {s: AnchorRayHint.from_hint(h, self._session.mapper.anchors[s].robot_pose[:3, 3])
                                   for s, h in self._session.templates.items()}
        self._hints = deepcopy(self._session.templates)
        # When old goals are far from measured wrists, update checker history
        # gradually while BOTH /reactivate and the gate lease remain disabled.
        if previous and any(np.linalg.norm(self._goals[s][:3,3]-previous[s][:3,3]) > .09 or
                            Rotation.from_matrix(self._goals[s][:3,:3] @ previous[s][:3,:3].T).magnitude() > np.deg2rad(25)
                            for s in SIDES):
            self._resume_bridge_targets = deepcopy(self._goals)
            self._goals = previous
            self._resume_bridge_rx = time.monotonic()
        self._path_frame = None
        # [CHANGED] Each capture starts from its current measured lift height.
        # The next snapshot must add only the displacement SINCE this capture.
        self._lift_follow_position = float(self.lift_joint_current_position)

    def _tracking_input_state(self, now=None):
        now = time.monotonic() if now is None else now
        samples = [self.last_body_pose_sec, *self.last_controller_pose_sec.values()]
        if len(samples) != 3 or any(t is None or now < t for t in samples):
            return 'expired'
        age = max(now - t for t in samples)
        if age <= self.arm_tracking_timeout_sec:
            return 'fresh'
        if age <= self.TRACKING_GRACE_SECONDS:
            return 'waiting'
        return 'expired'

    def _arm_tracking_is_fresh(self, now=None):
        now = time.monotonic() if now is None else now
        # Grace applies to an already ACTIVE session, never to arming.
        state = self._tracking_input_state(now)
        tracking_ok = state == 'fresh' or (
            state == 'waiting' and self.arm_control_enabled and self._phase == 'ACTIVE')
        limit = (self.TRACKING_GRACE_SECONDS
                 if self.arm_control_enabled and self._phase == 'ACTIVE' else self.FEEDBACK_MAX_AGE)
        return (tracking_ok and self._joint_rx is not None
                and 0.0 <= now - self._joint_rx <= limit)

    def _feedback_input_state(self, now):
        receipts = [self._joint_rx, *[self._command_rx.get(s) for s in SIDES]]
        if self._gate_state is not None:
            receipts.append(self._gate_rx)
        if any(t is None or now < t for t in receipts):
            return 'expired'
        age = max(now-t for t in receipts)
        if age > self.TRACKING_GRACE_SECONDS:
            return 'expired'
        if age > self.FEEDBACK_MAX_AGE or (self._gate_state or {}).get('feedback_waiting'):
            return 'waiting'
        return 'fresh'

    def _hold_for_tracking_gap(self):
        if not self.arm_control_enabled or self._phase != 'ACTIVE':
            return False
        now = time.monotonic()
        state = self._tracking_input_state(now)
        feedback = self._feedback_input_state(now)
        if state == 'fresh' and feedback == 'fresh':
            if self._tracking_waiting:
                self._tracking_waiting = False
                self._last_pub = now  # Never accumulate motion time across the gap.
                self._event('TRACKING_RECOVERED', 'VR/robot feedback restored; continuing from held goals')
            return False
        try:
            if not self._arm_tracking_is_fresh(now):
                raise ValueError(self._tracking_timeout_reason(now))
            if feedback == 'expired':
                raise ValueError('robot/Cyclo feedback exceeded 5s wait')
            self._graph_check()
            if feedback == 'fresh':
                self._snapshot()
            if not self._tracking_waiting:
                self._tracking_waiting = True
                self._aux_base = [0., 0., 0.]
                self._aux_joints.clear()
                self._event('TRACKING_WAIT', 'VR/robot feedback late; holding last goals until inputs are fresh '
                            f'or their age exceeds {self.TRACKING_GRACE_SECONDS:g}s')
                self._heartbeat()
            if self._last_pub is None or now - self._last_pub >= 1 / 30:
                self._publish_pair()  # Republish unchanged goals, never extrapolate missing input.
                self._last_pub = now
        except Exception as error:
            self._set_arm_control_enabled(False, str(error))
        return True

    def _publish_synced_pose_frame_if_ready(self):
        if self._hold_for_tracking_gap():
            return
        if self._resume_bridge_targets is None or not self.arm_control_enabled:
            return super()._publish_synced_pose_frame_if_ready()
        with self._xr_lock:
            now = time.monotonic()
            if self._last_pub is not None and now-self._last_pub < 1/30:
                return
            try:
                if not self._arm_tracking_is_fresh(now):
                    raise ValueError('tracking/feedback stopped during inactive resume preparation')
                self._graph_check()
                # Inactive seed updates must also satisfy the native reference
                # checker (10cm/30deg), even with nonbinding Cartesian rates.
                dt = min(max(.001, now-self._resume_bridge_rx),
                         .09/self.goal_speed, np.deg2rad(25.)/self.angular_speed)
                for side in SIDES:
                    self._goals[side] = limited_pose(self._goals[side], self._resume_bridge_targets[side],
                                                    dt, self.goal_speed, self.angular_speed)
                self._publish_pair()
                self._last_pub = self._resume_bridge_rx = now
                if all(np.linalg.norm(self._goals[s][:3,3]-target[:3,3]) < .001 and
                       Rotation.from_matrix(self._goals[s][:3,:3] @ target[:3,:3].T).magnitude() < np.deg2rad(.5)
                       for s,target in self._resume_bridge_targets.items()):
                    self._goals = self._resume_bridge_targets
                    self._resume_bridge_targets = None
                    self._phase = 'SETTLING'
                    self._arming_until = now+self.SETTLING_SECONDS
                    self._heartbeat()
                    self._event('RESUME_SEED_READY', 'measured anchor ready; stationary startup follows')
            except Exception as error:
                self._set_arm_control_enabled(False, str(error))

    def joint_states_callback(self, msg):
        previous_source = self._last_joint_source
        super().joint_states_callback(msg)
        if self._joint_rx is not None and self._last_joint_source != previous_source:
            # XR's callback records arm freshness only. Restore measured
            # head/lift values used by the original incremental stick methods.
            VRTrajectoryPublisher.joint_states_callback(self, msg)

    def _set_arm_control_enabled(self, enabled, reason):
        if enabled and not self.hardware_allow_arm:
            if getattr(self, '_resume_ready', False):
                self._event('OBSERVE_ONLY', 'restart hardware PC wrapper with --arm for manual X+A activation')
            return
        if enabled and getattr(self, '_controller_fault_latched', False):
            self._request_fault_reset()
            return
        super()._set_arm_control_enabled(enabled, reason)
        if enabled and self.arm_control_enabled and self._resume_bridge_targets is not None:
            self._phase = 'RESUME_SEED'
            self._event('RESUME_PREPARE', 'robot held; preparing measured anchor with controller/gate inactive')
        if not enabled:
            self._tracking_waiting = False
            self._fault_reset_wait = None
            self._fault_reset_started = None
            if self._fault_reset_future is not None:
                future, self._fault_reset_future = self._fault_reset_future, None
                future.cancel()
            self._resume_bridge_targets = None
            self._aux_joints.clear()
            self._aux_base = [0., 0., 0.]
            self.last_head_command = self.last_lift_command = None
            self.last_cmd_vel_command = (0., 0., 0.)
        if hasattr(self, '_intent_pub'):
            self._heartbeat()

    def _check_arm_tracking_timeout(self):
        super()._check_arm_tracking_timeout()
        if (self._fault_reset_started is not None and
                time.monotonic()-self._fault_reset_started > 3.):
            self._fault_reset_wait = None
            self._fault_reset_started = None
            if self._fault_reset_future is not None:
                future,self._fault_reset_future = self._fault_reset_future,None
                future.cancel()
            self._event('RESET_REFUSED', 'fault reset timed out; release X/A and retry')

    def _command_feedback(self, msg, side):
        # [CHANGED] Use real selected bounds, not extension's -.45 experiment constant.
        if self._hardware_manifest is not None:
            try:
                if not msg.points:
                    raise ValueError('empty gated command')
                check_positions(side, msg.joint_names, msg.points[-1].positions,
                                self._hardware_manifest['limits'], arm_position_tolerance(self.hardware_model))
            except (ValueError, TypeError) as error:
                if self.arm_control_enabled:
                    self._set_arm_control_enabled(False, 'hardware command guard: ' + str(error))
                return
        XRResumePublisher._command_feedback(self, msg, side)

    async def on_controller_move(self, event, session):
        await super().on_controller_move(event, session)
        self._process_tact_buttons()  # [ADDED] works whether or not the arm is active, like the leader
        with self._xr_lock:
            if self.hardware_model != 'sg2' or not self._controller_aux_ready():
                return
            # The original implementation supplies stick axis swaps, deadzones,
            # both-click mode toggles, rate limits and incremental measured starts.
            VRTrajectoryPublisher.process_thumbstick(self)
            if self._head_target_clamped is not None:
                self.last_head_command = self._head_target_clamped
            if 'lift_joint' in self._aux_joints:
                self.last_lift_command = self._aux_joints['lift_joint']
            for side, name in GRIPPERS.items():
                state = getattr(self, side + '_controller_state')
                raw = state.get('triggerValue')
                bound = (self._hardware_manifest or {}).get('auxiliary_limits', {}).get(name)
                if bound and self.is_valid_float(raw):
                    value = self.calibrate_trigger(side, raw)
                    self._aux_joints[name] = bound['lower'] + value * (bound['upper'] - bound['lower'])

    def _process_tact_buttons(self):
        """[ADDED 2026-10-16] Stick clicks -> /leader/joystick_controller/tact_trigger (leader rules)."""
        left = getattr(self, 'left_controller_state', None)
        right = getattr(self, 'right_controller_state', None)
        left = left if isinstance(left, dict) else {}
        right = right if isinstance(right, dict) else {}
        events = self._tact.update(bool(left.get('thumbstick', False)),
                                   bool(right.get('thumbstick', False)), time.monotonic())
        for name in events:
            if name == 'both':
                continue  # both-click stays the original lift/head <-> base mode toggle
            self._tact_pub.publish(String(data=name))
            self.get_logger().info(f'[TACT] {name} -> {TACT_TRIGGER_TOPIC}')

    def _snapshot(self):
        links = super()._snapshot()
        if self._session is None:
            return links
        if self.hardware_model == 'sh5':
            # [CHANGED] The SH5 lift is a vertical prismatic joint. VR inputs
            # are relative to base_link; without transporting the reference the
            # arm QP counter-moves to HOLD a world-fixed wrist as lift descends.
            # Use real joint feedback (not the last commanded lift) to translate
            # BOTH arms by one common displacement, including clutch/HOLD frames.
            current = float(self.lift_joint_current_position)
            if not np.isfinite(current):
                raise ValueError('nonfinite measured SH5 lift')
            previous = self._lift_follow_position
            self._lift_follow_position = current
            dz = 0.0 if previous is None else current - previous
            deltas = {side: np.array([0.0, 0.0, dz]) for side in SIDES}
        else:
            deltas = {}
            for side, template in self._session.templates.items():
                delta = links[side]['shoulder'][:3, 3] - template.shoulder
                if not np.isfinite(delta).all():
                    raise ValueError('nonfinite measured shoulder change')
                # SG2 legacy behavior preserved: only measured vertical lift
                # translation may move the shoulder in base_link.
                if np.linalg.norm(delta[:2]) > .005:
                    raise ValueError('unexpected horizontal shoulder/model change')
                deltas[side] = delta
        # [CHANGED] Transport every anchor, hint and in-flight target exactly
        # once per measured lift increment. Preserve wrist orientations and
        # human input anchors; never modify an unverified motor joint command.
        for side, delta in deltas.items():
            if np.linalg.norm(delta) < 1e-9:
                continue
            self._session.mapper.anchors[side].robot_pose[:3, 3] += delta
            for hints in (self._session.templates, self._hints):
                hint = hints.get(side)
                if hint is None:
                    continue
                hint.shoulder += delta
                hint.previous += delta
                if hasattr(hint, 'path_anchor'):
                    hint.path_anchor += delta
            for poses in (self._goals, self._elbows, self._last_published,
                          self._resume_bridge_targets or {}):
                if side in poses:
                    poses[side][:3, 3] += delta
        return links


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
