#!/usr/bin/env python3
"""[ADDED] Robot-side arms-only relay with PC lease and measured-state watchdog.

Only this relay writes the follower command topics. Cyclo writes private input
topics. This is an additional software check, NOT a certified emergency stop.
The physical emergency stop and normal hardware controller remain required.
"""
import argparse
import hashlib
import json
import math
import os
import signal
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from dynamixel_interfaces.msg import DynamixelState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from hardware.aux_controls import (AUX_TOPICS, JOINT_GROUPS, GRIPPERS, validate_aux, slew_aux,
                                   BASE_LIMITS, mobile_speed, scale_base_command)

from hardware.xr_contract import (
    ARM_NAMES, SIDES, FOLLOWER_TOPICS, GATE_NAME, PC_NAME, LEASE_SECONDS,
    check_positions, arm_position_tolerance, check_step, limit_step, load_profile, private_topic, verify_description,
    motor_health, motor_read_health, configured_arm_ids, motor_readiness,
)
from hardware.torque_evidence import TorqueEvidence

FEEDBACK_FRESH_SECONDS = 5.0
FEEDBACK_WAIT_SECONDS = 50.0
PC_SESSION_TIMEOUT_SECONDS = 1.5

def deployment_domain():
    mode = os.environ.get('AI_WORKER_XR_EXECUTION', '')
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    if mode == 'validation' and domain == '232':
        return
    if mode != 'hardware' or domain != '30':
        raise RuntimeError('Explicit hardware mode requires domain30; validation requires domain232')


class HardwareCommandGate(Node):
    def __init__(self, profile_path):
        deployment_domain()
        self.profile = load_profile(profile_path)
        super().__init__(GATE_NAME)
        self.profile['hand_input'] = os.environ.get('AI_WORKER_SH5_INPUT','hands')
        self.profile['hand_mode'] = int(os.environ.get('AI_WORKER_SH5_HAND_MODE','1'))
        self.base_speed = mobile_speed(os.environ.get('AI_WORKER_BASE_SPEED', '.2'), BASE_LIMITS[0])
        self.turn_speed = mobile_speed(os.environ.get('AI_WORKER_TURN_SPEED', str(1/3)), BASE_LIMITS[2])
        self.profile['base_speed_m_s'] = self.base_speed
        self.profile['turn_speed_rad_s'] = self.turn_speed
        if self.profile['hand_mode'] not in (1,2):
            raise ValueError('hand mode must be 1 or 2')
        if self.profile['model']=='sh5' and self.profile.get('finger_limits'):
            from hardware.sh5_controller_hands import HAND_SPEED_SCALE, MODE2_TRIGGER_CLOSE_SPEED_SCALE
            direct = self.profile['hand_input']=='hands' or self.profile['hand_mode']==2
            self.profile['hand_speed_scale'] = None if direct else HAND_SPEED_SCALE
            # [ADDED 2026-10-28] Diagnostic only: controller mode 2 trigger closing.
            self.profile['hand_trigger_close_scale'] = (
                HAND_SPEED_SCALE*MODE2_TRIGGER_CLOSE_SPEED_SCALE if self.profile['hand_input']=='controllers'
                and self.profile['hand_mode']==2 else None)
            self.profile['hand_output_policy'] = ('xr_ai_dexpilot' if self.profile['hand_input']=='hands' and self.profile['hand_mode']==2 else
                                                   'stock_robotis' if self.profile['hand_input']=='hands' else
                                                   'direct_position' if direct else 'velocity_limited')
            self.profile['hand_input_timeout_s'] = None if direct else .3
            self.profile['hand_output_cadence'] = ('latest_xr_samples' if self.profile['hand_input']=='hands' and self.profile['hand_mode']==2 else
                                                  'native_retargeter' if self.profile['hand_input']=='hands' else 'gate_timer')
        public_profile = {k: v for k, v in self.profile.items()
                          if k not in ('model_report', 'initial_collision_report')}
        self.declare_parameter('profile_json', json.dumps(public_profile),
                               ParameterDescriptor(read_only=True))
        self.model = self.profile['model']
        self.controller_hands = None
        if self.model=='sh5' and self.profile.get('finger_limits') and (self.profile['hand_input']=='controllers' or self.profile['hand_mode']==2):
            from hardware.sh5_controller_hands import SH5HandOutput
            self.controller_hands = SH5HandOutput(self,self.profile['finger_limits'],self.profile['hand_input'])
        self.aux_collision = None
        if self.profile.get('manual_auxiliary'):
            from hardware.check_collision import AuxiliaryCollisionGuard
            self.aux_collision = AuxiliaryCollisionGuard(self.profile)
        # [CHANGED] Old profiles remain strict; normal launcher selects explicitly.
        self.readiness_policy = self.profile.get('readiness_policy', 'strict_torque')
        self.description_ok = False
        self.measured = {}
        self.measured_rx = -math.inf
        self.measured_stamp = -math.inf
        self.motor_rx = -math.inf
        self.motor_stamp = -math.inf
        self.motor_ids = None
        self.motor_ok = False
        # [FIX] Buffered service evidence disabled; compatibility object cannot authorise motion.
        self.torque_evidence = TorqueEvidence(self)
        self.motor_topic_ok = False
        self.motor_read_ok = False  # [CHANGED] Do not confuse read health with torque proof.
        self.intent_rx = -math.inf
        self.intent_enabled = False
        self.pc_intent_enabled = False
        self.pc_reconnect_disarm_required = False
        self.enabled_since = None
        self.commands = {}
        self.previous = {}
        self.last_output = None
        self.fault = None
        self.fault_detail = None  # [CHANGED 2026-10-14] Preserve the first rejected frame before hold clears it.
        self.output_active = False
        self.feedback_waiting = False
        self.aux_command = {'joints': {}, 'base': [0., 0., 0.]}
        self.aux_previous = {}
        self.aux_active = False
        self.manual_enabled = False
        self.manual_rx = -math.inf
        self.manual_last_output = time.monotonic()
        self.reason = 'waiting for measured robot_description/joints and manual X+A'
        self.sequence = -1
        self.gate_instance = uuid.uuid4().hex
        self.status_sequence = 0
        self.last_cleared_fault = None
        self.pc_session = None
        self.last_status = -math.inf
        self.last_graph = -math.inf
        self.graph_ok = False
        self.pub = {s: self.create_publisher(JointTrajectory, FOLLOWER_TOPICS[s], 10) for s in SIDES}
        self.aux_pub = ({group: self.create_publisher(JointTrajectory, topic, 10)
                        for group, topic in AUX_TOPICS.items()} if self.profile['auxiliary_limits'] else {})
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', 10) if self.aux_pub else None
        self.stop_pub = self.create_publisher(Bool, '/reactivate', 10)
        self.fault_pub = self.create_publisher(Bool, '/xr_hardware/fault', 10)
        self.status_pub = self.create_publisher(String, '/xr_hardware/gate_status', 10)
        self.measured_pub = self.create_publisher(JointState, f'/xr_hardware/{self.model}/joint_states', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        description_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs = [
            self.create_subscription(JointState, '/joint_states', self.joints, qos),
            self.create_subscription(DynamixelState, '/ffw_follower/dxl_state', self.motor_state, qos),
            self.create_subscription(String, '/robot_description', self.description, description_qos),
            self.create_subscription(String, '/xr_hardware/intent', self.intent, 10),
            self.create_subscription(Bool, '/reference_diverged', self.reference_fault, 10),
        ]
        for s in SIDES:
            self.subs.append(self.create_subscription(
                JointTrajectory, private_topic(self.model, s), lambda msg, side=s: self.command(msg, side), qos))
        # Match the native SH5 arm controller's 100 Hz streaming cadence.
        # The gate still applies measured-state checks and URDF velocity slew.
        self.timer = self.create_timer(.01 if self.model == 'sh5' else .02, self.tick)
        self.reset_service = self.create_service(Trigger, '/xr_hardware/reset_fault', self.reset_fault)
        self.get_logger().info(f'[HARDWARE GATE] {self.model}; inactive; auxiliary controls={bool(self.aux_pub)}.')
        self.get_logger().info('[HARDWARE GATE LIMITS] ' + json.dumps({
            'source': 'verified controller URDF', 'positions_rad': self.profile['limits'],
            'velocity_rad_s': self.profile['joint_velocity_limits'],
            'tracking_window_s': self.profile['tracking_window_seconds'],
            'tracking_budget_rad': self.profile['tracking_error_limits'],
            'tracking_policy': self.profile['tracking_policy'],
        }))

    def description(self, msg):
        try:
            verify_description(msg.data, self.model, self.profile['arm_signature'])
            if configured_arm_ids(msg.data, self.model) != self.profile['arm_dxl_ids']:
                raise ValueError('physical motor ID mapping differs from profile')
            if hashlib.sha256(msg.data.encode()).hexdigest() != self.profile['description_sha256']:
                raise ValueError('live robot_description changed since model snapshot')
            self.description_ok = True
        except Exception as error:
            self.description_ok = False
            self.reason = str(error)
            if self.intent_enabled:
                self.latch(str(error))

    def joints(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        required = set().union(*ARM_NAMES.values())
        if (len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name)
                or not required.issubset(msg.name) or not all(math.isfinite(x) for x in msg.position)):
            self.measured_rx = -math.inf
            return
        if stamp <= self.measured_stamp:
            return  # Replayed samples must not refresh the watchdog.
        self.measured = dict(zip(msg.name, msg.position))
        self.measured_stamp, self.measured_rx = stamp, time.monotonic()
        if self.intent_enabled and self.profile.get('arm_limit_policy') == 'sh5_bent_elbow_v1':
            for side in ('l','r'):
                name=f'arm_{side}_joint4'
                if self.measured[name] > self.profile['limits'][name][1]+.03:
                    self.latch('measured elbow exceeded operational extension bound: '+name)
                    return
        # [FIX] Old Cyclo caches name order from its FIRST JointState. Never feed
        # reordered/partial inputs or freshly stamped cached state without DXL health.
        canonical = self.profile['controller_joint_names']
        if (self.description_ok and self.motor_ok and time.monotonic()-self.motor_rx <= FEEDBACK_FRESH_SECONDS
                and set(canonical).issubset(self.measured)):
            clean = JointState()
            clean.header = msg.header
            clean.name = list(canonical)
            clean.position = [self.measured[k] for k in canonical]
            if self.profile.get('live_wheel_feedback'):
                from hardware.sh5_hands_model import scalar_sensor_position
                clean.position = [scalar_sensor_position(k,v) for k,v in zip(canonical,clean.position)]
            velocity = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
            clean.velocity = [float(velocity.get(k, 0.)) for k in canonical]
            if all(math.isfinite(v) for v in clean.velocity):
                self.measured_pub.publish(clean)

    def motor_state(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if stamp <= self.motor_stamp:
            return
        ids = tuple(sorted(msg.id))
        # [CHANGED] Keep bus/ID/error checks in BOTH policies. Normal operation
        # uses the existing driver's torque management, not a broken extra proof.
        bus_ok = motor_read_health(
            msg.id, msg.dxl_hw_state, msg.comm_state, self.profile['arm_dxl_ids'])
        if self.profile.get('finger_limits'):
            # Also reject errors on any reported hand/robot device. This does
            # not claim missing temperature/board registers were measured.
            bus_ok = bus_ok and motor_read_health(msg.id,msg.dxl_hw_state,msg.comm_state,{'reported':list(msg.id)})
        reported_ok = motor_health(
            msg.id, msg.torque_state, msg.dxl_hw_state, msg.comm_state,
            self.profile['arm_dxl_ids'])
        if self.motor_ids is not None and ids != self.motor_ids:
            bus_ok = False
        ok = bool(bus_ok and motor_readiness(
            msg.id, msg.torque_state, msg.dxl_hw_state, msg.comm_state,
            self.readiness_policy, self.profile['arm_dxl_ids']))
        self.motor_read_ok = bool(bus_ok and len(msg.torque_state) == len(msg.id))
        self.motor_topic_ok = bool(reported_ok)
        if ok:
            self.motor_ids = ids
        self.motor_stamp, self.motor_rx, self.motor_ok = stamp, time.monotonic(), ok
        if not ok and self.intent_enabled:
            self.latch('Dynamixel feedback rejected by ' + self.readiness_policy +
                       ': bus/ID/hardware error or required reported torque missing')

    def intent(self, msg):
        try:
            data = json.loads(msg.data)
            if data['model'] != self.model or not isinstance(data['enabled'], bool):
                raise ValueError('wrong model or invalid PC lease')
            sequence, session = int(data['sequence']), str(data['session_id'])
            if session == self.pc_session and sequence <= self.sequence:
                return
            if session != self.pc_session and data['enabled']:
                # A new sender/session must first be observed DISARMED.
                raise ValueError('new PC session attempted to begin already enabled')
            self.pc_session, self.sequence = session, sequence
            aux = validate_aux(data.get('aux', {}), self.profile['auxiliary_limits'])
            manual = data.get('manual_enabled', False)
            if not isinstance(manual, bool) or (manual and not self.profile.get('manual_auxiliary')):
                raise ValueError('manual controls require a measured-head SH5 profile')
            if manual:
                aux = validate_aux(data.get('manual_aux', {}), self.profile['auxiliary_limits'])
            self.intent_rx = time.monotonic()
            if self.controller_hands and self.profile['hand_input']=='controllers':
                if type(data.get('hand_mode',1)) is not int or data.get('hand_mode',1)!=self.profile['hand_mode']:
                    raise ValueError('PC/robot --hand-mode must match')
                self.controller_hands.receive(data.get('controller_hands'),self.intent_rx)
            elif data.get('controller_hands') is not None:
                raise ValueError('PC controller input does not match robot hand mode')
            enabled = data['enabled']
            if not enabled:
                self.pc_reconnect_disarm_required = False
            elif self.pc_reconnect_disarm_required:
                enabled = manual = False
            self.pc_intent_enabled = enabled
            if enabled and not self.intent_enabled:
                self.enabled_since = time.monotonic()
                self.commands.clear()
                self.previous = {k: self.measured[k] for k in self.profile['limits'] if k in self.measured}
                self.last_output = time.monotonic()
            if not enabled and self.intent_enabled:
                self.hold('manual disarm or publisher guard')
            self.intent_enabled = enabled
            if self.manual_enabled and not manual and self.aux_active:
                self.emit_aux({k: self.measured[k] for k in self.aux_previous if k in self.measured}, [0., 0., 0.])
                self.aux_previous.clear()
            self.manual_enabled, self.manual_rx = manual, time.monotonic()
            self.aux_command = aux if enabled or manual else {'joints': {}, 'base': [0., 0., 0.]}
        except (KeyError, ValueError, TypeError) as error:
            if self.intent_enabled or self.manual_enabled:
                self.latch('invalid lease: ' + str(error))

    def command(self, msg, side):
        if not msg.points:
            return
        try:
            # [CHANGED 2026-10-14] This relay is for native t=0 streaming only.
            # Never compress an intentional multi-point/future trajectory into
            # an immediate endpoint while preserving the streaming contract.
            if (len(msg.points) != 1 or msg.points[0].time_from_start.sec != 0
                    or msg.points[0].time_from_start.nanosec != 0):
                raise ValueError('expected one native immediate Cyclo setpoint (time_from_start=0)')
            target = check_positions(side, msg.joint_names, msg.points[-1].positions,
                                     self.profile['limits'], arm_position_tolerance(self.model))
            self.commands[side] = (target, time.monotonic())
        except (ValueError, TypeError) as error:
            if self.intent_enabled:
                self.latch('invalid Cyclo arm output: ' + str(error))

    def reference_fault(self, msg):
        if msg.data and self.intent_enabled:
            self.latch('reference checker/controller fault')

    def graph(self):
        if self.controller_hands:self.controller_hands.graph()
        elif self.model=='sh5' and self.profile.get('finger_limits'):
            from hardware.sh5_controller_hands import HAND_TOPICS
            for topic in HAND_TOPICS.values():
                writers=self.get_publishers_info_by_topic(topic)
                if len(writers)!=1 or writers[0].node_name!='retargeting_teleop':
                    raise ValueError(f'{topic}: require original ROBOTIS retargeting_teleop writer')
        if any(name == 'arm_retargeting_teleop' for name, _ in self.get_node_names_and_namespaces()):
            raise ValueError('legacy arm retargeter must be stopped')
        if self.aux_pub:
            for topic in (*AUX_TOPICS.values(), '/cmd_vel'):
                writers = self.get_publishers_info_by_topic(topic)
                if len(writers) != 1 or writers[0].node_name != GATE_NAME:
                    raise ValueError(f'{topic}: require one hardware gate writer')
        for side in SIDES:
            raw = self.get_publishers_info_by_topic(private_topic(self.model, side))
            actual = self.get_publishers_info_by_topic(FOLLOWER_TOPICS[side])
            if len(raw) != 1 or raw[0].node_name != 'vr_controller':
                raise ValueError('require one Cyclo private command writer')
            if len(actual) != 1 or actual[0].node_name != GATE_NAME:
                raise ValueError('another publisher bypasses the hardware command gate')
        intent = self.get_publishers_info_by_topic('/xr_hardware/intent')
        if len(intent) != 1 or intent[0].node_name != PC_NAME:
            raise ValueError('require exactly one selected hardware PC publisher')
        for topic in ('/joint_states', '/robot_description', '/ffw_follower/dxl_state'):
            if self.count_publishers(topic) != 1:
                raise ValueError(f'{topic}: require exactly one physical feedback authority')

    def emit(self, target):
        # [CHANGED 2026-10-14] Preserve native Cyclo streaming semantics:
        # zero header = start on receipt, single point at t=0 = current setpoint.
        # Replacing a 100ms interpolation every 20ms adds lag to an already
        # integrated/velocity-checked stream. Do not retime it inside this relay.
        for side in SIDES:
            msg = JointTrajectory()
            msg.joint_names = list(ARM_NAMES[side])
            gripper = GRIPPERS[side]
            grip_value = target.get(gripper, self.aux_previous.get(gripper))
            if grip_value is not None:
                msg.joint_names.append(gripper)
            point = JointTrajectoryPoint()
            point.positions = [float(target[k]) if k in target else float(grip_value) for k in msg.joint_names]
            msg.points = [point]
            self.pub[side].publish(msg)

    def emit_aux(self, joints, base):
        for group, publisher in self.aux_pub.items():
            names = JOINT_GROUPS[group]
            if not any(name in joints for name in names):
                continue
            if not all(name in joints or name in self.measured for name in names):
                raise ValueError(f'{group}: measured auxiliary state missing')
            msg = JointTrajectory(joint_names=list(names))
            msg.points = [JointTrajectoryPoint(positions=[float(joints.get(k, self.measured.get(k))) for k in names])]
            publisher.publish(msg)
        if self.base_pub is not None:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = scale_base_command(base, self.base_speed, self.turn_speed)
            self.base_pub.publish(msg)
        self.aux_active = bool(joints or any(base))

    def hold(self, reason):
        if self.controller_hands:
            self.controller_hands.hold(self.measured,time.monotonic()-self.measured_rx <= .5)
        self.manual_enabled = False
        self.feedback_waiting = False
        # [SAFETY] One measured position-hold, only after this gate was driving.
        if self.output_active and time.monotonic()-self.measured_rx <= FEEDBACK_FRESH_SECONDS:
            self.emit(self.measured)
        if self.aux_pub:
            held = ({k: self.measured[k] for k in self.aux_previous if k in self.measured}
                    if self.aux_active and time.monotonic()-self.measured_rx <= FEEDBACK_FRESH_SECONDS else {})
            self.emit_aux(held, [0., 0., 0.])
        self.aux_command = {'joints': {}, 'base': [0., 0., 0.]}
        self.aux_previous.clear()
        self.aux_active = False
        self.output_active = False
        self.commands.clear()
        self.reason = reason
        self.stop_pub.publish(Bool(data=False))

    def latch(self, reason):
        if self.fault is None:
            self.fault = reason
            now = time.monotonic()
            def finite(value):
                return float(value) if value is not None and math.isfinite(value) else None
            self.fault_detail = {
                'origin': 'xr_hardware_gate',
                'reason': str(reason),
                'ros_time_s': self.get_clock().now().nanoseconds * 1e-9,
                'measured_age_s': finite(now-self.measured_rx),
                'joints': {name: {
                    'target_rad': finite(value),
                    'measured_rad': finite(self.measured.get(name)),
                    'previous_output_rad': finite(self.previous.get(name)),
                    'urdf_velocity_rad_s': self.profile['joint_velocity_limits'][name],
                    'tracking_budget_rad': self.profile['tracking_error_limits'][name],
                    'command_age_s': finite(now-receipt),
                } for values, receipt in self.commands.values() for name, value in values.items()},
            }
            self.hold(reason)
            self.get_logger().error('[HARDWARE GATE FAULT] ' + reason + '; disarm, resolve cause, then X+A to reset/retry')
            self.get_logger().error('[HARDWARE GATE FRAME] ' + json.dumps(self.fault_detail, allow_nan=False))
        self.intent_enabled = False
        self.fault_pub.publish(Bool(data=True))

    def reset_fault(self, request, response):
        """Explicit manual retry; clear the latch only while disarmed and healthy."""
        try:
            now = time.monotonic()
            if self.intent_enabled or self.pc_intent_enabled:
                raise ValueError('disarm PC intent before resetting fault')
            if now-self.intent_rx > LEASE_SECONDS:
                raise ValueError('fresh disarmed PC lease required')
            if not self.description_ok or now-self.measured_rx > FEEDBACK_FRESH_SECONDS:
                raise ValueError('fresh verified model/joint feedback required')
            if not self.motor_ok or now-self.motor_rx > FEEDBACK_FRESH_SECONDS:
                raise ValueError('healthy motor feedback required')
            if self.readiness_policy == 'strict_torque' and not self.motor_topic_ok:
                raise ValueError('required torque telemetry not healthy')
            for name,(lower,upper) in self.profile['limits'].items():
                if name not in self.measured or not lower-.005 <= self.measured[name] <= upper+.005:
                    raise ValueError(f'{name}: measured joint remains outside URDF bounds')
            self.graph()
            self.last_cleared_fault = {'fault': self.fault, 'detail': self.fault_detail}
            self.hold('manual fault reset; waiting for fresh measured start')
            self.fault = self.fault_detail = None
            self.commands.clear()
            self.fault_pub.publish(Bool(data=False))
            self.last_status = -math.inf
            response.success = True
            response.message = json.dumps({'gate_instance': self.gate_instance,
                                           'status_sequence': self.status_sequence})
        except (ValueError, KeyError) as error:
            response.success = False
            response.message = str(error)
        return response

    def tick(self):
        # [FIX] No get_dxl_data polling inside the physical controller loop.
        now = time.monotonic()
        if self.readiness_policy == 'strict_torque' and not self.motor_topic_ok:
            self.motor_ok = False
        if (self.model=='sh5' and self.intent_enabled
                and now-self.intent_rx>PC_SESSION_TIMEOUT_SECONDS):
            # Vuer may be restarted independently. Disarm without killing the
            # follower or latching a fault; reconnection must explicitly enable.
            self.hold('PC/Vuer disconnected; robot session retained, waiting for manual resume')
            self.intent_enabled = self.pc_intent_enabled = False
            self.pc_reconnect_disarm_required = True
        try:
            # Manual keyboard motion can continue while arms are explicitly held.
            # It uses the same unique follower writer, physical limits and motor
            # checks, with a shorter lease than the VR tracking grace window.
            if self.manual_enabled and self.fault is None:
                fresh = (now-self.manual_rx <= .35 and self.description_ok and self.motor_ok
                         and now-self.motor_rx <= .5 and now-self.measured_rx <= .5)
                if not fresh:
                    self.hold('manual keyboard lease/feedback expired')
                elif not self.intent_enabled:
                    if now-self.last_graph > .25:
                        self.graph()
                        self.last_graph = now
                    output = slew_aux(self.aux_command['joints'], self.aux_previous, self.measured,
                                      self.profile['auxiliary_limits'], now-self.manual_last_output)
                    if self.aux_collision:self.aux_collision.validate(self.measured, output)
                    self.emit_aux(output, self.aux_command['base'])
                    self.aux_previous, self.manual_last_output = output, now
            if self.intent_enabled and self.fault is None:
                if now-self.intent_rx > FEEDBACK_WAIT_SECONDS:
                    raise ValueError('PC intent lease expired')
                if not self.description_ok:
                    raise ValueError('physical robot description not verified')
                if now-self.measured_rx > FEEDBACK_WAIT_SECONDS:
                    raise ValueError('measured joint feedback stale')
                if not self.motor_ok or now-self.motor_rx > FEEDBACK_WAIT_SECONDS:
                    raise ValueError('motor read health stale or unhealthy; JointState stamp alone is insufficient')
                if now-self.last_graph > .25:
                    self.graph()
                    self.last_graph = now
                for name, (lower, upper) in self.profile['limits'].items():
                    if not lower-.005 <= self.measured[name] <= upper+.005:
                        raise ValueError(f'{name} measured outside commissioning model limits')
                for name, frozen in self.profile.get('frozen_joints', {}).items():
                    if name not in self.measured or abs(self.measured[name]-frozen) > .01:
                        raise ValueError(f'{name}: frozen model joint moved; recapture profile with controller stopped')
                # Startup is allowed its existing3s arming, but never emits stale goals.
                if len(self.commands) < 2:
                    if now-self.enabled_since > 10.:
                        raise ValueError('no complete Cyclo output during startup')
                else:
                    if any(now-t > FEEDBACK_WAIT_SECONDS for _, t in self.commands.values()):
                        raise ValueError('Cyclo output stale')
                    waiting = (now-self.measured_rx > FEEDBACK_FRESH_SECONDS
                               or now-self.intent_rx > FEEDBACK_FRESH_SECONDS
                               or now-self.motor_rx > FEEDBACK_FRESH_SECONDS
                               or any(now-t > FEEDBACK_FRESH_SECONDS for _, t in self.commands.values())
                               or max(t for _, t in self.commands.values())
                                  - min(t for _, t in self.commands.values()) > 1.0)
                    if waiting:
                        self.feedback_waiting = True
                        # Keep the existing setpoint; never advance a stale QP output.
                        if self.output_active:
                            self.emit(self.previous)
                        if self.aux_pub:
                            self.emit_aux(self.aux_previous, [0., 0., 0.])
                        self.last_output = now
                        self.reason = f'waiting for robot/Cyclo feedback (maximum age {FEEDBACK_WAIT_SECONDS:g}s); holding targets'
                    else:
                        self.feedback_waiting = False
                    if waiting:
                        if self.controller_hands:self.controller_hands.hold(self.measured,now-self.measured_rx<=.5)
                        self.publish_status(now)
                        return
                    target = {k: v for values, _ in self.commands.values() for k, v in values.items()}
                    elapsed = max(.001, now-self.last_output)
                    if self.readiness_policy == 'driver_managed':
                        # [CHANGED 2026-10-14] The physical-only relay runs at a
                        # different cadence from Cyclo. Do not latch a fault when
                        # a valid fresh target advances faster than this relay's
                        # instantaneous tick spacing; slew-limit what reaches the
                        # follower. Normal mode uses URDF bounds/slew instead
                        # of latching extra following-error thresholds.
                        output = limit_step(target, self.measured, self.previous, elapsed,
                                            self.profile['joint_velocity_limits'], enforce_tracking=False)
                    else:
                        check_step(target, self.measured, self.previous, elapsed,
                                   self.profile['joint_velocity_limits'])
                        output = target
                    if self.aux_pub:
                        aux_output = slew_aux(self.aux_command['joints'], self.aux_previous,
                                              self.measured, self.profile['auxiliary_limits'], elapsed)
                        if self.aux_collision:self.aux_collision.validate(self.measured, aux_output)
                        self.aux_previous = aux_output
                    self.emit(output)
                    if self.aux_pub:
                        self.emit_aux(aux_output, self.aux_command['base'])
                    self.previous, self.last_output = output, now
                    self.output_active = True
                    self.reason = 'forwarding validated arm and auxiliary targets'
        except (ValueError, KeyError) as error:
            self.latch(str(error))
        if self.controller_hands:
            try:
                self.update_hand_output(now)
            except (ValueError,KeyError) as error:self.latch('controller hands: '+str(error))
        self.publish_status(now)

    def update_hand_output(self,now):
        hand=self.controller_hands
        allowed=(self.intent_enabled and self.fault is None and self.output_active
                 and not self.feedback_waiting and self.motor_ok and (hand.direct_output or (
                     now-self.intent_rx <= .3 and now-self.measured_rx <= .5 and now-self.motor_rx <= .5)))
        if hand.mode=='hands':
            # Optical targets belong solely to native input callbacks. The
            # arm/watchdog timer only changes hand authority on transitions.
            if hand.active!=allowed:hand.update_authority(self.measured,now,allowed)
        else:hand.tick(self.measured,now,allowed)

    def publish_status(self, now):
        if now-self.last_status >= .1:
            self.last_status = now
            self.status_sequence += 1
            self.status_pub.publish(String(data=json.dumps({
                'gate_instance': self.gate_instance, 'status_sequence': self.status_sequence,
                'model': self.model, 'fault': self.fault, 'reason': self.reason,
                'fault_detail': self.fault_detail,
                'description_ok': self.description_ok, 'enabled': self.intent_enabled,
                'forwarding': self.output_active, 'measured_fresh': now-self.measured_rx <= FEEDBACK_FRESH_SECONDS,
                'feedback_waiting': self.feedback_waiting,
                'manual_active': self.manual_enabled and self.aux_active,
                'motor_healthy': self.motor_ok and now-self.motor_rx <= FEEDBACK_FRESH_SECONDS,
                # [CHANGED] Status preserves uncertainty; never claims register ON.
                'readiness_policy': self.readiness_policy,
                'motor_read_healthy': self.motor_read_ok and now-self.motor_rx <= FEEDBACK_FRESH_SECONDS,
                'reported_torque_all_on': self.motor_topic_ok,
                'physical_torque_verified': False,
                'ros_time_s': self.get_clock().now().nanoseconds*1e-9,
            })))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True)
    a, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = HardwareCommandGate(a.profile)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.hold('hardware gate shutting down')
            rclpy.spin_once(node, timeout_sec=.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
