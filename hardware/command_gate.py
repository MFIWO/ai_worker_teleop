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

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from dynamixel_interfaces.msg import DynamixelState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from hardware.xr_contract import (
    ARM_NAMES, SIDES, FOLLOWER_TOPICS, GATE_NAME, PC_NAME, LEASE_SECONDS,
    check_positions, check_step, load_profile, private_topic, verify_description, motor_health, configured_arm_ids,
)


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
        public_profile = {k: v for k, v in self.profile.items()
                          if k not in ('model_report', 'initial_collision_report')}
        self.declare_parameter('profile_json', json.dumps(public_profile),
                               ParameterDescriptor(read_only=True))
        self.model = self.profile['model']
        self.description_ok = False
        self.measured = {}
        self.measured_rx = -math.inf
        self.measured_stamp = -math.inf
        self.motor_rx = -math.inf
        self.motor_stamp = -math.inf
        self.motor_ids = None
        self.motor_ok = False
        self.intent_rx = -math.inf
        self.intent_enabled = False
        self.enabled_since = None
        self.commands = {}
        self.previous = {}
        self.last_output = None
        self.fault = None
        self.output_active = False
        self.reason = 'waiting for measured robot_description/joints and manual X+A'
        self.sequence = -1
        self.pc_session = None
        self.last_status = -math.inf
        self.last_graph = -math.inf
        self.graph_ok = False
        self.pub = {s: self.create_publisher(JointTrajectory, FOLLOWER_TOPICS[s], 10) for s in SIDES}
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
        self.timer = self.create_timer(.02, self.tick)
        self.get_logger().info(f'[HARDWARE GATE] {self.model}; inactive, seven arm joints only; no fingers/base/lift.')

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
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        required = set().union(*ARM_NAMES.values())
        if (len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name)
                or not required.issubset(msg.name) or not all(math.isfinite(x) for x in msg.position)
                or not -.1 <= now - stamp <= .5):
            self.measured_rx = -math.inf
            return
        if stamp <= self.measured_stamp:
            return  # Replayed samples must not refresh the watchdog.
        self.measured = dict(zip(msg.name, msg.position))
        self.measured_stamp, self.measured_rx = stamp, time.monotonic()
        # [FIX] Old Cyclo caches name order from its FIRST JointState. Never feed
        # reordered/partial inputs or freshly stamped cached state without DXL health.
        canonical = self.profile['controller_joint_names']
        if (self.description_ok and self.motor_ok and time.monotonic()-self.motor_rx <= .5
                and set(canonical).issubset(self.measured)):
            clean = JointState()
            clean.header = msg.header
            clean.name = list(canonical)
            clean.position = [self.measured[k] for k in canonical]
            velocity = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
            clean.velocity = [float(velocity.get(k, 0.)) for k in canonical]
            if all(math.isfinite(v) for v in clean.velocity):
                self.measured_pub.publish(clean)

    def motor_state(self, msg):
        now_ros = self.get_clock().now().nanoseconds*1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if stamp <= self.motor_stamp or not -.1 <= now_ros-stamp <= .5:
            return
        ids = tuple(sorted(msg.id))
        ok = motor_health(msg.id, msg.torque_state, msg.dxl_hw_state, msg.comm_state,
                          self.profile['arm_dxl_ids'])
        if self.motor_ids is not None and ids != self.motor_ids:
            ok = False
        if ok:
            self.motor_ids = ids
        self.motor_stamp, self.motor_rx, self.motor_ok = stamp, time.monotonic(), bool(ok)
        if not ok and self.intent_enabled:
            self.latch('Dynamixel communication/torque/hardware-health failure')

    def intent(self, msg):
        try:
            data = json.loads(msg.data)
            stamp = float(data['ros_time_s'])
            now_ros = self.get_clock().now().nanoseconds*1e-9
            if (data['model'] != self.model or not isinstance(data['enabled'], bool)
                    or not math.isfinite(stamp) or not -.1 <= now_ros-stamp <= .5):
                raise ValueError('wrong model or invalid/stale PC lease')
            sequence, session = int(data['sequence']), str(data['session_id'])
            if session == self.pc_session and sequence <= self.sequence:
                return
            if session != self.pc_session and data['enabled']:
                # A new sender/session must first be observed DISARMED.
                raise ValueError('new PC session attempted to begin already enabled')
            self.pc_session, self.sequence = session, sequence
            self.intent_rx = time.monotonic()
            enabled = data['enabled']
            if enabled and not self.intent_enabled:
                self.enabled_since = time.monotonic()
                self.commands.clear()
                self.previous = {k: self.measured[k] for k in self.profile['limits'] if k in self.measured}
                self.last_output = time.monotonic()
            if not enabled and self.intent_enabled:
                self.hold('manual disarm or publisher guard')
            self.intent_enabled = enabled
        except (KeyError, ValueError, TypeError) as error:
            if self.intent_enabled:
                self.latch('invalid lease: ' + str(error))

    def command(self, msg, side):
        if not msg.points:
            return
        try:
            target = check_positions(side, msg.joint_names, msg.points[-1].positions, self.profile['limits'])
            self.commands[side] = (target, time.monotonic())
        except (ValueError, TypeError) as error:
            if self.intent_enabled:
                self.latch('invalid Cyclo arm output: ' + str(error))

    def reference_fault(self, msg):
        if msg.data and self.intent_enabled:
            self.latch('reference checker/controller fault')

    def graph(self):
        if any(name == 'arm_retargeting_teleop' for name, _ in self.get_node_names_and_namespaces()):
            raise ValueError('legacy arm retargeter must be stopped')
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
        stamp = self.get_clock().now().to_msg()
        for side in SIDES:
            msg = JointTrajectory()
            msg.header.stamp = stamp
            msg.joint_names = list(ARM_NAMES[side])
            point = JointTrajectoryPoint()
            point.positions = [float(target[k]) for k in msg.joint_names]
            point.time_from_start.nanosec = 100_000_000
            msg.points = [point]
            self.pub[side].publish(msg)

    def hold(self, reason):
        # [SAFETY] One measured position-hold, only after this gate was driving.
        if self.output_active and time.monotonic()-self.measured_rx <= .5:
            self.emit(self.measured)
        self.output_active = False
        self.commands.clear()
        self.reason = reason
        self.stop_pub.publish(Bool(data=False))

    def latch(self, reason):
        if self.fault is None:
            self.fault = reason
            self.hold(reason)
            self.get_logger().error('[HARDWARE GATE FAULT] ' + reason + '; inspect then restart gate and PC')
        self.intent_enabled = False
        self.fault_pub.publish(Bool(data=True))

    def tick(self):
        now = time.monotonic()
        try:
            if self.intent_enabled and self.fault is None:
                if now-self.intent_rx > LEASE_SECONDS:
                    raise ValueError('PC intent lease expired')
                if not self.description_ok:
                    raise ValueError('physical robot description not verified')
                if now-self.measured_rx > .5:
                    raise ValueError('measured joint feedback stale')
                if not self.motor_ok or now-self.motor_rx > .5:
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
                    if any(now-t > .5 for _, t in self.commands.values()):
                        raise ValueError('Cyclo output stale')
                    receipts = [t for _, t in self.commands.values()]
                    if max(receipts)-min(receipts) > .10:
                        raise ValueError('left/right Cyclo commands out of sync by >.10s')
                    target = {k: v for values, _ in self.commands.values() for k, v in values.items()}
                    check_step(target, self.measured, self.previous, max(.001, now-self.last_output))
                    self.emit(target)
                    self.previous, self.last_output = target, now
                    self.output_active = True
                    self.reason = 'forwarding validated arms-only targets'
        except (ValueError, KeyError) as error:
            self.latch(str(error))
        if now-self.last_status >= .1:
            self.last_status = now
            self.status_pub.publish(String(data=json.dumps({
                'model': self.model, 'fault': self.fault, 'reason': self.reason,
                'description_ok': self.description_ok, 'enabled': self.intent_enabled,
                'forwarding': self.output_active, 'measured_fresh': now-self.measured_rx <= .5,
                'motor_healthy': self.motor_ok and now-self.motor_rx <= .5,
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
