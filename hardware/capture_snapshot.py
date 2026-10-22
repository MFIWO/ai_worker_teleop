#!/usr/bin/env python3
"""[ADDED] Passive preflight. Never publishes arm, reactivate, or torque commands."""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from dynamixel_interfaces.msg import DynamixelState
from hardware.xr_contract import (
    configured_arm_ids, motor_health, motor_read_health, ARM_DXL_IDS,
    motor_readiness, validate_readiness_policy, READINESS_POLICIES,
)
from hardware.torque_evidence import TorqueEvidence


class Snapshot(Node):
    def __init__(self, model, allow_torque_off=False, readiness_policy='strict_torque'):
        super().__init__('xr_hardware_passive_preflight', enable_rosout=False, start_parameter_services=False)
        self.readiness_policy = validate_readiness_policy(readiness_policy)  # [CHANGED]
        self.model, self.xml, self.state = model, None, None
        # [ADDED] Only passive --check-only may accept all arms torque OFF.
        # Live command_gate and hardware activation keep strict motor_health.
        self.allow_torque_off = bool(allow_torque_off)
        self.motor_snapshot_ok = False
        self.motor_torque_off = False
        # [FIX] Buffered-service fallback is disabled; never mistake old ON for fresh evidence.
        self.torque_evidence = TorqueEvidence(self)
        self.js_rx = self.motor_rx = -math.inf
        self.last_source = -math.inf
        self.still_since = None
        self.previous = None
        self.motor_ok = False
        self.motor_stamp = -math.inf
        self.motor_ids = None
        self.error = 'waiting for /robot_description, /joint_states and /ffw_follower/dxl_state'
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs = [self.create_subscription(JointState, '/joint_states', self.joints, qos),
                     self.create_subscription(String, '/robot_description', self.description, latched),
                     self.create_subscription(DynamixelState, '/ffw_follower/dxl_state', self.motor, qos)]

    def description(self, msg):
        try:
            root = ET.fromstring(msg.data)
            if root.get('name') != f'ffw_{self.model}_follower':
                raise ValueError('robot_description model is not the explicitly selected model')
            configured_arm_ids(msg.data, self.model)
            if 'mock_components/GenericSystem' in msg.data or 'gazebo_ros2_control' in msg.data:
                raise ValueError('simulation/mock robot_description rejected for hardware snapshot')
            self.xml = msg.data
        except Exception as error:
            self.xml, self.error = None, str(error)

    def joints(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if stamp <= self.last_source:
            self.error = 'non-increasing measured joint sample order'
            return
        required = {f'arm_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 8)} | {'lift_joint'}
        if (len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name)
                or not required.issubset(msg.name) or not all(math.isfinite(p) for p in msg.position)):
            self.error = 'missing/invalid hardware joint states'
            return
        self.last_source, self.js_rx = stamp, time.monotonic()
        state = dict(zip(msg.name, msg.position))
        if self.previous is None or any(k not in self.previous or abs(v-self.previous[k]) > .003 for k, v in state.items()):
            self.still_since = time.monotonic()
            self.previous = state.copy()
        self.state = state

    def motor(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp <= self.motor_stamp:
            return
        ids = tuple(sorted(msg.id))
        # [ADDED] A valid motor READ in standby is different from a motor
        # that is torque-enabled and able to follow a physical trajectory.
        expected_ids = set().union(*(set(v) for v in ARM_DXL_IDS.values()))
        indexed = {identity: i for i, identity in enumerate(msg.id)}
        arrays_valid = (len(msg.id) == len(set(msg.id))
                        and len(msg.id) == len(msg.torque_state)
                        and len(msg.id) == len(msg.dxl_hw_state)
                        and expected_ids.issubset(indexed)
                        and (self.motor_ids is None or ids == self.motor_ids))
        physical_ok = (arrays_valid and motor_read_health(
            msg.id, msg.dxl_hw_state, msg.comm_state))
        reported_ok = bool(physical_ok and motor_health(
            msg.id, msg.torque_state, msg.dxl_hw_state, msg.comm_state))
        # [CHANGED] Do not force a driver repair through an unrelated snapshot.
        # Torque bits remain diagnostics in normal mode; real bus errors block.
        self.motor_reported_torque_on = reported_ok
        self.motor_ok = bool(physical_ok and motor_readiness(
            msg.id, msg.torque_state, msg.dxl_hw_state, msg.comm_state,
            self.readiness_policy))
        all_off = bool(physical_ok and
                       all(not bool(msg.torque_state[indexed[i]])
                           for i in expected_ids))
        self.motor_torque_off = all_off
        self.motor_snapshot_ok = self.motor_ok or (self.allow_torque_off and all_off)
        if self.motor_snapshot_ok:
            self.motor_ids = ids
        elif not physical_ok:
            self.error = 'Dynamixel ID/read/communication/hardware error; refuse snapshot'
        else:
            self.error = ('arm torque telemetry is OFF/mixed or unreliable; actual torque is unverified. '
                          'Buffered register polling is disabled; passive --allow-torque-off does not authorise motion')
        self.motor_stamp, self.motor_rx = stamp, time.monotonic()

    def ready(self):
        now = time.monotonic()
        return (self.xml is not None and self.state is not None and self.motor_snapshot_ok
                and now-self.js_rx <= .5 and now-self.motor_rx <= .5
                and self.still_since is not None and now-self.still_since >= .5
                and all(self.count_publishers(t) == 1 for t in
                        ('/robot_description', '/joint_states', '/ffw_follower/dxl_state')))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', choices=('sg2', 'sh5'), required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--timeout', type=float, default=12.)
    p.add_argument('--allow-torque-off', action='store_true',
                   help='Passive snapshot ONLY: allow all arm torques OFF with healthy DXL reads')
    p.add_argument('--readiness-policy', choices=READINESS_POLICIES,
                   default='strict_torque')  # [CHANGED] Wrapper explicitly selects normal policy.
    a = p.parse_args()
    rclpy.init(args=[])
    node = Snapshot(a.model, allow_torque_off=a.allow_torque_off,
                    readiness_policy=a.readiness_policy)
    try:
        start = time.monotonic()
        while time.monotonic()-start < a.timeout:
            rclpy.spin_once(node, timeout_sec=.05)
            if node.ready():
                a.output.mkdir(parents=True, exist_ok=False)
                (a.output/'robot_description.urdf').write_text(node.xml)
                (a.output/'joints.json').write_text(json.dumps(node.state, indent=2))
                if node.readiness_policy == 'driver_managed':
                    # [CHANGED] Persist the policy and uncertainty, not a false ON claim.
                    (a.output/'readiness.json').write_text(json.dumps({
                        'policy': node.readiness_policy,
                        'motor_read_healthy': True,
                        'reported_torque_all_on': node.motor_reported_torque_on,
                        'physical_torque_verified': False,
                    }, indent=2))
                    print('[PASS] Driver-managed snapshot: fresh motor reads, stable joints, matching model')
                    print('[TORQUE DIAGNOSTIC] reported_all_on=' + str(node.motor_reported_torque_on) +
                          '; physical torque was not independently verified or modified')
                elif node.motor_torque_off:
                    print('[PASS] READ-ONLY STANDBY SNAPSHOT: reported all arm torques OFF, healthy DXL reads')
                    print('[NOT MOTION READY] Controller launch still requires confirmed torque ON')
                else:
                    print('[PASS] passive hardware preflight; stable joints, healthy DXL read, matching model')
                print('[SAVED]', a.output)
                return
        raise SystemExit('[NOT READY] ' + node.error + '; no commands sent')
    except KeyboardInterrupt:
        print('[CANCELLED] Passive snapshot stopped; no commands sent')
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
