#!/usr/bin/env python3
"""[CHANGED] Single minimal, read-only DXL readiness check for SG2 ready pose.

Do not duplicate measured joint/action/competitor checks: the existing
prepare_sg2_ready_pose.py already owns them and verifies actual convergence.
The official driver's torque_state can incorrectly report False. In that case
require read-only positive Torque Enable register evidence for ALL 14 arm IDs.
No torque writes and no arm commands occur here.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from dynamixel_interfaces.msg import DynamixelState

from hardware.torque_evidence import TorqueEvidence
from hardware.xr_contract import ARM_DXL_IDS, motor_health, motor_read_health


class ReadyPosePrecheck(Node):
    def __init__(self):
        super().__init__('xr_sg2_ready_pose_precheck')
        self.proof = TorqueEvidence(self)
        self.expected = set(ARM_DXL_IDS['left'] + ARM_DXL_IDS['right'])
        self.bus_ok = False
        self.reported_torque_good = False
        self.motor_rx = self.motor_stamp = -math.inf
        self.good_since = None
        self.reason = 'waiting for fresh follower motor status'
        self.large_skew_count = 0
        self.large_skew_last_stamp = -math.inf
        self.large_skew_age = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            DynamixelState, '/ffw_follower/dxl_state', self.motor, qos)

    def motor(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if not math.isfinite(age) or not (-0.1 <= age <= 0.5) or stamp <= self.motor_stamp:
            self.bus_ok = False
            self.good_since = None
            self.reason = (
                f'DXL feedback time invalid: PC minus robot header={age:.3f}s; '
                'synchronize host clocks before VR')
            if math.isfinite(age) and abs(age) > 2.0 and stamp > self.large_skew_last_stamp:
                self.large_skew_count += 1
                self.large_skew_last_stamp = stamp
                self.large_skew_age = age
            return
        self.large_skew_count = 0
        self.motor_stamp = stamp
        self.motor_rx = time.monotonic()
        shape_ok = (len(msg.id) == len(set(msg.id))
                    and len(msg.id) == len(msg.torque_state)
                    and len(msg.id) == len(msg.dxl_hw_state)
                    and self.expected.issubset(msg.id))
        self.bus_ok = bool(shape_ok and motor_read_health(
            msg.id, msg.dxl_hw_state, msg.comm_state))
        self.reported_torque_good = bool(self.bus_ok and motor_health(
            msg.id, msg.torque_state, msg.dxl_hw_state, msg.comm_state))
        if not self.bus_ok:
            self.good_since = None
            self.reason = 'DXL transport/error/arm motor ID invalid'

    def ready(self):
        now = time.monotonic()
        fresh = self.bus_ok and now - self.motor_rx <= 0.5
        torque_ok = self.reported_torque_good or self.proof.verified()
        source_ok = self.count_publishers('/ffw_follower/dxl_state') == 1
        if fresh and torque_ok and source_ok:
            if self.good_since is None:
                self.good_since = now
            return now - self.good_since >= 0.3
        self.good_since = None
        if fresh and not torque_ok:
            self.reason = 'Torque Enable not verified: ' + self.proof.reason
        elif fresh and not source_ok:
            self.reason = 'need exactly one real follower motor-state publisher'
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args()
    if not 5.0 <= args.timeout <= 90.0:
        parser.error('--timeout must be 5..90 seconds')
    rclpy.init(args=[])
    node = ReadyPosePrecheck()
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            node.proof.tick()
            if node.large_skew_count >= 5:
                raise SystemExit(
                    '[CLOCK MISMATCH] PC minus robot is '
                    f'{node.large_skew_age:.3f}s. '
                    'Synchronize clocks before moving; no commands sent.')
            if node.ready():
                if not node.reported_torque_good:
                    print('[WARN] Driver torque_state=False; all 14 real arm Torque Enable registers returned 1')
                print('[PASS] SG2 motor feedback and live torque; existing ready-pose mover will check joint/action states')
                return 0
        raise SystemExit('[NOT READY] ' + node.reason + '; no ready-pose command sent')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
