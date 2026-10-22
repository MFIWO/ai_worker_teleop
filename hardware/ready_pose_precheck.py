#!/usr/bin/env python3
"""[CHANGED] Single minimal, read-only DXL readiness check for SG2 ready pose.

Do not duplicate measured joint/action/competitor checks: the existing
prepare_sg2_ready_pose.py already owns them and verifies actual convergence.
The installed driver's torque_state can incorrectly report False. Buffered
register-service replies do not prove freshness, so that fallback is disabled.
No torque writes, service polling or arm commands occur here.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from dynamixel_interfaces.msg import DynamixelState

from hardware.torque_evidence import TorqueEvidence
from hardware.xr_contract import (
    ARM_DXL_IDS, motor_health, motor_read_health, validate_readiness_policy,
    READINESS_POLICIES,
)


class ReadyPosePrecheck(Node):
    def __init__(self, readiness_policy='driver_managed'):
        super().__init__('xr_sg2_ready_pose_precheck')
        # [CHANGED] Native-driver operation is the normal ready-pose path.
        self.readiness_policy = validate_readiness_policy(readiness_policy)
        self.proof = TorqueEvidence(self)
        self.expected = set(ARM_DXL_IDS['left'] + ARM_DXL_IDS['right'])
        self.bus_ok = False
        self.reported_torque_good = False
        self.motor_rx = self.motor_stamp = -math.inf
        self.good_since = None
        self.reason = 'waiting for fresh follower motor status'
        self.unverified_since = None  # [FIX] Diagnose unsupported telemetry without a 30s silent wait.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            DynamixelState, '/ffw_follower/dxl_state', self.motor, qos)

    def motor(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= self.motor_stamp:
            return
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
        if self.bus_ok and not self.reported_torque_good:
            if self.unverified_since is None:
                self.unverified_since = time.monotonic()
        else:
            self.unverified_since = None
        if not self.bus_ok:
            self.good_since = None
            self.reason = 'DXL transport/error/arm motor ID invalid'

    def ready(self):
        now = time.monotonic()
        fresh = self.bus_ok and now - self.motor_rx <= 0.5
        # [CHANGED] Unknown torque telemetry is not an error in the driver's
        # normal path. Do not replace False by a fabricated positive sample.
        torque_ok = (self.readiness_policy == 'driver_managed'
                     or self.reported_torque_good)
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
    parser.add_argument('--readiness-policy', choices=READINESS_POLICIES,
                        default='driver_managed')  # [CHANGED]
    args = parser.parse_args()
    if not 5.0 <= args.timeout <= 90.0:
        parser.error('--timeout must be 5..90 seconds')
    rclpy.init(args=[])
    node = ReadyPosePrecheck(args.readiness_policy)
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if (node.readiness_policy == 'strict_torque' and node.unverified_since is not None and
                    time.monotonic() - node.unverified_since > 3.0):
                raise SystemExit('[TORQUE TELEMETRY UNVERIFIED] ' + node.proof.reason)
            if node.ready():
                if not node.reported_torque_good:
                    print('[TORQUE DIAGNOSTIC] Topic does not report all ON; physical torque is not verified. '
                          'Existing follower driver remains responsible for torque management.')
                print('[PASS] Fresh SG2 motor read feedback; policy=' + node.readiness_policy +
                      '; ready-pose mover still checks action acceptance and measured convergence')
                return 0
        raise SystemExit('[NOT READY] ' + node.reason + '; no ready-pose command sent')
    except KeyboardInterrupt:
        print('[CANCELLED] Read-only precheck stopped; no ready-pose command sent')
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
