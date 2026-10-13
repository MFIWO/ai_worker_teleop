#!/usr/bin/env python3
"""[ADDED] SG2 pre-move evidence check for the existing 5s follower ready-pose mover.

No writes, torque enabling, joint goal publication or changes to ROBOTIS code.
The previous topic torque_state array may incorrectly say False on this driver;
accept it only after ALL fourteen motor Torque Enable registers are read as 1.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from dynamixel_interfaces.msg import DynamixelState
from sensor_msgs.msg import JointState

from hardware.torque_evidence import TorqueEvidence
from hardware.xr_contract import ARM_DXL_IDS, motor_health, motor_read_health


class ReadyPosePrecheck(Node):
    def __init__(self):
        super().__init__('xr_sg2_ready_pose_precheck')
        self.proof = TorqueEvidence(self)
        self.motor_good = False
        self.reported_torque_good = False
        self.joint_good = False
        self.motor_rx = self.joint_rx = -math.inf
        self.motor_stamp = self.joint_stamp = -math.inf
        self.ready_since = None
        self.reason = 'waiting for SG2 follower feedback'
        self.large_skew_count = 0
        self.large_skew_age = 0.0
        self.large_skew_last_stamp = -math.inf
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subs = [
            self.create_subscription(JointState, '/joint_states', self.joints, qos),
            self.create_subscription(DynamixelState, '/ffw_follower/dxl_state', self.motor, qos),
        ]

    def _source_age(self, header):
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        return self.get_clock().now().nanoseconds * 1e-9 - stamp, stamp

    def joints(self, msg):
        age, stamp = self._source_age(msg.header)
        required = {f'arm_{side}_joint{i}' for side in ('l', 'r') for i in range(1, 8)}
        ok = (len(msg.name) == len(msg.position)
              and len(set(msg.name)) == len(msg.name)
              and required.issubset(msg.name)
              and all(math.isfinite(float(p)) for p in msg.position)
              and -.1 <= age <= .5 and stamp > self.joint_stamp)
        if ok:
            self.joint_stamp, self.joint_rx = stamp, time.monotonic()
            self.joint_good = True
        else:
            self.joint_good = False
            self.reason = f'joint feedback stale/invalid; PC minus robot header age={age:.3f}s'

    def motor(self, msg):
        age, stamp = self._source_age(msg.header)
        if not (-.1 <= age <= .5 and stamp > self.motor_stamp):
            self.motor_good = False
            self.reason = f'DXL message clock mismatch/replay; PC minus robot age={age:.3f}s; sync both host clocks'
            # [ADDED] Fail promptly on repeated live messages with >2s clock
            # difference instead of asking operators to wait a full timeout.
            if math.isfinite(age) and abs(age) > 2.0 and stamp > self.large_skew_last_stamp:
                self.large_skew_count += 1
                self.large_skew_age = age
                self.large_skew_last_stamp = stamp
            return
        self.large_skew_count = 0
        self.motor_stamp, self.motor_rx = stamp, time.monotonic()
        expected = set(ARM_DXL_IDS['left'] + ARM_DXL_IDS['right'])
        id_ok = (len(msg.id) == len(set(msg.id))
                 and len(msg.id) == len(msg.torque_state)
                 and len(msg.id) == len(msg.dxl_hw_state)
                 and expected.issubset(msg.id))
        self.motor_good = bool(id_ok and motor_read_health(
            msg.id, msg.dxl_hw_state, msg.comm_state))
        self.reported_torque_good = bool(self.motor_good and motor_health(
            msg.id, msg.torque_state, msg.dxl_hw_state, msg.comm_state))
        if not self.motor_good:
            self.reason = 'DXL bus fault, arm ID missing/duplicate or hardware error'

    def ready(self):
        now = time.monotonic()
        fresh = (self.joint_good and self.motor_good
                 and now - self.joint_rx <= .5
                 and now - self.motor_rx <= .5)
        torque_ok = self.reported_torque_good or self.proof.verified()
        publishers_ok = (self.count_publishers('/joint_states') == 1
                         and self.count_publishers('/ffw_follower/dxl_state') == 1)
        if fresh and torque_ok and publishers_ok:
            if self.ready_since is None:
                self.ready_since = now
            return now - self.ready_since >= .3
        self.ready_since = None
        if fresh and not torque_ok:
            self.reason = self.proof.reason
        elif fresh and not publishers_ok:
            self.reason = 'require exactly one publisher per measured robot topic'
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=30.)
    args = parser.parse_args()
    if not 5. <= args.timeout <= 90.:
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
                    f'[CLOCK MISMATCH] PC minus robot time {node.large_skew_age:.3f}s. '
                    'Synchronize host clocks before motion; no command sent.')
            if node.ready():
                if not node.reported_torque_good:
                    print('[WARN] Upstream torque_state reports False; all 14 Torque Enable registers READ as 1')
                print('[PASS] SG2 measured joints, healthy DXL bus, current arm torque evidence')
                print('[INFO] No commands sent by precheck. Starting existing ready-pose mover.')
                return 0
        raise SystemExit('[NOT READY] '+node.reason+'; no ready-pose command sent')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
