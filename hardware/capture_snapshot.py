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
from hardware.xr_contract import configured_arm_ids, motor_health


class Snapshot(Node):
    def __init__(self, model):
        super().__init__('xr_hardware_passive_preflight', enable_rosout=False, start_parameter_services=False)
        self.model, self.xml, self.state = model, None, None
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
        age = self.get_clock().now().nanoseconds*1e-9-stamp
        if not -.1 <= age <= .5 or stamp <= self.last_source:
            self.error = 'stale or non-increasing measured joints; synchronize PC/robot clocks'
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
        age = self.get_clock().now().nanoseconds*1e-9-stamp
        if stamp <= self.motor_stamp or not -.1 <= age <= .5:
            return
        ids = tuple(sorted(msg.id))
        self.motor_ok = (motor_health(msg.id, msg.torque_state, msg.dxl_hw_state, msg.comm_state)
                         and (self.motor_ids is None or ids == self.motor_ids))
        if self.motor_ok:
            self.motor_ids = ids
        self.motor_stamp, self.motor_rx = stamp, time.monotonic()

    def ready(self):
        now = time.monotonic()
        return (self.xml is not None and self.state is not None and self.motor_ok
                and now-self.js_rx <= .5 and now-self.motor_rx <= .5
                and self.still_since is not None and now-self.still_since >= .5
                and all(self.count_publishers(t) == 1 for t in
                        ('/robot_description', '/joint_states', '/ffw_follower/dxl_state')))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', choices=('sg2', 'sh5'), required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--timeout', type=float, default=12.)
    a = p.parse_args()
    rclpy.init(args=[])
    node = Snapshot(a.model)
    try:
        start = time.monotonic()
        while time.monotonic()-start < a.timeout:
            rclpy.spin_once(node, timeout_sec=.05)
            if node.ready():
                a.output.mkdir(parents=True, exist_ok=False)
                (a.output/'robot_description.urdf').write_text(node.xml)
                (a.output/'joints.json').write_text(json.dumps(node.state, indent=2))
                print('[PASS] passive hardware preflight; stable joints, healthy DXL read, matching model')
                print('[SAVED]', a.output)
                return
        raise SystemExit('[NOT READY] ' + node.error + '; no commands sent')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
