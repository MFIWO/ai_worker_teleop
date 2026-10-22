"""Simulation-only command relay and reliable measured-state adapter.

No motor-health simulation and no hardware gate/profile impersonation. A live
optical-input heartbeat is required for forwarding; stale input holds measured
arm/finger positions and deactivates Cyclo. It does not certify hand collisions.
"""
import json
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Bool

from sim.prepare_sh5_sim import PROFILE, require_sim, measured_names


class SimulationBridge(Node):
    FEEDBACK_MAX_AGE = 5.0
    INPUT_MAX_AGE = 4.0
    COMMAND_MAX_AGE = 4.0
    FINGER_SOFTWARE_SPEED = 30.0
    ARM_POSITION_TOLERANCE_RAD = 1e-4
    def __init__(self):
        require_sim()
        super().__init__('sh5_sim_bridge')
        self.profile = json.loads((PROFILE / 'profile.json').read_text())
        root = ET.parse(self.profile['urdf']).getroot()
        self.bounds = {j.get('name'): tuple(float(j.find('limit').get(k))
                       for k in ('lower', 'upper', 'velocity'))
                       for j in root.findall('joint') if j.get('type') == 'revolute'}
        self.names = {}
        for side, short in (('left', 'l'), ('right', 'r')):
            self.names[side + '_arm'] = [f'arm_{short}_joint{i}' for i in range(1, 8)]
            self.names[side + '_hand'] = [f'finger_{short}_joint{i}' for i in range(1, 21)]
        self.required = set().union(*map(set, self.names.values())) | {'lift_joint'}
        self.measured, self.raw, self.last_output = {}, {}, {}
        self.feedback_rx = self.input_rx = -1e9
        self.input = {}
        self.fault = ''
        self.was_enabled = False
        self.last_tick = time.monotonic()
        self.last_state_stamp = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.js = self.create_publisher(JointState, '/sh5_sim/controller_joint_states', 10)
        self.create_subscription(JointState, '/joint_states', self.joints, qos)
        self.create_subscription(String, '/sh5_sim/input_status', self.input_status, 10)
        self.stop = self.create_publisher(Bool, '/reactivate', 10)
        self.status = self.create_publisher(String, '/sh5_sim/bridge_status', 10)
        self.outputs = {}
        for key in self.names:
            side, part = key.split('_')
            suffix = side + ('_hand' if part == 'hand' else '')
            topic = f'/leader/joint_trajectory_command_broadcaster_{suffix}/joint_trajectory'
            self.outputs[key] = self.create_publisher(JointTrajectory, topic, 10)
            self.create_subscription(JointTrajectory, f'/sh5_sim/raw/{key}',
                lambda msg, k=key: self.command(k, msg), qos)
        self.create_timer(.01, self.tick)
        self.create_timer(.1, self.report)

    def joints(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self.last_state_stamp is not None and stamp <= self.last_state_stamp:
            return
        if (len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name)
                or not self.required.issubset(msg.name) or not all(map(math.isfinite, msg.position))):
            return
        self.last_state_stamp = stamp
        self.measured = measured_names(msg.name, msg.position)
        self.feedback_rx = time.monotonic()
        msg.name = list(self.measured)
        msg.position = list(self.measured.values())
        msg.velocity = []
        msg.effort = []
        self.js.publish(msg)

    def input_status(self, msg):
        try:
            value = json.loads(msg.data)
            if value.get('simulation_only') is True and value.get('input') == 'optical_wrists_and_fingers':
                self.input, self.input_rx = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    def command(self, key, msg):
        names = self.names[key]
        if (not msg.points or len(msg.joint_names) != len(names) or set(msg.joint_names) != set(names)
                or len(msg.points[-1].positions) != len(names)
                or not all(map(math.isfinite, msg.points[-1].positions))):
            self.fault = f'Invalid {key} trajectory'
            return
        target = dict(zip(msg.joint_names, msg.points[-1].positions))
        # Original hand optimizer occasionally overshoots a bound by <0.001 rad.
        # Accept only numerical tolerance, then clamp to the exact URDF bounds.
        tolerance = .002 if key.endswith('_hand') else self.ARM_POSITION_TOLERANCE_RAD
        outside = {n: target[n] for n in names if not
                   self.bounds[n][0] - tolerance <= target[n] <= self.bounds[n][1] + tolerance}
        if outside:
            detail = {n: {'target': value, 'lower': self.bounds[n][0],
                          'upper': self.bounds[n][1],
                          'excess_rad': max(self.bounds[n][0] - value, value - self.bounds[n][1])}
                      for n, value in outside.items()}
            self.fault = f'{key} outside SH5 joint limits: {detail}; tolerance={tolerance:g}rad'
            return
        # Accept tiny solver overshoot, but store only in-range commands.
        target = {n: max(self.bounds[n][0], min(self.bounds[n][1], v))
                  for n, v in target.items()}
        self.raw[key] = (target, time.monotonic())

    def tick(self):
        now = time.monotonic()
        dt = min(.05, max(0., now - self.last_tick))
        self.last_tick = now
        feedback = now - self.feedback_rx <= self.FEEDBACK_MAX_AGE
        enabled = (feedback and now - self.input_rx <= self.INPUT_MAX_AGE and self.input.get('enabled') is True
                   and self.input.get('input_fresh') is True and not self.fault)
        if self.was_enabled and not enabled:
            self.stop.publish(Bool(data=False))
            self.raw.clear()
            if feedback:
                self.last_output = {n: self.measured[n] for n in self.required}
        self.was_enabled = enabled
        if not feedback:
            return
        for name, value in self.profile['frozen_joints'].items():
            if name not in self.measured or abs(self.measured[name] - value) > .02:
                self.fault = f'{name}: non-arm/non-finger joint moved'
                enabled = False
        for key, names in self.names.items():
            target, receipt = self.raw.get(key, ({}, -1e9))
            active = enabled and now - receipt <= self.COMMAND_MAX_AGE
            if key.endswith('_hand'):
                active = active and self.input.get('phase') == 'ACTIVE'
            values = []
            for name in names:
                actual = self.measured[name]
                if active:
                    lower, upper, velocity = self.bounds[name]
                    previous = self.last_output.get(name, actual)
                    goal = max(lower, min(upper, target[name]))
                    # Match the updated hardware arm policy: physical URDF
                    # velocity, without the old 0.35 rad/s commissioning cap.
                    # Finger software cap is 10x; physical URDF speed still wins.
                    speed = velocity if key.endswith('_arm') else min(velocity, self.FINGER_SOFTWARE_SPEED)
                    value = previous + max(-speed * dt, min(speed * dt, goal - previous))
                else:
                    # Hold one captured target; do not chase gravity sag.
                    value = self.last_output.get(name, actual)
                self.last_output[name] = value
                values.append(float(value))
            msg = JointTrajectory()
            msg.joint_names = names
            msg.points = [JointTrajectoryPoint(positions=values)]
            self.outputs[key].publish(msg)

    def report(self):
        now = time.monotonic()
        # Check ownership continuously; never silently reuse old hand-test writers.
        for key, pub in self.outputs.items():
            writers = self.get_publishers_info_by_topic(pub.topic_name)
            if len(writers) != 1 or writers[0].node_name != self.get_name():
                self.fault = f'Competing trajectory publisher: {pub.topic_name}'
        self.status.publish(String(data=json.dumps({
            'simulation_only': True, 'feedback_fresh': now - self.feedback_rx <= self.FEEDBACK_MAX_AGE,
            'fault': self.fault, 'input_fresh': now - self.input_rx <= self.INPUT_MAX_AGE,
            'raw_ages': {k: round(now - t, 3) for k, (_, t) in self.raw.items()},
            'note': 'Current measured finger geometry, not hardware collision certification'})))


def main():
    require_sim()
    rclpy.init()
    node = SimulationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop.publish(Bool(data=False))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
