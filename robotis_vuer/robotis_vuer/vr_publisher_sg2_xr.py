"""[ADDED] Opt-in, arms-only SG2 XR delta mode. Original publishers are intact.

Run Cyclo's existing launch with controller_type:=vr hand:=false arm:=false.
This node supplies robot-frame /[lr]_goal_pose and /[lr]_subgoal_pose directly;
the original arm retargeter MUST be disabled to avoid a second target writer.
No joint trajectory, trigger, finger, base, lift or head commands are emitted.
"""

import json
import os
import signal
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from robotis_vuer.vr_publisher_sg2 import BODY_HEAD_INDEX, VRTrajectoryPublisher
from robotis_vuer.xr_arm_targets import (
    CalibratedWristMapper, ContinuousElbowHint, limited_pose, rigid_pose,
)


SIDES = {'left': 'l', 'right': 'r'}


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(quat).all() or abs(np.linalg.norm(quat) - 1.0) > 0.02:
        raise ValueError('invalid feedback quaternion')
    result = np.eye(4)
    result[:3, 3] = [p.x, p.y, p.z]
    result[:3, :3] = Rotation.from_quat(quat).as_matrix()
    return rigid_pose(result)


class XRDeltaPublisher(VRTrajectoryPublisher):
    # [ADDED 2026-10-14] Explicit opt-in subclass hooks. Legacy defaults unchanged.
    DEFAULT_GOAL_SPEED = 0.20
    DEFAULT_ANGULAR_SPEED = 1.0

    def _validate_runtime_domain(self):
        if os.environ.get('ROS_DOMAIN_ID') not in ('231', '232'):
            raise RuntimeError('XR experimental mode requires simulation domain 231 (232 for tests), not robot domain 30')

    def __init__(self, start_server=True):
        self._validate_runtime_domain()
        self._xr_ready = False
        self._xr_lock = threading.RLock()
        self._mapper = None
        self._robot_poses = {}
        self._joint_rx = None
        self._last_pub = None
        self._last_input = {}
        self._goals = {}
        self._elbows = {}
        self._arming_until = None
        self._stopped_at = time.monotonic()
        self._stopped_ros = None
        self._last_status = 0.0
        self._last_reason = None
        self.published_pairs = 0
        super().__init__()  # Real rclpy/Vuer; server startup is deferred below.

        # [CHANGED] Arms-only mode must not accidentally emit legacy raw poses,
        # trigger or joystick trajectories even if a controller button is used.
        unused = ('head_joint_pub', 'lift_joint_pub', 'left_squeeze_pub',
                  'right_squeeze_pub', 'left_gripper_pub', 'right_gripper_pub',
                  'cmd_vel_pub', 'left_wrist_rviz_pub', 'right_wrist_rviz_pub',
                  'left_elbow_rviz_pub', 'right_elbow_rviz_pub',
                  'left_shoulder_rviz_pub', 'right_shoulder_rviz_pub')
        for name in unused:
            self.destroy_publisher(getattr(self, name))
        self._goal_pub = {side: self.create_publisher(PoseStamped, f'/{short}_goal_pose', self.vr_stream_qos)
                          for side, short in SIDES.items()}
        self._elbow_pub = {side: self.create_publisher(PoseStamped, f'/{short}_subgoal_pose', self.vr_stream_qos)
                           for side, short in SIDES.items()}
        self._wrist_subs = [self.create_subscription(
            PoseStamped, f'/{short}_gripper_pose',
            lambda msg, s=side: self._robot_pose(msg, s), self.vr_stream_qos)
            for side, short in SIDES.items()]
        self._reference_sub = self.create_subscription(Bool, '/reference_diverged', self._reference_fault, 10)
        self._status_pub = self.create_publisher(String, '/sg2_xr/status', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.translation_scale = float(self.declare_parameter('xr_translation_scale', 1.0).value)
        self.goal_speed = float(self.declare_parameter('xr_goal_speed', self.DEFAULT_GOAL_SPEED).value)
        self.angular_speed = float(self.declare_parameter('xr_angular_speed', self.DEFAULT_ANGULAR_SPEED).value)
        if not 0.1 <= self.translation_scale <= 1.5:
            raise ValueError('xr_translation_scale must be in [0.1,1.5]')
        if not 0.02 <= self.goal_speed <= 0.30 or not 0.1 <= self.angular_speed <= 1.5:
            raise ValueError('xr_goal_speed in [0.02,0.30], xr_angular_speed in [0.1,1.5] required')
        self._xr_ready = True
        self._set_arm_control_enabled(False, 'XR mode startup; manual calibration required')
        self.get_logger().info(
            '[XR MODE] Original offsets/scales are NOT used. X+A captures measured wrists; '
            'Y+B stops. Fixed horizontal heading; independent wrist deltas; no fingers/base/lift/head. '
            'Cyclo launch must include arm:=false. Keep the controller startup checks enabled.')
        if start_server:
            super().start_vuer_server()

    def start_vuer_server(self):
        # [FIX] Parent constructor calls this before subclass ROS endpoints exist.
        if self._xr_ready:
            super().start_vuer_server()

    def __del__(self):
        # [FIX] Installed Vuer has no .stop(); do not invoke the legacy
        # destructor's unsupported call or log through an already destroyed
        # ROS node. main() sends False and owns node shutdown; the standalone
        # process owns the daemon web server lifetime.
        pass

    def joint_states_callback(self, msg):
        with self._xr_lock:
            required = {f'arm_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 8)}
            valid = (len(msg.name) == len(msg.position) and len(set(msg.name)) == len(msg.name)
                     and required.issubset(msg.name) and np.isfinite(msg.position).all())
            if not valid:
                self._joint_rx = None
                return
            self._joint_rx = time.monotonic()
            self.current_joint_states = msg

    def _robot_pose(self, msg, side):
        with self._xr_lock:
            try:
                if msg.header.frame_id != 'base_link':
                    raise ValueError('controller EE pose must use base_link')
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                age = self.get_clock().now().nanoseconds * 1e-9 - stamp
                if not -0.1 <= age <= 0.5:
                    raise ValueError('stale controller EE feedback timestamp')
                matrix = pose_matrix(msg.pose)
                self._robot_poses[side] = (matrix, time.monotonic(), stamp)
            except ValueError:
                self._robot_poses.pop(side, None)

    def _reference_fault(self, msg):
        if msg.data and self._xr_ready:
            with self._xr_lock:
                if self.arm_control_enabled:
                    self._set_arm_control_enabled(False, 'Cyclo reference checker fault; no automatic reset')

    def _arm_tracking_is_fresh(self, now=None):
        now = time.monotonic() if now is None else now
        return (super()._arm_tracking_is_fresh(now) and self._joint_rx is not None
                and 0 <= now - self._joint_rx <= 0.5)

    def _tf_pose(self, link):
        tf = self.tf_buffer.lookup_transform('base_link', link, Time())
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if not -0.1 <= age <= 0.5:
            raise ValueError(f'stale TF for {link}: {age:.3f}s')
        p, q = tf.transform.translation, tf.transform.rotation
        msg = PoseStamped()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p.x, p.y, p.z
        msg.pose.orientation = q
        return pose_matrix(msg.pose)

    def _snapshot(self):
        result = {}
        for side, short in SIDES.items():
            result[side] = {role: self._tf_pose(f'arm_{short}_link{number}')
                            for role, number in (('shoulder', 2), ('elbow', 4), ('wrist', 7))}
        return result

    def _graph_check(self):
        for short in SIDES.values():
            for topic in (f'/{short}_goal_pose', f'/{short}_subgoal_pose'):
                if self.count_publishers(topic) != 1:
                    raise ValueError(f'{topic} has another writer. Stop old Vuer and use Cyclo arm:=false')
        if any(node == 'arm_retargeting_teleop' for node, _ in self.get_node_names_and_namespaces()):
            raise ValueError('legacy arm_retargeting_teleop still running; restart Cyclo with arm:=false')

    def _capture(self):
        self._graph_check()
        now = time.monotonic()
        if not self._arm_tracking_is_fresh(now):
            raise ValueError('fresh head, both controllers and SG2 joint_states are required')
        if now - self._stopped_at < 0.25:
            raise ValueError('wait briefly after stop so Cyclo publishes measured startup poses')
        links = self._snapshot()
        robots = {}
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        for side in SIDES:
            controller_pose, receipt, source_stamp = self._robot_poses.get(side, (None, -1e9, -1e9))
            if (controller_pose is None or now - receipt > 0.5
                    or receipt < self._stopped_at + 0.15
                    or self._stopped_ros is None
                    or source_stamp < self._stopped_ros + 0.10
                    or not -0.1 <= now_ros - source_stamp <= 0.5):
                raise ValueError(f'missing inactive Cyclo {side} gripper pose; is the VR controller running?')
            actual = links[side]['wrist']
            dp = np.linalg.norm(controller_pose[:3, 3] - actual[:3, 3])
            dr = Rotation.from_matrix(controller_pose[:3, :3] @ actual[:3, :3].T).magnitude()
            if dp > 0.04 or dr > np.deg2rad(20):
                raise ValueError(f'{side}: Cyclo model vs PhysX TF mismatch {dp:.3f}m/{np.rad2deg(dr):.1f}deg')
            # [IMPORTANT] This topic is measured only while Cyclo is INACTIVE.
            # During motion the controller publishes commanded-FK poses there.
            robots[side] = controller_pose.copy()
        inputs = {s: getattr(self, f'{s}_controller_matrix').copy() for s in SIDES}
        self._mapper = CalibratedWristMapper(self.head_transform_matrix, inputs, robots,
                                           self.translation_scale)
        self._hints = {s: ContinuousElbowHint(links[s]['shoulder'][:3, 3],
                                             links[s]['elbow'][:3, 3], robots[s][:3, 3])
                       for s in SIDES}
        self._goals = {s: robots[s].copy() for s in SIDES}
        self._elbows = {s: links[s]['elbow'].copy() for s in SIDES}
        self._last_input = inputs
        self._last_pub = None

    def _set_arm_control_enabled(self, enabled, reason):
        with self._xr_lock:
            if not self._xr_ready:
                return
            if enabled and self.arm_control_enabled:
                # [FIX] Repeated X+A during ACTIVE must not recalibrate from
                # Cyclo's commanded-FK topic. Y+B then a new X+A is required.
                self.get_logger().info('[XR] Already active. Use Y+B before recapturing the startup pose.')
                return
            if not enabled:
                self._mapper = None
                self._arming_until = None
                self._stopped_at = time.monotonic()
                self._stopped_ros = self.get_clock().now().nanoseconds * 1e-9
                # [FIX] Never reuse a cached ACTIVE commanded-FK pose as the
                # next inactive measured anchor. Wait for a new stamped sample.
                self._robot_poses.clear()
                super()._set_arm_control_enabled(False, reason)
                return
            try:
                self._capture()
            except Exception as error:
                self.get_logger().warning(f'[XR START REFUSED] {error}')
                super()._set_arm_control_enabled(False, str(error))
                return
            # [FIX] Send stationary anchor goals BEFORE enabling the controller.
            # Allow several DDS frames to supersede old goals; manual start only.
            self.arm_control_enabled = True
            self._arming_until = time.monotonic() + 0.20
            self._publish_reactivate(False, 'XR anchor goals settling')
            self.get_logger().info('[XR CALIBRATED] Captured robot startup pose. Hold still until Controller activated.')

    async def on_body_tracking_move(self, event, session):
        with self._xr_lock:
            if not self._xr_ready:
                return
            try:
                body = np.asarray(event.value['body'], dtype=float).reshape(-1)
                start = BODY_HEAD_INDEX * 16
                head = rigid_pose(body[start:start + 16].reshape(4, 4, order='F'))
                self.head_transform_matrix = head
                # [CHANGED] Human shoulders/elbows are NOT used or required.
                self.last_body_pose_sec = time.monotonic()
            except (KeyError, TypeError, ValueError):
                self.last_body_pose_sec = None
                if self.arm_control_enabled:
                    self._set_arm_control_enabled(False, 'invalid head tracking')

    async def on_controller_move(self, event, session):
        with self._xr_lock:
            if not self._xr_ready:
                return
            data = event.value
            if not isinstance(data, dict):
                return
            now = time.monotonic()
            for side in SIDES:
                state = data.get(f'{side}State', {})
                setattr(self, f'{side}_controller_state', state if isinstance(state, dict) else {})
                try:
                    if not state:
                        raise ValueError('missing controller state')
                    raw = np.asarray(data[side], dtype=float).reshape(4, 4, order='F')
                    matrix = rigid_pose(raw)
                    if self.arm_control_enabled and side in self._last_input:
                        old = self._last_input[side]
                        if (np.linalg.norm(matrix[:3, 3] - old[:3, 3]) > 0.15
                                or Rotation.from_matrix(matrix[:3, :3] @ old[:3, :3].T).magnitude() > 1.2):
                            raise ValueError('tracking jump; release X/A and recalibrate')
                    setattr(self, f'{side}_controller_matrix', matrix)
                    self.last_controller_pose_sec[side] = now
                except (KeyError, TypeError, ValueError) as error:
                    self.last_controller_pose_sec[side] = None
                    if self.arm_control_enabled:
                        self._set_arm_control_enabled(False, str(error))
            # Reuse the original X+A / Y+B edge and release-to-rearm semantics.
            self._update_arm_activation()
            self._publish_synced_pose_frame_if_ready()

    def _publish_pair(self):
        stamp = self.get_clock().now().to_msg()
        for side in SIDES:
            for matrix, publisher in ((self._goals[side], self._goal_pub[side]),
                                      (self._elbows[side], self._elbow_pub[side])):
                msg = PoseStamped()
                msg.header.frame_id, msg.header.stamp = 'base_link', stamp
                p, q = matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_quat()
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
                (msg.pose.orientation.x, msg.pose.orientation.y,
                 msg.pose.orientation.z, msg.pose.orientation.w) = map(float, q)
                publisher.publish(msg)
        self.published_pairs += 1

    def _publish_synced_pose_frame_if_ready(self):
        if not self._xr_ready or not self.arm_control_enabled or self._mapper is None:
            return
        now = time.monotonic()
        if self._last_pub is not None and now - self._last_pub < 1 / 30:
            return
        try:
            if not self._arm_tracking_is_fresh(now):
                raise ValueError('tracking or joint feedback stale; manual restart required')
            self._graph_check()
            links = self._snapshot()
            if self._arming_until is not None:
                self._publish_pair()
                if now >= self._arming_until:
                    # Start deltas from the latest held controller pose, not its
                    # location while the DDS settling frames were in transit.
                    self._mapper = CalibratedWristMapper(
                        self.head_transform_matrix,
                        {s: getattr(self, f'{s}_controller_matrix') for s in SIDES},
                        self._goals, self.translation_scale)
                    self._arming_until = None
                    self._publish_reactivate(True, 'manual X+A calibration ready')
                self._last_pub = now
                return
            dt = 1 / 30 if self._last_pub is None else now - self._last_pub
            for side in SIDES:
                actual = links[side]['wrist']
                if np.linalg.norm(self._goals[side][:3, 3] - actual[:3, 3]) > 0.25:
                    raise ValueError(f'{side} wrist is not following goal (>0.25m); inspect Cyclo/IK')
                if Rotation.from_matrix(self._goals[side][:3, :3] @ actual[:3, :3].T).magnitude() > np.pi / 2:
                    raise ValueError(f'{side} wrist orientation is not following goal (>90deg); inspect Cyclo/IK')
                current_input = getattr(self, f'{side}_controller_matrix')
                requested = self._mapper.target(side, current_input)
                if np.linalg.norm(requested[:3, 3] - self._mapper.anchors[side].robot_pose[:3, 3]) > 0.5:
                    raise ValueError('operator exceeded 0.5m clutch range; stop and recapture')
                requested[:3, 3], _ = self._hints[side].project_wrist(requested[:3, 3])
                previous_position = self._goals[side][:3, 3].copy()
                self._goals[side] = limited_pose(self._goals[side], requested, dt,
                                                self.goal_speed, self.angular_speed)
                # Segment hints stabilize redundancy without noisy Quest elbows.
                wrist, _ = self._hints[side].project_wrist(self._goals[side][:3, 3])
                # [FIX] Inner-shell projection is non-convex; it must not
                # silently undo the Cartesian speed bound after filtering.
                if np.linalg.norm(wrist - previous_position) > self.goal_speed * min(dt, 0.05) + 1e-8:
                    raise ValueError('workspace boundary would exceed the target speed; stop and recapture')
                self._goals[side][:3, 3] = wrist
                elbow = self._elbows[side].copy()
                elbow[:3, 3] = self._hints[side].elbow(wrist)
                self._elbows[side] = limited_pose(self._elbows[side], elbow, dt,
                                                 self.goal_speed, self.angular_speed)
                self._last_input[side] = current_input.copy()
            self._publish_pair()
            self._last_pub = now
        except Exception as error:
            self._set_arm_control_enabled(False, f'XR guard: {error}')

    def _check_arm_tracking_timeout(self):
        if not self._xr_ready:
            return
        with self._xr_lock:
            if not self._arm_tracking_is_fresh():
                self.arm_start_armed = False
                if self.arm_control_enabled:
                    self._set_arm_control_enabled(False, 'tracking/feedback timeout')
            self._publish_synced_pose_frame_if_ready()
            now = time.monotonic()
            if now - self._last_status >= 1.0:
                self._last_status = now
                self._status_pub.publish(String(data=json.dumps({
                    'mode': 'xr_delta', 'active': self.arm_control_enabled,
                    'settling': self._arming_until is not None,
                    'fresh': self._arm_tracking_is_fresh(now),
                    'published_pairs': self.published_pairs,
                    'scale': self.translation_scale, 'goal_speed_m_s': self.goal_speed,
                })))


def main(args=None):
    # [FIX] Default rclpy SIGINT shutdown invalidates the context before the
    # final /reactivate=False can be sent. Own signal handling in this process.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def stop_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_signal)
    node = None
    try:
        node = XRDeltaPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node._set_arm_control_enabled(False, 'XR publisher shutdown')
                rclpy.spin_once(node, timeout_sec=0.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
