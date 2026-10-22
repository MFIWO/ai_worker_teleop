"""[ADDED] SG2 XR fixed-reference resume mode; legacy XR publisher is untouched.

First X+A calibrates. Subsequent X+A resumes the SAME mapping only after
the operator realigns. No automatic home motion and no safety reset messages.
"""
from collections import deque
from copy import deepcopy
import json
import signal
import time
import uuid

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from robotis_vuer.vr_publisher_sg2_xr import XRDeltaPublisher, SIDES
from robotis_vuer.xr_arm_targets import limited_pose, rigid_pose
from robotis_vuer.xr_resume_session import FixedResumeSession


class XRResumePublisher(XRDeltaPublisher):
    """No restart offset accumulation: immutable session + measured transient state."""

    WORKSPACE_FRACTION = .94
    SETTLING_SECONDS = 0.20
    CONTROLLER_ACTIVATION_DELAY = 3.0
    ALIGN_POSITION_M = .02
    ALIGN_ROTATION_DEG = 10.
    REANCHOR_ON_RESUME = False
    MANUAL_FAULT_RESET = False
    WARMUP_HOLD_SECONDS = 8.0
    STOP_SETTLE_SECONDS = 0.50
    STILL_WINDOW_SECONDS = 0.30

    def __init__(self, start_server=True):
        self._resume_ready = False
        self._session = None
        self._session_id = str(uuid.uuid4())
        self._resume_count = 0
        self._phase = 'WAIT_CALIBRATION'
        self._joint_history = deque(maxlen=120)
        self._last_joint_source = None
        self._command_rx = {}
        self._first_command_rx = {}
        self._enable_at = None
        self._controller_fault_latched = False
        self._diagnostics = {}
        self._last_diag = 0.0
        self._last_published = {}
        super().__init__(start_server=False)
        self._event_pub = self.create_publisher(String, '/sg2_xr/events', 10)
        self._diag_pub = self.create_publisher(String, '/sg2_xr/diagnostics', 10)
        self._command_subs = [self.create_subscription(
            JointTrajectory,
            f'/leader/joint_trajectory_command_broadcaster_{side}/joint_trajectory',
            lambda msg, s=side: self._command_feedback(msg, s), self.vr_stream_qos)
            for side in SIDES]
        self._resume_ready = True
        self._set_arm_control_enabled(False, 'fixed-reference startup')
        self.get_logger().info(
            '[XR RESUME] Y+B stops; release X/A then X+A resumes. ' +
            ('Resume anchors current hands to measured wrists, like xr_tele U release. '
             if self.REANCHOR_ON_RESUME else
             f'Original mapping requires alignment within {self.ALIGN_POSITION_M*1000:g}mm/{self.ALIGN_ROTATION_DEG:g}deg. ') +
            'Hold hands still until [XR RESUME READY].')
        if start_server:
            super().start_vuer_server()

    def _event(self, kind, reason):
        self._last_reason = str(reason)
        data = {'event': kind, 'reason': str(reason), 'session_id': self._session_id,
                'resume_count': self._resume_count, 'phase': self._phase,
                'ros_time_s': self.get_clock().now().nanoseconds * 1e-9,
                'alignment': self._diagnostics}
        self._event_pub.publish(String(data=json.dumps(data, allow_nan=False)))
        self.get_logger().info(f'[XR RESUME {kind}] {reason}')

    def joint_states_callback(self, msg):
        # [FIX] Receipt freshness alone does not establish fresh measured state.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        if not self.RECEIPT_TIME_ONLY and not -0.1 <= now_ros - stamp <= self.FEEDBACK_MAX_AGE:
            self._joint_rx = None
            self._joint_history.clear()
            return
        # [FIX] Replayed equal-stamp samples cannot refresh feedback liveness
        # or manufacture a stationary window from one measurement.
        if self._last_joint_source is not None and stamp <= self._last_joint_source:
            return
        super().joint_states_callback(msg)
        if self._joint_rx is not None:
            self._last_joint_source = stamp
            mapping = dict(zip(msg.name, msg.position))
            q = np.array([mapping[f'arm_{s}_joint{i}']
                          for s in ('l', 'r') for i in range(1, 8)])
            self._joint_history.append((time.monotonic(), stamp, q))

    def _command_feedback(self, msg, side):
        required = {f'arm_{SIDES[side]}_joint{i}' for i in range(1, 8)}
        if (not msg.points or len(msg.joint_names) != len(msg.points[-1].positions)
                or len(msg.joint_names) != len(set(msg.joint_names))
                or not required.issubset(msg.joint_names)
                or not np.isfinite(msg.points[-1].positions).all()):
            return
        now = time.monotonic()
        self._command_rx[side] = now
        if (self._phase == 'WARMUP' and self._enable_at is not None
                and now >= self._enable_at + self.CONTROLLER_ACTIVATION_DELAY):
            # Commands have zero source stamps in this stack; delayed packets
            # must not shorten the known 3s pending stage + 8s ramp hold.
            self._first_command_rx.setdefault(side, now)

    def _graph_check(self):
        super()._graph_check()
        # [ADDED] Warmup output observations are meaningful only with one
        # expected Cyclo writer. Refuse simultaneous test/manual publishers.
        for side in SIDES:
            topic = f'/leader/joint_trajectory_command_broadcaster_{side}/joint_trajectory'
            writers = self.get_publishers_info_by_topic(topic)
            if len(writers) != 1 or writers[0].node_name != 'vr_controller':
                raise ValueError(f'{topic}: require exactly one vr_controller command writer')

    def _require_still_robot(self, now):
        now_ros = self.get_clock().now().nanoseconds*1e-9
        samples = [item for item in self._joint_history
                   if now - item[0] <= self.FEEDBACK_MAX_AGE and item[0] >= self._stopped_at + 0.05
                   and (self.RECEIPT_TIME_ONLY or (item[1] >= self._stopped_ros + 0.10
                   and -0.1 <= now_ros-item[1] <= self.FEEDBACK_MAX_AGE))]
        if (len(samples) < 3 or samples[-1][0] - samples[0][0] < self.STILL_WINDOW_SECONDS
                or (not self.RECEIPT_TIME_ONLY and samples[-1][1] - samples[0][1] < self.STILL_WINDOW_SECONDS)
                or any(b[1] <= a[1] for a, b in zip(samples, samples[1:]))):
            raise ValueError(
                f'wait for {self.STILL_WINDOW_SECONDS:.2f}s of post-stop stable measured joints')
        if float(np.max(np.ptp(np.stack([x[2] for x in samples]), axis=0))) > 0.01:
            raise ValueError('robot still moving after stop; hold and press X+A again')

    def _capture(self):
        # [FIX] Reuse freshness/model checks, NOT the old capture operation
        # that changes head basis, wrist correspondence and shell each retry.
        self._graph_check()
        if self._controller_fault_latched:
            raise ValueError('controller reference fault latched: inspect cause and restart controller/publisher')
        now = time.monotonic()
        if not self._arm_tracking_is_fresh(now):
            raise ValueError('fresh head/controllers and measured joint_states required')
        if self._stopped_ros is None or now - self._stopped_at < self.STOP_SETTLE_SECONDS:
            raise ValueError('wait after stop for measured state to settle')
        self._require_still_robot(now)
        links = self._snapshot()
        robots = {}
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        for side in SIDES:
            pose, receipt, source = self._robot_poses.get(side, (None, -1e9, -1e9))
            if (pose is None or now - receipt > self.FEEDBACK_MAX_AGE or receipt < self._stopped_at + 0.15
                    or (not self.RECEIPT_TIME_ONLY and
                        (source < self._stopped_ros + 0.10 or not -0.1 <= now_ros - source <= self.FEEDBACK_MAX_AGE))):
                raise ValueError(f'missing inactive Cyclo {side} gripper pose')
            actual = links[side]['wrist']
            dp = np.linalg.norm(pose[:3, 3] - actual[:3, 3])
            dr = Rotation.from_matrix(pose[:3, :3] @ actual[:3, :3].T).magnitude()
            if dp > 0.04 or dr > np.deg2rad(20):
                raise ValueError(f'{side}: Cyclo/PhysX mismatch {dp:.3f}m/{np.rad2deg(dr):.1f}deg')
            robots[side] = actual.copy() if self.REANCHOR_ON_RESUME else pose.copy()
        # [FIX] Do not publish a far-away seed just to update the reference
        # checker's history. Repeated X+A must keep refusing the same unsafe
        # transition until the caller inspects/restarts the controller session.
        for side, previous in (() if self.REANCHOR_ON_RESUME else self._last_published.items()):
            dp = np.linalg.norm(robots[side][:3, 3] - previous[:3, 3])
            dr = Rotation.from_matrix(robots[side][:3, :3] @ previous[:3, :3].T).magnitude()
            if dp > 0.099 or dr > np.deg2rad(29.0):
                raise ValueError(f'SEED_JUMP_REFUSED {side}: {dp:.3f}m/{np.rad2deg(dr):.1f}deg; '
                                 'inspect and restart controller/publisher, no checker-history reset')
        inputs = {s: rigid_pose(getattr(self, f'{s}_controller_matrix')) for s in SIDES}
        candidate = (None if self.REANCHOR_ON_RESUME else self._session) or FixedResumeSession(
            self.head_transform_matrix, inputs, robots, links, self.translation_scale,
            self.ALIGN_POSITION_M, self.ALIGN_ROTATION_DEG,
            workspace_fraction=self.WORKSPACE_FRACTION)
        self._diagnostics = candidate.inspect(inputs, robots)
        if self._session is not None and not self.REANCHOR_ON_RESUME:
            candidate.require_alignment(inputs, robots)
        # [FIX] A controller-FK/PhysX model tolerance of40mm/20deg above
        # cannot weaken the20mm/10deg restart alignment to MEASURED poses.
        measured_robots = {s: links[s]['wrist'] for s in SIDES}
        self._diagnostics = candidate.inspect(inputs, measured_robots)
        candidate.require_alignment(inputs, measured_robots)
        hints = candidate.fresh_hints(
            links, enforce_shoulder_stationarity=self.ENFORCE_SHOULDER_STATIONARITY)
        # Session commit is atomic, after both arms and all preconditions pass.
        self._session = candidate
        self._mapper = candidate.mapper
        self._hints = hints
        self._goals = {s: robots[s].copy() for s in SIDES}
        self._elbows = {s: links[s]['elbow'].copy() for s in SIDES}
        self._last_input = inputs
        self._last_pub = None

    def _set_arm_control_enabled(self, enabled, reason):
        if not self._resume_ready:
            return super()._set_arm_control_enabled(enabled, reason)
        with self._xr_lock:
            if enabled and self.arm_control_enabled:
                self.get_logger().info('[XR RESUME] Already active; no recapture')
                return
            if not enabled:
                super()._set_arm_control_enabled(False, reason)
                # [FIX] Erase every transient command/filter continuation, not
                # the immutable calibration. No failed half-frame survives.
                self._goals, self._elbows, self._hints, self._last_input = {}, {}, {}, {}
                self._last_pub = self._enable_at = None
                self._command_rx.clear()
                self._first_command_rx.clear()
                # [CHANGED] Keep measured history flowing across Y+B. Restart
                # validation below still accepts only post-stop samples.
                self.arm_start_armed = False
                self._phase = 'ALIGN_REQUIRED' if self._session else 'WAIT_CALIBRATION'
                if self._controller_fault_latched:
                    self._phase = 'CONTROLLER_FAULT'
                self._event('STOP', reason)
                return
            try:
                self._capture()
            except Exception as error:
                self._set_arm_control_enabled(False, str(error))
                self._event('START_REFUSED', str(error))
                return
            self.arm_control_enabled = True
            self._phase = 'SETTLING'
            self._arming_until = time.monotonic() + self.SETTLING_SECONDS
            self._resume_count += 1
            self._publish_reactivate(False, 'fixed-reference measured goals settling')
            self._event('CAPTURE' if self._resume_count == 1 else 'RESUME',
                        ('measured anchor captured; hold hands still through controller startup' if self.REANCHOR_ON_RESUME else
                         'original calibration fixed; hold hands still through controller startup'))

    def _reference_fault(self, msg):
        if self._resume_ready and msg.data:
            self._controller_fault_latched = True
            self._set_arm_control_enabled(False, 'Cyclo reference fault latched; X+A cannot clear it')

    def _publish_pair(self):
        super()._publish_pair()
        self._last_published = deepcopy(self._goals)

    def _publish_synced_pose_frame_if_ready(self):
        if not self._resume_ready or not self.arm_control_enabled or self._mapper is None:
            return
        with self._xr_lock:
            now = time.monotonic()
            if self._last_pub is not None and now - self._last_pub < 1 / 30:
                return
            try:
                if not self._arm_tracking_is_fresh(now):
                    raise ValueError('tracking or measured feedback stale')
                self._graph_check()
                links = self._snapshot()
                inputs = {s: rigid_pose(getattr(self, f'{s}_controller_matrix')) for s in SIDES}
                actual = {s: links[s]['wrist'] for s in SIDES}
                self._diagnostics = self._session.inspect(inputs, actual)
                for side, hint in self._session.templates.items():
                    self._check_shoulder_stationarity(links, side, hint, 'session')
                if self._phase in ('SETTLING', 'WARMUP'):
                    # [FIX] Do not re-zero head/input at the end of settling.
                    # Nor queue human motion while Cyclo waits 3s + ramps 8s.
                    self._session.require_alignment(inputs, self._goals)
                    # [FIX] Holding input/targets is insufficient if PhysX is
                    # reset or moved during startup. Check LIVE measured poses
                    # before every held frame and before declaring readiness.
                    for side in SIDES:
                        dp = np.linalg.norm(self._goals[side][:3, 3] - actual[side][:3, 3])
                        dr = Rotation.from_matrix(self._goals[side][:3, :3] @ actual[side][:3, :3].T).magnitude()
                        if dp > 0.02 or dr > np.deg2rad(10):
                            raise ValueError(f'{side} measured pose moved during startup: '
                                             f'{dp*1000:.1f}mm/{np.rad2deg(dr):.1f}deg')
                    self._publish_pair()
                    self._last_input = deepcopy(inputs)
                    if self._phase == 'SETTLING' and now >= self._arming_until:
                        self._arming_until = None
                        self._enable_at = now
                        self._phase = 'WARMUP'
                        self._first_command_rx.clear()
                        self._publish_reactivate(True, 'manual aligned resume; stationary targets during warmup')
                    elif self._phase == 'WARMUP':
                        if len(self._first_command_rx) == 2:
                            if any(now - self._command_rx.get(s, -1e9) > self.FEEDBACK_MAX_AGE for s in SIDES):
                                raise ValueError('Cyclo arm command feedback stale during warmup')
                            if now - max(self._first_command_rx.values()) >= self.WARMUP_HOLD_SECONDS:
                                self._session.require_alignment(inputs, actual)
                                self._phase = 'ACTIVE'
                                self._event(
                                    'READY',
                                    f'startup outputs observed and {self.WARMUP_HOLD_SECONDS:g}s '
                                    'ramp hold complete; move arms now')
                        elif now - self._enable_at > self.CONTROLLER_ACTIVATION_DELAY + 3.0:
                            raise ValueError('Cyclo produced no complete arm commands after start')
                    self._last_pub = now
                    return
                if any(now - self._command_rx.get(s, -1e9) > self.FEEDBACK_MAX_AGE for s in SIDES):
                    raise ValueError('Cyclo command feedback stopped')
                dt = 1 / 30 if self._last_pub is None else now - self._last_pub
                goals, elbows, hints = deepcopy(self._goals), deepcopy(self._elbows), deepcopy(self._hints)
                for side in SIDES:
                    if np.linalg.norm(goals[side][:3, 3] - actual[side][:3, 3]) > 0.25:
                        raise ValueError(f'{side} wrist tracking error exceeds0.25m')
                    if Rotation.from_matrix(goals[side][:3, :3] @ actual[side][:3, :3].T).magnitude() > np.pi / 2:
                        raise ValueError(f'{side} wrist orientation tracking error exceeds90deg')
                    requested = self._mapper.target(side, inputs[side])
                    if np.linalg.norm(requested[:3, 3] - self._mapper.anchors[side].robot_pose[:3, 3]) > 0.5:
                        raise ValueError('original 0.5m clutch range exceeded; return hand, do not reclutch')
                    requested[:3, 3], _ = hints[side].project_wrist(requested[:3, 3])
                    old_position = goals[side][:3, 3].copy()
                    goals[side] = limited_pose(goals[side], requested, dt, self.goal_speed, self.angular_speed)
                    wrist, _ = hints[side].project_wrist(goals[side][:3, 3])
                    if np.linalg.norm(wrist - old_position) > self.goal_speed * min(dt, 0.05) + 1e-8:
                        raise ValueError('workspace projection exceeds speed bound')
                    goals[side][:3, 3] = wrist
                    elbow = elbows[side].copy()
                    elbow[:3, 3] = hints[side].elbow(wrist)
                    elbows[side] = limited_pose(elbows[side], elbow, dt, self.goal_speed, self.angular_speed)
                # [FIX] Commit a complete bilateral frame only after all guards.
                self._goals, self._elbows, self._hints = goals, elbows, hints
                self._last_input = deepcopy(inputs)
                self._publish_pair()
                self._last_pub = now
            except Exception as error:
                self._set_arm_control_enabled(False, f'XR resume guard: {error}')

    def _tracking_timeout_reason(self, now):
        samples = [('body', self.last_body_pose_sec, self.arm_tracking_timeout_sec),
                   *[(side + '_controller', self.last_controller_pose_sec.get(side),
                      self.arm_tracking_timeout_sec) for side in SIDES],
                   ('joint_feedback', self._joint_rx, self.FEEDBACK_MAX_AGE)]
        details = []
        for name, stamp, limit in samples:
            age = None if stamp is None else now - stamp
            if age is None or not 0.0 <= age <= limit:
                age_text = 'missing' if age is None else f'{age:.3f}s'
                details.append(f'{name} age={age_text} limit={limit:.3f}s')
        return 'tracking/feedback timeout: ' + ('; '.join(details) or 'freshness guard rejected input')

    def _check_arm_tracking_timeout(self):
        if not self._resume_ready:
            return
        with self._xr_lock:
            now = time.monotonic()
            if not self._arm_tracking_is_fresh(now):
                self.arm_start_armed = False
                if self.arm_control_enabled:
                    self._set_arm_control_enabled(False, self._tracking_timeout_reason(now))
            self._publish_synced_pose_frame_if_ready()
            if self._session is not None and not self.arm_control_enabled and self._arm_tracking_is_fresh(now):
                try:
                    links = self._snapshot()
                    self._diagnostics = self._session.inspect(
                        {s: getattr(self, f'{s}_controller_matrix') for s in SIDES},
                        {s: links[s]['wrist'] for s in SIDES})
                except Exception:
                    self._diagnostics = {}
            state = {'mode': 'xr_fixed_resume', 'active': self.arm_control_enabled,
                     'phase': self._phase, 'ready': self._phase == 'ACTIVE',
                     'settling': self._phase in ('SETTLING', 'WARMUP'),
                     'fresh': self._arm_tracking_is_fresh(now), 'session_id': self._session_id,
                     'resume_count': self._resume_count, 'reason': self._last_reason,
                     'controller_fault_latched': self._controller_fault_latched,
                     'published_pairs': self.published_pairs, 'scale': self.translation_scale,
                     'goal_speed_m_s': self.goal_speed, 'arms': self._diagnostics}
            if now - self._last_status >= 1.0:
                self._last_status = now
                self._status_pub.publish(String(data=json.dumps(state, allow_nan=False)))
                if self._phase == 'ALIGN_REQUIRED' and self._diagnostics:
                    line = ' '.join(f"{s}: {d['align_position_m']*1000:.0f}mm/"
                                    f"{d['align_rotation_deg']:.1f}deg "
                                    f"hand_correction_robot_xyz_mm="
                                    f"{np.round(-1000*np.array(d['raw_minus_measured_xyz_m']), 1).tolist()}"
                                    for s, d in self._diagnostics.items())
                    self.get_logger().info('[ALIGN] Return hands to existing robot mapping; ' + line)
            if now - self._last_diag >= 0.10:
                self._last_diag = now
                self._diag_pub.publish(String(data=json.dumps(state, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def stop_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_signal)
    node = None
    try:
        node = XRResumePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node._set_arm_control_enabled(False, 'fixed-reference publisher shutdown')
                rclpy.spin_once(node, timeout_sec=0.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
