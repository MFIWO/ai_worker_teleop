"""[ADDED 2026-10-14] Guided SIM overhead mode with non-warping ray saturation.

Remove ONLY the arbitrary 0.5m distance-from-initial-wrist stop. Keep original
calibration, startup handshake, tracking, feedback, QP, joint/CBF, manual stop,
source/graph checks and the finite calibrated arm workspace. Original modules
remain unchanged. A robot's redundant elbow posture need not retrace its path.
"""
from copy import deepcopy
import json
import signal
import time

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from std_msgs.msg import String

from robotis_vuer.vr_publisher_sg2_xr_guided import XRGuidedPublisher
from robotis_vuer.vr_publisher_sg2_xr import SIDES
from robotis_vuer.xr_arm_targets import limited_pose, rigid_pose
from robotis_vuer.xr_path_workspace import AnchorRayHint


class XROverheadPublisher(XRGuidedPublisher):
    def __init__(self, start_server=True):
        self._path_ready = False
        self._path_rx = -1e9
        self._path_frame = None
        self._path_diag_at = -1e9
        super().__init__(start_server=False)
        self._path_pub = self.create_publisher(String, '/sg2_xr/path_diagnostics', 10)
        self._path_ready = True
        self._log_path_mode()
        if start_server:
            self.start_vuer_server()

    def _log_path_mode(self):
        # [ADDED] Separate hardware subclass announces its own stricter defaults.
        self.get_logger().info('[OVERHEAD] SIM ONLY: no 0.5m clutch stop. '
                               'Captured-wrist ray saturation preserves straight out/back targets. '
                               'AR guides, fixed calibration, all OTHER checks remain enabled.')

    def _capture(self):
        first = self._session is None
        super()._capture()
        if first:
            # [CHANGED] Geometry/radii from the existing extension capture; no
            # recalibration on resume, and no replacement of the Cyclo solver.
            self._session.templates = {
                s: AnchorRayHint.from_hint(h, self._session.mapper.anchors[s].robot_pose[:3, 3])
                for s, h in self._session.templates.items()}
            self._hints = deepcopy(self._session.templates)
        self._path_frame = None

    def _held_startup_snapshot(self):
        # [CHANGED] Existing extension startup checks copied exactly except
        # the distance-from-initial-wrist 0.5m check. Misaligned hands still
        # require WAIT_START confirmation; no automatic movement or catch-up.
        if self._controller_fault_latched:
            raise ValueError('controller reference fault latched')
        if not self._arm_tracking_is_fresh():
            raise ValueError('tracking or measured feedback stale')
        self._graph_check()
        links = self._snapshot()
        inputs = {s: rigid_pose(getattr(self, f'{s}_controller_matrix')) for s in SIDES}
        actual = {s: links[s]['wrist'] for s in SIDES}
        self._diagnostics = self._session.inspect(inputs, actual)
        for side, hint in self._session.templates.items():
            if np.linalg.norm(links[side]['shoulder'][:3, 3] - hint.shoulder) > .005:
                raise ValueError('shoulder/base moved during startup')
            dp = np.linalg.norm(self._goals[side][:3, 3] - actual[side][:3, 3])
            dr = Rotation.from_matrix(self._goals[side][:3, :3] @ actual[side][:3, :3].T).magnitude()
            if dp > .02 or dr > np.deg2rad(10):
                raise ValueError(f'{side} measured pose moved during startup: '
                                 f'{dp*1000:.1f}mm/{np.rad2deg(dr):.1f}deg')
        return inputs, actual

    def _publish_synced_pose_frame_if_ready(self):
        if (not getattr(self, '_path_ready', False) or not self.arm_control_enabled
                or self._mapper is None or self._phase != 'ACTIVE'):
            return super()._publish_synced_pose_frame_if_ready()
        with self._xr_lock:
            now = time.monotonic()
            if self._last_pub is not None and now - self._last_pub < 1/30:
                return
            try:
                if self._controller_fault_latched:
                    raise ValueError('controller reference fault latched')
                if not self._arm_tracking_is_fresh(now):
                    raise ValueError('tracking or measured feedback stale')
                self._graph_check()
                links = self._snapshot()
                inputs = {s: rigid_pose(getattr(self, f'{s}_controller_matrix')) for s in SIDES}
                actual = {s: links[s]['wrist'] for s in SIDES}
                self._diagnostics = self._session.inspect(inputs, actual)
                if any(now - self._command_rx.get(s, -1e9) > .5 for s in SIDES):
                    raise ValueError('Cyclo command feedback stopped')
                dt = 1/30 if self._last_pub is None else now - self._last_pub
                goals, elbows, hints = deepcopy(self._goals), deepcopy(self._elbows), deepcopy(self._hints)
                frame = {}
                for side in SIDES:
                    hint = hints[side]
                    if np.linalg.norm(links[side]['shoulder'][:3, 3] - hint.shoulder) > .005:
                        raise ValueError('shoulder/base moved during session; inspect simulator and recalibrate')
                    if np.linalg.norm(goals[side][:3, 3] - actual[side][:3, 3]) > .25:
                        raise ValueError(f'{side} wrist tracking error exceeds0.25m')
                    if Rotation.from_matrix(goals[side][:3, :3] @ actual[side][:3, :3].T).magnitude() > np.pi/2:
                        raise ValueError(f'{side} wrist orientation tracking error exceeds90deg')
                    raw = self._mapper.target(side, inputs[side])
                    requested = raw.copy()
                    requested[:3, 3], clipped = hint.project_wrist(raw[:3, 3])
                    previous = goals[side][:3, 3].copy()
                    filtered = limited_pose(goals[side], requested, dt, self.goal_speed, self.angular_speed)
                    filtered[:3, 3], step_fraction = hint.limit_filtered_step(previous, filtered[:3, 3])
                    if np.linalg.norm(filtered[:3, 3] - previous) > self.goal_speed*min(dt,.05)+1e-8:
                        raise ValueError('workspace step exceeds speed bound')
                    goals[side] = filtered
                    elbow = elbows[side].copy()
                    elbow[:3, 3] = hint.elbow(filtered[:3, 3])
                    elbows[side] = limited_pose(elbows[side], elbow, dt, self.goal_speed, self.angular_speed)
                    # [ADDED] Observe all stages, including rotation, so input
                    # drift, workspace shaping, filtering and IK are separable.
                    frame[side] = {
                        'input_pose_webxr': inputs[side].tolist(), 'raw_pose_base': raw.tolist(),
                        'workspace_pose_base': requested.tolist(), 'published_pose_base': filtered.tolist(),
                        'actual_pose_base': actual[side].tolist(), 'workspace_clipped_m': clipped,
                        'filtered_step_fraction': step_fraction,
                        'anchor_displacement_m': float(np.linalg.norm(raw[:3,3]-hint.path_anchor)),
                        'raw_orientation_from_anchor_deg': float(np.degrees(Rotation.from_matrix(
                            raw[:3,:3] @ self._mapper.anchors[side].robot_pose[:3,:3].T).magnitude())),
                    }
                # [PRESERVED] Atomic bilateral publication after every check.
                self._goals, self._elbows, self._hints = goals, elbows, hints
                self._last_input = deepcopy(inputs)
                self._publish_pair()
                self._last_pub = now
                self._path_frame, self._path_rx = frame, now
            except Exception as error:
                self._set_arm_control_enabled(False, f'XR overhead guard: {error}')

    def _check_arm_tracking_timeout(self):
        super()._check_arm_tracking_timeout()
        if not getattr(self, '_path_ready', False):
            return
        with self._xr_lock:
            now = time.monotonic()
            if now - self._path_diag_at < .1:
                return
            self._path_diag_at = now
            fresh = (self.arm_control_enabled and self._phase == 'ACTIVE'
                     and self._arm_tracking_is_fresh(now) and now-self._path_rx <= .3)
            report = {'mode': 'xr_overhead_anchor_ray', 'phase': self._phase,
                      'session_id': self._session_id, 'fresh': bool(fresh),
                      'ros_time_s': self.get_clock().now().nanoseconds*1e-9,
                      'frame_age_s': max(0., now-self._path_rx),
                      'arms': self._path_frame if fresh else {}}
            self._path_pub.publish(String(data=json.dumps(report, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = XROverheadPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node._set_arm_control_enabled(False, 'overhead guided publisher shutdown')
                rclpy.spin_once(node, timeout_sec=.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
