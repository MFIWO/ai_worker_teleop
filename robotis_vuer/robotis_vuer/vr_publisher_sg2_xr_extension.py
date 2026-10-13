"""[ADDED] Opt-in SIM near-extension + fixed-calibration resume.

Original XR and fixed-resume modules are unchanged. No direct elbow command:
wrist-centric Cyclo IK chooses all seven joints, as in the H1_2/H2 principle.
Requires the separate SG2 near-extension controller model and elbow cost0.
"""
from copy import deepcopy
import signal
import time

import numpy as np
import rclpy
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation

from robotis_vuer.vr_publisher_sg2_xr_resume import XRResumePublisher
from robotis_vuer.vr_publisher_sg2_xr import SIDES
from robotis_vuer.xr_arm_targets import rigid_pose
from robotis_vuer.xr_extension_math import extended_templates, EXPERIMENT_Q4_UPPER


class XRExtensionPublisher(XRResumePublisher):
    def __init__(self, start_server=True):
        self._profile_verified = False
        self._profile_rx = -1e9
        self._profile_future = None
        self._extension_initialised = False
        # [FIX 2026-10-14] Hand drift during the stationary startup must not
        # repeatedly restart Cyclo's 3s delay + 8s ramp. Keep the SAME goals and
        # calibration, then require a fresh manual X+A after realignment.
        self._startup_reconfirm = False
        self._startup_log_at = -1e9
        self._wait_start_at = None
        super().__init__(start_server=False)
        self._profile_client = AsyncParameterClient(self, '/vr_controller')
        self._profile_timer = self.create_timer(1.0, self._query_profile)
        self._log_extension_mode()
        self.get_logger().info('[STARTUP] Hand motion during warmup is NOT followed. '
                               'If it exceeds the existing alignment window, targets stay held; '
                               'after [XR RESUME WAIT_START], realign and press a fresh X+A. '
                               'No calibration reset and no automatic catch-up motion.')
        if start_server:
            self.start_vuer_server()

    def _log_extension_mode(self):
        # [ADDED] Hardware subtype uses original bounds and must not claim SIM limits.
        self.get_logger().info('[EXTENSION] SIM ONLY: frozen calibration, radial fraction0.995, '
                               'separate joint4 upper=-0.45 model required. Not a hardware limit.')

    def _query_profile(self):
        if self._profile_future is not None and not self._profile_future.done():
            return
        if not self._profile_client.services_are_ready():
            return
        self._profile_future = self._profile_client.get_parameters(['urdf_path', 'weight_elbow_position', 'weight_position'])
        self._profile_future.add_done_callback(self._profile_result)

    def _profile_result(self, future):
        try:
            values = future.result().values
            # File contents are checked by the profile generator; this confirms
            # the *running* node selected that model, not its unused source file.
            ok = (values[0].string_value.endswith('/sg2_near_extension.urdf')
                  and values[1].type == 3 and values[1].double_value == 0.0
                  and values[2].type == 3 and values[2].double_value == 10.0)
            self._profile_verified = ok
            self._profile_rx = time.monotonic()
        except Exception:
            self._profile_verified = False

    def _graph_check(self):
        super()._graph_check()
        if not self._profile_verified or time.monotonic()-self._profile_rx > 4.0:
            raise ValueError('EXTENSION_PROFILE_REQUIRED: running Cyclo must use '
                             'sg2_near_extension.urdf, weight_elbow_position=0.0, weight_position=10.0')

    def _capture(self):
        first = self._session is None
        super()._capture()
        if first:
            # [ADDED] Expand ONCE, before the first goal publication. Every
            # later X+A keeps these exact bounds, input anchors and heading.
            self._session.templates = extended_templates(self._session.templates)
            self._hints = deepcopy(self._session.templates)
            self._extension_initialised = True
            self.get_logger().info('[EXTENSION] Original mapping retained; near-extension shell captured once')

    def _held_startup_snapshot(self):
        """[ADDED] Check physical safety BEFORE publishing any held frame.

        Only human input alignment may wait; model/tracking/fault/command
        failures still stop. This deliberately does not move any anchor.
        """
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
            if np.linalg.norm(links[side]['shoulder'][:3, 3] - hint.shoulder) > 0.005:
                raise ValueError('shoulder/base moved during startup')
            dp = np.linalg.norm(self._goals[side][:3, 3] - actual[side][:3, 3])
            dr = Rotation.from_matrix(self._goals[side][:3, :3] @ actual[side][:3, :3].T).magnitude()
            if dp > 0.02 or dr > np.deg2rad(10):
                raise ValueError(f'{side} measured pose moved during startup: '
                                 f'{dp*1000:.1f}mm/{np.rad2deg(dr):.1f}deg')
            raw = self._session.mapper.target(side, inputs[side])
            if np.linalg.norm(raw[:3, 3] - self._session.mapper.anchors[side].robot_pose[:3, 3]) > 0.5:
                raise ValueError('original 0.5m clutch range exceeded during startup')
        return inputs, actual

    def _operator_start_aligned(self, inputs, actual):
        """Keep the existing raw (not clipped) 20mm/10deg entrance contract."""
        try:
            self._session.require_alignment(inputs, self._goals)
            self._session.require_alignment(inputs, actual)
            return True
        except ValueError as error:
            if not str(error).startswith('ALIGN_REQUIRED'):
                raise
            return False

    def _set_arm_control_enabled(self, enabled, reason):
        if (getattr(self, '_resume_ready', False) and enabled
                and self.arm_control_enabled and self._phase == 'WAIT_START'):
            with self._xr_lock:
                try:
                    inputs, actual = self._held_startup_snapshot()
                    now = time.monotonic()
                    if any(now - self._command_rx.get(s, -1e9) > 0.5 for s in SIDES):
                        raise ValueError('Cyclo command feedback stale at startup confirmation')
                    if not self._operator_start_aligned(inputs, actual):
                        self._event('CONFIRM_REFUSED', 'return both hands to ORIGINAL mapping '
                                    'within 20mm/10deg; robot stays held, no new calibration')
                        return
                    self._require_still_robot(now)
                    # [FIX] No _capture(), no new goals, and no reactivate edge:
                    # the controller has already completed stationary warmup.
                    self._startup_reconfirm = False
                    self._wait_start_at = None
                    self._last_input = deepcopy(inputs)
                    self._last_pub = now
                    self._phase = 'ACTIVE'
                    self._event('READY', 'manual X+A confirmed original mapping; move arms now')
                except Exception as error:
                    self._set_arm_control_enabled(False, f'extension startup guard: {error}')
            return
        super()._set_arm_control_enabled(enabled, reason)
        if not enabled:
            self._startup_reconfirm = False
            self._wait_start_at = None
            self._startup_log_at = -1e9

    def _publish_synced_pose_frame_if_ready(self):
        # [FIX] The original resume publisher and ACTIVE path are unchanged.
        # Only this extension mode's stationary startup has the hold policy.
        if (not getattr(self, '_resume_ready', False) or not self.arm_control_enabled
                or self._mapper is None or self._phase not in ('SETTLING', 'WARMUP', 'WAIT_START')):
            return super()._publish_synced_pose_frame_if_ready()
        with self._xr_lock:
            now = time.monotonic()
            if self._last_pub is not None and now - self._last_pub < 1 / 30:
                return
            try:
                inputs, actual = self._held_startup_snapshot()
                aligned = self._operator_start_aligned(inputs, actual)
                if not aligned:
                    self._startup_reconfirm = True
                if self._phase in ('WARMUP', 'WAIT_START'):
                    if self._first_command_rx:
                        if any(now - self._command_rx.get(s, -1e9) > 0.5
                               for s in self._first_command_rx):
                            raise ValueError('Cyclo arm command feedback stale during warmup')
                    if len(self._first_command_rx) < 2 and now - self._enable_at > 10.0:
                        raise ValueError('Cyclo produced no complete arm commands after start')
                if self._phase == 'WAIT_START':
                    if any(now - self._command_rx.get(s, -1e9) > 0.5 for s in SIDES):
                        raise ValueError('Cyclo arm command feedback stale while awaiting X+A')
                    if self._wait_start_at is not None and now - self._wait_start_at > 60.0:
                        raise ValueError('startup confirmation timed out; stopping with original calibration retained')
                # Send only the captured held pair, never a queued human target.
                self._publish_pair()
                self._last_input = deepcopy(inputs)
                if self._phase == 'SETTLING' and now >= self._arming_until:
                    self._arming_until = None
                    self._enable_at = now
                    self._phase = 'WARMUP'
                    self._first_command_rx.clear()
                    self._publish_reactivate(True, 'stationary extension startup; human motion held')
                elif self._phase == 'WARMUP' and len(self._first_command_rx) == 2:
                    if now - max(self._first_command_rx.values()) >= 8.0:
                        if self._startup_reconfirm:
                            self._phase = 'WAIT_START'
                            self._wait_start_at = now
                            self.arm_start_armed = False  # Require release after this phase transition.
                            self._event('WAIT_START', 'controller warmup complete; robot HELD. '
                                        'Realign to ORIGINAL mapping, release X/A, then press X+A. '
                                        'No automatic catch-up or recalibration.')
                        else:
                            self._session.require_alignment(inputs, actual)
                            self._phase = 'ACTIVE'
                            self._event('READY', 'startup outputs observed and 8s ramp hold complete; move arms now')
                if now - self._startup_log_at >= 1.0 and self._phase != 'ACTIVE':
                    self._startup_log_at = now
                    errors = ' '.join(f"{s}={d['align_position_m']*1000:.1f}mm/"
                                      f"{d['align_rotation_deg']:.1f}deg"
                                      for s, d in self._diagnostics.items())
                    remaining = (f"ramp_remaining={max(0., 8.0-(now-max(self._first_command_rx.values()))):.1f}s"
                                 if len(self._first_command_rx) == 2 else 'waiting_for_both_Cyclo_outputs')
                    self.get_logger().info(f'[XR STARTUP] {self._phase} {remaining} '
                                           f'input_aligned={aligned} {errors}; goals HELD')
                self._last_pub = now
            except Exception as error:
                self._set_arm_control_enabled(False, f'extension startup guard: {error}')

    def _command_feedback(self, msg, side):
        name = 'arm_'+{'left': 'l', 'right': 'r'}[side]+'_joint4'
        if msg.points and name in msg.joint_names and len(msg.joint_names) == len(msg.points[-1].positions):
            q4 = msg.points[-1].positions[msg.joint_names.index(name)]
            # Fail closed if the QP exceeds this experiment's upper boundary;
            # no silent command clipping or safety-checker reset is performed.
            if np.isfinite(q4) and q4 > EXPERIMENT_Q4_UPPER+0.005 and self.arm_control_enabled:
                self._set_arm_control_enabled(False, 'near-extension joint4 command exceeds experiment bound')
                return
        super()._command_feedback(msg, side)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = XRExtensionPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node._set_arm_control_enabled(False, 'extension publisher shutdown')
                rclpy.spin_once(node, timeout_sec=0.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
