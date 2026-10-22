"""Bare-hand SH5 simulation: existing XR wrist delta plus ROBOTIS finger inputs.

Only the input source changes from controller grip pose to optical wrist pose.
CalibratedWristMapper, FixedResumeSession, AnchorRayHint and native Cyclo IK
remain the existing arm implementation. This module cannot run on domain 30.
"""
import asyncio
from copy import deepcopy
import json
import os
import signal
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Point32
from robotis_interfaces.msg import HandJoints
from std_msgs.msg import String
from std_srvs.srv import SetBool
from vuer.schemas import Body, Hands, Scene

from robotis_vuer.sh5_optical_hand import parse_hand_poses, retarget_sh5_hand
from robotis_vuer.vr_publisher_sg2_xr import XRDeltaPublisher, SIDES
from robotis_vuer.vr_publisher_sg2_xr_resume import XRResumePublisher
from robotis_vuer.vr_publisher_sg2_xr_overhead import XROverheadPublisher
from robotis_vuer.vr_publisher_ai_worker_hardware import HardwareXRPublisher
from robotis_vuer.xr_path_workspace import AnchorRayHint


def optical_wrist(data):
    positions, rotation = parse_hand_poses(data)
    wrist = np.eye(4)
    wrist[:3, :3], wrist[:3, 3] = rotation, positions[0]
    return wrist


class SH5SimulationPublisher(XROverheadPublisher):
    TRACKING_JUMP_POSITION_M = 0.30
    WORKSPACE_FRACTION = 1.0
    ENFORCE_SHOULDER_STATIONARITY = False
    # Match hardware: no extra Cartesian following-error shutdown.
    # Physical position/velocity limits remain in the QP and SimulationBridge.
    TRACKING_POSITION_LIMIT_M = float('inf')
    TRACKING_ROTATION_LIMIT_RAD = float('inf')
    # Requested 10x software limits; physical URDF bounds remain unchanged.
    DEFAULT_GOAL_SPEED = 2.0
    DEFAULT_ANGULAR_SPEED = 10.0
    MAX_GOAL_SPEED = 3.0
    MAX_ANGULAR_SPEED = 15.0
    FEEDBACK_MAX_AGE = 5.0
    ARM_TRACKING_TIMEOUT = 5.0
    OPTICAL_MAX_AGE = 3.5
    # Same measured-anchor startup used in the verified hardware wrist path.
    REANCHOR_ON_RESUME = True
    REZERO_ON_READY = True
    SETTLING_SECONDS = .1
    STOP_SETTLE_SECONDS = .2
    STILL_WINDOW_SECONDS = .15
    CONTROLLER_ACTIVATION_DELAY = .1
    WARMUP_HOLD_SECONDS = .1
    ALIGN_POSITION_M = .03
    ALIGN_ROTATION_DEG = 15.

    def _validate_runtime_domain(self):
        if any(os.environ.get(k) != v for k, v in {
                'ROS_DOMAIN_ID': '232', 'SH5_SIM_ONLY': '1',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}.items()):
            raise RuntimeError('SH5 bare-hand simulation requires FastDDS/domain232/SH5_SIM_ONLY=1')

    def __init__(self, start_server=True):
        self._hand_samples = {}
        self._hand_rx = {}
        self._hand_previous = {'left': None, 'right': None}
        self._gesture_start = None
        self._gesture_latched = False
        self._bridge_rx = -1e9
        self._bridge_status = {}
        super().__init__(start_server=False)
        self._hand_publishers = {s: self.create_publisher(
            HandJoints, f'/{s}_hand/hand_joint_pos', self.vr_stream_qos) for s in SIDES}
        self._sim_status = self.create_publisher(String, '/sh5_sim/input_status', 10)
        self.create_subscription(String, '/sh5_sim/bridge_status', self._bridge_feedback, 10)
        self.create_service(SetBool, '/sh5_sim/enable', self._enable_service)
        self.vuer.add_handler('HAND_MOVE')(self.on_hand_move)
        self.create_timer(.1, self._publish_sim_status)
        self.get_logger().info('[SH5 SIM] Bare wrists -> existing XR delta; fingers -> original ROBOTIS retarget. '
                               'One pinch + opposite squeeze held 3s starts/stops. Release before repeating.')
        if start_server:
            self.start_vuer_server()

    def _command_feedback(self, msg, side):
        # SH5 uses physical URDF bounds in the controller and simulation relay.
        # Skip SG2 near-extension's unrelated q4 <= -0.45 rad experiment check,
        # while retaining message validation and startup/feedback timestamps.
        XRResumePublisher._command_feedback(self, msg, side)

    def _log_extension_mode(self):
        self.get_logger().info('[SH5 SIM] Original arm limits, measured wrists, elbow cost 0; no SG2 extension model')

    def _log_path_mode(self):
        self.get_logger().info('[SH5 SIM] Existing wrist-delta; full geometric reach minus 10um; physical QP limits retained')

    def _bridge_feedback(self, msg):
        try:
            value = json.loads(msg.data)
            if value.get('simulation_only') is True:
                self._bridge_status, self._bridge_rx = value, time.monotonic()
        except (ValueError, TypeError):
            self._bridge_rx = -1e9

    def _query_profile(self):
        if self._profile_future is not None and not self._profile_future.done():
            return
        if self._profile_client.services_are_ready():
            self._profile_future = self._profile_client.get_parameters([
                'urdf_path', 'weight_elbow_position', 'weight_position', 'weight_orientation',
                'right_traj_topic', 'left_traj_topic', 'activation_delay_seconds', 'startup_ramp_seconds'])
            self._profile_future.add_done_callback(self._profile_result)

    def _profile_result(self, future):
        try:
            v = future.result().values
            self._profile_verified = (v[0].string_value.endswith('/sh5_sim_dynamic.urdf')
                and v[1].double_value == 0. and v[2].double_value == 10.
                and v[3].double_value == 1. and v[4].string_value == '/sh5_sim/raw/right_arm'
                and v[5].string_value == '/sh5_sim/raw/left_arm'
                and v[6].double_value == .1 and v[7].double_value == .1)
            self._profile_rx = time.monotonic()
        except Exception:
            self._profile_verified = False

    def _graph_check(self):
        XRDeltaPublisher._graph_check(self)
        now = time.monotonic()
        if not self._profile_verified or now - self._profile_rx > 40.:
            raise ValueError('Start sim/run_sh5_hybrid_sim.sh controller first; SH5 model not verified')
        if now - self._bridge_rx > self.FEEDBACK_MAX_AGE or not self._bridge_status.get('feedback_fresh'):
            raise ValueError('SH5 simulator feedback/bridge missing')
        if self._bridge_status.get('fault'):
            raise ValueError('SH5 simulation bridge: ' + self._bridge_status['fault'])
        nodes = [n for n, _ in self.get_node_names_and_namespaces()]
        if nodes.count('reference_checker') != 1 or nodes.count('retargeting_teleop') != 1:
            raise ValueError('Exactly one reference checker and one hand retargeter required')
        for side in SIDES:
            expected = {
                f'/sh5_sim/raw/{side}_arm': 'vr_controller',
                f'/sh5_sim/raw/{side}_hand': 'retargeting_teleop',
                f'/leader/joint_trajectory_command_broadcaster_{side}/joint_trajectory': 'sh5_sim_bridge',
                f'/leader/joint_trajectory_command_broadcaster_{side}_hand/joint_trajectory': 'sh5_sim_bridge',
            }
            for topic, owner in expected.items():
                writers = self.get_publishers_info_by_topic(topic)
                if len(writers) != 1 or writers[0].node_name != owner:
                    raise ValueError(f'{topic}: expected one {owner}; stop previous tests')
            if self.count_publishers(f'/{side}_hand/hand_joint_pos') != 1:
                raise ValueError('Competing hand input publisher')

    def _capture(self):
        # Use hardware's measured-anchor mapping, without the SG2 extension shell.
        XRResumePublisher._capture(self)
        self._session.templates = {s: AnchorRayHint.from_hint(
            h, self._session.mapper.anchors[s].robot_pose[:3, 3])
            for s, h in self._session.templates.items()}
        self._hints = deepcopy(self._session.templates)
        self._path_frame = None

    _rezero_start_inputs = HardwareXRPublisher._rezero_start_inputs

    def _arm_tracking_is_fresh(self, now=None):
        now = time.monotonic() if now is None else now
        return (super()._arm_tracking_is_fresh(now) and
                all(0 <= now - self._hand_rx.get(s, -1e9) <= self.OPTICAL_MAX_AGE for s in SIDES))

    async def on_controller_move(self, event, session):
        # Controller events cannot overwrite the optical wrists in this mode.
        return

    async def on_hand_move(self, event, session):
        value = getattr(event, 'value', None)
        with self._xr_lock:
            # A partial HAND_MOVE is missing input, not a command to stop.
            # Validate both hands before advancing any receipt time or output.
            try:
                if not isinstance(value, dict):
                    raise ValueError('expected both optical hands')
                wrists = {s: optical_wrist(value[s]) for s in SIDES}
            except (KeyError, ValueError, TypeError, OverflowError) as error:
                self._gesture_start = None
                self._gesture_latched = False
                self._hand_previous = dict.fromkeys(SIDES)
                now = time.monotonic()
                if now - getattr(self, '_invalid_hand_log_at', -1e9) >= 2.:
                    self.get_logger().warning(
                        '[SH5 INPUT DROP] Ignoring incomplete/invalid hand frame; '
                        'last valid targets retained, input timeout unchanged: ' + str(error))
                    self._invalid_hand_log_at = now
                # Do not clear OR refresh _hand_rx / controller receipts.
                # Persistent loss expires through the existing tracking watchdog.
                return
            try:
                now = time.monotonic()
                self._hand_samples = {s: value[s] for s in SIDES}
                self._hand_rx = {s: now for s in SIDES}
                # Existing parser/jump checks accept the wrists in WebXR world axes.
                adapted = {s: wrists[s].flatten(order='F').tolist() for s in SIDES}
                adapted.update({f'{s}State': {'aButton': False, 'bButton': False} for s in SIDES})
                await XRDeltaPublisher.on_controller_move(self, SimpleNamespace(value=adapted), session)
                states = [value.get(s + 'State', {}) for s in SIDES]
                combo = all(isinstance(x, dict) for x in states) and (
                    (states[0].get('pinch', False) and states[1].get('squeeze', False)) or
                    (states[1].get('pinch', False) and states[0].get('squeeze', False)))
                if combo:
                    if self._gesture_start is None:
                        self._gesture_start = now
                    if not self._gesture_latched and now - self._gesture_start >= 3.:
                        self._gesture_latched = True
                        enabled = self._phase == 'WAIT_START' or not self.arm_control_enabled
                        self._set_arm_control_enabled(enabled, 'optical pinch+squeeze held 3 seconds')
                else:
                    self._gesture_start = None
                    self._gesture_latched = False
                if self.arm_control_enabled and self._phase == 'ACTIVE' and self._arm_tracking_is_fresh():
                    self._graph_check()
                    results = {s: retarget_sh5_hand(value[s], self.head_transform_matrix,
                        previous=self._hand_previous[s]) for s in SIDES}
                    stamp = self.get_clock().now().to_msg()
                    for side, (points, previous) in results.items():
                        msg = HandJoints()
                        msg.header.stamp = stamp
                        msg.joints = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
                        self._hand_publishers[side].publish(msg)
                        self._hand_previous[side] = previous
                else:
                    self._hand_previous = dict.fromkeys(SIDES)
            except (KeyError, ValueError, TypeError, OverflowError) as error:
                self._hand_rx.clear()
                self._gesture_start = None
                if self.arm_control_enabled:
                    self._set_arm_control_enabled(False, 'optical hand input: ' + str(error))

    def _enable_service(self, request, response):
        with self._xr_lock:
            self._set_arm_control_enabled(request.data, 'manual simulation service')
            response.success = self.arm_control_enabled == request.data
            response.message = f'{self._phase}: {self._last_reason}'
        return response

    def _publish_sim_status(self):
        with self._xr_lock:
            fresh = self._arm_tracking_is_fresh()
            self._sim_status.publish(String(data=json.dumps({
                'simulation_only': True, 'enabled': bool(self.arm_control_enabled),
                'phase': self._phase, 'input_fresh': bool(fresh),
                'reason': self._last_reason, 'input': 'optical_wrists_and_fingers'})))

    async def main_hand_tracking(self, session):
        session.set @ Scene(
            Body(fps=self.fps, stream=True, key='body_tracking', leftHand=False, rightHand=False,
                 hideIndicate=False, showFrame=True, showBody=True, frameScale=.02),
            bgChildren=[Hands(fps=self.fps, stream=True, key='hands', hideLeft=False, hideRight=False)])
        self.get_logger().info('[SH5 SIM] Hands+Body session ready. Keep both hands visible.')
        while True:
            await asyncio.sleep(1. / self.fps)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = SH5SimulationPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node._set_arm_control_enabled(False, 'SH5 simulation exit')
                node._publish_sim_status()
                rclpy.spin_once(node, timeout_sec=.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
