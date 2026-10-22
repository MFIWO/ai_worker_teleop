"""Attach bare-hand input and native SH5 fingers to the verified hardware arm path.

HardwareXRPublisher retains capture, reanchor, wrist mapping, phase transitions,
tracking grace, speed settings and gate verification. Only the WebXR input and
finger landmark output are added here.
"""
import asyncio
import json
import os
import signal
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from geometry_msgs.msg import Point32
from robotis_interfaces.msg import HandJoints
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String
from vuer.schemas import Body, Hands, Scene

from robotis_vuer.sh5_optical_hand import parse_hand_poses, retarget_sh5_hand
from robotis_vuer.vr_publisher_ai_worker_hardware import HardwareXRPublisher
from rclpy.signals import SignalHandlerOptions
from hardware.keyboard_controls import MOTION_KEYS, keyboard_aux

SIDES = ('left', 'right')


class SH5HandsHardwarePublisher(HardwareXRPublisher):
    TRACKING_JUMP_POSITION_M = 0.30
    ENFORCE_SHOULDER_STATIONARITY = False
    def __init__(self, start_server=True):
        self._operator_keys = set()
        self._operator_rx = -1e9
        self._operator_tick = time.monotonic()
        self._operator_targets = {}
        self._operator_sequence = -1
        self._operator_instance = None
        self._base_profile_dirty = False
        self._base_last_input = -1e9
        self._recapture_future = None
        self._recapture_requested = False
        self._recapture_barrier = None
        self._recapture_retry_at = -1e9
        self._recapture_deadline = -1e9
        self._operator_release_required = False
        self._operator_block_log_at = -1e9
        self._loop_controls = os.environ.get('AI_WORKER_LOOP_CONTROLS') == '1'
        self._gesture_since = None
        self._gesture_ready = False
        self._gesture_latched = False
        self._hand_previous = dict.fromkeys(SIDES)
        self._last_hand_event = None
        super().__init__(start_server=False)
        if self._loop_controls:
            # Native leader joystick updates at 100 Hz. Held keys use the same
            # measured-position jog cadence; other modes retain their heartbeat.
            self._intent_timer.cancel()
            self.destroy_timer(self._intent_timer)
            self._intent_timer = self.create_timer(.01, self._heartbeat)
            self.create_subscription(String, '/xr_hardware/operator_keys', self._operator_input, 10)
            self._recapture_client = self.create_client(Trigger, '/xr_hardware/recapture_profile')
            self.create_timer(.2, self._maybe_recapture)
        if self.hardware_model != "sh5":
            raise ValueError("Optical SH5 hands require --model sh5")
        self._hand_publishers = {side: self.create_publisher(
            HandJoints, f'/{side}_hand/hand_joint_pos', self.vr_stream_qos)
            for side in SIDES}
        self.vuer.add_handler('HAND_MOVE')(self.on_hand_move)
        self.create_service(SetBool, '/sh5_teleop/enable', self._enable_service)
        self.get_logger().info(
            '[SH5 HANDS] Optical wrists -> existing HardwareXRPublisher; '
            'fingers -> original ROBOTIS retargeting_teleop. '
            'Release gestures, then pinch + opposite squeeze for 3s to start/stop.')
        if start_server:
            self.start_vuer_server()

    def _operator_input(self, msg):
        with self._xr_lock:
            try:
                data = json.loads(msg.data)
                sequence, instance = int(data['sequence']), str(data['instance'])
                if instance == self._operator_instance and sequence <= self._operator_sequence:
                    return
                keys = set(data['keys'])
                toggle = data.get('toggle', False)
                if (keys - MOTION_KEYS or not isinstance(data['focused'], bool) or
                        not isinstance(data['stop'], bool) or not isinstance(toggle, bool)):
                    raise ValueError('invalid Loop keyboard event')
                self._operator_sequence, self._operator_instance = sequence, instance
                self._operator_rx = time.monotonic()
                self._operator_keys = keys if data['focused'] else set()
                if not keys:self._operator_release_required = False
                if self._recapture_requested or self._recapture_future is not None or self._operator_release_required:
                    if self._operator_keys and self._operator_rx-self._operator_block_log_at>=2.:
                        self.get_logger().warning('[LOOP KEYS BLOCKED] Profile recapture in progress; release motion keys, wait for BASE RECAPTURED, then press again.')
                        self._operator_block_log_at=self._operator_rx
                    self._operator_keys.clear()
                if self._operator_keys & {'w', 'a', 's', 'd', 'o', 'p'}:
                    self._base_profile_dirty = True
                    self._base_last_input = self._operator_rx
                    if self.arm_control_enabled:
                        self._set_arm_control_enabled(False, 'Loop base reposition: hold arms; recapture before resuming')
                    self._phase = 'HOLD'
                if data['stop']:
                    self._operator_keys.clear()
                    self._operator_targets.clear()
                    self._set_arm_control_enabled(False, 'Loop operator HOLD; recording continues')
                    self._reset_hand_input()
                elif toggle and data['focused']:
                    self._operator_keys.clear()
                    self._operator_targets.clear()
                    response = self._enable_service(
                        SetBool.Request(data=not self.arm_control_enabled), SetBool.Response())
                    if not response.success:
                        self.get_logger().warning('[LOOP C REFUSED] '+response.message)
            except (ValueError, KeyError, TypeError):
                self._operator_keys.clear()
                self._operator_rx = -1e9

    def _maybe_recapture(self):
        with self._xr_lock:
            if self._recapture_future is not None:
                if not self._recapture_future.done():
                    if time.monotonic()>self._recapture_deadline:
                        self._recapture_future.cancel()
                        self._recapture_future=None
                        self._recapture_requested=False
                        self._operator_release_required=True
                        self.get_logger().error('[BASE RECAPTURE TIMEOUT] Robot service did not reply; arms stay held. Inspect robot session; release keys before manual controls.')
                    return
                try:
                    response=self._recapture_future.result()
                    if response.success:
                        self._base_profile_dirty=False
                        self.get_logger().info('[BASE RECAPTURED] Fresh collision profile ready; release motion keys, then C to resume arms. Keyboard controls are available.')
                    else:self.get_logger().error('[BASE RECAPTURE] '+response.message)
                except Exception as error:self.get_logger().error('[BASE RECAPTURE] '+str(error))
                self._recapture_future=None
                self._recapture_requested=False
                self._operator_release_required=True
                return
            # Keep the gate alive for continuous keyboard operation after base
            # repositioning. Recapture is requested only by an explicit start.
            if (not self._recapture_requested or not self._base_profile_dirty or self._operator_keys or
                    time.monotonic()-self._base_last_input < .7 or self.arm_control_enabled):return
            now=time.monotonic()
            if now>self._recapture_deadline:
                self._recapture_requested=False
                self._operator_release_required=True
                self.get_logger().error('[BASE RECAPTURE TIMEOUT] No eligible gate/service acknowledgement; keyboard controls unlock after key release. Inspect robot session; C retries, arms stay held.')
                return
            if now < self._recapture_retry_at:return
            gate=self._gate_state or {}
            barrier=self._recapture_barrier
            if (now-self._gate_rx>.5 or gate.get('enabled') or gate.get('manual_active') or
                    barrier and self._gate_instance==barrier[0] and self._gate_status_sequence<=barrier[1]):return
            if gate.get('fault'):
                self._recapture_requested=False
                self.get_logger().error('[BASE RECAPTURE REFUSED] '+str(gate['fault'])+'; inspect/reset this fault before recapture.')
                return
            if self._recapture_client.service_is_ready():
                self._recapture_future=self._recapture_client.call_async(Trigger.Request())
                self.get_logger().info('[BASE RECAPTURE] Waiting for stable wheels/joints and new collision profile.')
                self._recapture_retry_at=now+2.

    def _heartbeat(self):
        with self._xr_lock:
            super()._heartbeat()

    def _extra_intent(self, now):
        dt, self._operator_tick = now-self._operator_tick, now
        manifest = self._hardware_manifest or {}
        allowed = (self._loop_controls and self.hardware_allow_arm and manifest.get('manual_auxiliary')
                   and not self._recapture_requested and self._recapture_future is None
                   and now-self._operator_rx <= .3 and self._joint_rx is not None
                   and now-self._joint_rx <= .5 and not self._controller_fault_latched)
        if not allowed or not self._operator_keys:
            if self._operator_keys and now-self._operator_block_log_at>=2.:
                reasons=[]
                if not manifest.get('manual_auxiliary'):reasons.append('matching keyboard profile not received')
                if self._joint_rx is None or now-self._joint_rx>.5:reasons.append('joint feedback missing/older than 0.5s')
                if self._controller_fault_latched:reasons.append('controller fault: '+str(self._gate_fault_reason or self._last_reason))
                if now-self._operator_rx>.3:reasons.append('keyboard input older than 0.3s')
                if self._recapture_requested or self._recapture_future is not None:reasons.append('profile recapture in progress')
                self.get_logger().warning('[LOOP KEYS BLOCKED] '+('; '.join(reasons) or 'manual motion not enabled'))
                self._operator_block_log_at=now
            self._operator_targets.clear()
            return {'manual_enabled': False, 'recording_epoch_us':time.time_ns()//1000}
        try:
            msg = self.current_joint_states
            aux = keyboard_aux(self._operator_keys, dict(zip(msg.name, msg.position)),
                               manifest['auxiliary_limits'], self._operator_targets, dt)
            self._operator_targets = aux['joints']
            return {'manual_enabled': True, 'manual_aux': aux, 'recording_epoch_us':time.time_ns()//1000}
        except (ValueError, KeyError, TypeError):
            self._operator_targets.clear()
            return {'manual_enabled': False, 'recording_epoch_us':time.time_ns()//1000}

    def _graph_check(self):
        super()._graph_check()
        if (self._hardware_manifest or {}).get('hand_control') != 'robotis_native_measured_fingers':
            raise ValueError('Robot must run sh5 --hands with the measured-finger model')

    async def on_controller_move(self, event, session):
        """Only optical wrists may feed the existing arm path in this mode."""
        return

    def _reset_hand_input(self):
        self._hand_previous = dict.fromkeys(SIDES)
        self._gesture_since = None
        self._gesture_ready = False
        self._gesture_latched = False
        self._last_hand_event = None

    def _gesture_buttons(self, value, now):
        states = [value.get(side + 'State', {}) for side in SIDES]
        combo = all(isinstance(s, dict) for s in states) and bool(
            (states[0].get('pinch') and states[1].get('squeeze')) or
            (states[1].get('pinch') and states[0].get('squeeze')))
        if not combo:
            self._gesture_since = None
            self._gesture_latched = False
            self._gesture_ready = True
        elif self._gesture_ready and not self._gesture_latched:
            if self._gesture_since is None:
                self._gesture_since = now
            elif now - self._gesture_since >= 3.0:
                self._gesture_latched = True
                start = not self.arm_control_enabled or self._phase == "WAIT_START"
                return start, not start
        return False, False

    async def on_hand_move(self, event, session):
        with self._xr_lock:
            # Partial optical frames must not become commands or refresh liveness.
            try:
                value = getattr(event, 'value', None)
                if not isinstance(value, dict):
                    raise ValueError('expected both optical hands')
                parsed = {side: parse_hand_poses(value[side]) for side in SIDES}
            except (KeyError, TypeError, ValueError, OverflowError) as error:
                self._gesture_since = None
                self._gesture_ready = False
                self._gesture_latched = False
                self._hand_previous = dict.fromkeys(SIDES)
                now = time.monotonic()
                if now - getattr(self, '_invalid_hand_log_at', -1e9) >= 2.:
                    self.get_logger().warning(
                        '[SH5 INPUT DROP] Ignoring incomplete/invalid hand frame; '
                        'last valid targets retained, input timeout unchanged: ' + str(error))
                    self._invalid_hand_log_at = now
                # Preserve last successful receipts. Existing hardware hold/grace
                # still expires if no complete, valid hand input returns.
                return
            try:
                now = time.monotonic()
                if (self._last_hand_event is not None and
                        now - self._last_hand_event > self.arm_tracking_timeout_sec):
                    # Keep the inherited arm tracking-grace behavior. Reset
                    # only gesture timing/filter state after an optical gap.
                    self._reset_hand_input()
                self._last_hand_event = now
                fresh_body = (self.last_body_pose_sec is not None and
                              0 <= now - self.last_body_pose_sec <= self.arm_tracking_timeout_sec)
                start, stop = self._gesture_buttons(value, now) if fresh_body else (False, False)
                if self._base_profile_dirty:
                    start=False
                if not fresh_body:
                    self._reset_hand_input()
                adapted = {}
                for side, (positions, rotation) in parsed.items():
                    wrist = np.eye(4)
                    wrist[:3, :3], wrist[:3, 3] = rotation, positions[0]
                    adapted[side] = wrist.flatten(order='F').tolist()
                    # No trigger/stick values: never generate SG2 gripper,
                    # lift, head or base commands from a hand gesture.
                    adapted[side + 'State'] = {'aButton': start, 'bButton': stop}
                await HardwareXRPublisher.on_controller_move(
                    self, SimpleNamespace(value=adapted), session)
                if (not self.arm_control_enabled or self._phase != "ACTIVE" or
                        self._tracking_input_state() != "fresh" or
                        self._feedback_input_state(time.monotonic()) != "fresh"):
                    self._hand_previous = dict.fromkeys(SIDES)
                    return
                processed = {side: retarget_sh5_hand(
                    value[side], self.head_transform_matrix, self._hand_previous[side])
                    for side in SIDES}
                stamp = self.get_clock().now().to_msg()
                for side, (points, previous) in processed.items():
                    msg = HandJoints()
                    msg.header.stamp = stamp
                    msg.joints = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                                  for p in points]
                    self._hand_publishers[side].publish(msg)
                    self._hand_previous[side] = previous
            except (KeyError, TypeError, ValueError, OverflowError, AttributeError) as error:
                self.last_controller_pose_sec = dict.fromkeys(SIDES)
                self._reset_hand_input()
                if self.arm_control_enabled:
                    self._set_arm_control_enabled(False, 'invalid optical hands: ' + str(error))

    def _enable_service(self, request, response):
        with self._xr_lock:
            if request.data and self._base_profile_dirty:
                if not self._recapture_requested and self._recapture_future is None:
                    self._operator_keys.clear()
                    self._operator_targets.clear()
                    self._operator_release_required=True
                    self._recapture_requested=True
                    self._recapture_deadline=time.monotonic()+75.
                    self._recapture_barrier=(self._gate_instance,self._gate_status_sequence)
                    self._set_arm_control_enabled(False,'C: stop manual controls before base profile recapture')
                    self._phase='HOLD'
                    self._heartbeat()
                    self.get_logger().info('[BASE RECAPTURE REQUESTED] C requested a fresh wheel profile; arms held. Wait for BASE RECAPTURED, then C again.')
                response.success = False
                response.message = 'Base/wheel profile recapture requested; wait for BASE RECAPTURED, release keys, then C again.'
                return response
            if request.data and not self._arm_tracking_is_fresh():
                response.success = False
                response.message = 'Both hands and body tracking must be fresh.'
                return response
            self._set_arm_control_enabled(request.data, 'manual SH5 enable service')
            self._reset_hand_input()
            response.success = self.arm_control_enabled == request.data
            response.message = f'arm_enabled={self.arm_control_enabled}; phase={self._phase}; {self._last_reason}'
        return response

    async def main_hand_tracking(self, session):
        session.set @ Scene(
            Body(fps=self.fps, stream=True, key='body_tracking',
                 leftHand=False, rightHand=False, hideIndicate=False,
                 showFrame=True, showBody=True, frameScale=.02),
            bgChildren=[Hands(fps=self.fps, stream=True, key='hands',
                              hideLeft=False, hideRight=False)])
        while True:
            await asyncio.sleep(1.0 / self.fps)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = SH5HandsHardwarePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._set_arm_control_enabled(False, 'SH5 publisher shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
