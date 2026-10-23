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
from std_srvs.srv import SetBool
from std_msgs.msg import String
from vuer.schemas import Body, Hands, Scene, MotionControllers

from robotis_vuer.sh5_optical_hand import parse_hand_poses, retarget_sh5_hand
from robotis_vuer.vr_publisher_ai_worker_hardware import HardwareXRPublisher
from robotis_vuer.vr_publisher_sg2 import VRTrajectoryPublisher
from rclpy.signals import SignalHandlerOptions
from hardware.keyboard_controls import MOTION_KEYS, keyboard_aux
from hardware.controller_hand_modes import ControllerHandModes,POSE_CODES

SIDES = ('left', 'right')


class SH5HandsHardwarePublisher(HardwareXRPublisher):
    TRACKING_JUMP_POSITION_M = 0.30
    ENFORCE_SHOULDER_STATIONARITY = False
    SETTLING_SECONDS = 0.0
    CONTROLLER_ACTIVATION_DELAY = 0.0
    WARMUP_HOLD_SECONDS = 0.03
    STOP_SETTLE_SECONDS = 0.10
    STILL_WINDOW_SECONDS = 0.03
    POST_STOP_POSE_SECONDS = 0.05
    def __init__(self, start_server=True):
        self._fast_resume_at = None
        self._controller_mode = os.environ.get('AI_WORKER_SH5_INPUT','hands') == 'controllers'
        self._hand_mode = int(os.environ.get('AI_WORKER_SH5_HAND_MODE','1'))
        if self._hand_mode not in (1,2) or (self._hand_mode==2 and not self._controller_mode):
            raise ValueError('hand mode 2 requires SH5 controller input')
        self._hand_modes = ControllerHandModes()
        self._controller_event_rx = -1e9
        self._stick_log_at = -1e9
        self._operator_keys = set()
        self._operator_rx = -1e9
        self._operator_tick = time.monotonic()
        self._operator_targets = {}
        self._operator_sequence = -1
        self._operator_instance = None
        self._base_profile_dirty = False
        self._base_last_input = -1e9
        self._operator_release_required = False
        self._operator_block_log_at = -1e9
        self._loop_controls = os.environ.get('AI_WORKER_LOOP_CONTROLS') == '1' and not self._controller_mode
        self._gesture_since = None
        self._gesture_ready = False
        self._gesture_latched = False
        self._hand_previous = dict.fromkeys(SIDES)
        self._last_hand_event = None
        super().__init__(start_server=False)
        if self._loop_controls or self._controller_mode:
            # Native leader joystick updates at 100 Hz. Held keys use the same
            # measured-position jog cadence; other modes retain their heartbeat.
            self._intent_timer.cancel()
            self.destroy_timer(self._intent_timer)
            self._intent_timer = self.create_timer(.01, self._heartbeat)
            if self._loop_controls:
                self.create_subscription(String, '/xr_hardware/operator_keys', self._operator_input, 10)
        if self.hardware_model != "sh5":
            raise ValueError("Optical SH5 hands require --model sh5")
        self._hand_publishers = {} if self._controller_mode else {side: self.create_publisher(
            HandJoints, f'/{side}_hand/hand_joint_pos', self.vr_stream_qos)
            for side in SIDES}
        if not self._controller_mode:self.vuer.add_handler('HAND_MOVE')(self.on_hand_move)
        self.create_service(SetBool, '/sh5_teleop/enable', self._enable_service)
        if self._controller_mode:
            self.get_logger().info('[SH5 CONTROLLERS] X+A start / Y+B HOLD; SG2 sticks; trigger=0..1 grasp, grip=3-finger pinch. Loop motion keys disabled.')
            self.get_logger().info('[STICKS] Initial HEAD mode: left=head, right up/down=lift. Both stick clicks toggle MOBILE: left=translation, right left/right=yaw. Requires ACTIVE.')
            self.get_logger().info(f'[HAND MODE {self._hand_mode}] '+(
                'Original proportional trigger grasp; grip=thumb/index/middle pinch.' if self._hand_mode==1 else
                'Tap X=full grip, Y=angled grip, B=point, A=thumb-out grip. Each trigger captures its own pose until released; grip pinch overrides. AX/BY reserved.'))
        else:
            self.get_logger().info(
                '[SH5 HANDS] Optical wrists -> existing HardwareXRPublisher; '
                'fingers -> original ROBOTIS retargeting_teleop through the hand gate. '
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
                if self._operator_release_required:
                    if self._operator_keys and self._operator_rx-self._operator_block_log_at>=2.:
                        self.get_logger().warning('[LOOP KEYS BLOCKED] Release motion keys before another manual command.')
                        self._operator_block_log_at=self._operator_rx
                    self._operator_keys.clear()
                if self._operator_keys & {'w', 'a', 's', 'd', 'o', 'p'}:
                    self._base_profile_dirty = True
                    self._base_last_input = self._operator_rx
                    if self.arm_control_enabled:
                        self._set_arm_control_enabled(False, 'Loop base reposition: hold arms; C reanchors without controller restart')
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

    def _heartbeat(self):
        with self._xr_lock:
            if self._controller_mode and time.monotonic()-self._controller_event_rx>.3:
                self._aux_base = [0.,0.,0.]
                self._aux_joints.clear()
            super()._heartbeat()

    def _controller_aux_selected(self):
        return (self._controller_mode and time.monotonic()-self._controller_event_rx <= .3
                and bool((self._hardware_manifest or {}).get('manual_auxiliary')))

    def publish_right_joystick(self, thumbstick_value):
        # Leader-style exclusive modes: HEAD has lift; MOBILE has yaw only.
        # The inherited stick parser still handles axis swaps and mode clicks.
        if self.joystick_mode:
            super().publish_right_joystick(thumbstick_value)

    def _report_sticks(self, now, allowed):
        active = False
        for side in SIDES:
            state = getattr(self,side+'_controller_state',None)
            if not isinstance(state,dict):continue
            active = active or bool(state.get('thumbstick'))
            values = state.get('thumbstickValue',[])
            if isinstance(values,(list,tuple)):
                active = active or any(isinstance(v,(int,float)) and np.isfinite(v) and abs(v)>.1 for v in values[:2])
        if not active or now-self._stick_log_at<2.:return
        self._stick_log_at = now
        mode = 'HEAD' if self.joystick_mode else 'MOBILE'
        if allowed:
            self.get_logger().info(f'[STICK INTENT] mode={mode}; joints={self._aux_joints}; base={self._aux_base}')
        else:
            reasons=[]
            manifest=self._hardware_manifest or {}
            if not self.arm_control_enabled or self._phase!='ACTIVE':reasons.append(f'phase={self._phase}; A+X must reach ACTIVE')
            if not self.hardware_allow_arm:reasons.append('PC --arm missing')
            if not manifest.get('manual_auxiliary') or not manifest.get('auxiliary_limits'):reasons.append('robot controller-stick profile missing; restart matching robot command')
            if self._controller_fault_latched:reasons.append('controller/gate fault')
            if self._tracking_input_state(now)!='fresh':reasons.append('controller/body tracking not fresh')
            if self._feedback_input_state(now)!='fresh':reasons.append('robot feedback not fresh')
            self.get_logger().warning(f'[STICK BLOCKED] mode={mode}; '+('; '.join(reasons) or 'controller auxiliary input unavailable'))

    def _extra_intent(self, now):
        if self._controller_mode:
            controls = None
            fresh = now-self._controller_event_rx <= .3 and self._tracking_input_state(now)=='fresh'
            if not fresh:
                self._aux_base = [0.,0.,0.]
                self._aux_joints.clear()
            if fresh:
                try:
                    controls = {}
                    for side in SIDES:
                        state = getattr(self,side+'_controller_state')
                        raw = float(state['triggerValue'])
                        grip = float(state.get('squeezeValue',float(bool(state.get('squeeze',False)))))
                        if not np.isfinite([raw,grip]).all():raise ValueError('nonfinite controller hand input')
                        controls[side] = dict(trigger=self.calibrate_trigger(side,raw),grip=min(1.,max(0.,grip)))
                except (KeyError,TypeError,ValueError):controls = None
            enabled = self.arm_control_enabled and self._phase=='ACTIVE' and not self._controller_fault_latched
            hand_controls = controls if enabled else None
            if self._hand_mode==2:
                if not enabled:self._hand_modes.reset_grasps()
                if hand_controls is not None:hand_controls=self._hand_modes.controls(hand_controls)
            active_codes=[]
            for side in SIDES:
                item=(hand_controls or {}).get(side,{})
                active_codes.append(5 if item.get('grip',0.)>.05 else
                    POSE_CODES.get(item.get('pose','full_grip'),0) if item.get('trigger',0.)>.02 else 0)
            return {'manual_enabled':False,'hand_input':'controllers','controller_values':controls,
                    'hand_mode':self._hand_mode,'controller_hands':hand_controls,
                    'hand_pose_state':[self._hand_mode,POSE_CODES[self._hand_modes.selected] if self._hand_mode==2 else 0,*active_codes],
                    'recording_epoch_us':time.time_ns()//1000}
        dt, self._operator_tick = now-self._operator_tick, now
        manifest = self._hardware_manifest or {}
        allowed = (self._loop_controls and self.hardware_allow_arm and manifest.get('manual_auxiliary')
                   and now-self._operator_rx <= .3 and self._joint_rx is not None
                   and now-self._joint_rx <= .5 and not self._controller_fault_latched)
        if not allowed or not self._operator_keys:
            if self._operator_keys and now-self._operator_block_log_at>=2.:
                reasons=[]
                if not manifest.get('manual_auxiliary'):reasons.append('matching keyboard profile not received')
                if self._joint_rx is None or now-self._joint_rx>.5:reasons.append('joint feedback missing/older than 0.5s')
                if self._controller_fault_latched:reasons.append('controller fault: '+str(self._gate_fault_reason or self._last_reason))
                if now-self._operator_rx>.3:reasons.append('keyboard input older than 0.3s')
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
        expected = 'controllers' if self._controller_mode else 'hands'
        if (self._hardware_manifest or {}).get('hand_input','hands') != expected:
            raise ValueError('PC/robot --input must match: '+expected)
        if self._controller_mode and (self._hardware_manifest or {}).get('hand_mode',1)!=self._hand_mode:
            raise ValueError('PC/robot --hand-mode must match')

    async def on_controller_move(self, event, session):
        if not self._controller_mode:return
        self._controller_event_rx = time.monotonic()
        with self._xr_lock:
            if self._hand_mode==2:
                data=event.value
                if (self._xr_ready and isinstance(data,dict)
                        and all(isinstance(data.get(s+'State'),dict) for s in SIDES)):
                    changed=self._hand_modes.buttons(data['leftState'],data['rightState'])
                    if changed:self.get_logger().info(f'[HAND POSE] {changed[0]} -> {changed[1]}; released hands adopt selection (Y: C-shaped rest), held poses retained')
                else:self._hand_modes.reset_buttons()
        await HardwareXRPublisher.on_controller_move(self,event,session)
        with self._xr_lock:
            now=time.monotonic()
            allowed=self._controller_aux_ready(now)
            if allowed:
                VRTrajectoryPublisher.process_thumbstick(self)
                if not self.joystick_mode:self._aux_joints.pop('lift_joint',None)
                if self._head_target_clamped is not None:self.last_head_command=self._head_target_clamped
                if 'lift_joint' in self._aux_joints:self.last_lift_command=self._aux_joints['lift_joint']
            self._report_sticks(now,allowed)
            self._heartbeat()

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
            began = time.monotonic()
            if request.data and self._base_profile_dirty:
                if not (self._hardware_manifest or {}).get('live_wheel_feedback'):
                    response.success = False
                    response.message = 'Update robot code/build once for live wheel feedback; no automatic recapture or restart.'
                    return response
                gate = self._gate_state or {}
                if (time.monotonic()-self._base_last_input < .1 or time.monotonic()-self._gate_rx > .5
                        or gate.get('enabled') or gate.get('manual_active')):
                    response.success = False
                    response.message = 'Release base keys and wait for fresh disarmed gate acknowledgement, then C.'
                    return response
            if request.data and not self._arm_tracking_is_fresh():
                response.success = False
                response.message = 'Both hands and body tracking must be fresh.'
                return response
            self._fast_resume_at = began if request.data else None
            self._set_arm_control_enabled(request.data, 'manual SH5 enable service')
            self._reset_hand_input()
            if request.data and self.arm_control_enabled:
                self._base_profile_dirty = False
                self.get_logger().info('[BASE REANCHORED] Current measured wrists; controller stayed running, no snapshot/profile rebuild.')
                # Publish the stationary seed/reactivate in this callback,
                # rather than adding another 30 Hz frame before startup.
                self._publish_synced_pose_frame_if_ready()
            response.success = self.arm_control_enabled == request.data
            response.message = f'arm_enabled={self.arm_control_enabled}; phase={self._phase}; {self._last_reason}'
        return response

    def _event(self, tag, message):
        if tag == 'READY' and self._fast_resume_at is not None:
            message += f'; C-to-ready={(time.monotonic()-self._fast_resume_at)*1000:.1f}ms'
            self._fast_resume_at = None
        elif tag in ('STOP','START_REFUSED'):
            self._fast_resume_at = None
        super()._event(tag,message)

    async def main_hand_tracking(self, session):
        if self._controller_mode:
            return await VRTrajectoryPublisher.main_hand_tracking(self,session)
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
