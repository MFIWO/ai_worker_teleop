"""Attach bare-hand input and native SH5 fingers to the verified hardware arm path.

HardwareXRPublisher retains capture, reanchor, wrist mapping, phase transitions,
tracking grace, speed settings and gate verification. Only the WebXR input and
finger landmark output are added here.
"""
import asyncio
import signal
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from geometry_msgs.msg import Point32
from robotis_interfaces.msg import HandJoints
from std_srvs.srv import SetBool
from vuer.schemas import Body, Hands, Scene

from robotis_vuer.sh5_optical_hand import parse_hand_poses, retarget_sh5_hand
from robotis_vuer.vr_publisher_ai_worker_hardware import HardwareXRPublisher
from rclpy.signals import SignalHandlerOptions

SIDES = ('left', 'right')


class SH5HandsHardwarePublisher(HardwareXRPublisher):
    TRACKING_JUMP_POSITION_M = 0.30
    ENFORCE_SHOULDER_STATIONARITY = False
    def __init__(self, start_server=True):
        self._gesture_since = None
        self._gesture_ready = False
        self._gesture_latched = False
        self._hand_previous = dict.fromkeys(SIDES)
        self._last_hand_event = None
        super().__init__(start_server=False)
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
