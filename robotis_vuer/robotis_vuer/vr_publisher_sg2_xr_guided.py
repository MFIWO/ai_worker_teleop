"""[ADDED 2026-10-14] Existing extension teleop + display-only AR alignment.

Only the Vuer scene is changed. All tracking handlers, calibration, X+A/Y+B,
startup gates, target limits and reference faults use the unchanged parent.
Never run this beside another Vuer publisher: it replaces that process.
"""
import asyncio
from copy import deepcopy
import math
import signal
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from vuer.schemas import Body, MotionControllers, Scene

from robotis_vuer.vr_publisher_sg2_xr_extension import XRExtensionPublisher
from robotis_vuer.xr_calibration_overlay import overlay_state, render_overlay


class XRGuidedPublisher(XRExtensionPublisher):
    def __init__(self, start_server=True):
        self._overlay_last_error = None
        super().__init__(start_server=False)
        self.overlay_hz = float(self.declare_parameter('xr_calibration_overlay_hz', 10.).value)
        if not math.isfinite(self.overlay_hz) or not 2 <= self.overlay_hz <= 20:
            raise ValueError('xr_calibration_overlay_hz must be in [2,20]')
        self.get_logger().info('[CAL OVERLAY] Display only: L cyan / R orange targets, '
                               'white live origins, matching RGB axes. Green=pose match, '
                               'not automatic activation. Enter the usual passthrough XR session.')
        if start_server:
            self.start_vuer_server()

    def _overlay_snapshot(self):
        # Copy a coherent snapshot without touching any control state. No await
        # while holding the parent lock, and no extra ROS publisher/subscriber.
        with self._xr_lock:
            head = deepcopy(self.head_transform_matrix)
            phase = self._phase
            fault = self._controller_fault_latched
            reason = self._last_reason or ''
            fresh = self._arm_tracking_is_fresh()
            session = deepcopy(self._session)
            inputs = {s: deepcopy(getattr(self, s + '_controller_matrix'))
                      for s in ('left', 'right')}
            measured = None
            if fresh:
                links = self._snapshot()
                measured = {s: links[s]['wrist'] for s in inputs}
            held = deepcopy(self._goals) if phase in ('SETTLING', 'WARMUP', 'WAIT_START') else None
            remaining = None
            if phase == 'WARMUP' and len(self._first_command_rx) == 2:
                remaining = max(0., 8.0 - (time.monotonic() - max(self._first_command_rx.values())))
        return overlay_state(session, inputs, measured, phase, fresh, head,
                             held=held, fault=fault, ramp_remaining=remaining, reason=reason)

    async def main_hand_tracking(self, session):
        # [CHANGED VIEW ONLY] Same stream components; hide the body skeleton to
        # reduce clutter. No SceneBackground/skybox/ground obscures passthrough.
        session.set @ Scene(
            Body(fps=self.fps, stream=True, key='body_tracking', leftHand=False,
                 rightHand=False, hideIndicate=False, showFrame=False,
                 showBody=False, frameScale=.02),
            render_overlay(overlay_state(None, {}, None, 'WAIT_CALIBRATION', False)),
            bgChildren=[MotionControllers(stream=True, key='motion-controller', left=True, right=True)],
            up=[0, 1, 0],
        )
        self.get_logger().info('[CAL OVERLAY] Scene connected; use the usual Enter VR/passthrough, same 8012 server.')
        while True:
            try:
                state = self._overlay_snapshot()
                self._overlay_last_error = None
            except Exception as error:
                # [DEBUG] Display errors never bypass a controller guard or
                # change its phase. Hide markers rather than showing stale green.
                message = str(error)
                if message != self._overlay_last_error:
                    self.get_logger().warning('[CAL OVERLAY] Display unavailable: ' + message)
                    self._overlay_last_error = message
                state = overlay_state(None, {}, None, 'DISPLAY_UNAVAILABLE', False)
            session.upsert(render_overlay(state), strict=True)
            await asyncio.sleep(1.0 / self.overlay_hz)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = XRGuidedPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node._set_arm_control_enabled(False, 'guided extension publisher shutdown')
                rclpy.spin_once(node, timeout_sec=.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
