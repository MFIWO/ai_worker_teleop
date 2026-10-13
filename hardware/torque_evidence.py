"""[CHANGED] Non-blocking verification of all fourteen physical SG2 arm torques.

The ROBOTIS torque_state message can report False despite energized actuators.
Verify by the official GetDataFromDxl service, NEVER SetBool torque-enable.
The upstream read service enqueues a read from its own ROS callback and may
time out before the subsequent hardware read cycle processes it. A short,
bounded first request primes the read buffer; the second request verifies
the returned register. Do not block the 100Hz hardware driver for 0.5s.
If any register is OFF, unresolved, or stale, proof expires (fail closed).
"""
import time

from dynamixel_interfaces.srv import GetDataFromDxl
from hardware.xr_contract import ARM_DXL_IDS


class TorqueEvidence:
    """Read-only, limited-rate rolling Torque Enable register evidence."""

    def __init__(self, node, max_age=6.0):
        self.node = node
        self.ids = tuple(ARM_DXL_IDS["right"] + ARM_DXL_IDS["left"])
        self.client = node.create_client(GetDataFromDxl, "/ffw_follower/get_dxl_data")
        self.max_age = float(max_age)
        self.good_at = {}
        self.cursor = 0
        self.phase = 0  # [CHANGED] 0=prime service read, 1=confirm its result
        self.future = None
        self.pending_id = None
        self.pending_phase = None
        self.pending_at = 0.0
        self.last_request = -1e12
        self.reason = "waiting for all fourteen real Torque Enable registers"
        self._warned = False

    def verified(self):
        now = time.monotonic()
        return all(now - self.good_at.get(i, -1e12) <= self.max_age
                   for i in self.ids)

    def _complete(self, succeeded):
        """[CHANGED] Only the second-stage confirmed read establishes proof."""
        identity = self.pending_id
        if self.pending_phase == 1:
            if succeeded:
                self.good_at[identity] = time.monotonic()
            else:
                self.good_at.pop(identity, None)
            self.phase = 0
            self.cursor = (self.cursor + 1) % len(self.ids)
        else:
            self.phase = 1
        self.pending_id = None
        self.pending_phase = None
        self.future = None

    def tick(self):
        """Nonblocking: call from owning rclpy event loop; no torque writes."""
        now = time.monotonic()
        if self.future is not None:
            if self.future.done():
                ok = False
                try:
                    response = self.future.result()
                    ok = (response is not None and bool(response.result)
                          and int(response.item_data) == 1)
                    if not ok and self.pending_phase == 1:
                        self.reason = (f"DXL {self.pending_id} Torque Enable OFF "
                                       "or register read failed")
                except Exception as exc:
                    self.reason = f"DXL {self.pending_id} register read failed: {exc}"
                self._complete(ok)
            elif now - self.pending_at > 0.3:
                self.reason = f"DXL {self.pending_id} torque service unresponsive"
                try:
                    self.future.cancel()
                except Exception:
                    pass
                self._complete(False)
            else:
                return
        # [CHANGED] 80ms spacing bounds extra serial read requests to 12.5/s.
        if now - self.last_request < 0.08:
            return
        if not self.client.service_is_ready():
            self.reason = "waiting for /ffw_follower/get_dxl_data service"
            return
        identity = self.ids[self.cursor]
        req = GetDataFromDxl.Request()
        req.id = identity
        req.item_name = "Torque Enable"
        # [CHANGED] Upstream driver spins callbacks from hardware read(); its
        # service waits on a read-buffer operation performed on the NEXT cycle.
        # A 10ms upper bound avoids the previous 500ms hardware-loop stall.
        req.timeout_sec = 0.01
        try:
            future = self.client.call_async(req)
        except Exception as exc:
            self.good_at.pop(identity, None)
            self.reason = f"DXL {identity} request failed: {exc}"
            return
        self.future = future
        self.pending_id = identity
        self.pending_phase = self.phase
        self.pending_at = now
        self.last_request = now

    def warning_once(self):
        if not self._warned:
            self.node.get_logger().warn(
                "[TORQUE TELEMETRY] Upstream /dxl_state reports False, "
                "but all 14 physical Torque Enable registers returned ON. "
                "Fresh DXL bus, joint, collision and reference guards remain."
            )
            self._warned = True
