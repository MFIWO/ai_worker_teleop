"""[ADDED] Read-only proof of arm Torque Enable from real Dynamixel registers.

The upstream published torque_state can report False when its internal map
is never populated. Never assume a service SetBool acknowledgement is proof.
Asynchronously read all 14 motor registers via the existing official service.
No torque write or joint command is performed by this module.
"""
import time

from dynamixel_interfaces.srv import GetDataFromDxl
from hardware.xr_contract import ARM_DXL_IDS


class TorqueEvidence:
    """Asynchronous rolling confirmation of all arm Torque Enable registers."""

    def __init__(self, node, max_age=4.0):
        self.node = node
        self.ids = tuple(ARM_DXL_IDS["right"] + ARM_DXL_IDS["left"])
        self.client = node.create_client(GetDataFromDxl, "/ffw_follower/get_dxl_data")
        self.max_age = float(max_age)
        self.good_at = {}
        self.cursor = 0
        self.future = None
        self.pending_id = None
        self.pending_at = 0.0
        self.last_request = 0.0
        self.reason = "waiting for real Dynamixel Torque Enable register reads"
        self._warned = False

    def verified(self):
        now = time.monotonic()
        return all(now - self.good_at.get(i, -1e12) <= self.max_age for i in self.ids)

    def tick(self):
        """Nonblocking: call from the owning ROS node's timer / event loop."""
        now = time.monotonic()
        if self.future is not None:
            if self.future.done():
                try:
                    response = self.future.result()
                    if response is not None and bool(response.result) and int(response.item_data) == 1:
                        self.good_at[self.pending_id] = now
                    else:
                        self.good_at.pop(self.pending_id, None)
                        self.reason = f"DXL {self.pending_id} Torque Enable is OFF/unreadable"
                except Exception as exc:
                    self.good_at.pop(self.pending_id, None)
                    self.reason = f"DXL {self.pending_id} torque register read failed: {exc}"
                self.future = None
                self.pending_id = None
            elif now - self.pending_at > 1.2:
                # Do not retain a stale proof after a nonresponsive service.
                self.good_at.pop(self.pending_id, None)
                self.reason = f"DXL {self.pending_id} torque register read timed out"
                self.future = None
                self.pending_id = None
            else:
                return
        if now - self.last_request < 0.05:
            return
        if not self.client.service_is_ready():
            self.reason = "waiting for /ffw_follower/get_dxl_data"
            return
        identity = self.ids[self.cursor]
        self.cursor = (self.cursor + 1) % len(self.ids)
        req = GetDataFromDxl.Request()
        req.id = identity
        req.item_name = "Torque Enable"
        req.timeout_sec = 0.5
        try:
            self.future = self.client.call_async(req)
        except Exception as exc:
            self.good_at.pop(identity, None)
            self.reason = f"DXL {identity} torque service call failed: {exc}"
            return
        self.pending_id = identity
        self.pending_at = now
        self.last_request = now

    def warning_once(self):
        if not self._warned:
            self.node.get_logger().warn(
                "[TORQUE TELEMETRY] /dxl_state reports torque False but actual "
                "Torque Enable register reads confirm all 14 arm motors ON. "
                "Retaining live hardware-read, joint, reference and collision guards."
            )
            self._warned = True
