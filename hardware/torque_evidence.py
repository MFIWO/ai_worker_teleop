"""[FIX 2026-10-14] Disable unsound buffered-service torque evidence.

The installed ROBOTIS GetDataFromDxl callback has no sample timestamp or request
identity. It returns the oldest matching queued item, not necessarily the item
requested by this client. An offline reproduction with the installed C++ source
showed old ON values authorising motion after the real register changed OFF.
Therefore this compatibility object never supplies motor-authorisation evidence
and never calls that service. Fresh native driver telemetry remains required.
"""


class TorqueEvidence:
    """Fail-closed compatibility API; no ROS client and no hardware traffic."""

    def __init__(self, node, max_age=6.0):
        self.node = node
        self.max_age = float(max_age)
        self.reason = (
            'Torque state is UNVERIFIED: installed driver telemetry may be '
            'incorrect; buffered GetDataFromDxl responses cannot prove freshness. '
            'No register polling or automatic torque enabling is performed.'
        )

    def verified(self):
        return False

    def tick(self):
        return None

    def warning_once(self):
        # [FIX] Never print a false claim that registers have been verified.
        raise RuntimeError('Buffered torque evidence is disabled and cannot authorise motion')
