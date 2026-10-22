"""[ADDED 2026-10-14] No ROS graph or hardware needed for this regression."""
import unittest

from hardware.torque_evidence import TorqueEvidence


class NoClientsAllowed:
    def create_client(self, *args, **kwargs):
        raise AssertionError('Read service must not be called by this fallback')


class DisabledEvidenceTests(unittest.TestCase):
    def test_does_not_create_client_or_poll(self):
        proof = TorqueEvidence(NoClientsAllowed())
        for _ in range(1000):
            proof.tick()
            self.assertFalse(proof.verified())

    def test_cannot_claim_hardware_evidence(self):
        proof = TorqueEvidence(NoClientsAllowed())
        self.assertIn('UNVERIFIED', proof.reason)
        with self.assertRaises(RuntimeError):
            proof.warning_once()


if __name__ == '__main__':
    unittest.main()
