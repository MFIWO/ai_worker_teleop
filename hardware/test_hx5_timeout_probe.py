"""No-device status validation and timeout-unit regressions."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from hardware.hx5_timeout_probe import Audit, status_payload


def frame(ident, data, error=0):
    size = len(data)+4
    return [255, 255, 253, 0, ident, size & 255, size >> 8, 0x55, error]+data+[0, 0]


class TimeoutAuditTests(unittest.TestCase):
    def test_wrong_id_short_ack_length_and_device_error_are_rejected(self):
        valid = frame(111, [1, 2, 3, 4, 5, 6])
        self.assertEqual(status_payload(valid, 111, 6), [1, 2, 3, 4, 5, 6])
        for raw in (frame(112, [1]*6), frame(111, []), frame(111, [1]*5),
                    frame(111, [1]*6, 4), [0]*17):
            self.assertIsNone(status_payload(raw, 111, 6))
        self.assertEqual(status_payload(frame(111, [1]*6, 128), 111, 6), [1]*6)

    def test_millisecond_api_is_used_and_old_ack_cannot_satisfy_a_read(self):
        port = SimpleNamespace(setPacketTimeoutMillis=Mock(), setPacketTimeout=Mock())
        packet = SimpleNamespace(readTx=Mock(return_value=0), rxPacket=Mock(side_effect=[
            (frame(111, []), 0), (frame(111, [1, 2, 3, 4, 5, 6]), 0)]))
        result = Audit(port, packet).read(111, 224, 6, 600.)
        port.setPacketTimeoutMillis.assert_called_once_with(600.)
        port.setPacketTimeout.assert_not_called()
        self.assertTrue(result['ok'])
        self.assertEqual([f['accepted'] for f in result['frames']], [False, True])
        packet.readTx.assert_called_once_with(port, 111, 224, 6)

    def test_timeout_does_not_resend_the_query_or_turn_into_success(self):
        port = SimpleNamespace(setPacketTimeoutMillis=Mock())
        packet = SimpleNamespace(readTx=Mock(return_value=0), rxPacket=Mock(return_value=([], -3001)))
        result = Audit(port, packet).read(111, 224, 6)
        self.assertFalse(result['ok'])
        self.assertEqual(result['comm'], -3001)
        packet.readTx.assert_called_once()


if __name__ == '__main__':unittest.main()
