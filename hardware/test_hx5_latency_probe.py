"""No-device diagnostic guard and restoration tests."""
import copy
import itertools
import unittest
from unittest.mock import patch

from hardware.hx5_latency_probe import probe


class ProbeTests(unittest.TestCase):
    def fixture(self):
        config = {'boards': {'110': list(range(111, 131)), '140': list(range(141, 161))}}
        values = {}
        for b in config['boards']:
            b = int(b)
            values.update({(b, 0, 2): 260, (b, 33, 1): 0, (b, 70, 1): 0, (b, 8, 1): 100})
            for f in range(5):
                values[b, 1030+198*f, 2] = 224
                for j in range(6):values[b, 1066+198*f+2*j, 2] = 4 if j < 4 else 0
        for motors in config['boards'].values():
            for m in motors:values[m, 64, 1] = 0
        writes = []
        def read(i, a, n):return values[i, a, n]
        def write(i, a, n, v):
            self.assertIn(str(i), config['boards'])
            self.assertNotIn(a, (10, 64, 108, 112, 116, 230, 1150))
            writes.append((i, a, n, v));values[i, a, n] = v
        return config, values, writes, read, write

    def test_default_is_read_only_and_torque_on_aborts_before_writes(self):
        config, values, writes, read, write = self.fixture()
        probe(config, read, write, lambda _: None)
        self.assertEqual(writes, [])
        values[111, 64, 1] = 1
        with self.assertRaisesRegex(RuntimeError, 'torques'):
            probe(config, read, write, lambda _: None, execute=True)
        self.assertEqual(writes, [])

    def test_success_restores_every_modified_field(self):
        config, values, writes, read, write = self.fixture()
        before = copy.deepcopy(values)
        with patch('hardware.hx5_latency_probe.time.monotonic', side_effect=itertools.count()):
            result = probe(config, read, write, lambda _: None, execute=True, duration=1.)
        self.assertTrue(result['restored'])
        self.assertEqual(values, before)
        for b in config['boards']:
            start = writes.index((int(b), 70, 1, 1))
            for f in range(5):
                for j in range(6):self.assertIn((int(b), 1066+198*f+2*j, 2, 0), writes[:start])

    def test_setup_failure_still_restores_both_boards(self):
        config, values, writes, read, write = self.fixture()
        before = copy.deepcopy(values)
        failed = False
        def fault(i, a, n, v):
            nonlocal failed
            if not failed and a == 1030 and v == 120:
                failed = True
                raise RuntimeError('injected setup failure')
            write(i, a, n, v)
        reports = []
        with self.assertRaisesRegex(RuntimeError, 'injected'):
            probe(config, read, fault, lambda d: reports.append(copy.deepcopy(d)), execute=True, duration=0.)
        self.assertEqual(values, before)
        self.assertTrue(reports[-1]['restored'])


if __name__ == '__main__':unittest.main()
