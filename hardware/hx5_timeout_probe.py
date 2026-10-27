"""READ-only HX5 timeout audit. No torque, goal, LED or configuration writes.

Compare a single finger read after arm traffic, collecting late and wrong-size
status packets instead of treating an increased timeout as a latency fix.
Requires an idle follower port, TableSync OFF and all forty finger torques OFF.
"""
import argparse
import json
from pathlib import Path
import sys
import time

from hardware.hand_shutdown import assert_idle, configuration

EXTERNAL_READS = [(i, 698, 13 if i == 1 else 11)
                  for i in list(range(1, 8))+list(range(31, 38))+[81, 101, 103, 105]]
EXTERNAL_READS += [(61, 634, 13), (62, 634, 11)]


def status_payload(frame, ident, size):
    """Validate a CRC-checked, unstuffed SDK status frame's identity and size."""
    if len(frame) < 11 or frame[:4] != [255, 255, 253, 0] or frame[7] != 0x55:
        return None
    if frame[4] != ident or len(frame) != size+11 or frame[5]+256*frame[6] != size+4:
        return None
    if frame[8] & 0x7f:
        return None
    return frame[9:-2]


class Audit:
    def __init__(self, port, packet):
        self.port, self.packet = port, packet

    def read(self, ident, address, size, timeout_ms=600.):
        started = time.monotonic()
        comm = self.packet.readTx(self.port, ident, address, size)
        if comm:
            raise RuntimeError(f'READ TX ID{ident}: {comm}')
        # Python SDK has separate methods; setPacketTimeout takes BYTES.
        self.port.setPacketTimeoutMillis(float(timeout_ms))
        frames = []
        while True:
            raw, comm = self.packet.rxPacket(self.port, False)
            now = time.monotonic()
            if comm:
                return dict(ok=False, comm=comm, elapsed_ms=(now-started)*1000, frames=frames)
            data = status_payload(raw, ident, size)
            frames.append(dict(id=raw[4], size=len(raw)-11, error=raw[8],
                               ms=(now-started)*1000, accepted=data is not None, hex=bytes(raw).hex()))
            if data is not None:
                return dict(ok=True, elapsed_ms=(now-started)*1000, data=data, frames=frames)

    def external_bulk(self, fast=False, entries=None):
        entries = EXTERNAL_READS if entries is None else entries
        param = []
        for ident, address, size in entries:
            param.extend([ident, address & 255, address >> 8, size, 0])
        started = time.monotonic()
        comm = self.packet.bulkReadTx(self.port, param, len(param), fast)
        if comm:
            raise RuntimeError(f'arm bulk TX: {comm}')
        self.port.setPacketTimeoutMillis(100.)
        pending = {i: size for i, _, size in entries}
        frames = []
        while pending:
            raw, comm = self.packet.rxPacket(self.port, fast)
            if comm:
                return dict(ok=False, comm=comm, elapsed_ms=(time.monotonic()-started)*1000, frames=frames)
            ident = raw[4]
            size = len(raw)-11
            valid = fast and ident == 254
            if not fast and ident in pending:
                valid = status_payload(raw, ident, pending[ident]) is not None
                if valid:
                    del pending[ident]
            frames.append(dict(id=ident, size=size, error=raw[8], ms=(time.monotonic()-started)*1000, accepted=valid))
            if fast:
                return dict(ok=valid, elapsed_ms=(time.monotonic()-started)*1000, frames=frames)
        return dict(ok=True, elapsed_ms=(time.monotonic()-started)*1000, frames=frames)

    def external_sync(self, entries, fast=False):
        address, size = entries[0][1:]
        if any((a, n) != (address, size) for _, a, n in entries):
            raise ValueError('Sync Read needs matching address and size')
        ids = [i for i, _, _ in entries]
        started = time.monotonic()
        comm = self.packet.syncReadTx(self.port, address, size, ids, len(ids), fast)
        if comm:
            raise RuntimeError(f'arm sync TX: {comm}')
        self.port.setPacketTimeoutMillis(100.)
        pending = set(ids)
        frames = []
        while pending:
            raw, comm = self.packet.rxPacket(self.port, fast)
            if comm:
                return dict(ok=False, comm=comm, elapsed_ms=(time.monotonic()-started)*1000, frames=frames)
            ident = raw[4]
            valid = fast and ident == 254
            if not fast and ident in pending:
                valid = status_payload(raw, ident, size) is not None
                if valid:pending.remove(ident)
            frames.append(dict(id=ident, size=len(raw)-11, ms=(time.monotonic()-started)*1000, accepted=valid))
            if fast:
                return dict(ok=valid, elapsed_ms=(time.monotonic()-started)*1000, frames=frames)
        return dict(ok=True, elapsed_ms=(time.monotonic()-started)*1000, frames=frames)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--payload-matrix', action='store_true', help='isolate position bytes without changing any register')
    parser.add_argument('--address-matrix', action='store_true', help='compare direct/indirect register ranges without changing any register')
    parser.add_argument('--direct-arm-matrix', action='store_true', help='compare direct telemetry and error status reads; no remapping writes')
    parser.add_argument('--protocol-matrix', action='store_true', help='isolate broadcast instruction and device count')
    args = parser.parse_args()
    if not 1 <= args.repeats <= 5:
        parser.error('repeats must be 1..5')
    config = configuration(args.description.read_text())
    assert_idle(config['port'])
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'vendor'))
    from dynamixel_sdk import PortHandler, PacketHandler
    port, packet = PortHandler(config['port']), PacketHandler(2.)
    report = dict(read_only=True, preconditions=False, cases=[])
    def save():
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2)+'\n')
    if not port.openPort():
        raise RuntimeError('idle serial port unavailable')
    try:
        if not port.setBaudRate(config['baud']):
            raise RuntimeError('baud setup failed')
        audit = Audit(port, packet)
        time.sleep(.6)
        for b, motors in config['boards'].items():
            board = int(b)
            for address, expected in [(33, 0), (70, 0)]:
                r = audit.read(board, address, 1)
                if not r['ok'] or r['data'] != [expected]:
                    raise RuntimeError(f'ID{board}: Direct mode/TableSync OFF required')
            for motor in motors:
                r = audit.read(motor, 64, 1)
                if not r['ok'] or r['data'] != [0]:
                    raise RuntimeError(f'ID{motor}: confirmed torque OFF required')
                time.sleep(.002)
        report['preconditions'] = True
        save()
        workloads = [('hands_only', None), ('arm1_model', (1, 0, 2)),
                    ('arm3_model', (3, 0, 2)), ('arm3_indirect', (3, 698, 11)),
                    ('arm_bulk', 'bulk'), ('arm_fast_bulk', 'fast')]
        if args.payload_matrix:
            workloads = [('hands_only', None)] + [(f'arm3_{a}_{n}', (3, a, n))
                for a, n in [(698, 1), (698, 2), (698, 3), (698, 4),
                              (700, 1), (700, 2), (701, 1), (702, 4), (706, 2), (708, 1)]]
        if args.address_matrix:
            workloads = [('hands_only', None)] + [(f'arm3_{a}_{n}', (3, a, n))
                for a, n in [(153, 1), (546, 10), (698, 10), (707, 2),
                              (708, 1), (709, 1), (710, 11), (720, 11), (730, 11)]]
        if args.direct_arm_matrix:
            trimmed = [(i, a, min(n, 10)) for i, a, n in EXTERNAL_READS]
            data = [(i, 574 if i in (7, 37) else 126 if i in (61, 62) else 546, 10)
                    for i, _, _ in EXTERNAL_READS]
            errors = [(i, 518 if i in (7, 37) else 70 if i in (61, 62) else 153, 1)
                      for i, _, _ in EXTERNAL_READS]
            workloads = [('hands_only', None), ('bulk_trimmed', [trimmed]),
                         ('bulk_direct_data', [data]), ('bulk_direct_data_status', [data, errors])]
        if args.protocol_matrix:
            all_models = [(i, 0, 2) for i, _, _ in EXTERNAL_READS]
            workloads = [('hands_only', None), ('unicast3_direct', (3, 546, 10)),
                ('bulk1_model', [[(1, 0, 2)]]), ('bulk3_model', [[(3, 0, 2)]]),
                ('bulk3_direct', [[(3, 546, 10)]]), ('bulk3_prefix10', [[(3, 698, 10)]]),
                ('bulk_all_models', [all_models]),
                ('sync3_direct', dict(sync=[(3, 546, 10)], fast=False)),
                ('fastsync3_direct', dict(sync=[(3, 546, 10)], fast=True)),
                ('sync_all_models', dict(sync=all_models, fast=False)),
                ('fastsync_all_models', dict(sync=all_models, fast=True))]
        for repeat in range(args.repeats):
            for label, stimulus in workloads:
                for motor in (111, 141):
                    time.sleep(.4)
                    port.clearPort()
                    if stimulus is None:
                        arm = dict(ok=True)
                    elif isinstance(stimulus, tuple):
                        arm = audit.read(*stimulus)
                    elif isinstance(stimulus, list):
                        parts = [audit.external_bulk(entries=entries) for entries in stimulus]
                        arm = dict(ok=all(p['ok'] for p in parts), parts=parts)
                    elif isinstance(stimulus, dict):
                        arm = audit.external_sync(stimulus['sync'], stimulus['fast'])
                    else:
                        arm = audit.external_bulk(stimulus == 'fast')
                    result = audit.read(motor, 224, 6) if arm['ok'] else None
                    row = dict(repeat=repeat, workload=label, motor=motor, arm=arm, hand=result)
                    report['cases'].append(row)
                    save()
                    print(label, motor, 'arm_ok', arm['ok'], 'hand',
                          None if result is None else {k: v for k, v in result.items() if k not in ('frames', 'data')}, flush=True)
        report['completed'] = True
    except BaseException as error:
        report['error'] = str(error)
        raise
    finally:
        port.closePort()
        save()


if __name__ == '__main__':main()
