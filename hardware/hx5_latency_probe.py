"""Idle HX5 clock diagnostic. Default: read only; --execute: reversible board setup.

Never writes torque, motor goals or profiles. Requires all forty torques OFF
and an idle follower port. Before TableSync starts, all motor write sizes are
zeroed. Original board fields are persisted before mutation and restored in
finally. An internal motor clock separates board caching from ROS traffic.
"""
import argparse
import json
from pathlib import Path
import signal
import sys
import time

from hardware.hand_shutdown import assert_idle, configuration


def probe(config, read, write, save, execute=False, duration=2., traffic=()):
    state = dict(backups={}, phases={}, restored=False, torque_off=False)
    fields = [(8, 1), (70, 1)]
    fields += [(1030 + 198*f, 2) for f in range(5)]
    fields += [(1066 + 198*f + 2*j, 2) for f in range(5) for j in range(6)]
    for board, motors in config['boards'].items():
        ident = int(board)
        if (read(ident, 0, 2) not in (260, 261) or read(ident, 33, 1) != 0
                or read(ident, 70, 1) != 0):
            raise RuntimeError('Idle HX5 Direct mode with TableSync OFF required')
        if any(read(m, 64, 1) != 0 for m in motors):
            raise RuntimeError('All forty finger torques must be OFF')
        state['backups'][board] = [[a, n, read(ident, a, n)] for a, n in fields]
    state['torque_off'] = True
    save(state)
    if not execute:
        return state

    allowed = set(fields)
    def set_field(ident, address, size, value):
        if str(ident) not in config['boards'] or (address, size) not in allowed:
            raise ValueError('Only saved board configuration may be written')
        write(ident, address, size, value)
        if read(ident, address, size) != value:
            raise RuntimeError(f'ID{ident} address {address}: write not verified')

    try:
        for board in config['boards']:
            ident = int(board)
            for f in range(5):
                for j in range(6):
                    set_field(ident, 1066+198*f+2*j, 2, 0)
                set_field(ident, 1030+198*f, 2, 120)
            set_field(ident, 8, 1, 0)
            set_field(ident, 70, 1, 1)
        # ID3 is the physical arm motor that correlated with the old stalls.
        phases = [('hands_only', None), ('arm_model', (3, 0, 2)),
                  ('arm_tick', (3, 120, 2)), ('arm_position', (3, 132, 4)),
                  ('arm_indirect', (3, 698, 11)),
                  ('arm_indirect_short', (3, 698, 2))] + list(traffic)
        for label, extra in phases:
            rows = {b: [] for b in config['boards']}
            start = deadline = time.monotonic()
            while time.monotonic()-start < duration:
                if callable(extra):
                    extra()
                elif extra:
                    read(*extra)
                for board in rows:
                    rows[board].append([time.monotonic()-start, read(int(board), 1078, 2)])
                deadline += .01
                time.sleep(max(0., deadline-time.monotonic()))
            summary = {}
            for board, data in rows.items():
                changes = [r[0] for p, r in zip(data, data[1:]) if r[1] != p[1]]
                gaps = [1000*(b-a) for a, b in zip(changes, changes[1:])]
                summary[board] = dict(hz=len(changes)/duration, max_gap_ms=max(gaps, default=None))
            state['phases'][label] = dict(summary=summary, rows=rows)
            save(state)
            print(label, summary, flush=True)
    finally:
        failures = []
        for board, snapshot in state['backups'].items():
            ident = int(board)
            try:
                set_field(ident, 70, 1, 0)
            except Exception as error:
                failures.append(str(error))
                continue  # Never restore goal write mappings while TableSync is ON.
            for a, n, value in snapshot:
                if a == 70:
                    continue
                try:
                    set_field(ident, a, n, value)
                except Exception as error:
                    failures.append(str(error))
            for motor in config['boards'][board]:
                try:
                    if read(motor, 64, 1) != 0:
                        raise RuntimeError(f'ID{motor}: torque unexpectedly ON')
                except Exception as error:
                    failures.append(str(error))
        state['restore_errors'] = failures
        state['restored'] = not failures
        save(state)
        if failures:
            raise RuntimeError('Board restore incomplete: '+str(failures))
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--fast-arm-read', action='store_true', help='also compare Fast Sync/Bulk Read for external motors only')
    args = parser.parse_args()
    config = configuration(args.description.read_text())
    assert_idle(config['port'])
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'vendor'))
    from dynamixel_sdk import PortHandler, PacketHandler
    port, packet = PortHandler(config['port']), PacketHandler(2.)
    if not port.openPort():
        raise RuntimeError('Idle follower port unavailable')
    def read(ident, address, size):
        deadline = time.monotonic()+1.5
        while True:
            raw, comm, error = packet.readTxRx(port, ident, address, size)
            if comm == 0 and not error & 127:
                return int.from_bytes(bytes(raw), 'little')
            if time.monotonic() >= deadline:
                raise RuntimeError(f'ID{ident} read {address}: {comm}/{error}')
            time.sleep(.025)
    def write(ident, address, size, value):
        method = packet.write1ByteTxRx if size == 1 else packet.write2ByteTxRx
        method(port, ident, address, value)  # Readback resolves an uncertain ACK.
    def save(state):
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(state, indent=2)+'\n')
    def stop(*_):
        raise KeyboardInterrupt
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGTERM, signal.SIGHUP)}
    try:
        if not port.setBaudRate(config['baud']):
            raise RuntimeError('Baud setup failed')
        traffic = []
        if args.fast_arm_read:
            def fast_sync(ids):
                def action():
                    comm = packet.syncReadTx(port, 698, 11, ids, len(ids), True)
                    if comm == 0:
                        port.setPacketTimeout(10.)
                        _, comm = packet.rxPacket(port, True)
                    if comm != 0:
                        raise RuntimeError(f'Fast Sync Read {ids}: {comm}')
                return action
            traffic = [('fast_arm3', fast_sync([3])),
                       ('fast_right_arm', fast_sync(list(range(1, 8)))),
                       ('fast_both_arms', fast_sync(list(range(1, 8))+list(range(31, 38))))]
        state = probe(config, read, write, save, execute=args.execute, traffic=traffic)
        print('READ ONLY' if not args.execute else 'RESTORED '+str(state['restored']), flush=True)
    finally:
        port.closePort()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
