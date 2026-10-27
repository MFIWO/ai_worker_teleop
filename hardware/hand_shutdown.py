"""SH5 terminal-exit hand torque release. Never enables torque or sends positions."""
import argparse
import os
from pathlib import Path
import xml.etree.ElementTree as ET


def configuration(xml):
    root=ET.fromstring(xml)
    if root.get('name')!='ffw_sh5_follower':raise ValueError('SH5 physical description required')
    control=next(c for c in root.findall('ros2_control') if c.get('name')=='ffw_sh5_follower')
    params={p.get('name'):p.text for p in control.findall('hardware/param')}
    if control.findtext('hardware/plugin')!='dynamixel_hardware_interface/DynamixelHardware':
        raise ValueError('unexpected SH5 hardware driver')
    result={'port':params['port_name'],'baud':int(params['baud_rate']),'boards':{}}
    for side in ('l','r'):
        board=control.find(f"gpio[@name='hand_{side}_controller']")
        if board is None:raise ValueError('missing hand board')
        values={p.get('name'):p.text for p in board.findall('param')}
        ident=int(values['ID'])
        motors=[int(values[f'SyncTable {finger} ID {joint}']) for finger in range(1,6) for joint in range(1,5)]
        virtual={int(g.findtext("param[@name='ID']")) for g in control.findall('gpio')
                 if g.findtext("param[@name='type']")=='virtual_dxl'
                 and g.findtext("param[@name='comm_id']")==str(ident)}
        if len(set(motors))!=20 or set(motors)!=virtual:raise ValueError('hand motor mapping mismatch')
        result['boards'][str(ident)]=motors
    all_ids=[int(n) for n in result['boards']]+[n for ids in result['boards'].values() for n in ids]
    if len(set(all_ids))!=42 or any(not 0<n<253 for n in all_ids):raise ValueError('invalid hand IDs')
    return result


def assert_idle(port):
    device=os.stat(port).st_rdev
    if not device:raise RuntimeError('not a serial device')
    owners=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
        try:
            for fd in (proc/'fd').iterdir():
                try:
                    stat=fd.stat()
                    if stat.st_rdev==device:owners.append(proc.name)
                except FileNotFoundError:pass
        except FileNotFoundError:pass
        except PermissionError:raise RuntimeError('cannot establish idle serial ownership')
    if owners:raise RuntimeError('follower port still owned by PIDs '+','.join(sorted(set(owners))))


def release(config):
    # No second connection while ros2_control or another driver owns the port.
    assert_idle(config['port'])
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'vendor'))
    from dynamixel_sdk import PortHandler,PacketHandler,COMM_SUCCESS
    port=PortHandler(config['port']);packet=PacketHandler(2.)
    if not port.openPort():raise RuntimeError('cannot open idle follower port')
    failures=[]
    try:
        if not port.setBaudRate(config['baud']):raise RuntimeError('baud rate setup failed')
        def read(ident,address):
            value,comm,error=packet.read1ByteTxRx(port,ident,address)
            # Alert bit is telemetry (e.g. latched board watchdog), not a
            # packet execution error. Actual torque is checked below.
            if comm!=COMM_SUCCESS or error&0x7f:raise RuntimeError(f'ID{ident} read {address}: comm={comm} error={error}')
            return value
        def write(ident,address,value):
            comm,error=packet.write1ByteTxRx(port,ident,address,value)
            if comm!=COMM_SUCCESS or error&0x7f:raise RuntimeError(f'ID{ident} write {address}: comm={comm} error={error}')
        for board,motors in config['boards'].items():
            try:
                # Torque OFF must work in either Operating Mode(33), including
                # Preset(1). Startup separately verifies Direct(0) for teleop.
                # Disable TableSync and verify every motor's Torque Enable.
                write(int(board),70,0)
                if read(int(board),70)!=0:raise RuntimeError('TableSync disable not verified')
                print(f'[HAND TORQUE OFF] Board {board}: TableSync OFF verified',flush=True)
            except Exception as error:
                failures.append(str(error));continue
            for ident in motors:
                try:
                    # XM335 Torque Enable(64), value 0 only. No broadcast.
                    write(ident,64,0)
                    if read(ident,64)!=0:raise RuntimeError(f'ID{ident} torque remained ON')
                    print(f'[HAND TORQUE OFF] ID{ident}: verified 0',flush=True)
                except Exception as error:failures.append(str(error))
    finally:port.closePort()
    if failures:raise RuntimeError('hand release incomplete: '+'; '.join(failures))
    print('[HAND TORQUE OFF] Verified 40/40 finger motors',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--description',type=Path,required=True)
    a=p.parse_args()
    release(configuration(a.description.read_text()))


if __name__=='__main__':main()
