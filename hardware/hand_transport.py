"""Verify HX5 direct-control mode on an idle, torque-OFF hand bus.

Only board Operating Mode(33) may be written, from Preset(1) to Direct(0).
Never enables torque, starts TableSync, changes a motor profile or sends goals.
"""
import argparse
import json
from pathlib import Path
import sys
import time

from hardware.hand_shutdown import assert_idle,configuration

CONTROL_TABLE_URL='https://docs.robotis.com/docs/systems/hx5_d20/specifications/control_table/'


def prepare(config, *, report=None, check_only=False):
    assert_idle(config['port'])
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'vendor'))
    from dynamixel_sdk import PortHandler,PacketHandler,COMM_SUCCESS
    port=PortHandler(config['port']);packet=PacketHandler(2.)
    state=dict(timestamp_ns=time.time_ns(),control_table=CONTROL_TABLE_URL,
               port=config['port'],check_only=check_only,boards={},verified=False)
    def save():
        if report is not None:
            path=Path(report);path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(json.dumps(state,indent=2)+'\n')
    if not port.openPort():raise RuntimeError('cannot open idle hand transport')
    try:
        if not port.setBaudRate(config['baud']):raise RuntimeError('baud setup failed')
        def read(ident,address,size=1):
            fn=packet.read1ByteTxRx if size==1 else packet.read2ByteTxRx
            value,comm,error=fn(port,ident,address)
            if comm!=COMM_SUCCESS or error&0x7f:
                raise RuntimeError(f'ID{ident} read {address}: comm={comm} error={error}')
            return value
        # Complete all preconditions before the first write to either hand.
        for board,motors in config['boards'].items():
            ident=int(board)
            data=dict(model=read(ident,0,2),firmware=read(ident,6),
                      mode_before=read(ident,33),table_sync=read(ident,70),
                      torques={str(m):read(m,64) for m in motors})
            state['boards'][board]=data
            if data['model'] not in (260,261):raise RuntimeError(f'ID{board}: unexpected HX5 board model')
            if data['mode_before'] not in (0,1):raise RuntimeError(f'ID{board}: unsupported Operating Mode')
            if data['table_sync']!=0:raise RuntimeError(f'ID{board}: stop follower/TableSync before transport setup')
            if any(v!=0 for v in data['torques'].values()):
                raise RuntimeError(f'ID{board}: verify all finger torques OFF before transport setup')
        save()
        for board,data in state['boards'].items():
            ident=int(board)
            if data['mode_before']==1 and not check_only:
                comm,error=packet.write1ByteTxRx(port,ident,33,0)
                if comm!=COMM_SUCCESS or error&0x7f:
                    raise RuntimeError(f'ID{board}: Direct Control write failed: comm={comm} error={error}')
                # Firmware 10 briefly stops replying while applying this
                # EEPROM mode change. Retry reads only, never repeat the write.
                time.sleep(.05)
                deadline=time.monotonic()+1.
                while True:
                    try:
                        data['mode_after']=read(ident,33)
                        break
                    except RuntimeError:
                        if time.monotonic()>=deadline:raise
                        time.sleep(.025)
            else:data['mode_after']=read(ident,33)
            print(f'[HAND TRANSPORT] Board {board}: Operating Mode {data["mode_before"]} -> {data["mode_after"]}; all finger torques OFF',flush=True)
            save()
            if data['mode_after']!=0:raise RuntimeError(f'ID{board}: Direct Control Mode(33)=0 required')
        for board,data in state['boards'].items():
            data['torques_after']={str(m):read(m,64) for m in config['boards'][board]}
            if read(int(board),70)!=0 or any(v!=0 for v in data['torques_after'].values()):
                raise RuntimeError(f'ID{board}: torque/TableSync changed unexpectedly during setup')
        state['verified']=True;save()
    except Exception as error:
        state['error']=str(error);save();raise
    finally:port.closePort()
    return state


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description',type=Path,required=True)
    parser.add_argument('--report',type=Path)
    parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args()
    prepare(configuration(args.description.read_text()),report=args.report,check_only=args.check_only)


if __name__=='__main__':main()
