"""Shutdown verification with fake devices; never opens a real serial port."""
from contextlib import redirect_stdout
import io
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from hardware.hand_shutdown import release


class HandShutdownTests(unittest.TestCase):
    def run_release(self, *, stuck_board=None, failed_motor=None):
        config=dict(port='/fake/follower',baud=4000000,
                    boards={'140':list(range(141,161)),'110':list(range(111,131))})
        motors={i for ids in config['boards'].values() for i in ids}
        registers={(i,64):1 for i in motors}
        registers.update({(int(i),70):1 for i in config['boards']})
        writes=[]
        def read(port,ident,addr):
            self.assertIn((ident,addr),registers)  # address 33 must never be read
            return registers[ident,addr],0,128 if ident in (110,140) else 0
        def write(port,ident,addr,value):
            self.assertEqual(value,0)  # no torque ON, position or mode writes
            self.assertIn((ident,addr),registers)
            writes.append((ident,addr,value))
            if ident==failed_motor:return 0,1
            if ident!=stuck_board:registers[ident,addr]=value
            return 0,128 if ident in (110,140) else 0
        port=Mock()
        port.openPort.return_value=port.setBaudRate.return_value=True
        packet=SimpleNamespace(read1ByteTxRx=read,write1ByteTxRx=write)
        sdk=SimpleNamespace(PortHandler=lambda _:port,PacketHandler=lambda _:packet,COMM_SUCCESS=0)
        output=io.StringIO()
        with patch.dict(sys.modules,dynamixel_sdk=sdk),patch('hardware.hand_shutdown.assert_idle'),redirect_stdout(output):
            if stuck_board or failed_motor:
                with self.assertRaises(RuntimeError):release(config)
                self.assertNotIn('Verified 40/40',output.getvalue())
            else:
                release(config)
                self.assertTrue(all(registers[i,64]==0 for i in motors))
                self.assertIn('Verified 40/40',output.getvalue())
        port.closePort.assert_called_once()
        return writes

    def test_all_forty_torques_verified_without_undocumented_control_mode(self):
        writes=self.run_release()
        self.assertEqual(sum(addr==64 for _,addr,_ in writes),40)

    def test_table_sync_failure_blocks_only_affected_board_and_never_claims_success(self):
        writes=self.run_release(stuck_board=140)
        self.assertEqual(sum(addr==64 for _,addr,_ in writes),20)

    def test_motor_write_failure_is_reported_but_other_motors_are_attempted(self):
        writes=self.run_release(failed_motor=141)
        self.assertEqual(sum(addr==64 for _,addr,_ in writes),40)


if __name__=='__main__':unittest.main()
