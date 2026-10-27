"""HX5 transport setup regressions with fake devices; never uses serial I/O."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

from hardware.hand_transport import prepare


class HandTransportTests(unittest.TestCase):
    def setup_transport(self,mode=1):
        config=dict(port='/fake/follower',baud=4000000,
                    boards={'110':list(range(111,131)),'140':list(range(141,161))})
        registers={}
        for board,motors in config['boards'].items():
            ident=int(board)
            registers.update({(ident,0):260 if ident==110 else 261,
                              (ident,6):10,(ident,33):mode,(ident,70):0})
            registers.update({(m,64):0 for m in motors})
        writes=[]
        def read(port,ident,addr):return registers[ident,addr],0,128 if ident in (110,140) else 0
        def write(port,ident,addr,value):
            self.assertIn(ident,(110,140));self.assertEqual((addr,value),(33,0))
            registers[ident,addr]=value;writes.append((ident,addr,value));return 0,128
        port=Mock();port.openPort.return_value=port.setBaudRate.return_value=True
        packet=SimpleNamespace(read1ByteTxRx=read,read2ByteTxRx=read,write1ByteTxRx=write)
        sdk=SimpleNamespace(PortHandler=lambda _:port,PacketHandler=lambda _:packet,COMM_SUCCESS=0)
        return config,registers,writes,port,sdk

    def test_preset_to_direct_changes_only_board_mode_and_records_original_values(self):
        config,regs,writes,port,sdk=self.setup_transport()
        with tempfile.TemporaryDirectory() as folder,patch.dict(sys.modules,dynamixel_sdk=sdk),\
             patch('hardware.hand_transport.assert_idle'),redirect_stdout(io.StringIO()):
            report=Path(folder)/'transport.json'
            prepare(config,report=report)
            result=json.loads(report.read_text())
            self.assertTrue(result['verified'])
            self.assertTrue(all(d['mode_before']==1 and d['mode_after']==0 for d in result['boards'].values()))
        self.assertEqual(writes,[(110,33,0),(140,33,0)])
        self.assertTrue(all(value==0 for (ident,addr),value in regs.items() if addr==64))
        port.closePort.assert_called_once()

    def test_already_direct_is_idempotent_with_no_eeprom_writes(self):
        config,_,writes,_,sdk=self.setup_transport(mode=0)
        with patch.dict(sys.modules,dynamixel_sdk=sdk),patch('hardware.hand_transport.assert_idle'),redirect_stdout(io.StringIO()):
            self.assertTrue(prepare(config)['verified'])
        self.assertFalse(writes)

    def test_temporary_post_write_timeout_retries_read_without_rewriting_eeprom(self):
        config,_,writes,_,sdk=self.setup_transport()
        packet=sdk.PacketHandler(2.)
        original=packet.read1ByteTxRx
        failed=set()
        def read(port,ident,addr):
            if addr==33 and any(w[0]==ident for w in writes) and ident not in failed:
                failed.add(ident);return 0,-3001,0
            return original(port,ident,addr)
        packet.read1ByteTxRx=read
        with patch.dict(sys.modules,dynamixel_sdk=sdk),patch('hardware.hand_transport.assert_idle'),\
             patch('hardware.hand_transport.time.sleep'),redirect_stdout(io.StringIO()):
            self.assertTrue(prepare(config)['verified'])
        self.assertEqual(writes,[(110,33,0),(140,33,0)])
        self.assertEqual(failed,{110,140})

    def test_bad_preconditions_on_either_hand_prevent_all_writes(self):
        for entry,value in (((141,64),1),((140,70),1),((140,0),999),((140,33),7)):
            with self.subTest(entry=entry):
                config,regs,writes,port,sdk=self.setup_transport();regs[entry]=value
                with patch.dict(sys.modules,dynamixel_sdk=sdk),patch('hardware.hand_transport.assert_idle'):
                    with self.assertRaises(RuntimeError):prepare(config)
                self.assertFalse(writes);port.closePort.assert_called_once()

    def test_busy_port_is_rejected_before_opening_sdk(self):
        config,_,_,port,sdk=self.setup_transport()
        with patch.dict(sys.modules,dynamixel_sdk=sdk),\
             patch('hardware.hand_transport.assert_idle',side_effect=RuntimeError('busy')):
            with self.assertRaises(RuntimeError):prepare(config)
        port.openPort.assert_not_called()

    def test_check_only_detects_wrong_mode_without_writes(self):
        config,_,writes,_,sdk=self.setup_transport()
        with patch.dict(sys.modules,dynamixel_sdk=sdk),patch('hardware.hand_transport.assert_idle'),redirect_stdout(io.StringIO()):
            with self.assertRaises(RuntimeError):prepare(config,check_only=True)
        self.assertFalse(writes)


if __name__=='__main__':unittest.main()
