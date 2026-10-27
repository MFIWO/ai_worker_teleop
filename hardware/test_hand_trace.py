"""Timing instrumentation tests; never connect to robot, ROS graph or serial."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from hardware import build_hand_trace as trace
from hardware import hand_trace_capture as capture
from hardware.teleop_cli import parser,command

class TraceDeploymentTests(unittest.TestCase):
    def test_unknown_source_rejected(self):
        with self.assertRaises(ValueError):trace.patch_source('unknown')
        with self.assertRaises(ValueError):trace.patch_hardware('unknown')

    def test_explicit_child_environment_keeps_default_environment_intact(self):
        with tempfile.TemporaryDirectory() as d, patch.object(trace,'verify'):
            original={'LD_LIBRARY_PATH':'/sdk','UNRELATED':'keep'}
            env=trace.follower_environment(Path(d)/'capture',root=Path('/trace'),base_env=original)
        self.assertEqual(env[trace.ENV],d+'/capture')
        self.assertEqual(env['AI_WORKER_HX5_WRITE_HZ'],'30')
        self.assertEqual(original,{'LD_LIBRARY_PATH':'/sdk','UNRELATED':'keep'})
        self.assertTrue(env['LD_LIBRARY_PATH'].endswith(':/sdk'))

    def test_unverified_overlay_cannot_be_selected(self):
        with patch.object(trace,'verify',side_effect=ValueError('changed source')):
            with self.assertRaises(ValueError):trace.follower_environment('/unused')

    def test_cli_passes_trace_only_when_requested(self):
        prefix=['robot','--pc-ip','192.0.2.1','--robot-ip','robot']
        ordinary=command(parser().parse_args(prefix),False)[-1]
        diagnostic=command(parser().parse_args(prefix+['--hand-trace']),False)[-1]
        self.assertNotIn('--hand-trace',ordinary)
        self.assertIn('--hand-trace',diagnostic)

    def test_capture_starts_only_a_file_request_and_collects_after_finalization(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)/'driver_trace';root.mkdir()
            dest=Path(d)/'log';dest.mkdir();manifest=dest/'manifest.json';manifest.write_text('{}')
            (root/'trace_1234.bin').write_bytes(b'fake trace')
            with patch.object(trace,'verify'),patch.object(capture,'running_trace',return_value=(7,root)),\
                    patch.object(capture.time,'time_ns',return_value=1234):
                capture.prepare(manifest,120)
            self.assertEqual((root/'capture.request').read_text(),'1234 125\n')
            (root/'trace_1234.json').write_text(json.dumps({'written':1,'dropped':0,'io_ok':True}))
            (root/'sources.tsv').write_text('1\t/dev/fake\n')
            capture.finish(manifest)
            self.assertFalse((root/'capture.request').exists())
            self.assertEqual((dest/'driver_trace/trace_1234.bin').read_bytes(),b'fake trace')

    def test_capture_duration_rejected_before_any_process_or_file_access(self):
        with patch.object(trace,'verify'),patch.object(capture,'running_trace') as running:
            with self.assertRaises(ValueError):capture.prepare(Path('/nonexistent'),181)
        running.assert_not_called()


class TraceCodecTests(unittest.TestCase):
    def test_crc_and_split_status_frame(self):
        from hardware.analyse_hand_trace import crc16,Frames,RECORD
        self.assertEqual(RECORD.size,104)
        ping=bytes.fromhex('fffffd0001030001')
        self.assertEqual(crc16(ping),0x4e19)
        body=bytes.fromhex('fffffd006e060055000102')
        frame=body+crc16(body).to_bytes(2,'little')
        parser=Frames()
        self.assertEqual(parser.feed(frame[:5]),[])
        self.assertEqual(parser.feed(frame[5:]),[frame])
        self.assertEqual(parser.frame_end_offsets,[len(frame)])
        broken=bytearray(frame);broken[-1]^=1
        self.assertEqual(parser.feed(broken),[])
        self.assertEqual(parser.bad_crc,1)

    def test_resynchronized_frame_retains_its_original_stream_offset(self):
        from hardware.analyse_hand_trace import crc16,Frames
        body=bytes.fromhex('fffffd006e060055000102')
        frame=body+crc16(body).to_bytes(2,'little')
        # An invalid long header delays parsing; the valid older frame must
        # retain the byte offset where it actually ended, not the later feed.
        prefix=bytes.fromhex('fffffd00016400')
        parser=Frames()
        self.assertEqual(parser.feed(prefix+frame),[])
        self.assertEqual(parser.feed(bytes(120)),[frame])
        self.assertEqual(parser.frame_end_offsets,[len(prefix)+len(frame)])

    def test_port_wrapper_forwards_one_call_and_preserves_return(self):
        import shutil,subprocess
        compiler=shutil.which('g++')
        if not compiler:self.skipTest('g++ unavailable')
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);sdk=root/'dynamixel_sdk';sdk.mkdir()
            (sdk/'port_handler_linux.h').write_text('''#include <stdint.h>
namespace dynamixel {class PortHandlerLinux {public:
static int reads,writes,read_result,write_result;
explicit PortHandlerLinux(const char *){} virtual ~PortHandlerLinux(){}
virtual int readPort(uint8_t * p,int n){++reads;if(read_result>0)for(int i=0;i<read_result&&i<n;++i)p[i]=42;return read_result;}
virtual int writePort(uint8_t *,int){++writes;return write_result;}
};}''')
            (root/'main.cpp').write_text('''#include "hx5_trace_port.hpp"
#include <cassert>
int dynamixel::PortHandlerLinux::reads=0,dynamixel::PortHandlerLinux::writes=0;
int dynamixel::PortHandlerLinux::read_result=0,dynamixel::PortHandlerLinux::write_result=0;
int main(){HX5TracePort p("/fake",nullptr);uint8_t data[8]{};
for(int r:{-1,0,5}){dynamixel::PortHandlerLinux::read_result=r;assert(p.readPort(data,8)==r);}
for(int r:{-1,8}){dynamixel::PortHandlerLinux::write_result=r;assert(p.writePort(data,8)==r);}
assert(dynamixel::PortHandlerLinux::reads==3&&dynamixel::PortHandlerLinux::writes==2);
assert(data[0]==42);}
''')
            subprocess.run([compiler,'-std=c++14','-pthread','-I',str(root),'-I',str(Path(__file__).parent),str(root/'main.cpp'),'-o',str(root/'test')],check=True,capture_output=True)
            subprocess.run([str(root/'test')],check=True)

class RecorderReadinessTests(unittest.TestCase):
    def test_failed_bag_start_never_arms_driver_capture(self):
        from hardware import hand_capture_runner as runner
        from unittest.mock import Mock
        import subprocess
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            with patch.object(runner.subprocess,'Popen',return_value=Mock()) as popen, \
                 patch.object(runner,'wait_ready',side_effect=RuntimeError('not ready')), \
                 patch.object(runner,'stop_recorder') as stop,patch.object(runner,'prepare') as prepare, \
                 patch.object(runner,'finish') as finish:
                with self.assertRaisesRegex(RuntimeError,'not ready'):runner.capture(root,10,['/rosout'],True)
            prepare.assert_not_called();finish.assert_not_called();stop.assert_called_once()
            self.assertEqual(popen.call_args.kwargs['stdin'],subprocess.DEVNULL)
            self.assertTrue(popen.call_args.kwargs['start_new_session'])

    def test_missing_ready_message_times_out(self):
        from hardware import hand_capture_runner as runner
        from unittest.mock import Mock
        with tempfile.TemporaryDirectory() as d:
            log=Path(d)/'recorder.log';log.write_text('')
            process=Mock();process.poll.return_value=None
            with patch.object(runner.time,'monotonic',side_effect=[0,21]):
                with self.assertRaisesRegex(RuntimeError,'did not become ready'):runner.wait_ready(process,log)

if __name__=='__main__':unittest.main()
