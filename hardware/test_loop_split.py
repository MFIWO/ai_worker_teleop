"""Verify robot/hand source separation without ROS, network, or motors."""
import sys
import types
import unittest
from unittest.mock import patch

from hardware.loop_clock import PCRecordingClock, SplitRobotStreamer


class SplitSourceTests(unittest.TestCase):
    def test_installed_sdk_schemas_include_pressure_in_hand(self):
        try:
            import loop_sdk
        except ImportError:
            self.skipTest('run in robot container with loop-sdk installed')
        schemas = []
        class Producer:
            def stats(self):return types.SimpleNamespace(connected=True)
            def send_robot(self, *args):pass
            def send_tactile(self, stamp, seq, channels):
                self.frame = loop_sdk.TactileFrame(timestamp_us=stamp, sequence=seq, channels=channels)
            def close(self):pass
        def connect(addr, schema, **kwargs):
            schemas.append(schema)
            return Producer()
        infos = [types.SimpleNamespace(key='ffw_sh5_vr.observation.'+part+'.joint_position',
                 label=part, unit='rad', group=part) for part in ('left_arm', 'left_hand')]
        infos.append(types.SimpleNamespace(key='ffw_sh5_vr.observation.left_hand.tactile_pressure',
            label='pressure', unit='raw', group='left_hand'))
        sender = SplitRobotStreamer('unused', types.SimpleNamespace(channel_infos=lambda:infos), PCRecordingClock())
        with patch.object(loop_sdk.SourceProducer, 'connect', side_effect=connect):
            sender.connect()
        self.assertEqual([schema.kind.value for schema in schemas], ['robot', 'robot'])
        self.assertEqual(schemas[1].robot.name, 'HX5-D20')
        self.assertIn('ffw_sh5_vr.observation.left_hand.tactile_pressure',
                      [c.key for c in schemas[1].robot.channels])
        sender.close()

    def test_synchronized_sources_and_no_cross_component_pressure(self):
        clock = PCRecordingClock()
        clock.receive(1_000_000_000, 10.)
        sender = SplitRobotStreamer('unused', None, clock)
        class Producer:
            def __init__(self):self.frames = []
            def send_robot(self, *args):self.frames.append(args)
            def send_tactile(self, *args):self.frames.append(args)
        for stream in sender.streams:stream.producer = Producer()
        robot, hand = sender.streams
        prefix = 'ffw_sh5_vr.'
        payload = {
            prefix+'observation.left_arm.joint_position': [1.]*7,
            prefix+'action.mobile.velocity': [0.]*3,
            prefix+'observation.left_hand.joint_position': [2.]*20,
            prefix+'observation.left_hand.tactile_pressure': list(range(45)),
            prefix+'observation.right_hand.tactile_pressure': None,
            prefix+'teleop.flags': [0., 1., 0., 0., 1.],
        }
        fake_sdk = types.SimpleNamespace(TactileChannelValue=lambda **kw:types.SimpleNamespace(**kw))
        with patch.dict(sys.modules, loop_sdk=fake_sdk), patch('hardware.loop_clock.time.monotonic', return_value=10.2):
            self.assertTrue(sender.send(payload))
        frames = [s.producer.frames[0] for s in sender.streams]
        self.assertEqual(len({(f[0],f[1]) for f in frames}), 1)
        self.assertEqual(frames[0][1], 0)
        self.assertIn(prefix+'action.mobile.velocity', frames[0][2])
        self.assertFalse(any('.left_hand.' in k for k in frames[0][2]))
        self.assertIn(prefix+'observation.left_hand.joint_position', frames[1][2])
        self.assertFalse(any('.mobile.' in k or '.left_arm.' in k for k in frames[1][2]))
        self.assertEqual(len(frames), 2)
        self.assertFalse(any(k.endswith('.tactile_pressure') for k in frames[0][2]))
        self.assertEqual(frames[1][2][prefix+'observation.left_hand.tactile_pressure'], list(range(45)))
        self.assertIsNone(frames[1][2][prefix+'observation.right_hand.tactile_pressure'])
        self.assertEqual(frames[0][2][prefix+'teleop.flags'], frames[1][2][prefix+'teleop.flags'])

    def test_no_clock_no_samples_and_partial_connect_cleanup(self):
        sender = SplitRobotStreamer('unused', None, PCRecordingClock())
        self.assertFalse(sender.send({}))
        with patch.object(sender.streams[0], 'connect'), patch.object(sender.streams[1], 'connect', side_effect=ConnectionError), patch.object(sender, 'close') as close:
            with self.assertRaises(ConnectionError):sender.connect()
            close.assert_called_once()


if __name__ == '__main__':unittest.main()
