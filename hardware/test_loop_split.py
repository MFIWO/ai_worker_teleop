"""Offline regression for one Robot State, optional typed tactile and channel filters."""
import sys
import types
import unittest
from unittest.mock import patch

from hardware.loop_channels import select_robot_channels
from hardware.loop_clock import PCRecordingClock, SplitRobotStreamer

PREFIX = 'ffw_sh5_vr.'


def layout():
    suffixes = [
        'observation.left_arm.joint_position',
        'observation.right_arm.joint_position',
        'observation.left_hand.joint_position',
        'observation.right_hand.joint_position',
        'observation.head.joint_position',
        'observation.lift.joint_position',
        'observation.mobile.velocity',
        'action.left_arm.joint_position',
        'action.right_arm.joint_position',
        'action.left_hand.joint_position',
        'action.right_hand.joint_position',
        'action.head.joint_position',
        'action.lift.joint_position',
        'action.mobile.velocity',
        'teleop.flags', 'teleop.phase', 'teleop.action_fresh',
        'teleop.action_valid', 'teleop.ages', 'teleop.clock',
        'teleop.controller_inputs', 'teleop.hand_pose_state',
        # Defensive test: raw tactile must be excluded even if mapper has it.
        'observation.left_hand.tactile_pressure',
    ]
    return [types.SimpleNamespace(key=PREFIX + key, label=key,
                                  unit='rad', group=key.split('.')[1])
            for key in suffixes]


class DummyMapper:
    robot_type = 'ffw_sh5_vr'

    def channel_infos(self):
        return layout()


class FakeSourceSchema:
    def __init__(self, *, robot=None, tactile=None):
        self.robot = robot
        self.tactile = tactile
        self.kind = types.SimpleNamespace(value='robot' if robot else 'tactile')
        self.source_key = robot.source_key if robot else tactile.source_key


class FakeProducer:
    schemas = []

    def __init__(self, schema):
        self.schema = schema
        self.frames = []
        self.closed = False

    @classmethod
    def connect(cls, _addr, schema, **_kwargs):
        cls.schemas.append(schema)
        return cls(schema)

    def stats(self):
        return types.SimpleNamespace(connected=True)

    def send_robot(self, *args):
        self.frames.append(args)

    def send_tactile(self, *args):
        self.frames.append(args)

    def close(self):
        self.closed = True


def fake_sdk():
    spec = lambda **kw: types.SimpleNamespace(**kw)
    return types.SimpleNamespace(
        SourceSchema=FakeSourceSchema, SourceProducer=FakeProducer,
        RobotStreamSchema=spec, RobotConfigOptions=spec, ChannelSpec=spec,
        TactileStreamSchema=spec, TactileChannelSpec=spec,
        TactileChannelValue=spec,
    )


class LoopSourceTests(unittest.TestCase):
    def setUp(self):
        FakeProducer.schemas = []

    def test_schema_selection_training_qpos_all_and_custom(self):
        infos = layout()
        training = select_robot_channels(infos, 'ffw_sh5_vr')
        self.assertEqual(len(training), 14)  # 6 obs + 6 action + 2 quality
        self.assertIn(PREFIX + 'observation.left_hand.joint_position', training)
        self.assertIn(PREFIX + 'action.right_hand.joint_position', training)
        self.assertIn(PREFIX + 'teleop.action_valid', training)
        self.assertNotIn(PREFIX + 'observation.mobile.velocity', training)
        self.assertFalse(any('.tactile_' in key for key in training))
        observed = select_robot_channels(infos, 'ffw_sh5_vr', 'qpos')
        self.assertEqual(len(observed), 6)
        self.assertTrue(all('.observation.' in key and key.endswith('.joint_position')
                            for key in observed))
        all_channels = select_robot_channels(infos, 'ffw_sh5_vr', 'all')
        self.assertEqual(len(all_channels), 22)
        self.assertFalse(any('.tactile_' in key for key in all_channels))
        selected = select_robot_channels(infos, 'ffw_sh5_vr',
            'observation.left_hand.joint_position,ffw_sh5_vr.action.right_arm.joint_position')
        self.assertEqual(selected, (
            PREFIX + 'observation.left_hand.joint_position',
            PREFIX + 'action.right_arm.joint_position',
        ))
        for bad in ('', 'missing', 'observation.left_hand.tactile_pressure',
                    'teleop.flags,,teleop.clock', 'teleop.flags,teleop.flags'):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                select_robot_channels(infos, 'ffw_sh5_vr', bad)

    def test_one_robot_schema_and_separate_typed_tactile(self):
        clock = PCRecordingClock()
        clock.receive(1_000_000_000, 10.)
        sender = SplitRobotStreamer('unused', DummyMapper(), clock)
        with patch.dict(sys.modules, loop_sdk=fake_sdk()):
            with patch('hardware.loop_clock.time.monotonic', return_value=10.2):
                sender.connect()
                self.assertEqual([s.kind.value for s in FakeProducer.schemas],
                                 ['robot', 'tactile'])
                robot_schema, tactile_schema = FakeProducer.schemas
                self.assertEqual(robot_schema.robot.name, 'aiworker')
                self.assertEqual(robot_schema.robot.source_key, 'robotis-vr-sh5')
                self.assertEqual(tactile_schema.tactile.name, 'HX5-D20')
                self.assertEqual(tactile_schema.tactile.source_key, 'robotis-vr-sh5-tactile')
                self.assertEqual(tactile_schema.tactile.channels[0].sample_count, 45)
                declared = {c.key for c in robot_schema.robot.channels}
                self.assertIn(PREFIX + 'observation.left_arm.joint_position', declared)
                self.assertIn(PREFIX + 'observation.left_hand.joint_position', declared)
                self.assertNotIn(PREFIX + 'observation.left_hand.tactile_pressure', declared)
                payload = {
                    PREFIX + 'observation.left_arm.joint_position': [1.]*7,
                    PREFIX + 'observation.left_hand.joint_position': [2.]*20,
                    PREFIX + 'action.right_hand.joint_position': [3.]*20,
                    PREFIX + 'observation.left_hand.tactile_pressure': list(range(45)),
                    PREFIX + 'observation.right_hand.tactile_pressure': None,
                    PREFIX + 'teleop.flags': [1., 0., 0., 0., 1.],
                    PREFIX + 'teleop.action_valid': [1.]*7,
                    PREFIX + 'teleop.phase': [5.],
                }
                self.assertTrue(sender.send(payload))
                robot_frame = sender.robot.producer.frames[0]
                tactile_frame = sender.tactile.producer.frames[0]
                self.assertEqual(robot_frame[:2], tactile_frame[:2])
                self.assertEqual(robot_frame[1], 0)
                self.assertEqual(set(robot_frame[2]), declared)
                self.assertEqual(robot_frame[2][PREFIX + 'observation.left_hand.joint_position'],
                                 [2.]*20)
                self.assertEqual(robot_frame[2][PREFIX + 'action.right_hand.joint_position'],
                                 [3.]*20)
                self.assertNotIn(PREFIX + 'teleop.phase', robot_frame[2])
                self.assertEqual(len(tactile_frame[2]), 1)
                self.assertEqual(tactile_frame[2][0].key, 'left_hand')
                self.assertEqual(tactile_frame[2][0].raw, bytes(range(45)))
                payload[PREFIX + 'observation.left_hand.tactile_pressure'] = None
                self.assertTrue(sender.send(payload))
                self.assertEqual(len(sender.robot.producer.frames), 2)
                self.assertEqual(len(sender.tactile.producer.frames), 1)
                self.assertEqual(sender.sent_count, 2)
        sender.close()
        self.assertTrue(all(s.producer is None for s in sender.streams))

    def test_qpos_only_and_no_tactile(self):
        clock = PCRecordingClock()
        clock.receive(1_000_000_000, 10.)
        sender = SplitRobotStreamer(
            'unused', DummyMapper(), clock, channels='qpos', enable_tactile=False)
        self.assertEqual(len(sender.streams), 1)
        with patch.dict(sys.modules, loop_sdk=fake_sdk()):
            with patch('hardware.loop_clock.time.monotonic', return_value=10.2):
                sender.connect()
                self.assertTrue(sender.send({
                    PREFIX + 'observation.left_hand.joint_position': [2.]*20,
                    PREFIX + 'teleop.flags': [1.]*5,
                }))
        self.assertEqual([s.kind.value for s in FakeProducer.schemas], ['robot'])
        self.assertEqual(len(sender.robot.producer.frames[0][2]), 6)
        self.assertNotIn(PREFIX + 'teleop.flags', sender.robot.producer.frames[0][2])
        sender.close()

    def test_no_pc_clock_and_partial_connect_cleanup(self):
        sender = SplitRobotStreamer('unused', DummyMapper(), PCRecordingClock())
        self.assertFalse(sender.send({}))
        with patch.object(sender.streams[0], 'connect'), \
             patch.object(sender.streams[1], 'connect', side_effect=ConnectionError), \
             patch.object(sender, 'close') as close:
            with self.assertRaises(ConnectionError):
                sender.connect()
            close.assert_called_once()

    def test_installed_sdk_schema_if_available(self):
        try:
            import loop_sdk
        except ImportError:
            self.skipTest('run against installed loop-sdk in robot container')
        # The real SDK should accept both source schema kinds and uint8 frames.
        clock = PCRecordingClock()
        sender = SplitRobotStreamer('unused', DummyMapper(), clock)
        schemas = []

        class Producer:
            def stats(self):return types.SimpleNamespace(connected=True)
            def send_robot(self, *_args):pass
            def send_tactile(self, stamp, seq, channels):
                self.frame = loop_sdk.TactileFrame(timestamp_us=stamp,
                                                   sequence=seq, channels=channels)
            def close(self):pass

        def connect(addr, schema, **kwargs):
            schemas.append(schema)
            return Producer()

        with patch.object(loop_sdk.SourceProducer, 'connect', side_effect=connect):
            sender.connect()
        self.assertEqual([schema.kind.value for schema in schemas],
                         ['robot', 'tactile'])
        sender.tactile.send({
            PREFIX + 'observation.left_hand.tactile_pressure': [255]*45,
        }, stamp=123)
        self.assertEqual(sender.tactile.producer.frame.channels[0].raw, bytes([255]*45))
        sender.close()


if __name__ == '__main__':
    unittest.main()
