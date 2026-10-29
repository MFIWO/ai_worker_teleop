"""PC epoch anchored to robot monotonic time, without changing system clocks."""
import time

from hardware.hx5_tactile import HAND_MODEL, HAND_SIDES
from hardware.loop_channels import select_robot_channels


class PCRecordingClock:
    def __init__(self):
        self.anchor = None
        self.last_receipt = None
        self.last_stamp = 0

    def receive(self, epoch_us, now):
        if type(epoch_us) is not int or epoch_us <= 0:
            raise ValueError('invalid PC recording clock')
        candidate = (epoch_us / 1e6) - now
        if self.anchor is None:
            self.anchor = candidate
        # Allow gradual drift correction; never follow large wall-clock steps.
        elif abs(candidate - self.anchor) < 1.:
            self.anchor += max(-.0001, min(.0001, candidate - self.anchor))
        else:
            raise ValueError('PC recording clock stepped; restart this recording source')
        self.last_receipt = now

    def timestamp(self, now):
        if self.anchor is None:
            return None
        value = int((self.anchor + now) * 1e6)
        self.last_stamp = max(self.last_stamp + 1, value)
        return self.last_stamp


class ClockedRobotStreamer:
    """One aiworker SDK Robot State with robot/hand joints and numeric pressure."""
    def __init__(self, loop_addr, mapper, clock, *, selected_channels=None):
        self.loop_addr, self.mapper, self.clock = loop_addr, mapper, clock
        self.producer = None
        self.sent_count = 0
        # CHANGED(2026-10-28): Both hands and robot are in this same schema.
        self.channel_keys = (tuple(selected_channels) if selected_channels is not None
                             else tuple(i.key for i in mapper.channel_infos())
                             if mapper is not None else None)

    @staticmethod
    def select_config(requested):
        """Validate recording identity; Preview selection never commands motors."""
        if requested.action_space != 'joint_position':
            raise ValueError('VR recording supports joint_position only')
        if requested.robot_type not in ('', 'ffw_sh5_vr'):
            raise ValueError('VR recording requires robot type ffw_sh5_vr')
        if requested.finger_type not in ('', HAND_MODEL) or requested.gripper_type:
            raise ValueError('SH5 recording requires finger type ' + HAND_MODEL)
        return requested.model_copy(update=dict(robot_type='ffw_sh5_vr', finger_type=HAND_MODEL))

    def connect(self):
        from loop_sdk import ChannelSpec, RobotConfigOptions, RobotStreamSchema, SourceSchema, SourceProducer
        if self.mapper is None or not self.channel_keys:
            raise ValueError('aiworker Loop source requires at least one configured channel')
        selected = set(self.channel_keys)
        infos = [i for i in self.mapper.channel_infos() if i.key in selected]
        if len(infos) != len(selected):
            raise ValueError('Loop Robot State channel selection does not match mapper layout')
        schema = SourceSchema(robot=RobotStreamSchema(
            source_key='robotis-vr-sh5', name='aiworker',
            channels=tuple(ChannelSpec(key=i.key, label=i.label, unit=i.unit, group=i.group,
                                       range=None) for i in infos),
            available_options=RobotConfigOptions(
                action_space=('joint_position',), robot_type=('ffw_sh5_vr',),
                finger_type=(HAND_MODEL,))))
        self.producer = SourceProducer.connect(
            self.loop_addr, schema, on_select=self.select_config)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if self.producer.stats().connected:
                return
            time.sleep(.1)
        self.close()
        raise ConnectionError('Loop aiworker Robot State Source Bus did not connect')

    def send(self, payload, *, stamp=None):
        if stamp is None:
            stamp = self.clock.timestamp(time.monotonic())
        if stamp is None:
            return False  # wait for PC anchor, not robot wall time
        values = ({key: payload.get(key) for key in self.channel_keys}
                  if self.channel_keys is not None else dict(payload))
        self.producer.send_robot(stamp, self.sent_count, values)
        self.sent_count += 1
        return True

    def close(self):
        if self.producer is not None:
            self.producer.close()
            self.producer = None


class ClockedTactileStreamer:
    """Typed Loop Tactile source HX5-D20; no joint qpos in this source."""
    def __init__(self, loop_addr):
        self.loop_addr = loop_addr
        self.producer = None
        self.sent_count = 0

    def connect(self):
        # CHANGED(2026-10-28): The previously validated SDK typed-tactile path.
        from loop_sdk import SourceProducer, SourceSchema, TactileStreamSchema, TactileChannelSpec
        schema = SourceSchema(tactile=TactileStreamSchema(
            source_key='robotis-vr-sh5-tactile', name=HAND_MODEL,
            channels=tuple(TactileChannelSpec(
                key=side + '_hand',
                label=side + ' thumb,index,middle,ring,little; row-major 3x3; raw counts 0..255',
                sample_format='uint8', sample_count=45) for side in HAND_SIDES)))
        self.producer = SourceProducer.connect(self.loop_addr, schema)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if self.producer.stats().connected:
                return
            time.sleep(.1)
        self.close()
        raise ConnectionError('Loop HX5-D20 Tactile Source Bus did not connect')

    def send(self, payload, *, stamp):
        from loop_sdk import TactileChannelValue
        channels = []
        for side in HAND_SIDES:
            key = f'ffw_sh5_vr.observation.{side}_hand.tactile_pressure'
            values = payload.get(key)
            if values is None:
                continue  # unavailable/stale is missing, never zero-filled
            if len(values) != 45 or any(type(v) is not int or v < 0 or v > 255
                                         for v in values):
                raise ValueError(f'invalid HX5-D20 {side} tactile frame; expected 45 uint8')
            channels.append(TactileChannelValue(key=side + '_hand', raw=bytes(values),
                                                values_i16=tuple(values)))
        if channels:
            self.producer.send_tactile(stamp, self.sent_count, tuple(channels))
        self.sent_count += 1  # sequences follow the same 30 Hz ticks as aiworker

    def close(self):
        if self.producer is not None:
            self.producer.close()
            self.producer = None


class SplitRobotStreamer:
    """Compatibility name: one Robot State and an optional typed Tactile source."""
    def __init__(self, loop_addr, mapper, clock, *, channels='training',
                 enable_tactile=True):
        self.clock = clock
        selected = (select_robot_channels(mapper.channel_infos(), mapper.robot_type,
                                          channels) if mapper is not None else None)
        self.robot = ClockedRobotStreamer(
            loop_addr, mapper, clock, selected_channels=selected)
        self.tactile = ClockedTactileStreamer(loop_addr) if enable_tactile else None
        self.streams = (self.robot,) + ((self.tactile,) if self.tactile else ())

    @property
    def sent_count(self):
        return self.robot.sent_count

    def connect(self):
        try:
            # A declared empty tactile source can be selected in Preview but
            # cannot supply the first frame needed for recording readiness.
            self.robot.connect()
        except Exception:
            self.close()
            raise

    def send(self, payload, *, tactile_payload=None):
        stamp = self.clock.timestamp(time.monotonic())
        if stamp is None:
            return False
        self.robot.send(payload, stamp=stamp)
        raw = payload if tactile_payload is None else tactile_payload
        if self.tactile is not None:
            if self.tactile.producer is None:
                ready = all(raw.get(f'ffw_sh5_vr.observation.{s}_hand.tactile_pressure') is not None
                            for s in HAND_SIDES)
                if not ready:
                    return True
                self.tactile.connect()
            self.tactile.sent_count=self.robot.sent_count-1
            self.tactile.send(raw, stamp=stamp)
        return True

    def close(self):
        for stream in self.streams:
            stream.close()
