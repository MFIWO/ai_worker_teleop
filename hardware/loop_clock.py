"""PC epoch anchored to robot monotonic time, without changing either system clock."""
import time

from hardware.hx5_tactile import HAND_MODEL


class PCRecordingClock:
    def __init__(self):
        self.anchor=None
        self.last_receipt=None
        self.last_stamp=0

    def receive(self, epoch_us, now):
        if type(epoch_us) is not int or epoch_us<=0:raise ValueError('invalid PC recording clock')
        candidate=(epoch_us/1e6)-now
        if self.anchor is None:self.anchor=candidate
        # Allow a gradual drift correction; never follow large wall-clock steps.
        elif abs(candidate-self.anchor)<1.:
            self.anchor+=max(-.0001,min(.0001,candidate-self.anchor))
        else:raise ValueError('PC recording clock stepped; restart this recording source')
        self.last_receipt=now

    def timestamp(self, now):
        if self.anchor is None:return None
        value=int((self.anchor+now)*1e6)
        self.last_stamp=max(self.last_stamp+1,value)
        return self.last_stamp


class ClockedRobotStreamer:
    """Use the SDK's public SourceProducer.send_robot explicit timestamp API."""
    def __init__(self, loop_addr, mapper, clock, *, component='robot'):
        self.loop_addr,self.mapper,self.clock=loop_addr,mapper,clock
        self.producer=None
        self.sent_count=0
        self.component=component

    def owns_channel(self, key):
        if self.component == 'hand' and key.endswith(('.flags', '.phase', '.clock', '.ages')):
            return True
        parts = key.split('.')
        hand = any(part in ('left_hand', 'right_hand') for part in parts)
        hand = hand or key.endswith(('.controller_inputs', '.hand_pose_state'))
        return hand if self.component == 'hand' else not hand

    @staticmethod
    def select_config(requested):
        """Confirm recording identity only; Preview selection never commands motors."""
        if requested.action_space != 'joint_position':
            raise ValueError('VR recording supports joint_position only')
        if requested.robot_type not in ('','ffw_sh5_vr'):
            raise ValueError('VR recording requires robot type ffw_sh5_vr')
        if requested.finger_type not in ('',HAND_MODEL) or requested.gripper_type:
            raise ValueError('SH5 recording requires finger type '+HAND_MODEL)
        return requested.model_copy(update=dict(robot_type='ffw_sh5_vr',finger_type=HAND_MODEL))

    def connect(self):
        from loop_sdk import ChannelSpec,RobotConfigOptions,RobotStreamSchema,SourceSchema,SourceProducer
        hand = self.component == 'hand'
        schema=SourceSchema(robot=RobotStreamSchema(source_key='robotis-vr-sh5-hand' if hand else 'robotis-vr-sh5',
            name=HAND_MODEL if hand else 'aiworker',
            channels=tuple(ChannelSpec(key=i.key,label=i.label,unit=i.unit,group=i.group,
                                      range=(0.,255.) if i.key.endswith('.tactile_pressure') else None)
                           for i in self.mapper.channel_infos() if self.owns_channel(i.key)),
            available_options=RobotConfigOptions(action_space=('joint_position',),
                robot_type=('ffw_sh5_vr',),finger_type=(HAND_MODEL,))))
        self.producer=SourceProducer.connect(self.loop_addr,schema,on_select=self.select_config)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if self.producer.stats().connected:return
            time.sleep(.1)
        self.close()
        raise ConnectionError('Loop Source Bus did not connect')

    def send(self,payload, *, stamp=None):
        if stamp is None:stamp=self.clock.timestamp(time.monotonic())
        if stamp is None:return False  # wait for the first PC heartbeat, not robot wall time
        self.producer.send_robot(stamp,self.sent_count,
                                 {k:v for k,v in payload.items() if self.owns_channel(k)})
        self.sent_count+=1
        return True

    def close(self):
        if self.producer is not None:
            self.producer.close()
            self.producer=None


class SplitRobotStreamer:
    """Independent Loop sources with identical timestamps and sequence numbers."""
    def __init__(self, loop_addr, mapper, clock):
        self.clock = clock
        self.streams = tuple(ClockedRobotStreamer(loop_addr, mapper, clock, component=part)
                             for part in ('robot', 'hand'))

    @property
    def sent_count(self):
        return self.streams[0].sent_count

    def connect(self):
        try:
            for stream in self.streams:stream.connect()
        except Exception:
            self.close()
            raise

    def send(self, payload):
        stamp = self.clock.timestamp(time.monotonic())
        if stamp is None:return False
        for stream in self.streams:stream.send(payload, stamp=stamp)
        return True

    def close(self):
        for stream in self.streams:stream.close()
