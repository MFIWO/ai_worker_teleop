"""PC epoch anchored to robot monotonic time, without changing either system clock."""
import time


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
    def __init__(self, loop_addr, mapper, clock):
        self.loop_addr,self.mapper,self.clock=loop_addr,mapper,clock
        self.producer=None
        self.sent_count=0

    def connect(self):
        from loop_sdk import ChannelSpec,RobotConfigOptions,RobotStreamSchema,SourceSchema,SourceProducer
        schema=SourceSchema(robot=RobotStreamSchema(source_key='robotis-vr-sh5',name='robotis-vr-sh5',
            channels=tuple(ChannelSpec(key=i.key,label=i.label,unit=i.unit,group=i.group)
                           for i in self.mapper.channel_infos()),
            available_options=RobotConfigOptions(action_space=('joint_position',),robot_type=('ffw_sh5_vr',))))
        self.producer=SourceProducer.connect(self.loop_addr,schema)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if self.producer.stats().connected:return
            time.sleep(.1)
        self.close()
        raise ConnectionError('Loop Source Bus did not connect')

    def send(self,payload):
        stamp=self.clock.timestamp(time.monotonic())
        if stamp is None:return False  # wait for the first PC heartbeat, not robot wall time
        self.producer.send_robot(stamp,self.sent_count,dict(payload))
        self.sent_count+=1
        return True

    def close(self):
        if self.producer is not None:
            self.producer.close()
            self.producer=None
