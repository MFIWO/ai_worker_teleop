"""Read-only VR adapter around the robot's installed Loop camera/SDK implementation."""
import argparse
import json
from pathlib import Path
import signal
import tempfile
import time

from hardware.loop_vr_contract import RecordingStatus, vr_parameters
from hardware.loop_clock import PCRecordingClock,SplitRobotStreamer
from hardware.hx5_tactile import HAND_MODEL,PRESSURE_TOPICS,TactileReadings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--loop-addr', required=True)
    parser.add_argument('--robot-ip', required=True)
    parser.add_argument('--fps', type=float, default=30.)
    parser.add_argument('--no-camera', action='store_true')
    # CHANGED(2026-10-28): recording-only channel profile; VR control is unchanged.
    parser.add_argument('--channels', default='training', help='training | qpos | all | comma-separated channel keys')
    parser.add_argument('--no-tactile', action='store_true', help='do not register the HX5-D20 Tactile source')
    parser.add_argument('--describe', action='store_true')
    args = parser.parse_args()
    import yaml
    from ament_index_python.packages import get_package_share_directory
    config = Path(get_package_share_directory('ffw_loop_streamer'))/'config/ffw_sg2_rev1.yaml'
    params = vr_parameters(yaml.safe_load(config.read_text()), loop_addr=args.loop_addr,
                           fps=args.fps, robot_ip=args.robot_ip)
    params['/**']['ros__parameters']['enable_camera'] = not args.no_camera
    if args.describe:
        print(yaml.safe_dump(params, sort_keys=False))
        return
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
    from std_msgs.msg import String
    from ffw_loop_streamer.loop_streamer_node import LoopStreamerNode
    from ffw_loop_streamer.loop_streamer import RobotStepMapper, ChannelInfo,parse_camera_rotation

    class VRMapper(RobotStepMapper):
        def channel_infos(self):
            return super().channel_infos() + [ChannelInfo(
                key=f'{self.robot_type}.teleop.{name}', label=label, unit=unit, group='teleop')
                for name, label, unit in (
                    ('flags', 'active,paused,manual_active,fault,status_fresh', 'bool'),
                    ('phase', 'phase code (see VR_LOOP.md)', 'enum'),
                    ('action_fresh', 'left_arm,right_arm,left_hand,right_hand,head,lift,mobile', 'bool'),
                    ('action_valid', 'left_arm,right_arm,left_hand,right_hand,head,lift,mobile', 'bool'),
                    ('ages', 'joint_receipt,gate_receipt,intent_receipt (-1=missing)', 's'),
                    ('clock', 'pc_anchor_age,robot_minus_pc_wallclock', 's'),
                    ('controller_inputs', 'left_trigger,left_grip,right_trigger,right_grip', 'normalized'),
                    ('hand_pose_state', 'hand_mode,pending_pose,left_pose,right_pose (see VR_LOOP.md)', 'enum'))]

    class VRNode(LoopStreamerNode):
        def __init__(self):
            self.recording_status = RecordingStatus()
            self.recording_clock = PCRecordingClock()
            self.record_channels = args.channels
            self.enable_tactile = not args.no_tactile
            self.tactile = TactileReadings()
            self._tactile_reported = {}
            # Reuse installed subscriber/camera methods, while selecting a public
            # SDK sender that supports an explicit PC-clock timestamp. No global
            # monkeypatches or installed Loop package changes.
            Node.__init__(self,'loop_vr_streamer')
            defaults=dict(robot_type='',loop_addr=args.loop_addr,fps=30.,source_key='robotis-vr-sh5',
                enable_camera=True,rtsp_advertise_host='',rtsp_base_port=8554,jpeg_quality=75,
                jpeg_recompress_cameras='',camera_rotation='',rotated_jpeg_quality=90,
                camera_probe_timeout_s=120.,connect_timeout_s=5.)
            for name,value in defaults.items():self.declare_parameter(name,value)
            value=lambda name:self.get_parameter(name).value
            self.robot_type,self.loop_addr,self.fps=value('robot_type'),value('loop_addr'),float(value('fps'))
            self.enable_camera=bool(value('enable_camera'))
            self.jpeg_quality=int(value('jpeg_quality'))
            self.jpeg_recompress_cameras=tuple(n.strip() for n in str(value('jpeg_recompress_cameras')).split(',') if n.strip())
            self.camera_rotation=parse_camera_rotation(str(value('camera_rotation')))
            self.rotated_jpeg_quality=int(value('rotated_jpeg_quality'))
            self._rotation_failed_once={}
            self.camera_probe_timeout_s=float(value('camera_probe_timeout_s'))
            self.connect_timeout_s=float(value('connect_timeout_s'))
            if not 1<=self.jpeg_quality<=100 or not 1<=self.rotated_jpeg_quality<=100:
                raise ValueError('JPEG quality must be 1..100')
            self._load_robot_config()
            self._camera_msgs={}
            self._follower_joint_msgs={}
            self._leader_joint_msgs={}
            self._follower_mobile_msg=self._leader_mobile_msg=None
            self._warned_missing_joints=False
            self._qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
            self._init_subscribers()
            if self.enable_tactile:
                self._init_tactile_subscribers()
            # CHANGED(2026-10-28): one aiworker Robot State for arm/hand qpos.
            self.robot_streamer = SplitRobotStreamer(
                self.loop_addr, self.mapper, self.recording_clock,
                channels=self.record_channels, enable_tactile=self.enable_tactile)
            selected_keys = self.robot_streamer.robot.channel_keys
            if (any('.action.' in key for key in selected_keys) and
                    f'{self.robot_type}.teleop.action_valid' not in selected_keys):
                self.get_logger().warning(
                    '[Loop training] Action channels selected without action_valid; '
                    'cached HOLD commands may be unsuitable for imitation learning')
            self.robot_streamer.connect()
            self.get_logger().info(
                f'Loop aiworker ({len(selected_keys)} selected channels: '
                f'{self.record_channels}); typed HX5-D20 tactile={self.enable_tactile}; '
                f'keys={selected_keys}')
            self.camera_streamer=None
            self._probe_started_at=time.monotonic()
            self._probe_timer=(self.create_timer(self.PROBE_PERIOD_S,self._probe_cameras)
                               if self.enable_camera and self.camera_topics else None)
            self._stream_timer=self.create_timer(1./self.fps,self._stream_tick)
            self._tick_count=0
            self.create_subscription(String, '/xr_hardware/gate_status', lambda msg:self.receive_status('gate',msg), 10)
            self.create_subscription(String, '/xr_hardware/intent', lambda msg:self.receive_status('intent',msg), 10)
            self.get_logger().info('VR continuous recording: HOLD never stops data/camera transmission. No recording API calls.')

        def _init_tactile_subscribers(self):
            try:
                from robotis_interfaces.msg import HandPressures
            except ImportError:
                self.get_logger().warning(
                    '[HX5-D20] HandPressures is unavailable in this workspace; pressure channels stay missing. '
                    'Use a driver/workspace with robotis_interfaces/HandPressures and pressure broadcasters.')
                return
            self._tactile_subscriptions = [self.create_subscription(
                HandPressures, topic, lambda msg, side=side:self.receive_tactile(side,msg), self._qos)
                for side,topic in PRESSURE_TOPICS.items()]
            self.get_logger().info(
                f'[{HAND_MODEL}] Loop fingerType={HAND_MODEL}; subscribing to '+', '.join(PRESSURE_TOPICS.values()))

        def receive_tactile(self, side, msg):
            accepted = self.tactile.receive(side,msg,time.monotonic())
            error = self.tactile.errors[side]
            previous = self._tactile_reported.get(side)
            state = '' if accepted else error
            if previous != state:
                self._tactile_reported[side] = state
                if accepted:
                    self.get_logger().info(f'[{HAND_MODEL}] {side}: valid 5x3x3 raw tactile received')
                else:
                    self.get_logger().warning(f'[{HAND_MODEL}] {side}: invalid tactile omitted: {error}')

        def _load_robot_config(self):
            super()._load_robot_config()
            self.mapper = VRMapper(self.robot_type, self.mapper.groups)

        def receive_status(self, kind, msg):
            try:
                data=json.loads(msg.data)
                now=time.monotonic()
                accepted=self.recording_status.status(kind,data,now)
                if accepted and kind=='intent' and data.get('recording_epoch_us'):
                    self.recording_clock.receive(data['recording_epoch_us'],now)
            except (ValueError, KeyError, TypeError, AttributeError):
                setattr(self.recording_status, kind+'_rx', None)

        def _follower_joint_callback(self, name, msg):
            super()._follower_joint_callback(name,msg)
            self.recording_status.observation_rx=time.monotonic()

        def _follower_mobile_callback(self,msg):
            super()._follower_mobile_callback(msg)
            self.recording_status.mobile_rx=time.monotonic()

        def _leader_joint_callback(self,name,msg):
            super()._leader_joint_callback(name,msg)
            if msg.points:
                self.recording_status.action_rx[name.removeprefix('leader_')]=time.monotonic()

        def _leader_mobile_callback(self,msg):
            super()._leader_mobile_callback(msg)
            self.recording_status.action_rx['mobile']=time.monotonic()

        def _stream_tick(self):
            now=time.monotonic()
            observed=self._collect_follower_positions()
            if self.recording_status.observation_rx is None or now-self.recording_status.observation_rx>.5:
                observed=None
            mobile=(self._odometry_vector(self._follower_mobile_msg) if self.recording_status.mobile_rx is not None
                    and now-self.recording_status.mobile_rx<=.5 else None)
            payload=self.mapper.build_payload(follower_positions=observed,
                leader_positions=self._collect_leader_positions(),mobile_state=mobile,
                mobile_command=self._twist_vector(self._leader_mobile_msg))
            payload.update({f'{self.robot_type}.teleop.{k}':v for k,v in self.recording_status.values(now).items()})
            if self.enable_tactile:
                payload.update(self.tactile.payload(self.robot_type,now))
            # Keep raw button depths during HOLD; they are inputs, not actions.
            controls = None
            receipt = self.recording_status.intent_rx
            if receipt is not None and 0 <= now-receipt <= .5:
                raw = self.recording_status.intent.get('controller_values')
                if raw is not None:
                    from hardware.sh5_controller_hands import control_values
                    try:
                        values = control_values(raw)
                        controls = [values[side][key] for side in ('left','right') for key in ('trigger','grip')]
                    except ValueError:pass
            payload[f'{self.robot_type}.teleop.controller_inputs'] = controls
            state=None
            if receipt is not None and 0<=now-receipt<=.5:
                raw_state=self.recording_status.intent.get('hand_pose_state')
                if (isinstance(raw_state,list) and len(raw_state)==4 and all(type(v) is int for v in raw_state)
                        and raw_state[0] in (1,2) and 0<=raw_state[1]<=4
                        and all(0<=v<=5 for v in raw_state[2:])):
                    state=[float(v) for v in raw_state]
            payload[f'{self.robot_type}.teleop.hand_pose_state']=state
            if self.enable_tactile and self._tick_count and self._tick_count % max(1,int(self.fps*10)) == 0:
                for side,receipt in self.tactile.receipts.items():
                    if receipt is None or now-receipt > self.tactile.freshness:
                        self.get_logger().warning(
                            f'[{HAND_MODEL}] {side}: tactile missing/stale on {PRESSURE_TOPICS[side]}; '
                            'pressure is absent in Loop, not a zero reading')
            anchor_age=(-1. if self.recording_clock.last_receipt is None else now-self.recording_clock.last_receipt)
            skew=(-1. if self.recording_clock.anchor is None else time.time()-self.recording_clock.anchor-now)
            payload[f'{self.robot_type}.teleop.clock']=[anchor_age,skew]
            self.robot_streamer.send(payload)
            self._tick_count+=1

    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    with tempfile.TemporaryDirectory(prefix='loop_vr_') as directory:
        generated=Path(directory)/'parameters.yaml'
        generated.write_text(yaml.safe_dump(params,sort_keys=False))
        rclpy.init(args=['--ros-args','--params-file',str(generated),'-r','__node:=loop_vr_streamer'],
                   signal_handler_options=SignalHandlerOptions.NO)
        node=None
        try:
            node=VRNode()
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            if node is not None:
                node.close()
                node.destroy_node()
            if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
