"""HX5-D20 controller synergies, applied only by the existing robot gate."""
import json
import math
from pathlib import Path

from hardware.sh5_hands_model import FINGER_NAMES
from hardware.controller_hand_modes import POSE_CODES

HAND_TOPICS = {side: f'/leader/joint_trajectory_command_broadcaster_{side}_hand/joint_trajectory'
               for side in FINGER_NAMES}
NATIVE_TOPICS = {side:f'/xr_hardware/sh5/native_{side}_hand' for side in FINGER_NAMES}
HAND_SPEED_SCALE = .85
# [CHANGED 2026-10-28] Controller mode 2 trigger CLOSING only: one quarter of the configured
# finger velocity. Optical modes, grip-pinch and opening are unaffected.
MODE2_TRIGGER_CLOSE_SPEED_SCALE = .25
# ROBOTIS DexPilotOptimizer.set_joint_limit expands bounds by 1e-3 rad.
# Accept that solver margin at input, then clamp to the physical URDF bounds.
# This never expands the limits of a command sent to the hand.
NATIVE_SOLVER_MARGIN = 1e-3 + 1e-6


EFFORT_TOPICS = {side:f'/effort_{side[0]}_controller/commands' for side in FINGER_NAMES}
STOCK_HAND_CURRENT_RAW = 300.0  # Existing follower launch setting, NOT milliamps.

def pointing_currents(controls):
    """Trigger scales index current ceiling 150..300 raw, never above stock.

    Keep the other sixteen motor ceilings at the original value. This changes
    available holding current, not torque calibration or a physical force goal.
    """
    result = {}
    for side,item in controls.items():
        values=[STOCK_HAND_CURRENT_RAW]*20
        if item.get('pose')=='pointing':
            value=150.0+150.0*item['trigger']
            values[4:8]=[value]*4
        result[side]=values
    return result


def control_values(payload):
    if not isinstance(payload,dict) or set(payload) != set(FINGER_NAMES):
        raise ValueError('controller hands require both left/right inputs')
    result = {}
    for side in FINGER_NAMES:
        item = payload[side]
        if not isinstance(item,dict) or set(item)-{'trigger','grip','pose'} or not {'trigger','grip'}<=set(item):
            raise ValueError('expected trigger/grip per hand')
        values = {}
        for key in ('trigger','grip'):
            value=item[key]
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0 <= value <= 1:
                raise ValueError('trigger/grip must be finite values in 0..1')
            values[key] = float(value)
        if 'pose' in item:
            if item['pose'] not in POSE_CODES:raise ValueError('unknown controller hand preset')
            values['pose'] = item['pose']
        result[side] = values
    return result


def load_poses(limits):
    config = json.loads(Path(__file__).with_name('hx5_controller_poses.json').read_text())
    if config.get('schema') not in (1,2) or config.get('model') != 'HX5-D20':
        raise ValueError('unsupported controller hand pose layout')
    for side,names in FINGER_NAMES.items():
        expected = {'open','grasp','pinch'} | (set(POSE_CODES)|{'angled_prepare','pointing_prepare','index_prepare','full_prepare','ulnar_prepare'} if config['schema']==2 else set())
        if set(config['poses'][side]) != expected:
            raise ValueError('hand pose definitions missing')
        for pose in config['poses'][side].values():
            if len(pose) != 20:raise ValueError('hand pose requires 20 joint positions')
            for name,value in zip(names,pose):
                bound = limits[name]
                if not math.isfinite(value) or not bound['lower'] <= value <= bound['upper']:
                    raise ValueError(f'{name}: controller hand pose outside physical URDF bounds')
    return config['poses']


def targets(poses,controls):
    result = {}
    for side,item in controls.items():
        # Grip has priority: thumb/index/middle pinch, ring/little stay open.
        mode = 'pinch' if item['grip'] > .05 else item.get('pose','grasp')
        depth = item['grip'] if mode == 'pinch' else item['trigger']
        # Mode 2: selecting angled grip establishes the C-shaped rest pose.
        # Trigger travel closes continuously from that pose, including release.
        baseline = (poses[side]['angled_prepare'] if mode=='angled_grip' else
                    poses[side]['pointing_prepare'] if mode=='pointing' else
                    poses[side]['index_prepare'] if mode=='index_pinch' else
                    poses[side]['ulnar_prepare'] if mode=='ulnar_pinch' else
                    poses[side]['full_prepare'] if mode=='full_grip' else poses[side]['open'])
        result.update({name: opened+depth*(closed-opened)
                       for name,opened,closed in zip(FINGER_NAMES[side],baseline,poses[side][mode])})
    return result


class SH5HandOutput:
    @property
    def direct_output(self):
        return self.mode=='hands' or self.hand_mode==2

    def __init__(self,node,limits,mode):
        if mode not in ('hands','controllers'):raise ValueError('unsupported SH5 input mode')
        from trajectory_msgs.msg import JointTrajectory
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        self.node,self.mode = node,mode
        self.hand_mode = int(node.profile.get('hand_mode',1))
        if self.hand_mode not in (1,2):raise ValueError('unsupported controller hand mode')
        self.limits = {n:dict(b,velocity=b['velocity']*HAND_SPEED_SCALE) for n,b in limits.items()}
        self.poses = load_poses(limits) if mode=='controllers' else None
        if mode=='controllers' and self.hand_mode==2 and not set(POSE_CODES)<=set(self.poses['left']):
            raise ValueError('install mode-2 hand pose definitions on robot')
        self.preparing = {}
        self.native = {}
        self.publishers = {side:node.create_publisher(JointTrajectory,topic,10) for side,topic in HAND_TOPICS.items()}
        if mode=='hands':
            qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
            self.subscriptions = [node.create_subscription(JointTrajectory,topic,
                lambda msg,side=side:self.native_trajectory(side,msg),qos) for side,topic in NATIVE_TOPICS.items()]
        self.effort_publishers = {}
        if mode == 'controllers' and self.hand_mode == 2:
            from std_msgs.msg import Float64MultiArray
            self.effort_publishers = {side:node.create_publisher(Float64MultiArray,topic,1)
                                      for side,topic in EFFORT_TOPICS.items()}
        self.contact_goals = {}
        self.tactile = None
        if mode == 'controllers' and self.hand_mode == 2:
            from hardware.hx5_tactile import TactileReadings, PRESSURE_TOPICS
            self.tactile = TactileReadings(freshness=.3)
            try:
                from robotis_interfaces.msg import HandPressures
            except ImportError:
                node.get_logger().warning('HandPressures unavailable: pointing contact lock disabled')
            else:
                import time
                self.pressure_subscriptions = [node.create_subscription(HandPressures, topic,
                    lambda msg, side=side: self.tactile.receive(side,msg,time.monotonic()),
                    QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
                    for side,topic in PRESSURE_TOPICS.items()]
        self.controls = None
        self.rx = -math.inf
        self.previous = {}
        self.last_trigger = {}  # [ADDED] Per-hand direction; trigger release is never slowed.
        self.active = False
        self.last_output = 0.
        node.get_logger().info('[SH5 HAND OUTPUT] '+(
            'Optical input callbacks only; no hand output timer or interpolation. Global HOLD/fault remains.'
            if self.mode=='hands' else
            'Controller mode 2: trigger closure <= 25% of configured finger velocity; '
            'continuous joint travel, bounded opening; global HOLD/fault retained.'
            if self.mode=='controllers' and self.hand_mode==2 else
            'Direct position relay; no added smoothing, slew or hand-age gate. Global HOLD/fault remains.'
            if self.direct_output else 'Mode 1: original hand velocity limits and input watchdog.'))

    def receive(self,payload,now):
        self.controls = None if payload is None else control_values(payload)
        if self.controls is not None:
            if any(('pose' in item)!=(self.hand_mode==2) for item in self.controls.values()):
                self.controls = None
                raise ValueError('PC/robot controller hand modes do not match')
        # [CHANGED] In mode 2, physical grips are ARM clutch / ready-reset controls,
        # never a competing finger command. Mode 1 retains its old tripod pinch.
        if self.controls is not None and self.hand_mode == 2:
            for item in self.controls.values():
                item['grip'] = 0.0
        self.rx = now

    def native_trajectory(self,side,msg):
        import time
        try:
            if (len(msg.points)!=1 or len(msg.joint_names)!=20 or len(set(msg.joint_names))!=20
                    or set(msg.joint_names)!=set(FINGER_NAMES[side]) or len(msg.points[0].positions)!=20):
                raise ValueError('expected complete named 20-joint native hand trajectory')
            values = dict(zip(msg.joint_names,msg.points[0].positions))
            for n,v in values.items():
                lower,upper=self.limits[n]['lower'],self.limits[n]['upper']
                if not math.isfinite(v) or not lower-NATIVE_SOLVER_MARGIN<=v<=upper+NATIVE_SOLVER_MARGIN:
                    raise ValueError(f'native hand target outside physical bounds: {n}={v:.9g}, '
                                     f'limits=[{lower:.9g}, {upper:.9g}], solver margin={NATIVE_SOLVER_MARGIN:.9g} rad')
            values = {n:min(self.limits[n]['upper'],max(self.limits[n]['lower'],v)) for n,v in values.items()}
            self.native[side] = (values,time.monotonic())
            # Relay each native result on arrival, independently per side.
            # No resampling, smoothing, slew limit or hand-specific age gate.
            if self.active and self.node.intent_enabled and self.node.fault is None:
                self.publish(values)
        except ValueError as error:
            self.native.pop(side,None)
            if self.node.intent_enabled:self.node.latch(str(error))

    def publish_effort(self):
        if not getattr(self,'effort_publishers',None):return
        from std_msgs.msg import Float64MultiArray
        for side,values in pointing_currents(self.controls).items():
            self.effort_publishers[side].publish(Float64MultiArray(data=values))

    def graph(self):
        if getattr(self,'effort_publishers',None):
            for topic in EFFORT_TOPICS.values():
                writers=self.node.get_publishers_info_by_topic(topic)
                if len(writers)!=1 or writers[0].node_name!=self.node.get_name():
                    raise ValueError('hand current output requires one gate writer; stop other effort publishers')
        for topic in HAND_TOPICS.values():
            writers = self.node.get_publishers_info_by_topic(topic)
            if len(writers) != 1 or writers[0].node_name != self.node.get_name():
                raise ValueError('controller hand output requires one gate writer; stop native/other hand retargeters')
        if self.mode=='hands' and self.hand_mode==2:
            for topic in NATIVE_TOPICS.values():
                writers = self.node.get_publishers_info_by_topic(topic)
                if len(writers)!=1 or writers[0].node_name!='xr_ai_hand_retargeter':
                    raise ValueError(f'{topic}: require one xr_ai_hand_retargeter writer')

    def publish(self,values):
        from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
        for side,names in FINGER_NAMES.items():
            if not all(n in values for n in names):continue
            self.publishers[side].publish(JointTrajectory(joint_names=list(names),
                points=[JointTrajectoryPoint(positions=[float(values[n]) for n in names])]))

    def hold(self,measured,fresh):
        if self.active and fresh:
            # Preserve commanded preload: measured angles can relax a loaded grasp.
            held = dict(measured)
            if self.mode == 'controllers' and self.hand_mode == 2:
                held.update(self.previous)
            self.publish(held)
        self.active = False
        if not (self.mode == 'controllers' and self.hand_mode == 2):
            self.previous.clear()
        self.last_trigger.clear()  # [ADDED] Do not carry pre-HOLD trigger states to restart.
        self.preparing.clear()

    def update_authority(self,measured,now,allowed):
        """Optical hand lifecycle only; enabling never replays a saved goal."""
        if allowed:self.active=True
        else:self.hold(measured,now-self.node.measured_rx <= .5)

    def prepare_targets(self,desired,measured,now):
        for side,item in self.controls.items():
            pose = item.get('pose')
            if pose != 'tripod_deep' or item['trigger']<=.02:  # [CHANGED] prepare both selected pinches
                self.preparing.pop(side,None)
                continue
            names=FINGER_NAMES[side]
            desired.update(zip(names[:4],self.poses[side][pose][:4]))
            if side not in self.preparing or self.preparing[side]['pose']!=pose:
                self.preparing[side]={'pose':pose,'start':now,'done':False,
                    'held':{name:measured[name] for name in names}}
            state=self.preparing[side]
            if state['done']:continue
            # [CHANGED] Preposition the thumb for either pinch before closing the
            # selected fingers. Measured arrival and URDF slew are preserved.
            prep=dict(state['held'])
            prep.update(zip(names[:4],self.poses[side][pose][:4]))
            checked=names[:4]
            reached=all(abs(measured[n]-prep[n])<.10 for n in checked)
            if reached and now-state['start']>=.12:
                state['done']=True
            else:
                if now-state['start']>8.:
                    raise ValueError(f'{side}: {pose} preparation not reached; holding fingers')
                desired.update(prep)
        return desired

    def tick(self,measured,now,allowed):
        from hardware.aux_controls import slew_aux
        if self.mode=='hands':
            raise ValueError('optical hand targets must be forwarded on input, never on timer ticks')
        ready = self.controls is not None and (self.direct_output or now-self.rx<=.3)
        if not allowed or not ready:
            self.hold(measured,now-self.node.measured_rx <= .5)
            return
        desired = (targets(self.poses,self.controls) if self.mode=='controllers' else
                   {n:v for values,_ in self.native.values() for n,v in values.items()})
        if not set(desired).issubset(measured):
            raise ValueError('controller hands require all 40 fresh measured finger joints')
        if self.mode=='controllers' and self.hand_mode==2:
            desired=self.prepare_targets(desired,measured,now)
        output = (desired if self.direct_output else
                  slew_aux(desired,self.previous,measured,self.limits,
                           now-self.last_output if self.active else .01))
        if self.mode=='controllers' and self.hand_mode==2:
            # Bound every trigger output transition, including thumb prep and
            # tiny trigger reversals. Direction comes from remaining joint
            # travel, not noisy trigger derivatives. Release remains faster,
            # but never jumps directly to a distant target.
            dt = now-self.last_output if self.active else .01
            for side,item in self.controls.items():
                if item['grip'] > .05:
                    continue  # legacy mode-1 pinch override in offline callers
                pose = item.get('pose','grasp')
                baseline_name = {'angled_grip':'angled_prepare',
                                 'pointing':'pointing_prepare',
                                 'index_pinch':'index_prepare',
                                 'full_grip':'full_prepare',
                                 'ulnar_pinch':'ulnar_prepare'}.get(pose,'open')
                baseline = self.poses[side][baseline_name]
                closed = self.poses[side][pose]
                paced = {}
                for i,name in enumerate(FINGER_NAMES[side]):
                    old = self.previous.get(name,measured[name])
                    opening = (desired[name]-old)*(closed[i]-baseline[i]) < -1e-9
                    scale = 1.0 if opening else MODE2_TRIGGER_CLOSE_SPEED_SCALE
                    paced[name] = dict(self.limits[name],velocity=self.limits[name]['velocity']*scale)
                output.update(slew_aux({n:desired[n] for n in FINGER_NAMES[side]},
                                       self.previous,measured,paced,dt))
                self.last_trigger[side] = item['trigger']
        if self.mode == 'controllers' and self.hand_mode == 2:
            self.lock_pointing_contact(output,now)
        self.publish_effort()
        self.publish(output)
        self.previous,self.last_output,self.active = output,now,True

    def lock_pointing_contact(self, output, now):
        """Hold the configured index pose while trigger is held, even without tactile.

        Enter the straight pose through the existing velocity bound first.
        Never capture a deflected measured finger as the new holding target.
        Current ceilings are handled separately and never exceed stock 300 raw.
        """
        if not hasattr(self,'contact_goals'):self.contact_goals={}
        for side,item in self.controls.items():
            if item.get('pose')!='pointing' or item['trigger']<=.02:
                self.contact_goals.pop(side,None)
                continue
            names=FINGER_NAMES[side][4:8]
            goal=dict(zip(names,self.poses[side]['pointing'][4:8]))
            if side not in self.contact_goals and all(abs(output[n]-goal[n])<=1e-6 for n in names):
                self.contact_goals[side]=goal
            output.update(self.contact_goals.get(side,{}))
