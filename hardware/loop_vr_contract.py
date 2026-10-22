"""VR recording layout and quality markers; no ROS or recording side effects."""
from copy import deepcopy
import math

PHASES = ('WAIT_CALIBRATION', 'ALIGN_REQUIRED', 'SETTLING', 'WAIT_START',
          'RESUME_SEED', 'ACTIVE', 'HOLD', 'FAULT')
GROUPS = {
    'left_arm': tuple(f'arm_l_joint{i}' for i in range(1, 8)),
    'right_arm': tuple(f'arm_r_joint{i}' for i in range(1, 8)),
    'left_hand': tuple(f'finger_l_joint{i}' for i in range(1, 21)),
    'right_hand': tuple(f'finger_r_joint{i}' for i in range(1, 21)),
    'head': ('head_joint1', 'head_joint2'), 'lift': ('lift_joint',),
    'mobile': ('linear_x', 'linear_y', 'angular_z'),
}
TOPICS = {
    'left_arm': '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
    'right_arm': '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
    'left_hand': '/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory',
    'right_hand': '/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory',
    'head': '/leader/joystick_controller_left/joint_trajectory',
    'lift': '/leader/joystick_controller_right/joint_trajectory', 'mobile': '/cmd_vel',
}


def vr_parameters(existing, *, loop_addr, fps=30., robot_ip=''):
    """Reuse the installed camera setup; introduce a separate SH5 source layout."""
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError('fps must be finite and positive')
    result = deepcopy(existing)
    params = result['/**']['ros__parameters']
    camera = params['ffw_sg2_rev1']['camera_topic_list']
    params['ffw_sh5_vr'] = {
        'camera_topic_list': list(camera),
        'joint_topic_list': ['follower_upper_body:/joint_states', 'follower_mobile:/odom'] +
                            [f'leader_{name}:{TOPICS[name]}' for name in GROUPS],
        'joint_list': [f'leader_{name}' for name in GROUPS],
        'joint_order': {f'leader_{name}': list(names) for name, names in GROUPS.items()},
    }
    params.update(robot_type='ffw_sh5_vr', source_key='robotis-vr-sh5', loop_addr=loop_addr,
                  fps=fps, rtsp_advertise_host=robot_ip, camera_probe_timeout_s=120.)
    return result


def age(now, receipt):
    return -1. if receipt is None else max(0., now-receipt)


class RecordingStatus:
    def __init__(self):
        self.gate, self.intent = {}, {}
        self.gate_rx = self.intent_rx = None
        self.observation_rx = self.mobile_rx = None
        self.action_rx = {}

    def status(self, kind, data, now):
        if data.get('model') != 'sh5':
            raise ValueError('VR recorder requires SH5 status')
        old = getattr(self, kind)
        identity, sequence = ('gate_instance', 'status_sequence') if kind == 'gate' else ('session_id', 'sequence')
        if not isinstance(data.get(identity), str) or type(data.get(sequence)) is not int:
            raise ValueError('status identity/sequence missing')
        if old.get(identity) == data[identity] and data[sequence] <= old.get(sequence, -1):
            return False
        setattr(self, kind, dict(data))
        setattr(self, kind+'_rx', now)
        return True

    def values(self, now, *, freshness=.5):
        fresh = lambda stamp: stamp is not None and 0 <= now-stamp <= freshness
        health = fresh(self.gate_rx) and fresh(self.intent_rx)
        fault = bool(self.gate.get('fault')) if fresh(self.gate_rx) else False
        active = bool(health and self.intent.get('enabled') and self.intent.get('phase') == 'ACTIVE'
                      and self.intent.get('tracking_fresh', True) and self.intent.get('robot_feedback_fresh', True)
                      and self.gate.get('forwarding') and not self.gate.get('feedback_waiting') and not fault)
        manual = bool(health and self.gate.get('manual_active') and not fault)
        phase = self.intent.get('phase') if fresh(self.intent_rx) else None
        return {
            'flags': [float(active), float(not active), float(manual), float(fault), float(health)],
            'phase': [float(PHASES.index(phase)) if phase in PHASES else -1.],
            'action_fresh': [float(fresh(self.action_rx.get(name))) for name in GROUPS],
            'action_valid': [float(fresh(self.action_rx.get(name)) and
                                   (active if name.endswith(('arm', 'hand')) else active or manual)) for name in GROUPS],
            'ages': [age(now, self.observation_rx), age(now, self.gate_rx), age(now, self.intent_rx)],
        }
