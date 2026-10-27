"""xr_ai_teleop HX5 DexPilot on latest hand samples; outputs only private hand goals."""
import json
import time
from pathlib import Path

import numpy as np

from hardware.sh5_controller_hands import NATIVE_TOPICS
from hardware.sh5_hands_model import FINGER_NAMES
from hardware.xr_hand_input import XR_HAND_TOPICS


def hand_sample(msg):
    if len(msg.points) != 25 or msg.header.frame_id != 'xr_ai_wrist_local':
        raise ValueError('expected xr_ai wrist-local 25-point hand skeleton')
    points = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=float)
    if not np.isfinite(points).all() or np.count_nonzero(np.linalg.norm(points, axis=1) > 1e-7) < 20:
        raise ValueError('invalid hand skeleton')
    return points


class LatestHandSamples:
    """No backlog, no stale replay, and no replay across authority transitions."""
    def __init__(self):
        self.samples = {}

    def clear(self):
        self.samples.clear()

    def receive(self, side, points, now):
        self.samples[side] = (points, now)

    def take(self, now):
        samples, self.samples = self.samples, {}
        return {s: p for s, (p, stamp) in samples.items() if 0 <= now - stamp <= .2}


def make_retargeters():
    from ament_index_python.packages import get_package_share_directory
    from hardware.xr_ai_hx5_retarget import HX5D20DexPilotRetargeter
    root = Path(get_package_share_directory('ffw_description')) / 'urdf/common/hx5_d20'
    return {s: HX5D20DexPilotRetargeter(s, smoothing_alpha=1.,
                 urdf_path=root / f'hx5_d20_{s}.urdf') for s in FINGER_NAMES}


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    class XRHandRetargetNode(Node):
        def __init__(self):
            super().__init__('xr_ai_hand_retargeter')
            self.retargeters = make_retargeters()
            self.pending = LatestHandSamples()
            self.intent, self.gate = {}, {}
            self.intent_rx = self.gate_rx = -float('inf')
            self._reported_active = self._reported_output = False
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.pubs = {s: self.create_publisher(JointTrajectory, t, qos)
                         for s, t in NATIVE_TOPICS.items()}
            self.subs = [self.create_subscription(PointCloud, t,
                         lambda msg, s=s: self.receive(s, msg), qos)
                         for s, t in XR_HAND_TOPICS.items()]
            for kind, topic in (('intent', '/xr_hardware/intent'), ('gate', '/xr_hardware/gate_status')):
                self.subs.append(self.create_subscription(String, topic,
                                 lambda msg, k=kind: self.status(k, msg), 10))
            self.create_timer(.02, self.tick)
            self.get_logger().info('[XR AI HAND MODE 2] HX5 DexPilot; latest 25-point hands at up to 50 Hz; private gate output only')

        def allowed(self):
            now = time.monotonic()
            return (now-self.intent_rx <= .3 and now-self.gate_rx <= .3
                    and self.intent.get('enabled') and self.intent.get('phase') == 'ACTIVE'
                    and self.gate.get('forwarding') and not self.gate.get('fault')
                    and not self.gate.get('feedback_waiting'))

        def status(self, kind, msg):
            try:
                data = json.loads(msg.data)
                if not isinstance(data, dict) or data.get('model') != 'sh5':
                    raise ValueError('invalid hand authority status')
                setattr(self, kind, data)
                setattr(self, kind+'_rx', time.monotonic())
            except (TypeError, ValueError):
                setattr(self, kind+'_rx', -float('inf'))
            if not self.allowed():
                self.pending.clear()
            elif not self._reported_active:
                self._reported_active=True
                self.get_logger().info('[XR AI HAND MODE 2] Hand authority active')

        def receive(self, side, msg):
            if not self.allowed():
                self.pending.clear()
                return
            try:
                self.pending.receive(side, hand_sample(msg), time.monotonic())
            except ValueError as error:
                self.pending.samples.pop(side, None)
                self.get_logger().warning(f'[XR AI HAND INPUT] {side}: {error}', throttle_duration_sec=2.)

        def tick(self):
            if not self.allowed():
                self.pending.clear()
                return
            for side, points in self.pending.take(time.monotonic()).items():
                try:
                    q = self.retargeters[side].retarget(points)
                except ValueError as error:
                    self.get_logger().warning(f'[XR AI HAND DROP] {side}: {error}', throttle_duration_sec=2.)
                    continue
                if q.shape != (20,) or not np.isfinite(q).all():
                    raise ValueError('invalid HX5 DexPilot output')
                msg = JointTrajectory(joint_names=list(FINGER_NAMES[side]),
                                      points=[JointTrajectoryPoint(positions=q.tolist())])
                self.pubs[side].publish(msg)
                if not self._reported_output:
                    self._reported_output=True
                    self.get_logger().info('[XR AI HAND MODE 2] First 20-joint private hand goal published')

    rclpy.init()
    node = XRHandRetargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
