"""WebXR 25-pose hand conversion for xr_ai_teleop HX5 DexPilot; no arm outputs."""
import numpy as np

# xr_ai_teleop's T_TO_UNITREE_HAND @ T_ROBOT_OPENXR, applied in wrist space.
OPENXR_TO_XR_HAND = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
XR_HAND_TOPICS = {side: f'/xr_hardware/sh5/xr_{side}_landmarks'
                  for side in ('left', 'right')}


def xr_hand_points(flat_poses):
    """Match the wrist-local 25-point convention consumed by xr_ai_teleop."""
    raw = np.asarray(flat_poses, dtype=np.float64)
    if raw.size != 400 or not np.isfinite(raw).all():
        raise ValueError('expected 25 finite WebXR hand poses')
    poses = raw.reshape(25, 16).reshape(25, 4, 4).transpose(0, 2, 1)
    wrist = poses[0]
    rotation = wrist[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=.02) or np.linalg.det(rotation) < .98:
        raise ValueError('invalid wrist rotation')
    local = (poses[:, :3, 3] - wrist[:3, 3]) @ rotation
    if np.count_nonzero(np.linalg.norm(local, axis=1) > 1e-7) < 20:
        raise ValueError('degenerate hand skeleton')
    return local @ OPENXR_TO_XR_HAND.T


class XRHandInput:
    def __init__(self, node):
        from sensor_msgs.msg import PointCloud
        self.node = node
        self.publishers = {s: node.create_publisher(PointCloud, t, node.vr_stream_qos)
                           for s, t in XR_HAND_TOPICS.items()}

    def publish_event(self, event, enabled):
        from geometry_msgs.msg import Point32
        from sensor_msgs.msg import PointCloud
        if not enabled or not isinstance(event.value, dict):
            return
        for side, pub in self.publishers.items():
            if side not in event.value:
                continue
            try:
                points = xr_hand_points(event.value[side])
            except (TypeError, ValueError):
                continue
            msg = PointCloud()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'xr_ai_wrist_local'
            msg.points = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
            pub.publish(msg)
