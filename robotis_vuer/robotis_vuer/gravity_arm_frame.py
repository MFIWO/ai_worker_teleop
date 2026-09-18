"""Express head-relative arm poses in a gravity-aligned frame (WebXR Y-up)."""

import numpy as np
from scipy.spatial.transform import Rotation


class GravityArmFrame:
    """Remove head tilt, retaining its horizontally projected forward direction."""

    def __init__(self, head_to_ros):
        self.head_to_ros = np.asarray(head_to_ros, dtype=float)
        self.forward = None
        self.correction = None
        self.reference = None

    def update(self, head):
        """Update from body-head pose; retain heading near vertical, never invent one."""
        head = np.asarray(head, dtype=float)
        self.correction = self.reference = None
        if head.shape != (4, 4) or not np.all(np.isfinite(head)):
            return False
        rotation = head[:3, :3]
        if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-3)):
            return False
        up = np.array([0., 1., 0.])
        # XR body head is +Y forward, +X down, +Z right (not viewer-camera axes).
        forward = rotation[:, 1].copy()
        forward -= np.dot(forward, up) * up
        norm = np.linalg.norm(forward)
        if norm >= .2:
            self.forward = forward / norm
        elif self.forward is None:
            return False
        left = np.cross(up, self.forward)
        world_to_level = np.stack([self.forward, left, up])
        # Input poses are B @ R_head.T @ (p_world - p_head).
        self.correction = world_to_level @ rotation @ self.head_to_ros.T
        self.reference = np.eye(4)
        self.reference[:3, :3] = world_to_level.T
        self.reference[:3, 3] = head[:3, 3]
        return True

    def transform(self, position, quaternion):
        """Apply the same basis change to position and orientation."""
        if self.correction is None:
            raise RuntimeError('No valid gravity-aligned arm reference')
        position = self.correction @ np.asarray(position, dtype=float)
        orientation = Rotation.from_matrix(self.correction) * Rotation.from_quat(quaternion)
        return position, orientation.as_quat()
