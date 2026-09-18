"""Hold arm references across missing tracking and discontinuous reacquisition."""

import numpy as np
from scipy.spatial.transform import Rotation


class ArmTrackingGuard:
    """Accept whole pose batches, never interpolate through an unobserved jump."""

    def __init__(self):
        self.last = None
        self.candidate = None
        self.stable_since = None
        self.holding = True
        self.reason = 'waiting_for_tracking'

    @staticmethod
    def close(a, b, distance, degrees):
        if a.keys() != b.keys():
            return False
        for key in a:
            pa, qa = a[key]
            pb, qb = b[key]
            if np.linalg.norm(pa - pb) > distance:
                return False
            angle = (Rotation.from_quat(qa).inv() * Rotation.from_quat(qb)).magnitude()
            if angle > np.deg2rad(degrees):
                return False
        return True

    def hold(self, reason):
        self.holding = True
        self.reason = reason
        self.candidate = None
        self.stable_since = None

    def accept(self, poses, now):
        if not poses or any(not np.all(np.isfinite(p)) or not np.all(np.isfinite(q))
                            or np.linalg.norm(q) < 1e-6 for p, q in poses.values()):
            self.hold('invalid_pose')
            return False
        if not self.holding and not self.close(poses, self.last, .08, 20.):
            self.hold('reference_jump')
        if self.holding:
            if self.last is not None and not self.close(poses, self.last, .05, 15.):
                self.reason = 'return_to_held_pose'
                self.candidate = self.stable_since = None
                return False
            if self.candidate is None or not self.close(poses, self.candidate, .02, 8.):
                self.candidate = poses
                self.stable_since = now
            if now - self.stable_since < .3:
                return False
        self.last = poses
        self.holding = False
        self.reason = 'tracking'
        return True
