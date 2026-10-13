"""[ADDED] Calibrated wrist targets; no ROS, hardware, or IK side effects.

References (Apache-2.0): MFIWO/xr_tele, commit
28d35e311581b30540929d262f5727791d17a6c6,
teleop/robot_control/robotis_ai_worker.py: AIWorkerArmIK._target.
The measured-start spatial rotation/translation delta is adapted, not its
joint solver, robot dimensions, zero-elbow extension, or fixed wrist offsets.
H1_2/H2 wrist-centric IK is the design reference; the existing Cyclo QP remains.
Copyright 2025 Unitree Robotics (referenced XR implementation).
Licensed under the Apache License, Version 2.0.
"""

from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial.transform import Rotation


def rigid_pose(value):
    """Reject corrupt/projective/reflected tracking matrices, without repairing."""
    pose = np.asarray(value, dtype=np.float64)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError('pose must be a finite 4x4 matrix')
    r = pose[:3, :3]
    if (not np.allclose(pose[3], [0., 0., 0., 1.], atol=1e-3)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-3)
            or not np.isclose(np.linalg.det(r), 1., atol=1e-3)):
        raise ValueError('tracking pose is not a proper rigid transform')
    return pose.copy()


def level_basis_from_body_head(head):
    """Body head +Y is forward, WebXR world +Y is up (NOT camera -Z).

    [CHANGED] Capture this basis only on manual activation. It does not follow
    subsequent HMD yaw/pitch/roll or position while the operator holds still.
    """
    head = rigid_pose(head)
    up = np.array([0., 1., 0.])
    forward = head[:3, 1].copy()
    forward -= up * np.dot(forward, up)
    length = np.linalg.norm(forward)
    if length < 0.2:
        raise ValueError('head forward is near vertical; look forward to calibrate')
    forward /= length
    left = np.cross(up, forward)
    return np.stack([forward, left, up])


@dataclass
class WristAnchor:
    input_pose: np.ndarray
    robot_pose: np.ndarray


class CalibratedWristMapper:
    """Independent left/right spatial deltas in one fixed horizontal basis."""

    def __init__(self, head, inputs, robots, translation_scale=1.0):
        if not math.isfinite(translation_scale) or not 0.1 <= translation_scale <= 1.5:
            raise ValueError('translation_scale must be finite, within [0.1,1.5]')
        if set(inputs) != {'left', 'right'} or set(robots) != {'left', 'right'}:
            raise ValueError('both wrists are required to capture an anchor')
        self.basis = level_basis_from_body_head(head)
        self.scale = translation_scale
        self.anchors = {
            side: WristAnchor(rigid_pose(inputs[side]), rigid_pose(robots[side]))
            for side in ('left', 'right')
        }

    def target(self, side, input_pose):
        live = rigid_pose(input_pose)
        anchor = self.anchors[side]
        q = self.basis
        output = np.eye(4)
        # [ADAPTED] Same spatial-delta multiplication order as XR AIWorkerArmIK.
        output[:3, 3] = anchor.robot_pose[:3, 3] + self.scale * q @ (
            live[:3, 3] - anchor.input_pose[:3, 3])
        world_delta = live[:3, :3] @ anchor.input_pose[:3, :3].T
        output[:3, :3] = q @ world_delta @ q.T @ anchor.robot_pose[:3, :3]
        return rigid_pose(output)


def limited_pose(previous, requested, dt, speed=0.20, angular_speed=1.0,
                 position_tau=0.07, rotation_tau=0.10):
    """SO(3) interpolation plus bounded increments; never average Euler angles."""
    previous, requested = rigid_pose(previous), rigid_pose(requested)
    values = (dt, speed, angular_speed, position_tau, rotation_tau)
    if not all(math.isfinite(v) and v > 0 for v in values):
        raise ValueError('filter timing/rates must be finite and positive')
    dt = min(dt, 0.05)  # No large catch-up jump after a stalled callback.
    result = previous.copy()
    delta = (requested[:3, 3] - previous[:3, 3]) * (-math.expm1(-dt / position_tau))
    norm = np.linalg.norm(delta)
    if norm > speed * dt:
        delta *= speed * dt / norm
    result[:3, 3] += delta
    rotvec = Rotation.from_matrix(requested[:3, :3] @ previous[:3, :3].T).as_rotvec()
    rotvec *= -math.expm1(-dt / rotation_tau)
    angle = np.linalg.norm(rotvec)
    if angle > angular_speed * dt:
        rotvec *= angular_speed * dt / angle
    result[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix() @ previous[:3, :3]
    return rigid_pose(result)


class ContinuousElbowHint:
    """Two-segment geometric hint for the existing QP's elbow position task.

    [ADDED] Uses measured SG2 shoulder/elbow/wrist, not inferred human elbow or
    H1/H2 lengths. This is a continuity hint, NOT exact 7-DoF IK nor a proof of
    reachability under wrist orientation/collision/joint limits. Cyclo retains
    all those constraints. The nearest previous bend plane is preserved.
    """

    def __init__(self, shoulder, elbow, wrist):
        self.shoulder = self._point(shoulder)
        self.previous = self._point(elbow)
        wrist = self._point(wrist)
        self.upper = float(np.linalg.norm(self.previous - self.shoulder))
        self.forearm = float(np.linalg.norm(wrist - self.previous))
        if not 0.05 < self.upper < 0.8 or not 0.05 < self.forearm < 0.8:
            raise ValueError('invalid measured SG2 segment length')
        d0 = np.linalg.norm(wrist - self.shoulder)
        self.minimum = abs(self.upper - self.forearm) + 0.01
        if d0 < self.minimum:
            raise ValueError('captured wrist is too close to the shoulder; use a non-folded startup pose')
        # Conservative outer shell, preserving the captured pose exactly.
        self.maximum = max(d0, 0.94 * (self.upper + self.forearm))
        self.maximum = min(self.maximum, self.upper + self.forearm - 1e-5)
        if d0 > self.maximum:
            raise ValueError('captured arm is fully straight/singular; bend the elbow before calibration')
        self.last_bend = None

    @staticmethod
    def _point(value):
        p = np.asarray(value, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError('point must be finite xyz')
        return p.copy()

    def project_wrist(self, wrist):
        wrist = self._point(wrist)
        offset = wrist - self.shoulder
        length = np.linalg.norm(offset)
        if length < 1e-6:
            raise ValueError('wrist coincides with shoulder')
        bounded = float(np.clip(length, self.minimum, self.maximum))
        return self.shoulder + offset * (bounded / length), abs(length - bounded)

    def elbow(self, wrist):
        wrist = self._point(wrist)
        direction = wrist - self.shoulder
        distance = np.linalg.norm(direction)
        if not self.minimum - 1e-6 <= distance <= self.maximum + 1e-6:
            raise ValueError('project wrist to geometric shell before elbow hint')
        direction /= distance
        axial = (self.upper**2 - self.forearm**2 + distance**2) / (2 * distance)
        height = math.sqrt(max(0., self.upper**2 - axial**2))
        bend = self.previous - self.shoulder
        bend -= direction * np.dot(direction, bend)
        if np.linalg.norm(bend) < 1e-6 and self.last_bend is not None:
            bend = self.last_bend - direction * np.dot(direction, self.last_bend)
        if np.linalg.norm(bend) < 1e-6:
            axis = np.eye(3)[np.argmin(np.abs(direction))]
            bend = axis - direction * np.dot(direction, axis)
        bend /= np.linalg.norm(bend)
        self.last_bend = bend.copy()
        self.previous = self.shoulder + axial * direction + height * bend
        return self.previous.copy()
