"""SH5 optical finger geometry, following the original ROBOTIS Vuer mapping.

Pure functions here are independently testable without ROS, WebXR or motors.
HAND_MOVE supplies 25 column-major 4x4 poses, of which the original SH5
publisher selects 21. Output is 21 wrist-local, URDF-axis Point32 positions;
the SH5 hand retargeter consumes these points, rather than finger angles.
"""

import numpy as np


# [ADDED SH5 hybrid] Exact original vr_publisher_sh5.required_vr_frames order.
SH5_HAND_FRAMES = (0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14,
                   16, 17, 18, 19, 21, 22, 23, 24)
VR_HAND_TO_SH5 = np.array([[0., -1., 0.], [-1., 0., 0.],
                            [0., 0., -1.]], dtype=float)


def parse_hand_poses(data):
    """Reject partial/nonrigid hand frames before any ROS publication."""
    try:
        raw = np.asarray(data, dtype=np.float64)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError('hand input must contain 25 finite 4x4 poses') from error
    if raw.size != 400 or not np.isfinite(raw).all():
        raise ValueError('hand input must contain 400 finite values')
    poses = raw.reshape(25, 4, 4, order='C') if raw.shape == (25, 4, 4) else None
    if poses is None:
        poses = np.stack([raw.reshape(-1)[i*16:(i+1)*16].reshape(4, 4, order='F')
                          for i in range(25)])
    selected = poses[list(SH5_HAND_FRAMES)]
    if not np.allclose(selected[:, 3, :], [0., 0., 0., 1.], atol=1e-3):
        raise ValueError('hand transform has invalid homogeneous row')
    rotations = selected[:, :3, :3]
    ortho = rotations @ np.swapaxes(rotations, 1, 2)
    if (not np.allclose(ortho, np.eye(3), atol=2e-2)
            or not np.allclose(np.linalg.det(rotations), 1., atol=2e-2)):
        raise ValueError('hand transform has invalid rotation')
    positions = selected[:, :3, 3]
    return positions, rotations[0]


def retarget_sh5_hand(data, head_world, previous=None, alpha=.9):
    """Return (21,3) wrist-local points and current smoothed head-frame samples.

    [ADDED SH5 hybrid] Same operation order as original SH5 process_hand_joints:
    world -> head-relative points -> low-pass -> subtract wrist -> wrist-local
    -> original vr_hand_to_urdf. No arm wrist pose is published from this path.
    """
    positions, wrist_world_rotation = parse_hand_poses(data)
    head = np.asarray(head_world, dtype=np.float64)
    if head.shape != (4, 4) or not np.isfinite(head).all():
        raise ValueError('missing finite head transform')
    if (not np.allclose(head[3], [0., 0., 0., 1.], atol=1e-3)
            or not np.allclose(head[:3, :3].T @ head[:3, :3], np.eye(3), atol=2e-2)
            or not np.isclose(np.linalg.det(head[:3, :3]), 1., atol=2e-2)):
        raise ValueError('invalid head rigid transform')
    if not np.isfinite(alpha) or not 0. < alpha <= 1.:
        raise ValueError('invalid hand low pass alpha')
    head_rotation_inv = head[:3, :3].T
    relative = (head_rotation_inv @ (positions - head[:3, 3]).T).T
    wrist_rotation = head_rotation_inv @ wrist_world_rotation
    if previous is not None:
        old = np.asarray(previous, dtype=np.float64)
        if old.shape != (21, 3) or not np.isfinite(old).all():
            raise ValueError('invalid previous hand filter state')
        relative = alpha * relative + (1. - alpha) * old
    fingers = (VR_HAND_TO_SH5 @ (wrist_rotation.T @
               (relative - relative[0]).T)).T
    if not np.isfinite(fingers).all():
        raise ValueError('nonfinite SH5 retarget points')
    return fingers, relative

