"""Independent HX5 thumb MCP/IP flexion from the four tracked thumb points."""
import numpy as np


def thumb_flexion(points, side):
    """Return physical joint3/4 radians. Both 21 and 25 layouts start 0,1..4.

    Thumb points are CMC, MCP, IP, tip. Distances/rigid wrist rotations do not
    affect the two local bend angles. HX5 left flexes negative, right positive.
    """
    if side not in ('left', 'right'):
        raise ValueError('Expected left or right hand')
    points = np.asarray(points, dtype=float)
    if points.shape not in ((21, 3), (25, 3)) or not np.isfinite(points).all():
        raise ValueError('Expected finite 21 or 25 hand landmarks')
    bones = np.diff(points[1:5], axis=0)
    lengths = np.linalg.norm(bones, axis=1)
    if np.any(lengths < 1e-6):
        raise ValueError('Degenerate thumb bones')
    bones /= lengths[:, None]
    angles = np.arccos(np.clip(np.sum(bones[:-1]*bones[1:], axis=1), -1., 1.))
    return np.clip(angles, 0., 1.57) * (-1. if side == 'left' else 1.)


def apply_thumb_flexion(q, points, side):
    result = np.array(q, dtype=float, copy=True)
    if result.shape != (20,) or not np.isfinite(result).all():
        raise ValueError('Expected finite twenty-joint hand goal')
    result[2:4] = thumb_flexion(points, side)
    return result
