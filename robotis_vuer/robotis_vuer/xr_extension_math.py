"""[ADDED] Near-extension experiment helpers, independent of ROS and actuation."""
from copy import deepcopy
import math
import numpy as np

EXTENSION_FRACTION = 0.995
EXPERIMENT_Q4_UPPER = -0.45


def bend_degrees(shoulder, elbow, endpoint):
    """S->E versus E->P chord bend: 0=collinear/straight, NOT raw joint4 angle."""
    points = [np.asarray(p, dtype=float) for p in (shoulder, elbow, endpoint)]
    if any(p.shape != (3,) or not np.isfinite(p).all() for p in points):
        raise ValueError('Geometric elbow points must be finite xyz')
    u, f = points[1]-points[0], points[2]-points[1]
    if min(np.linalg.norm(u), np.linalg.norm(f)) < 1e-5:
        raise ValueError('Degenerate elbow segment')
    return float(np.degrees(np.arctan2(np.linalg.norm(np.cross(u, f)), np.dot(u, f))))


def extended_templates(templates, fraction=EXTENSION_FRACTION):
    """Copy the calibrated shell; do not recapture/map hands or change heading.

    This enlarges a geometric *target* bound, not an exact reachable set for
    every orientation. The unchanged QP, measured-following and speed guards
    remain authoritative. Keep a nonzero straight-arm singularity margin.
    """
    if not math.isfinite(fraction) or not 0.94 < fraction <= EXTENSION_FRACTION:
        raise ValueError('Extension fraction must be in (0.94, 0.995]')
    result = deepcopy(templates)
    for hint in result.values():
        new_max = max(hint.maximum, fraction*(hint.upper+hint.forearm))
        hint.maximum = min(new_max, hint.upper+hint.forearm-1e-5)
    return result
