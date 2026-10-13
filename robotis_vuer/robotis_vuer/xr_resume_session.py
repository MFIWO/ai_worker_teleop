"""[ADDED] Fixed-calibration resume policy; pure math, no ROS or actuation.

Safety stops must not redefine the correspondence between a human controller
pose and a robot wrist pose. Only creation of a NEW session calibrates them.
The legacy reclutch mapper and its limits remain unchanged in xr_arm_targets.
"""
from copy import deepcopy

import numpy as np
from scipy.spatial.transform import Rotation

from robotis_vuer.xr_arm_targets import CalibratedWristMapper, ContinuousElbowHint


class FixedResumeSession:
    """Preserve the original heading, wrist anchors and radial bounds.

    On resume only temporary elbow-plane continuation is seeded from measured
    elbows. Segment lengths and original target mapping are NOT recaptured.
    """

    def __init__(self, head, inputs, robots, links, scale):
        self.mapper = CalibratedWristMapper(head, inputs, robots, scale)
        self.templates = {
            side: ContinuousElbowHint(links[side]['shoulder'][:3, 3],
                                      links[side]['elbow'][:3, 3], robots[side][:3, 3])
            for side in ('left', 'right')
        }

    def inspect(self, inputs, robots):
        """Raw request BEFORE projection, measured error, and saved bounds."""
        result = {}
        for side, hint in self.templates.items():
            requested = self.mapper.target(side, inputs[side])
            projected, clipped = hint.project_wrist(requested[:3, 3])
            actual = robots[side]
            delta = requested[:3, 3] - actual[:3, 3]
            result[side] = {
                'align_position_m': float(np.linalg.norm(delta)),
                'align_rotation_deg': float(np.rad2deg(Rotation.from_matrix(
                    requested[:3, :3] @ actual[:3, :3].T).magnitude())),
                'raw_minus_measured_xyz_m': delta.tolist(),
                'raw_goal_xyz_m': requested[:3, 3].tolist(),
                'projected_goal_xyz_m': projected.tolist(),
                'raw_radius_m': float(np.linalg.norm(requested[:3, 3] - hint.shoulder)),
                'projection_m': float(clipped),
                'shell_min_m': float(hint.minimum),
                'shell_max_m': float(hint.maximum),
            }
        return result

    def require_alignment(self, inputs, robots, position_m=0.02, rotation_deg=10.0):
        """A fresh X+A is not permission to jump, chase a stale goal or reclutch.

        Comparison uses UNPROJECTED targets. Clipping a far-away human hand
        onto the current robot pose must not make a restart appear aligned.
        """
        result = self.inspect(inputs, robots)
        bad = {s: x for s, x in result.items()
               if x['align_position_m'] > position_m or x['align_rotation_deg'] > rotation_deg
               or x['projection_m'] > 1e-5}
        if bad:
            detail = ', '.join(
                f"{s}: {x['align_position_m'] * 1000:.1f}mm/"
                f"{x['align_rotation_deg']:.1f}deg, clipped={x['projection_m'] * 1000:.1f}mm"
                for s, x in bad.items())
            raise ValueError('ALIGN_REQUIRED (original calibration retained): ' + detail)
        return result

    def fresh_hints(self, links):
        hints = deepcopy(self.templates)
        for side, hint in hints.items():
            # The bridge is arms-only. A moved shoulder/base/lift invalidates
            # this session rather than silently redefining its workspace.
            if np.linalg.norm(links[side]['shoulder'][:3, 3] - hint.shoulder) > 0.005:
                raise ValueError('shoulder/base moved; restart publisher for a new calibration')
            hint.previous = links[side]['elbow'][:3, 3].copy()
            hint.last_bend = None
        return hints
