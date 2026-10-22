"""[ADDED 2026-10-14] Captured-wrist-ray workspace limiting, not radial warping.

Legacy shoulder-centred sphere projection changes ALL Cartesian components
when one axis exceeds reach. Here a raw segment from the immutable captured
wrist is shortened by ONE scalar. Straight rays out/back therefore stay on
the same line. This is not an IK reachability proof or an arbitrary-path planner.
"""
from copy import deepcopy
import math
import numpy as np

from robotis_vuer.xr_arm_targets import ContinuousElbowHint


def clip_segment_to_shell(start, requested, shoulder, minimum, maximum):
    """Return the longest initial segment staying in the existing spherical shell.

    Stop at the FIRST inner-sphere entry, even if the endpoint exits its far
    side. Never teleport through the hole or bend a segment around the sphere.
    The outer ball is convex; numerical tolerance does not expand its radius.
    """
    start, requested, shoulder = (ContinuousElbowHint._point(x)
                                  for x in (start, requested, shoulder))
    if not all(math.isfinite(x) for x in (minimum, maximum)) or not 0 < minimum < maximum:
        raise ValueError('invalid workspace shell')
    offset, direction = start - shoulder, requested - start
    radius = float(np.linalg.norm(offset))
    if not minimum - 1e-8 <= radius <= maximum + 1e-8:
        raise ValueError('segment start outside saved workspace; do not silently reanchor')
    aa = float(direction @ direction)
    if not math.isfinite(aa):
        raise ValueError('workspace segment overflow')
    if aa < 1e-24:
        return start.copy(), 1.0
    bb = float(offset @ direction)
    alpha = 1.0
    # aa*t*t + 2*bb*t + cc = 0; choose the outgoing outer intersection.
    cc = float(offset @ offset) - maximum * maximum
    outer = (-bb + math.sqrt(max(0., bb*bb - aa*cc))) / aa
    alpha = min(alpha, max(0., outer))
    # For an entering ray, two roots enclose the forbidden inner interval.
    cc_inner = float(offset @ offset) - minimum * minimum
    discriminant = bb*bb - aa*cc_inner
    if bb < 0. and discriminant > 0.:
        first = (-bb - math.sqrt(discriminant)) / aa
        second = (-bb + math.sqrt(discriminant)) / aa
        if second > 0. and first < alpha:
            alpha = min(alpha, max(0., first))
    alpha = float(np.clip(alpha, 0., 1.))
    result = requested.copy() if alpha == 1.0 else start + alpha * direction
    radius_out = float(np.linalg.norm(result - shoulder))
    if not minimum - 1e-8 <= radius_out <= maximum + 1e-8:
        raise ValueError('workspace intersection numerical failure')
    return result, alpha


class AnchorRayHint(ContinuousElbowHint):
    """Same measured geometry/elbow continuation; only wrist projection changes."""

    @classmethod
    def from_hint(cls, hint, anchor):
        instance = cls.__new__(cls)
        instance.__dict__.update(deepcopy(hint.__dict__))
        instance.path_anchor = cls._point(anchor)
        clip_segment_to_shell(instance.path_anchor, instance.path_anchor,
                              instance.shoulder, instance.minimum, instance.maximum)
        return instance

    def project_wrist(self, wrist):
        requested = self._point(wrist)
        projected, _ = clip_segment_to_shell(
            self.path_anchor, requested, self.shoulder, self.minimum, self.maximum)
        return projected, float(np.linalg.norm(requested - projected))

    def limit_filtered_step(self, previous, filtered):
        # [FIX] Do not re-project a filtered point from the shoulder. Stop on
        # the same finite step instead; its speed bound can only get smaller.
        previous, filtered = self._point(previous), self._point(filtered)
        candidate, shell_fraction = clip_segment_to_shell(
            previous, filtered, self.shoulder, self.minimum, self.maximum)
        # [FIX] A shell-safe chord can enter the inner sphere's SHADOW viewed
        # from the immutable anchor. Such a point cannot be resumed under this
        # mapping. Split the step at tangent-cone roots and keep only its first
        # anchor-visible prefix. This also covers valid endpoints whose middle
        # is occluded. No sampling-only collision/visibility approximation.
        a = self.path_anchor - self.shoulder
        d = previous - self.path_anchor
        v = candidate - previous
        k = float(a @ a) - self.minimum**2
        av, ad = float(a @ v), float(a @ d)
        coefficients = [k*float(v@v)-av*av,
                        2*(k*float(d@v)-ad*av),
                        k*float(d@d)-ad*ad]
        # [FIX] Absolute trimming can erase real roots of a micrometre-scale
        # step near the inner boundary. Scale the polynomial first and discard
        # only EXACT zero leading terms (tiny nonzero leading terms matter).
        scale=max(abs(c) for c in coefficients)
        if scale>0.:
            coefficients=[c/scale for c in coefficients]
        while len(coefficients)>1 and coefficients[0]==0.:
            coefficients.pop(0)
        cuts = [0.,1.]
        if len(coefficients)>1:
            cuts += [float(root.real) for root in np.roots(coefficients)
                     if abs(root.imag)<1e-9 and 0<root.real<1]
        cuts = sorted(set(cuts))
        def visible(point):
            projected, _ = self.project_wrist(point)
            return np.linalg.norm(projected-point) <= 1e-8
        if not visible(previous):
            raise ValueError('previous filtered goal outside anchor-visible workspace')
        fraction = 1.
        for lo,hi in zip(cuts,cuts[1:]):
            if not visible(previous + .5*(lo+hi)*v):
                fraction = max(0., lo-1e-9)
                break
        result = previous + fraction*v
        if not visible(result):
            raise ValueError('anchor-visible step intersection failed')
        return result, shell_fraction*fraction
