"""[ADDED] Pure geometry: no ROS publishers, simulator, or physical commands."""
from copy import deepcopy
import unittest
import numpy as np

from robotis_vuer.xr_arm_targets import ContinuousElbowHint, limited_pose
from robotis_vuer.xr_path_workspace import AnchorRayHint, clip_segment_to_shell


class WorkspacePathTests(unittest.TestCase):
    def setUp(self):
        self.center = np.zeros(3)
        self.anchor = np.array([.30, 0., -.30])
        self.old = ContinuousElbowHint(self.center, [0.,0.,-.30], self.anchor)
        self.old.maximum = .595
        self.hint = AnchorRayHint.from_hint(self.old, self.anchor)

    def test_interior_exact_identity(self):
        p = np.array([.40, .03, -.30])
        got, cut = self.hint.project_wrist(p)
        np.testing.assert_allclose(got, p, atol=1e-15)
        self.assertEqual(cut, 0.)

    def test_straight_extension_return_has_no_lateral_warp(self):
        previous = np.eye(4); previous[:3,3] = self.anchor
        old_warp = []
        for x in np.r_[np.linspace(.3,.9,200), np.linspace(.9,.3,200)]:
            raw = np.array([x,0.,-.3])
            requested = previous.copy()
            requested[:3,3], _ = self.hint.project_wrist(raw)
            candidate = limited_pose(previous, requested, 1/30)
            candidate[:3,3], _ = self.hint.limit_filtered_step(previous[:3,3],candidate[:3,3])
            self.assertAlmostEqual(candidate[2,3], -.3, places=12)
            self.assertAlmostEqual(candidate[1,3], 0., places=12)
            self.assertLessEqual(np.linalg.norm(candidate[:3,3]-previous[:3,3]), .2/30+1e-10)
            previous = candidate
            projected, _ = self.old.project_wrist(raw)
            old_warp.append(abs(projected[2]+.3))
        self.assertGreater(max(old_warp), .10)

    def test_same_raw_point_identical_on_reverse(self):
        points = [self.anchor+np.array([d,.12*d,.03*d]) for d in np.linspace(0,.8,30)]
        forward = [self.hint.project_wrist(p)[0] for p in points]
        self.hint.previous += [.03,.02,0.]  # Elbow continuation cannot affect the ray.
        reverse = [self.hint.project_wrist(p)[0] for p in points[::-1]][::-1]
        np.testing.assert_array_equal(forward, reverse)

    def test_overhead_displacement_over_half_meter_is_not_rejected(self):
        requested = np.array([.3,0.,.35])
        projected, cut = self.hint.project_wrist(requested)
        self.assertGreater(np.linalg.norm(requested-self.anchor),.5)
        np.testing.assert_allclose(projected,requested,atol=1e-15)
        self.assertEqual(cut,0.)

    def test_inner_sphere_crossing_stops_at_first_entry(self):
        p,alpha = clip_segment_to_shell([.3,0,0],[-.3,0,0],[0,0,0],.1,.6)
        np.testing.assert_allclose(p,[.1,0,0],atol=1e-12)
        self.assertAlmostEqual(alpha,1/3)

    def test_boundary_outward_holds_inward_returns(self):
        p,a = clip_segment_to_shell([.6,0,0],[.7,0,0],[0,0,0],.1,.6)
        np.testing.assert_allclose(p,[.6,0,0],atol=1e-12)
        self.assertAlmostEqual(a,0.)
        p,a = clip_segment_to_shell([.6,0,0],[.3,0,0],[0,0,0],.1,.6)
        np.testing.assert_allclose(p,[.3,0,0],atol=1e-12)
        self.assertAlmostEqual(a,1.)
        p,a = clip_segment_to_shell([.1,0,0],[0,0,0],[0,0,0],.1,.6)
        np.testing.assert_allclose(p,[.1,0,0],atol=1e-12)

    def test_segment_randomized_stays_in_shell_and_on_line(self):
        rng=np.random.default_rng(14)
        for _ in range(250):
            start=rng.normal(size=3); start *= rng.uniform(.08,.6)/np.linalg.norm(start)
            end=rng.normal(size=3)
            p,a=clip_segment_to_shell(start,end,np.zeros(3),.05,.65)
            np.testing.assert_allclose(p,start+a*(end-start),atol=1e-12)
            samples=start+np.linspace(0,a,50)[:,None]*(end-start)
            radii=np.linalg.norm(samples,axis=1)
            self.assertGreaterEqual(radii.min(),.05-1e-8)
            self.assertLessEqual(radii.max(),.65+1e-8)

    def test_legacy_bounds_and_anchor_not_mutated(self):
        saved=deepcopy(self.old.__dict__)
        for p in ([1,0,1],[.1,0,.2],[.3,0,-.3]):
            self.hint.project_wrist(p)
        for k,v in saved.items():
            np.testing.assert_equal(v,self.old.__dict__[k])
        np.testing.assert_array_equal(self.hint.path_anchor,self.anchor)

    def test_filtered_chord_cannot_enter_anchor_shadow(self):
        hint=deepcopy(self.hint)
        hint.path_anchor=np.array([.4,0.,0.]); hint.minimum=.1; hint.maximum=.8
        before=np.array([-.4,.4,0.]); after=np.array([-.4,-.4,0.])
        self.assertLess(hint.project_wrist(before)[1],1e-8)
        self.assertLess(hint.project_wrist(after)[1],1e-8)
        limited,fraction=hint.limit_filtered_step(before,after)
        self.assertLess(fraction,1.)
        for t in np.linspace(0,1,80):
            p=before+t*(limited-before)
            self.assertLess(hint.project_wrist(p)[1],1e-7)

    def test_invalid_input_fails_closed(self):
        for start,end in (([.7,0,0],[.5,0,0]),([0,0,0],[.2,0,0]),([.3,0,0],[float('nan'),0,0])):
            with self.assertRaises(ValueError):
                clip_segment_to_shell(start,end,[0,0,0],.1,.6)

    def test_micrometre_step_keeps_small_tangent_polynomial_roots(self):
        # Analytic narrow shadow at t in(.1,.3): midpoint and endpoint alone
        # would miss it if tiny but NONZERO quadratic coefficients were erased.
        hint=deepcopy(self.hint)
        a=.100001; h=1e-7; x=.099996; k=a*a-.1*.1; dx=x-a
        z=np.sqrt(.1**2*dx**2/k-h*h)
        hint.path_anchor=np.array([a,0,0]); hint.minimum=.1; hint.maximum=.8
        start=np.array([x,-2*h,z]); end=np.array([x,8*h,z])
        self.assertLess(hint.project_wrist(start)[1],1e-8)
        self.assertLess(hint.project_wrist(end)[1],1e-8)
        self.assertGreater(hint.project_wrist(start+.2*(end-start))[1],1e-5)
        result,fraction=hint.limit_filtered_step(start,end)
        self.assertLess(fraction,.101)
        self.assertLess(hint.project_wrist(result)[1],1e-8)


if __name__=='__main__':
    unittest.main()
