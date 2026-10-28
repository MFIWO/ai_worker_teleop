import unittest
import numpy as np
from hardware.arm_posture import operating_limits,apply_limits
from hardware.base_collision_cover import triangle_cover

class PostureTests(unittest.TestCase):
    def test_operating_bounds_only_narrow_elbows(self):
        physical={f'arm_{s}_joint{i}':[-2.9,1.0786 if i==4 else 3.14] for s in ('l','r') for i in range(1,8)}
        result=operating_limits(physical)
        for name,value in result.items():
            self.assertEqual(value[0],physical[name][0])
            self.assertEqual(value[1],-.45 if name.endswith('joint4') else physical[name][1])
        self.assertEqual(physical['arm_l_joint4'][1],1.0786)

    def test_triangle_cover_preserves_faces_and_leaves_empty_gap(self):
        triangles=np.array([[[0,0,0],[0,.3,0],[0,0,.3]],[[1,0,0],[1,.3,0],[1,0,.3]]],float)
        boxes=triangle_cover(triangles)
        for tri in triangles:
            for u in np.linspace(0,1,13):
                for v in np.linspace(0,1-u,13):
                    point=tri[0]*(1-u-v)+tri[1]*u+tri[2]*v
                    self.assertTrue(any(np.all(point>=a) and np.all(point<=b) for a,b in boxes))
        point=np.array([.5,.1,.1])
        self.assertFalse(any(np.all(point>=a) and np.all(point<=b) for a,b in boxes))

    def test_invalid_cover_fails(self):
        for data in ([],np.full((1,3,3),np.nan)):
            with self.assertRaises(ValueError):triangle_cover(data)

if __name__=='__main__':unittest.main()
