"""[ADDED] Passive initial-pose collision check using the generated Cyclo model.

Checks enabled pairs whose relative pose can change with an arm joint. Static
pairs (e.g. the frozen lift cover) are still reported and remain in the native
QP; they are not silently removed from the collision model.
"""
import json
from pathlib import Path

import numpy as np
import pinocchio as pin

from hardware.xr_contract import ARM_NAMES


def check_initial_pose(urdf, srdf, snapshot, minimum_distance):
    model = pin.buildModelFromUrdf(str(urdf))
    if model.nq != model.nv or model.nq != 15:
        raise ValueError('Cyclo control model must have15 scalar DOFs, no continuous wheels/fingers')
    data = model.createData()
    geom = pin.buildGeomFromUrdf(model, str(urdf), pin.GeometryType.COLLISION)
    geom.addAllCollisionPairs()
    pin.removeCollisionPairs(model, geom, str(srdf))
    geom_data = geom.createData()
    q = pin.neutral(model)
    for joint_id in range(1, model.njoints):
        name = model.names[joint_id]
        if name not in snapshot:
            raise ValueError('missing measured model joint: '+name)
        q[model.joints[joint_id].idx_q] = float(snapshot[name])
    pin.computeDistances(model, data, geom, geom_data, q)
    arm_ids = {model.getJointId(name) for names in ARM_NAMES.values() for name in names}
    def ancestry(joint_id):
        result = set()
        while joint_id:
            result.add(joint_id)
            joint_id = int(model.parents[joint_id])
        return result
    moving, static = [], []
    for pair, distance in zip(geom.collisionPairs, geom_data.distanceResults):
        a, b = geom.geometryObjects[pair.first], geom.geometryObjects[pair.second]
        item = {'distance_m': float(distance.min_distance), 'first': a.name, 'second': b.name}
        relative_arm_joints = (ancestry(a.parentJoint) ^ ancestry(b.parentJoint)) & arm_ids
        (moving if relative_arm_joints else static).append(item)
    moving.sort(key=lambda p:p['distance_m']);static.sort(key=lambda p:p['distance_m'])
    if not moving or not all(np.isfinite(p['distance_m']) for p in moving):
        raise ValueError('missing/nonfinite collision-distance result')
    report = {'arm_movable_pairs':len(moving),'static_pairs':len(static),
              'nearest_arm_movable':moving[:8],'nearest_static':static[:8],
              'minimum_required_m':float(minimum_distance)}
    if moving[0]['distance_m'] < minimum_distance:
        raise ValueError('Initial measured pose violates native collision clearance: '+json.dumps(report)+
                         '; no controller started. Use the established robot ready-pose procedure and recapture.')
    return report
