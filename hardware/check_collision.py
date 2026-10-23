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


class AuxiliaryCollisionGuard:
    """Cached FK/distance check for keyboard head/lift targets at measured arms."""
    def __init__(self, profile):
        self.model = pin.buildModelFromUrdf(profile['urdf'])
        self.live_wheels = bool(profile.get('live_wheel_feedback'))
        expected_dof = 63 if self.live_wheels else 57
        if self.model.nq != self.model.nv or self.model.nq != expected_dof:
            raise ValueError(f'Manual SH5 controls require the {expected_dof} scalar-joint model')
        self.data = self.model.createData()
        self.geom = pin.buildGeomFromUrdf(self.model, profile['urdf'], pin.GeometryType.COLLISION)
        self.geom.addAllCollisionPairs()
        pin.removeCollisionPairs(self.model, self.geom, profile['srdf'])
        self.geom_data = self.geom.createData()
        # Refine only a possible auxiliary collision using the original meshes.
        # Native arm QP keeps its conservative boxes and all enabled pairs.
        self.exact = pin.buildGeomFromUrdf(self.model, profile['manual_collision_urdf'], pin.GeometryType.COLLISION)
        self.exact.addAllCollisionPairs()
        pin.removeCollisionPairs(self.model, self.exact, profile['srdf'])
        if ([o.name for o in self.exact.geometryObjects] != [o.name for o in self.geom.geometryObjects] or
                [(p.first,p.second) for p in self.exact.collisionPairs] !=
                [(p.first,p.second) for p in self.geom.collisionPairs]):
            raise ValueError('Exact auxiliary model differs from conservative collision topology')
        self.exact_data = self.exact.createData()
        def ancestors(index):
            result=set()
            while index:
                result.add(index)
                index=int(self.model.parents[index])
            return result
        self.pair_joints = [ancestors(self.geom.geometryObjects[p.first].parentJoint) ^
                            ancestors(self.geom.geometryObjects[p.second].parentJoint)
                            for p in self.geom.collisionPairs]
        if not self.pair_joints:raise ValueError('Manual collision model has no pairs')

    def validate(self, measured, targets):
        if not targets:return
        positions=dict(measured, **targets)
        moving={self.model.getJointId(n) for n,v in targets.items() if abs(v-measured[n])>1e-12}
        q=pin.neutral(self.model)
        from hardware.sh5_hands_model import scalar_sensor_position
        for i in range(1,self.model.njoints):
            name = self.model.names[i]
            q[self.model.joints[i].idx_q]=scalar_sensor_position(name,positions[name]) if self.live_wheels else positions[name]
        if not np.isfinite(q).all():raise ValueError('Nonfinite manual collision state')
        pin.computeDistances(self.model,self.data,self.geom,self.geom_data,q)
        candidates=[]
        for i,joints in enumerate(self.pair_joints):
            if not moving & joints:continue
            distance=self.geom_data.distanceResults[i].min_distance
            if not np.isfinite(distance):raise ValueError('Nonfinite manual collision distance')
            if distance<=0:candidates.append(i)
        if candidates:
            pin.updateGeometryPlacements(self.model,self.data,self.exact,self.exact_data,q)
            for i in candidates:
                distance=pin.computeDistance(self.exact,self.exact_data,i).min_distance
                if not np.isfinite(distance) or distance<=0:
                    raise ValueError('Manual head/lift target has original-mesh contact/penetration; hold and inspect')


def check_initial_pose(urdf, srdf, snapshot, minimum_distance, *, enforce_margin=True, measured_fingers=False, measured_head=False, measured_wheels=False):
    """[CHANGED] Distinguish native CBF margin from an extra startup predicate.

    In normal operation positive clearance below the CBF target is reported;
    the unchanged native QP still enforces its collision constraints. Actual
    contact/penetration, missing geometry and nonfinite inputs are rejected.
    This does not certify a path or start any recovery movement.
    """
    if not np.isfinite(minimum_distance) or minimum_distance <= 0:
        raise ValueError('native collision margin must be finite and positive')
    model = pin.buildModelFromUrdf(str(urdf))
    expected_dof = (57 if measured_head else 55) if measured_fingers else 15
    if measured_wheels:expected_dof += 6
    if model.nq != model.nv or model.nq != expected_dof:
        raise ValueError(f'Cyclo control model must have {expected_dof} scalar DOFs')
    if measured_fingers:
        from hardware.sh5_hands_model import CONTROLLER_JOINTS, HEAD_JOINTS, WHEEL_JOINTS
        expected = CONTROLLER_JOINTS | HEAD_JOINTS if measured_head else CONTROLLER_JOINTS
        if measured_wheels:expected |= WHEEL_JOINTS
        if set(model.names[1:]) != expected:
            raise ValueError('Unexpected joints in measured-finger model')
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
        value = float(snapshot[name])
        if measured_wheels:
            from hardware.sh5_hands_model import scalar_sensor_position
            value = scalar_sensor_position(name,value)
        q[model.joints[joint_id].idx_q] = value
    if not np.isfinite(q).all():
        raise ValueError('nonfinite measured model joint')
    pin.computeDistances(model, data, geom, geom_data, q)
    arm_ids = {model.getJointId(name) for names in ARM_NAMES.values() for name in names}
    # Startup concerns pairs affected by arm motion. Like the measured fingers,
    # measured head joints have zero QP velocity. Their own motion is checked
    # separately by AuxiliaryCollisionGuard, including original-mesh refinement.
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
    if not moving or not all(np.isfinite(p['distance_m']) for p in moving + static):
        raise ValueError('missing/nonfinite collision-distance result')
    report = {'arm_movable_pairs':len(moving),'static_pairs':len(static),
              'nearest_arm_movable':moving[:8],'nearest_static':static[:8],
              'minimum_required_m':float(minimum_distance),
              'native_margin_met': moving[0]['distance_m'] >= minimum_distance,
              'startup_policy': 'strict_margin' if enforce_margin else 'native_cbf'}
    # [CHANGED] Never turn an actual modeled collision into a warning.
    if moving[0]['distance_m'] <= 0:
        raise ValueError('Initial measured pose has contact/penetration: '+json.dumps(report))
    if enforce_margin and not report['native_margin_met']:
        raise ValueError('Initial measured pose violates native collision clearance: '+json.dumps(report)+
                         '; no controller started. Use the established robot ready-pose procedure and recapture.')
    return report
