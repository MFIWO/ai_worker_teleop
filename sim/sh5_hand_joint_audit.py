"""Offline SH5 D20 finger joint-axis audit. Never commands a real robot.

Finger abduction is at the MCP splay joint, NOT the PIP hinge. The STL name
"finger_pip" refers to a geometry segment, not a roll-axis joint.
"""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path


def audit(urdf):
    root = ET.parse(urdf).getroot()
    result = {}
    for side in ('l', 'r'):
        joints = {}
        for i in range(1, 21):
            name = f'finger_{side}_joint{i}'
            matched = [j for j in root.findall('joint') if j.get('name') == name and j.find('axis') is not None]
            if len(matched) != 1:
                raise ValueError(f'{name}: expected exactly one actuated URDF joint')
            joint = matched[0]
            bounds = joint.find('limit')
            if bounds is None:
                raise ValueError(f'{name}: missing mechanical limits')
            axis = joint.find('axis').get('xyz')
            joints[name] = {'axis': axis, 'lower': float(bounds.get('lower')),
                            'upper': float(bounds.get('upper')),
                            'role': ('thumb_base' if i <= 4 else
                                     'mcp_splay' if i in (5, 9, 13, 17) else
                                     'flexion')}
        result[side] = joints
    for side, joints in result.items():
        for i in (5, 9, 13, 17):
            j = joints[f'finger_{side}_joint{i}']
            if j['axis'] != '1 0 0' or not (j['lower'] < 0 < j['upper']):
                raise ValueError('unexpected MCP splay axis/range')
        for i in (7, 8, 11, 12, 15, 16, 19, 20):
            if joints[f'finger_{side}_joint{i}']['axis'] != '0 1 0':
                raise ValueError('unexpected PIP/DIP flexion axis')
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    result = audit(a.urdf)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(result, indent=2))
    print('[SH5 D20] MCP splay joints are 5,9,13,17 per hand; PIP is flexion, not roll')
