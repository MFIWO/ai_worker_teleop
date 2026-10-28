"""SH5 operational limits strictly inside physical bounds; no actuation."""
from copy import deepcopy
import math
import xml.etree.ElementTree as ET

ELBOW_UPPER = -.45
POLICY = 'sh5_bent_elbow_v1'

def operating_limits(physical):
    result=deepcopy(physical)
    for side in ('l','r'):
        name=f'arm_{side}_joint4'
        lower,upper=map(float,result[name])
        if not all(map(math.isfinite,(lower,upper))) or lower>=min(upper,ELBOW_UPPER):
            raise ValueError('invalid physical elbow bounds')
        result[name]=[lower,min(upper,ELBOW_UPPER)]
    return result

def apply_limits(text,physical):
    limits=operating_limits(physical)
    root=ET.fromstring(text)
    for side in ('l','r'):
        name=f'arm_{side}_joint4'
        root.find(f"joint[@name='{name}']/limit").set('upper',str(limits[name][1]))
    return ET.tostring(root,encoding='unicode'),limits
