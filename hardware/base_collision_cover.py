"""Conservative local boxes covering every source triangle AABB; offline only.

A single lightweight watertight-box triangle soup preserves one URDF collision
object and existing SRDF pair topology. No collision pair is disabled.
"""
import hashlib
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np


def triangle_cover(triangles, max_cell=.10):
    triangles=np.asarray(triangles,dtype=float)
    if triangles.ndim!=3 or triangles.shape[1:]!=(3,3) or not len(triangles) or not np.isfinite(triangles).all():
        raise ValueError('finite nonempty triangles required')
    if not np.isfinite(max_cell) or not .02<=max_cell<=.2:raise ValueError('invalid cover cell size')
    lows,highs=triangles.min(axis=1),triangles.max(axis=1)
    boxes=[]
    def visit(indices,lower,upper,depth):
        lower=np.maximum(lower,lows[indices].min(axis=0))
        upper=np.minimum(upper,highs[indices].max(axis=0))
        axis=int(np.argmax(upper-lower))
        if upper[axis]-lower[axis]<=max_cell:
            boxes.append((lower-1e-6,upper+1e-6));return
        if depth>24:raise ValueError('cover subdivision did not converge')
        split=(lower[axis]+upper[axis])*.5
        a=upper.copy();a[axis]=split
        b=lower.copy();b[axis]=split
        left=indices[lows[indices,axis]<=split]
        right=indices[highs[indices,axis]>=split]
        if len(left):visit(left,lower,a,depth+1)
        if len(right):visit(right,b,upper,depth+1)
    visit(np.arange(len(triangles)),lows.min(axis=0),highs.max(axis=0),0)
    return boxes


def write_cover(boxes,path):
    corners=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])
    faces=((0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
           (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7))
    with Path(path).open('wb') as f:
        f.write(b'Conservative SH5 base triangle-AABB cover'.ljust(80,b'\0'))
        f.write(struct.pack('<I',len(boxes)*12))
        for lower,upper in boxes:
            v=lower+corners*(upper-lower)
            for face in faces:f.write(struct.pack('<12fH',0.,0.,0.,*v[list(face)].reshape(-1),0))


def refine_base(urdf_text,destination):
    try:import coal
    except ModuleNotFoundError:import hppfcl as coal
    root=ET.fromstring(urdf_text)
    meshes=root.findall("./link[@name='base_link']/collision/geometry/mesh")
    if len(meshes)!=1:raise ValueError('expected one base collision mesh')
    mesh=meshes[0];source=Path(mesh.get('filename'));scale=np.array([float(x) for x in mesh.get('scale','1 1 1').split()])
    geom=coal.MeshLoader().load(str(source),scale)
    vertices=np.asarray(geom.vertices(),dtype=float)
    faces=np.array([[geom.tri_indices(i)[j] for j in range(3)] for i in range(geom.num_tris)])
    boxes=triangle_cover(vertices[faces])
    destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True)
    write_cover(boxes,destination)
    mesh.set('filename',str(destination.resolve()));mesh.set('scale','1 1 1')
    report={'type':'conservative_triangle_aabb_cover','max_cell_m':.10,'padding_m':1e-6,
            'original_triangles':int(len(faces)),'boxes':len(boxes),'proxy_triangles':len(boxes)*12,
            'source_mesh_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'proxy_path':str(destination.resolve()),'proxy_sha256':hashlib.sha256(destination.read_bytes()).hexdigest(),
            'triangle_aabb_partition_coverage':True,'collision_pairs_changed':False}
    return ET.tostring(root,encoding='unicode'),report
