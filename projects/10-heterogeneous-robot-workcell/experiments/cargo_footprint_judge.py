"""Independent physical footprint judge, never used to select control targets.

Body centres alone do not establish that complete cargo fits a tray cell. Uses
actual collision-geom orientation and the existing 10mm footprint tolerance.
Cylinder/capsule enclosing boxes are conservative, not falsely precise meshes.
"""
import itertools
import numpy as np
import mujoco
from workcell.tray import derive_cells,FOOTPRINT_TOL_M


def contained(points,centre,rotation,half_size,lo,hi,axis=1):
    points=np.asarray(points,float);centre=np.asarray(centre,float)
    rotation=np.asarray(rotation,float);half_size=np.asarray(half_size,float)
    if (points.ndim!=2 or points.shape[1]!=3 or not len(points)
            or centre.shape!=(3,) or half_size.shape!=(3,) or np.any(half_size<=0)
            or axis not in (0,1) or not np.isfinite(lo) or not np.isfinite(hi) or lo>=hi or not all(
            np.all(np.isfinite(v)) for v in (points,centre,rotation,half_size))):
        return dict(status='UNKNOWN',reason='INVALID_GEOMETRY')
    if (rotation.shape!=(3,3) or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6)
            or not np.isclose(np.linalg.det(rotation),1.,atol=1e-6)):
        return dict(status='UNKNOWN',reason='INVALID_ROTATION')
    local=(points-centre)@rotation
    other=1-axis;minimum=local.min(axis=0);maximum=local.max(axis=0)
    passed=(minimum[other]>=-half_size[other]-FOOTPRINT_TOL_M and
            maximum[other]<=half_size[other]+FOOTPRINT_TOL_M and
            minimum[axis]>=lo-FOOTPRINT_TOL_M and maximum[axis]<=hi+FOOTPRINT_TOL_M)
    return dict(status='PASS' if passed else 'FAIL',local_min_m=minimum.tolist(),
                local_max_m=maximum.tolist(),cell_span_m=[float(lo),float(hi)],
                tolerance_m=FOOTPRINT_TOL_M)


def judge(model,data,names):
    floor=model.geom('c_tray_floor').id
    centre=np.array(data.geom_xpos[floor]);rotation=np.array(data.geom_xmat[floor]).reshape(3,3)
    half=model.geom_size[floor]
    cells=derive_cells(model,data)
    signs=np.array(list(itertools.product((-1,1),repeat=3)))
    results={}
    for name in names:
        body=model.body(name).id;vertices=[];unsupported=[]
        for g in range(model.ngeom):
            if model.geom_bodyid[g]!=body or not (model.geom_contype[g] or model.geom_conaffinity[g]):continue
            size=model.geom_size[g];kind=model.geom_type[g]
            if kind==mujoco.mjtGeom.mjGEOM_BOX:bounds=size.copy()
            elif kind==mujoco.mjtGeom.mjGEOM_CYLINDER:bounds=np.array([size[0],size[0],size[1]])
            elif kind==mujoco.mjtGeom.mjGEOM_SPHERE:bounds=np.full(3,size[0])
            elif kind==mujoco.mjtGeom.mjGEOM_CAPSULE:bounds=np.array([size[0],size[0],size[0]+size[1]])
            else:unsupported.append(model.geom(g).name);continue
            vertices.extend((signs*bounds)@np.array(data.geom_xmat[g]).reshape(3,3).T+data.geom_xpos[g])
        if unsupported or not vertices:
            results[name]=dict(status='UNKNOWN',reason='UNSUPPORTED_OR_MISSING_COLLISION_GEOMETRY');continue
        choices=[contained(vertices,centre,rotation,half,c['lo'],c['hi'],cells['axis'])
                 for c in cells['cells']]
        matched=[i for i,c in enumerate(choices) if c['status']=='PASS']
        results[name]=dict(status='PASS' if len(matched)==1 else 'FAIL',
                          matching_cells=matched,cell_checks=choices)
    return dict(scope='INDEPENDENT_COLLISION_GEOMETRY_JUDGE_NOT_SENSOR_OR_CONTROL',parts=results,
                status='PASS' if results and all(r['status']=='PASS' for r in results.values()) else 'FAIL')
