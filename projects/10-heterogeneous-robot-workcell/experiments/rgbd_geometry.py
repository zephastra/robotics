"""XY outline fit from measured points, no labels or runtime pose inputs."""
import numpy as np


def image_components(mask):
    """Eight-connected measured pixels, no semantic labels or extra library."""
    seen=np.zeros(mask.shape,bool)
    height,width=mask.shape
    for y,x in zip(*np.nonzero(mask)):
        if seen[y,x]:continue
        pending=[(int(y),int(x))];seen[y,x]=True;pixels=[]
        while pending:
            py,px=pending.pop();pixels.append((py,px))
            for dy in (-1,0,1):
                for dx in (-1,0,1):
                    ny,nx=py+dy,px+dx
                    if 0<=ny<height and 0<=nx<width and mask[ny,nx] and not seen[ny,nx]:
                        seen[ny,nx]=True;pending.append((ny,nx))
        yield np.asarray(pixels,dtype=int)


def rectangle(points):
    """Minimum-area XY rectangle; canonical yaw is bounded to +/-45 degrees."""
    ordered=sorted(set(map(tuple,points[:,:2])))
    if len(ordered)<3:raise ValueError('degenerate outline')
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    hull=[]
    for sequence in (ordered,ordered[::-1]):
        part=[]
        for p in sequence:
            while len(part)>1 and cross(part[-2],part[-1],p)<=0:part.pop()
            part.append(p)
        hull.extend(part[:-1])
    hull=np.asarray(hull)
    candidates=[]
    for edge in np.roll(hull,-1,axis=0)-hull:
        angle=(np.arctan2(edge[1],edge[0])+np.pi/4)%(np.pi/2)-np.pi/4
        c,s=np.cos(angle),np.sin(angle)
        rotation=np.array([[c,-s],[s,c]])
        local=points[:,:2]@rotation
        lo,hi=local.min(axis=0),local.max(axis=0)
        candidates.append((float(np.prod(hi-lo)),angle,rotation,lo,hi))
    _,angle,rotation,lo,hi=min(candidates,key=lambda x:x[0])
    return (lo+hi)/2@rotation.T,hi-lo,float(angle)
