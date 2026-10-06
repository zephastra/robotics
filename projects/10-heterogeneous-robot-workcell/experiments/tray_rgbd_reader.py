"""Restricted chromatic-part counting from RGB-D, no geom labels/order/truth.

Fixed calibrated camera and declared stationary fixture pose are assumptions.
The reader must see some floor in EVERY cell; otherwise count is UNKNOWN, not 0.
This is a restricted RGB-D reader, not PCL/general detection or real-camera proof.
"""
import numpy as np

import probe_p3_vision as pv
from probe_p4_vision import EXTENT_TOL_M
from workcell.tray import cell_of
from rgbd_geometry import rectangle

COLOR_DOMINANCE = .4  # saturated red/blue parts versus cyan tray, declared choice
MIN_VISIBLE_FLOOR_PIXELS = 20
FLOOR_BAND_M = .006
MAX_DECLARED_CONTENT_HEIGHT_M = .075  # 50mm parts, 60mm tray walls + margin


def read(model, data, rgb, depth, cells, half_size, convention):
    """No body/geom identity inputs. Return independent counts or UNKNOWN."""
    if np.shape(depth)!=(pv.H,pv.W) or np.shape(rgb)!=(pv.H,pv.W,3):
        return dict(status='UNKNOWN',counts=None,reason='INVALID_FRAME_SHAPE',parts=[])
    world, valid, _ = pv.unproject(model,data,depth,**convention)
    return read_cloud(rgb,depth,world,valid,cells,half_size)


def read_cloud(rgb,depth,world,valid,cells,half_size):
    """Same reader on recorded calibrated points; no synthetic replay world."""
    if (np.shape(depth)!=(pv.H,pv.W) or np.shape(rgb)!=(pv.H,pv.W,3)
            or np.shape(world)!=(pv.H,pv.W,3) or np.shape(valid)!=(pv.H,pv.W)):
        return dict(status='UNKNOWN',counts=None,reason='INVALID_FRAME_SHAPE',parts=[])
    if not np.all(np.isfinite(rgb)):
        return dict(status='UNKNOWN',counts=None,reason='INVALID_RGB',parts=[])
    deck = cells['deck']
    yaw=float(cells.get('observed_yaw_rad',0.))
    rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    def tray_coordinates(points):
        if yaw==0:return points
        points=points.copy()
        centre=np.asarray(deck['centre'])[:2]
        points[...,:2]=(points[...,:2]-centre)@rotation+centre
        return points
    world=tray_coordinates(world)
    x0,x1 = deck['x_span']
    y0,y1 = deck['y_span']
    inside = valid & (world[...,0]>=x0) & (world[...,0]<=x1) & (
        world[...,1]>=y0) & (world[...,1]<=y1)
    if np.any(inside & (world[...,2]>deck['top_z']+MAX_DECLARED_CONTENT_HEIGHT_M)):
        return dict(status='UNKNOWN',counts=None,reason='OCCLUDER_ABOVE_CONTENT_ENVELOPE',parts=[])
    floor = inside & (np.abs(world[...,2]-deck['top_z'])<=FLOOR_BAND_M)
    coverage = {}
    for cell in cells['cells']:
        axis = cells['axis']
        offset = world[...,axis]-deck['centre'][axis]
        coverage[cell['index']] = int(np.sum(floor & (offset>=cell['lo']) &
                                              (offset<=cell['hi'])))
    if any(v<MIN_VISIBLE_FLOOR_PIXELS for v in coverage.values()):
        return dict(status='UNKNOWN',counts=None,reason='TRAY_NOT_FULLY_OBSERVABLE',
                    visible_floor_pixels=coverage,parts=[])
    counts, objects = {'red':0,'blue':0}, []
    color = np.asarray(rgb,float)
    for cls, channel in (('red',0),('blue',2)):
        size = np.asarray(half_size[cls] if isinstance(half_size,dict) else half_size)
        others = [a for a in range(3) if a!=channel]
        chromatic = np.all(color[...,others]<COLOR_DOMINANCE*color[...,channel,None],axis=-1)
        # Colour mask BEFORE connectivity: prevents a red part merging into a
        # physically adjacent grey rib, without labels or changing geometry.
        masked_depth = np.where(inside & chromatic, depth, 0.)
        w=world
        ok=inside & chromatic
        comps = pv.segment_parts(w,masked_depth,float(deck['top_z']),ok,
                                 size, (x0,x1,y0,y1))
        above_pixels = int(np.sum(ok & (w[...,2]>float(deck['top_z'])+
                                             .5*float(min(size)))))
        if above_pixels != sum(c['n_pixels'] for c in comps):
            return dict(status='UNKNOWN',counts=None,reason='UNRESOLVED_COLORED_FRAGMENT',
                        visible_floor_pixels=coverage,parts=objects)
        for comp in comps:
            extent = np.asarray(comp['bbox_extent_m'])
            if 'observed_yaw_rad' in cells:
                # Each free part can rotate independently of the tray. Fit its
                # own observed outline; retain the SAME 6mm shape criterion.
                points=np.asarray([w[y,x] for y,x in comp['pixels']])
                _,extent_xy,part_yaw=rectangle(points)
                error=float(np.max(abs(np.sort(extent_xy)-np.sort(2*size[:2]))))
            else:
                error = float(np.max(np.abs(extent[:2]-2*size[:2])))
            cell = cell_of(cells,comp['centroid'])
            if comp['n_pixels']<pv.MIN_PART_PIXELS or error>EXTENT_TOL_M or cell is None:
                return dict(status='UNKNOWN',counts=None,reason='UNRESOLVED_COLORED_COMPONENT',
                    visible_floor_pixels=coverage,parts=objects,
                    rejected=dict(cls=cls, pixels=comp['n_pixels'],size_error_m=error,cell=cell))
            counts[cls] += 1
            centroid=np.asarray(comp['centroid']).copy()
            centroid[:2]=(centroid[:2]-np.asarray(deck['centre'])[:2])@rotation.T+np.asarray(deck['centre'])[:2]
            objects.append(dict(cls=cls,cell=cell,centroid=centroid.tolist(),
                                pixels=comp['n_pixels'],size_error_m=error))
    return dict(status='RESOLVED',counts=counts,reason='RGBD_OBSERVED',
                visible_floor_pixels=coverage,parts=objects)
