"""Restricted axis-aligned tray localization from RGB-D at a declared receiver.

No live tray pose, labels or requested count enter this reader. The catalog
template, receiver ROI and support height are declared fixture calibration.
Partial/multiple/rotated trays outside the shape gate return UNKNOWN.
"""
import copy
import numpy as np
from tray_rgbd_reader import FLOOR_BAND_M, EXTENT_TOL_M, MIN_VISIBLE_FLOOR_PIXELS
from rgbd_geometry import rectangle,image_components


def locate(template, world, rgb, valid, roi, floor_z):
    if world.shape != (*valid.shape,3) or rgb.shape != world.shape:
        return dict(status='UNKNOWN',reason='INVALID_FRAME_SHAPE',cells=None)
    color=np.asarray(rgb,float)
    x0,x1,y0,y1=roi
    cyan=(color[...,1]>.45*color[...,2]) & (color[...,1]<.9*color[...,2]) & (
          color[...,0]<.5*color[...,2])
    mask=valid & np.all(np.isfinite(world),axis=-1) & np.all(np.isfinite(color),axis=-1)
    mask &= cyan & (world[...,0]>=x0) & (world[...,0]<=x1)
    mask &= (world[...,1]>=y0) & (world[...,1]<=y1)
    mask &= abs(world[...,2]-floor_z)<=FLOOR_BAND_M
    floor_points=world[mask]
    if len(floor_points)<3*MIN_VISIBLE_FLOOR_PIXELS:
        return dict(status='UNKNOWN',reason='TRAY_FLOOR_NOT_OBSERVABLE',cells=None)
    if 'rim_height_m' in template:
        rim_mask=valid & cyan & np.all(np.isfinite(world),axis=-1)
        rim_mask &= (world[...,0]>=x0)&(world[...,0]<=x1)&(world[...,1]>=y0)&(world[...,1]<=y1)
        rim_mask &= abs(world[...,2]-floor_z-template['rim_height_m'])<=FLOOR_BAND_M
        # The catalog handle spheres are also cyan and can intersect the rim
        # height plane. Keep the long, connected wall outline, not disconnected
        # compact handle patches. No pose/geom labels enter this measurement.
        minimum_wall_extent=float(min(2*np.asarray(template['deck']['size'])[:2]))/2
        wall_patches=[]
        for pixels in image_components(rim_mask):
            patch=world[pixels[:,0],pixels[:,1]]
            if len(patch)>=3 and np.max(np.ptp(patch[:,:2],axis=0))>=minimum_wall_extent:
                wall_patches.append(patch)
        points=np.concatenate(wall_patches) if wall_patches else np.empty((0,3))
        if len(points)<3*MIN_VISIBLE_FLOOR_PIXELS:
            return dict(status='UNKNOWN',reason='TRAY_RIM_NOT_OBSERVABLE',cells=None)
    else:
        points=floor_points
    centre_xy,extent,yaw=rectangle(points)
    expected=2*np.asarray(template['deck']['size'])[:2]
    error=float(np.max(abs(extent-expected)))
    if error>EXTENT_TOL_M:
        return dict(status='UNKNOWN',reason='TRAY_TEMPLATE_UNRESOLVED',cells=None,
                    observed_extent_m=extent.tolist(),size_error_m=error)
    cells=copy.deepcopy(template)
    centre=np.asarray(cells['deck']['centre'],float).copy()
    centre[:2]=centre_xy
    measured_z=float(np.median(floor_points[:,2]))
    centre[2]=measured_z-float(cells['deck']['size'][2])
    cells['deck']['centre']=centre
    cells['deck']['top_z']=measured_z
    cells['observed_yaw_rad']=yaw
    for axis,key in ((0,'x_span'),(1,'y_span')):
        half=cells['deck']['size'][axis]
        cells['deck'][key]=(centre[axis]-half,centre[axis]+half)
    return dict(status='RESOLVED',reason='RGBD_TRAY_TEMPLATE_FIT',cells=cells,
                centre_xy_m=centre[:2].tolist(),floor_z_m=measured_z,
                observed_yaw_rad=yaw,
                visible_floor_pixels=len(floor_points),fitted_outline_pixels=len(points),
                outline='catalog_rim' if 'rim_height_m' in template else 'floor',size_error_m=error)
