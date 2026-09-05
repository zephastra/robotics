"""Detect the single blue marked box from actual camera pixels, not world poses.

Known horizontal support height + calibrated downward camera gives a ray/plane
intersection. This is a constrained colour-marker detector, not general grasp AI.
"""
import cv2
import numpy as np

CAMERA_X, CAMERA_Y, CAMERA_Z = 0.30, 0.0, 1.80


def detect_box(rgb, fx, fy, cx, cy, top_z, color='blue'):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    if color == 'blue':
        mask = cv2.inRange(hsv, np.array([90,100,50]), np.array([135,255,255]))
    elif color == 'red':
        mask = cv2.inRange(hsv, np.array([0,140,70]), np.array([8,255,255])) | cv2.inRange(hsv, np.array([172,140,70]), np.array([179,255,255]))
    elif color == 'magenta':
        mask = cv2.inRange(hsv,np.array([140,100,60]),np.array([169,255,255]))
    else:
        raise ValueError('Unsupported marker colour')
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    candidates = [i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA] >= 12]
    if len(candidates) != 1:
        raise ValueError(f'Expected one {color} target, observed {len(candidates)}')
    i = candidates[0]
    u, v = centroids[i]
    if stats[i,0] <= 1 or stats[i,1] <= 1 or stats[i,0]+stats[i,2] >= rgb.shape[1]-1 or stats[i,1]+stats[i,3] >= rgb.shape[0]-1:
        raise ValueError('Box is clipped by camera image')
    height = CAMERA_Z-top_z
    if fx <= 0 or fy <= 0 or height <= 0:
        raise ValueError('Invalid camera calibration or support height')
    # Camera looks along -Z; image right=-Y, image down=-X in base_link.
    x = CAMERA_X-(v-cy)*height/fy
    y = CAMERA_Y-(u-cx)*height/fx
    metadata={'u':float(u),'v':float(v),'pixels':int(stats[i,4])}
    if color == 'magenta':
        rows,cols=np.nonzero(labels==i)
        points=np.stack((-(rows-v)/fy,-(cols-u)/fx))
        values,vectors=np.linalg.eigh(np.cov(points))
        if values[1] < 2.*values[0]: raise ValueError('Home marker orientation is ambiguous')
        axis=vectors[:,1]
        if axis[0]<0: axis=-axis
        metadata['yaw']=float(np.arctan2(axis[1],axis[0]))
    return float(x), float(y), metadata
