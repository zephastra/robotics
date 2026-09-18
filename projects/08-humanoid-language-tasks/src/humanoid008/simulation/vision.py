"""RGB-D marker perception. No model/body/object identifiers or truth poses.

Adapted from the 007 baseline (section 6.1). The detector is colour-marker
based and deliberately does **not** see ground-truth object poses: it only
segments colour regions in the rendered RGB image and fits a plane to their
depth.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    rgb: np.ndarray
    depth: np.ndarray
    position: np.ndarray
    rotation: np.ndarray
    fovy: float
    time: float


@dataclass
class Detection:
    center: np.ndarray
    normal: np.ndarray
    long_axis: np.ndarray
    pixels: int
    time: float
    pixel_center: np.ndarray


def detect(frame, kind):
    if (
        frame.rgb.ndim != 3
        or frame.rgb.shape[2] != 3
        or frame.depth.shape != frame.rgb.shape[:2]
        or not np.isfinite(frame.position).all()
        or not np.isfinite(frame.rotation).all()
        or not np.isfinite(frame.fovy)
        or not 1 < frame.fovy < 179
    ):
        return None
    rgb = frame.rgb.astype(float)
    red, green, blue = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    if kind == "box":
        mask = (red > 70) & (blue > 70) & (green < 0.65 * red) & (green < 0.65 * blue)
    elif kind == "destination":
        mask = (green > 70) & (green > 1.5 * red) & (green > 1.3 * blue)
    elif kind == "destination_c":
        # The C-station marker is pure blue (.1 .1 1). The sky/background is also
        # blue-ish, so the threshold must be strict enough to reject it: the
        # marker's red/green channels are ~25 while the sky's green is ~64+.
        mask = (blue > 70) & (blue > 2 * red) & (blue > 2 * green)
    else:
        raise ValueError("Unknown marker class")
    mask &= np.isfinite(frame.depth) & (frame.depth > 0.05) & (frame.depth < 4.0)
    cropped = bool(mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any())
    if cropped:
        return None
    # Exclude antialiased color/depth boundaries before fitting a plane.
    mask &= (
        np.roll(mask, 1, 0)
        & np.roll(mask, -1, 0)
        & np.roll(mask, 1, 1)
        & np.roll(mask, -1, 1)
    )
    v, u = np.nonzero(mask)
    if len(u) < 30:
        return None
    h, w = frame.depth.shape
    if min(u) == 0 or max(u) == w - 1 or min(v) == 0 or max(v) == h - 1:
        return None
    depth = frame.depth[v, u]
    f = 0.5 * h / np.tan(np.deg2rad(frame.fovy) / 2)
    local = np.column_stack(
        ((u - (w - 1) / 2) * depth / f, -(v - (h - 1) / 2) * depth / f, -depth)
    )
    points = local @ frame.rotation.T + frame.position
    origin = points.mean(axis=0)
    _, singular, basis = np.linalg.svd(points - origin, full_matrices=False)
    if singular[1] < 1e-4 or singular[2] / singular[1] > 0.08:
        return None
    normal = basis[2]
    if normal[2] < 0:
        normal = -normal
    axis = basis[0]
    if axis[1] < 0:
        axis = -axis
    transverse = np.cross(axis, normal)
    coordinates = (points - origin) @ np.column_stack((axis, transverse, normal))
    midpoint = (coordinates.min(axis=0) + coordinates.max(axis=0)) / 2
    center = origin + np.column_stack((axis, transverse, normal)) @ midpoint
    return Detection(center, normal, axis, len(u), frame.time, np.array([u.mean(), v.mean()]))
