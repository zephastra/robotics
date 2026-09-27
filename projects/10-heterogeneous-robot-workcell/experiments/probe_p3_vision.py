"""P3-VISION-03: an RGB-D camera over the arm's table, and can it find the parts?

WHAT THIS ANSWERS
-----------------
`P3-VISION-03` asks for "RGB-D / point cloud, tabletop and separated-part localization, latency
and coordinate validation". `MASTER_PLAN` asks for "red and blue simple parts, placed apart, not
piled up", and the P3 exit gate asks for "coordinate/time/sensor calibration".

So this file renders a depth image of the arm's bench, separates the bench surface from whatever
stands on it, localizes each part in world coordinates, and compares that against the SIMULATION
TRUTH. Truth is read only by the judge; nothing here feeds it back to a controller.

WHAT THE FIRST RUN OF THIS PROBE GOT WRONG -- ALL FIXED HERE
-----------------------------------------------------------
The first run scored 6/8 and reported the blue part "localized 46.9 mm off". Every part of that
sentence was wrong.

1. **The blue part was never on the bench.** A `<keyframe>` pins every joint in the model, and for
   a free joint added afterwards MuJoCo pads the keyframe's qpos with ZERO. So
   `spec.worldbody.add_body(pos=...)` plus `add_freejoint()` produced a part sitting at the world
   ORIGIN, and `d.qpos[:] = model.key_qpos[0]` restored it there on every render. A body `pos`
   write is a DEAD WRITE for a free-jointed body whenever a keyframe exists.
2. **`truth_blue` was a TYPED constant** (`truth_red[1] + 0.10 + 0.06`) rather than read off the
   model, so nothing could notice the part had moved. It also disagreed with the placement code,
   which uses `2*size[1] + PART_CLEARANCE` = 0.13, not 0.16.
3. **The colour check passed on a GREY blob.** It only tested channel ORDERING (`B > R`), and the
   arm's hand renders at RGB [93.5, 93.7, 93.8] -- blue by 0.3 of a level. A grey object is not a
   blue part.
4. **The falsifiability probe could not fire.** It "removed" the parts with the same dead body-pos
   write, so a part stayed on the bench, the component count did not change, and the probe
   reported that it had failed to make the check fail -- the only reason the run was honest.
5. **The convention statistic was too weak to calibrate anything.** It scored a candidate by the
   distance from the truth point to the NEAREST unprojected point. The scene is full of surfaces
   clustered around the bench, so a correct convention and a vertically MIRRORED one both scored
   15.2 mm and the choice was a tie. A statistic a scrambled cloud can satisfy does not validate
   coordinates.
6. **The convention was unobservable anyway.** Both parts sat on the image centre column (cols
   154..165 of W=320), so a horizontal mirror mapped each part's own pixels back onto itself.
   Fitted on the parts alone, a horizontally mirrored convention scored 9.005 mm against the
   winner's 9.005 mm. A calibration target on the optical axis is not a target. Fixed by fitting
   on TWO known targets, one of which (the bench) is 70 mm off the axis: the horizontal rival now
   costs 54.1 mm.
7. **The blue part was never occluded.** Occlusion by the arm's hand was the suspected cause of
   the 46.9 mm. With the part actually on the bench it renders 186 px at rows 95..110 while the
   hand sits at rows 0..28.

The fixes are structural, not tolerance changes. One function (`prepare`) writes the state and
every render, truth read and probe goes through it; truth is read off the compiled model; the
placement is asserted to have landed; a candidate must be CHROMATIC and must REST ON THE BENCH
before it can represent a part; and the convention is fitted on labelled targets and then
CROSS-VALIDATED on the part that was not used to fit it.

WHAT THE PROBE ADDS, AND WHY IT IS NOT IN THE SCENE
---------------------------------------------------
The P3 cell has one camera (`h_eyes`, on the humanoid's head) and one part (`a_payload`). Neither
is enough: the humanoid is at its own station and cannot see the bench, and "separated parts"
needs two. So this probe builds a VARIANT of the judged world through `MjSpec`, adding

  * `bench_cam` -- a fixed camera looking down at the arm's bench, and
  * `a_payload_blue` -- a second part, beside the existing one.

The judged artefact `assets/world_p3_cell.xml` is left byte-identical, because P3-WORLD-01's
evidence points at it. Where the cameras and parts finally live is a P3-FREEZE-04 decision, and
pretending it is already made would put an unearned claim in the world's provenance.

CALIBRATION vs PERCEPTION -- KEEPING THEM SEPARATE
--------------------------------------------------
Calibration uses MuJoCo's segmentation renderer, which labels each pixel with its geom id. That is
an ORACLE: a real camera does not hand you object identities. It is legitimate here for exactly
the purpose a real calibration board serves -- fitting a known sensor model against a known target
-- and it is used for nothing else. The localization claim is BLIND: it works from depth + colour
only, and it is CROSS-VALIDATED on the blue part, whose pixels took no part in the fit.

WHAT IS NOT ESTABLISHED
-----------------------
No ROS, no point-cloud library, no timestamped topic: this is the perception half of the interface
in-process. `MASTER_PLAN` defers the PCL choice to "after checking compatibility", so no PCL claim
is made. No real sensor noise, blur or exposure is modelled, so none of these numbers describe a
physical camera. Two parts on one bench is not a bin-picking scene. The bench's planar top is
removed by a height threshold plus a footprint, not by RANSAC plane fitting.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import build_p3_world as bpw  # noqa: E402

bpw.install()
import merge_world as mw  # noqa: E402
from evaluate_p3_world import Results  # noqa: E402

WORLD = mw.ASSETS / 'world_p3_cell.xml'
W, H = 320, 240
CAM = 'bench_cam'
RED = 'a_payload'
BLUE = 'a_payload_blue'
BLUE_J = 'a_payload_blue_free'
TABLE = 'a_table'

#: DECLARED design parameters (chosen, not measured). Everything the WORLD decides -- the bench
#: top, the footprint, the part size, the initial poses -- is derived at runtime instead.
PART_CLEARANCE = 0.10       # face-to-face gap between the two parts
CAM_BACK_OFF = 0.75         # camera standoff behind the parts' midpoint, along -y
CAM_HEIGHT = 0.70           # camera height above the bench top
ON_BENCH_TOL = 0.025        # a blob may rise half a part above the bench and still "rest on" it
CHROMA_RATIO = 1.5          # a class colour must beat the other two channels by this factor
MIN_PART_PIXELS = 50        # a part with fewer visible pixels than this is not "localized"
LOC_TOL = 0.020             # the x,y localization tolerance the claim is made at
HAND_RGB = np.array([93.5, 93.7, 93.8])   # what the arm's hand renders as; a grey, not a part
RED_AWAY = [-5.0, 0.0, 0.2]

#: V1's two part classes, set as GEOM rgba rather than through a material. A named material is an
#: extra indirection that can silently fail to resolve, and the colour is the whole point of the
#: classification check -- so it is set on the thing the renderer actually reads, and then read
#: back off the compiled model.
RED_RGBA = [0.85, 0.05, 0.05, 1.0]
BLUE_RGBA = [0.05, 0.05, 0.85, 1.0]

#: Short keys -> check names. Every check is registered here, so the falsifiability contract can be
#: written in keys and cannot silently decouple from a renamed check.
KEYS = {
    'colour': 'the part colours are actually applied to the geoms the renderer reads',
    'render': 'RGB-D renders',
    'placement': 'the second part is where it was placed, and the bench top is DERIVED',
    'calibration': 'the camera convention is calibrated against TWO known targets, not assumed',
    'observable': 'the calibration is not degenerate: the runner-up is far worse, and no sign is '
                  'unobservable',
    'footprint': "the localized cloud has the part's own size in world coordinates",
    'visibility': "both parts are inside the camera's field of view and not occluded",
    'bench': 'the bench surface is separated from what stands on it',
    'clutter': 'clutter standing above the bench is rejected, not matched to a part',
    'localization': 'two separated parts are found as TWO components and localized',
    'chroma': 'the two parts are distinguishable by CHROMA, and a grey blob is not a part',
    'stream': 'every observation carries its frame, timestamp, age and intrinsics',
}

#: Checks with no input that can be injected to make them silently wrong, and why.
STRUCTURAL = {
    'render': 'a render either returns a finite depth buffer or raises; there is no input that '
              'makes it quietly wrong',
    'bench': 'the failure mode is a dead camera, which the render row already covers',
    'stream': 'a wall-clock measurement with no failure mode to inject',
}


# =============================================================================================
# state: ONE writer, so the render and the truth cannot diverge
# =============================================================================================
def free_addr(model, body):
    """qpos address of `body`'s free joint, or None. Searched by OWNERSHIP, not by name."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    for j in range(model.njnt):
        if int(model.jnt_bodyid[j]) == bid and int(model.jnt_type[j]) == int(
                mujoco.mjtJoint.mjJNT_FREE):
            return int(model.jnt_qposadr[j])
    return None


def pose_of(model, d, body):
    return np.array(d.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)])


def prepare(model, d, *, poses=None):
    """THE one place the state is set. Render, truth and every probe call this.

    `poses` maps BODY NAME -> absolute world position. It must go through the free joint's qpos:
    with a `<keyframe>` in the model, `body.pos` is a dead write for a free-jointed body, which is
    how the blue part spent a whole run parked at the world origin.
    """
    d.qpos[:] = model.key_qpos[0]
    for body, p in (poses or {}).items():
        adr = free_addr(model, body)
        if adr is None:
            raise SystemExit(f'{body} has no free joint, so its pose cannot be set')
        d.qpos[adr:adr + 3] = np.asarray(p, float)
        d.qpos[adr + 3:adr + 7] = (1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, d)
    return d


# =============================================================================================
# geometry, derived from the model in hand
# =============================================================================================
def _geom_of_body(model, body):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    for g in range(model.ngeom):
        if int(model.geom_bodyid[g]) == bid:
            return g
    raise SystemExit(f'{body} has no geom')


def bench_geometry(model, d):
    """(top z, roi) of the bench, from the bench's own geom -- NOT typed.

    An earlier version carried `BENCH_TOP = 0.24` annotated "from a_table's own geometry" while the
    derived value was 0.2000. A typed constant cannot notice it has gone stale, so it is gone.
    """
    g = _geom_of_body(model, TABLE)
    c = np.array(d.geom_xpos[g], float)
    s = np.array(model.geom_size[g], float)
    return (float(c[2] + s[2]),
            (float(c[0] - s[0]), float(c[0] + s[0]), float(c[1] - s[1]), float(c[1] + s[1])))


def part_size(model):
    return np.array(model.geom_size[_geom_of_body(model, RED)], float)


def intended_blue_pose(model, d, bench_z):
    """Where the second part is meant to stand, derived from the first part's own pose."""
    g = part_size(model)
    t = pose_of(model, d, RED)
    return np.array([t[0], t[1] + 2.0 * g[1] + PART_CLEARANCE, bench_z + g[2]]), g


# =============================================================================================
# rendering: ONE Renderer per model, mode-switched. A Renderer per pass leaks GL contexts and
# produced garbage geom ids (measured: index 8467080 into a 241-entry lookup table).
# =============================================================================================
class View:
    def __init__(self, model):
        self.r = mujoco.Renderer(model, H, W)

    def capture(self, d, mode):
        r = self.r
        r.disable_depth_rendering()
        r.disable_segmentation_rendering()
        if mode == 'depth':
            r.enable_depth_rendering()
        elif mode == 'seg':
            r.enable_segmentation_rendering()
        r.update_scene(d, camera=CAM)
        out = r.render().copy()
        return out[:, :, 0] if mode == 'seg' else out

    def close(self):
        self.r.close()


def look_at(pos, target, up=(0.0, 0.0, 1.0)):
    """Quaternion putting the camera's LOCAL -z on `target`.

    The -z is not a guess: with an identity quaternion a camera at +0.79 m renders the bench below
    it (min depth 0.183 m), and the +90-degree-about-x candidate renders a surface at 1.625 m --
    the cell's own y half-extent, which is the wall. Both are only consistent with "forward is
    local -z".
    """
    f = np.array(target, float) - np.array(pos, float)
    f = f / np.linalg.norm(f)
    z = -f
    u = np.array(up, float)
    x = np.cross(u, z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    tr = float(np.trace(R))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = [(1.0 + tr) / s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
             (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2.0
        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = (R[k, j] - R[j, k]) / s
        q[i + 1] = (1.0 + R[i, i] - R[j, j] - R[k, k]) / s
        q[j + 1] = (R[i, j] + R[j, i]) / s
        q[k + 1] = (R[i, k] + R[k, i]) / s
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q)


def build_scene(*, cam_quat=None, cam_pos=None):
    """The judged world + a bench camera + a second, blue part. Compiles to an MjModel.

    Returns (model, bench_z, blue_pose). `blue_pose` is what the caller must hand to `prepare`;
    it is returned so that `main` can ASSERT the compiled model agrees, instead of assuming it.
    """
    spec = mujoco.MjSpec.from_file(str(WORLD))
    part = spec.body(RED)
    if part is None:
        raise SystemExit(f'{RED} is missing, so there is nothing to localize')
    a_geom = part.geoms[0]
    a_size = [float(v) for v in a_geom.size]
    a_pos = [float(v) for v in a_geom.pos]
    a_geom.rgba = RED_RGBA

    m0 = mujoco.MjModel.from_xml_path(str(WORLD))
    d0 = mujoco.MjData(m0)
    d0.qpos[:] = m0.key_qpos[0]
    mujoco.mj_forward(m0, d0)
    bench_z, _roi = bench_geometry(m0, d0)
    blue_pose, _g = intended_blue_pose(m0, d0, bench_z)
    red_pose = pose_of(m0, d0, RED)

    b = spec.worldbody.add_body(name=BLUE, pos=[float(v) for v in blue_pose])
    b.add_freejoint(name=BLUE_J)
    g = b.add_geom(name=f'{BLUE}_geom', type=mujoco.mjtGeom.mjGEOM_BOX,
                   size=a_size, pos=a_pos)
    g.contype = 1
    g.conaffinity = 1
    g.rgba = BLUE_RGBA

    mid_y = (float(red_pose[1]) + float(blue_pose[1])) / 2.0
    if cam_pos is None:
        # DECLARED mounting choice, aimed at the parts' midpoint. The first version backed it off
        # 0.75 m to dodge the arm's hand; the hand never occluded anything, the part was simply
        # not on the bench. What the choice must satisfy is measured below, by counting the
        # labelled pixels each part actually renders.
        cam_pos = [float(red_pose[0]), mid_y - CAM_BACK_OFF, bench_z + CAM_HEIGHT]
    if cam_quat is None:
        cam_quat = look_at(cam_pos, [float(red_pose[0]), mid_y, bench_z])
    cam = spec.worldbody.add_camera()
    cam.name = CAM
    cam.pos = list(cam_pos)
    cam.quat = list(cam_quat)
    cam.fovy = 45.0
    model = spec.compile()
    d = mujoco.MjData(model)
    prepare(model, d)
    here, _r2 = bench_geometry(model, d)
    if abs(here - bench_z) > 1e-9:
        raise SystemExit(f'the bench moved during the probe build: {bench_z} -> {here}')
    return model, bench_z, blue_pose


# =============================================================================================
# projection
# =============================================================================================
def unproject(model, d, depth, *, forward, fwd_sign, up_sign, v_sign):
    """Depth pixels -> world points, under an explicit camera convention.

    `forward` is the camera-local axis that points AWAY from the camera (0=x, 1=y, 2=z); `fwd_sign`
    its direction; `up_sign`/`v_sign` the two IMAGE directions. All four are calibrated, never
    guessed -- and the calibration must leave none of them unobservable, which is why it is fitted
    on two targets and one of them is off the optical axis.
    """
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, CAM)
    cpos = np.array(d.cam_xpos[cid])
    cmat = np.array(d.cam_xmat[cid]).reshape(3, 3)
    fy = (H / 2.0) / math.tan(math.radians(float(model.cam_fovy[cid])) / 2.0)
    fx = fy
    us, vs = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    a = np.zeros_like(depth)
    ok = np.isfinite(depth) & (depth > 0.01) & (depth < 10.0)
    a[ok] = depth[ok]
    xcam = v_sign * (us - W / 2.0) / fx * a
    ycam = up_sign * (vs - H / 2.0) / fy * a
    local = np.zeros((H, W, 3))
    ax = [0, 1, 2]
    ax.remove(forward)
    local[..., ax[0]] = xcam
    local[..., ax[1]] = ycam
    local[..., forward] = fwd_sign * a
    world = local.reshape(-1, 3) @ cmat.T + cpos
    return world.reshape(H, W, 3), ok, (fx, fy)


def calibration_error(model, d, depth, conv, seg, bench_z, g, truth_red):
    """Score a candidate convention against TWO KNOWN TARGETS, from their labelled pixels.

    Target 1: the red part. Its own pixels must unproject onto its own known pose -- the x,y
    centroid near the body origin and the top at bench_z + 2*half_height.

    Target 2: the bench. Its labelled pixels' world bounding box must match the bench geom's own
    footprint. The bench is 70 mm off the camera's optical axis, and that is the whole point: the
    parts sit on the axis, so a horizontal mirror maps each part's pixels back onto itself and a
    part-only fit cannot see the horizontal sign at all (measured: 9.005 mm for both).
    """
    world, _ok, _fx = unproject(model, d, depth, **conv)
    gid = _geom_of_body(model, RED)
    ys, xs = np.where(seg == gid)
    if ys.size < 10:
        return float('inf'), {}
    pts = world[ys, xs]
    red_xy = float(np.linalg.norm(pts[:, :2].mean(axis=0) - truth_red[:2]))
    red_z = abs(float(pts[:, 2].max()) - (bench_z + 2.0 * g[2]))

    tid = _geom_of_body(model, TABLE)
    ty, tx = np.where(seg == tid)
    if ty.size < 100:
        return float('inf'), {}
    tb = world[ty, tx]
    c = np.array(d.geom_xpos[tid], float)
    s = np.array(model.geom_size[tid], float)
    bench = (abs(float(tb[:, 0].min()) - (c[0] - s[0])) + abs(float(tb[:, 0].max()) - (c[0] + s[0]))
             + abs(float(tb[:, 1].min()) - (c[1] - s[1]))
             + abs(float(tb[:, 1].max()) - (c[1] + s[1])))
    return red_xy + red_z + bench, {'red_xy': red_xy, 'red_z': red_z, 'bench': bench,
                                    'red_n': int(ys.size), 'bench_n': int(ty.size)}


def calibrate(model, d, depth, seg, bench_z, g, truth_red):
    """Rank all 24 conventions. Returns (best, conv, ranked)."""
    ranked = []
    for forward in (0, 1, 2):
        for fs in (1.0, -1.0):
            for up in (1.0, -1.0):
                for vs in (1.0, -1.0):
                    conv = {'forward': forward, 'fwd_sign': fs, 'up_sign': up, 'v_sign': vs}
                    s, det = calibration_error(model, d, depth, conv, seg, bench_z, g, truth_red)
                    ranked.append((s, conv, det))
    ranked.sort(key=lambda t: t[0])
    return ranked[0][0], ranked[0][1], ranked


# =============================================================================================
# perception (BLIND: depth + colour only)
# =============================================================================================
def segment_parts(world, depth, bench_z, ok, g_size, roi=None):
    """Points above the bench AND inside the bench's footprint, clustered into components.

    The ROI is not optional. With an oblique camera the bench plane is not the only thing above
    `bench_z`: the floor beyond it and the walls are too, and without a footprint test the whole
    frame becomes one component (measured: a single 32400 px blob).
    """
    above = ok & (world[..., 2] > bench_z + 0.5 * float(min(g_size)))
    if roi is not None:
        x0, x1, y0, y1 = roi
        m = 0.02
        above &= ((world[..., 0] > x0 - m) & (world[..., 0] < x1 + m)
                  & (world[..., 1] > y0 - m) & (world[..., 1] < y1 + m))
    comps = []
    seen = np.zeros((H, W), dtype=bool)
    # iterate the candidate pixels only: scanning all H*W in Python cost ~500 ms of a frame
    for i, j in np.argwhere(above):
        i, j = int(i), int(j)
        if seen[i, j]:
            continue
        stack, cur = [(i, j)], []
        seen[i, j] = True
        while stack:
            y, x = stack.pop()
            cur.append((y, x))
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                yy, xx = y + dy, x + dx
                if 0 <= yy < H and 0 <= xx < W and above[yy, xx] and not seen[yy, xx]:
                    seen[yy, xx] = True
                    stack.append((yy, xx))
        if len(cur) >= 30:
            comps.append(cur)
    out = []
    for cur in comps:
        pts = np.array([world[y, x] for y, x in cur])
        out.append({'n_pixels': len(cur), 'centroid': pts.mean(axis=0).tolist(),
                    'bbox_extent_m': [float(pts[:, k].max() - pts[:, k].min()) for k in range(3)],
                    'z_min': float(pts[:, 2].min()), 'z_max': float(pts[:, 2].max()),
                    'pixels': cur})
    return out


def mean_rgb(rgb, comp):
    px = np.array([rgb[y, x] for y, x in comp['pixels']], dtype=float)
    return px.mean(axis=0)


def chroma_class(colour):
    """Which declared part class a colour is, by CHROMA -- or 'grey' for anything else.

    Ordering alone is not enough. The arm's hand renders at [93.5, 93.7, 93.8]; `B > R` calls that
    blue, and the first version's colour check therefore PASSED on it. A class colour has to beat
    the other two channels by `CHROMA_RATIO`, which is the standard saturation argument.
    """
    r, g, b = (float(v) for v in colour)
    if r >= CHROMA_RATIO * max(g, b):
        return 'red'
    if b >= CHROMA_RATIO * max(r, g):
        return 'blue'
    return 'grey'


def order_only_class(colour):
    """The FIRST version's rule, kept as the regression the chroma test replaced."""
    r, g, b = (float(v) for v in colour)
    if r > g and r > b:
        return 'red'
    if b > r and b > g:
        return 'blue'
    return 'grey'


def rests_on_bench(z_max, bench_z, g_size):
    """Does a blob's top sit at the height a part standing on the bench would have?

    A part resting on the bench has its top at `bench_z + 2*half_height`; the tolerance is half a
    part. The arm's hand measures 0.4022 m above the bench top, i.e. 16x the tolerance.
    """
    return z_max <= bench_z + 2.0 * float(g_size[2]) + ON_BENCH_TOL


def analyze(model, d, rgb, depth, bench_z, g, conv):
    """Depth + colour -> components, classes and eligibility. No truth, no oracle."""
    t0 = time.perf_counter()
    world, ok, (fx, fy) = unproject(model, d, depth, **conv)
    _top, roi = bench_geometry(model, d)
    comps = segment_parts(world, depth, bench_z, ok, g, roi)
    for c in comps:
        c['rgb'] = mean_rgb(rgb, c)
        c['cls'] = chroma_class(c['rgb'])
        c['on_bench'] = rests_on_bench(c['z_max'], bench_z, g)
    return {'rgb': rgb, 'depth': depth, 'world': world, 'ok': ok, 'comps': comps,
            'total_s': time.perf_counter() - t0, 'fx': fx, 'fy': fy}


def localize(per, truth, g_size):
    """Match each truth part to the component that can actually be IT.

    NOT "nearest centroid". The arm's hand happens to sit 16.6 mm in x,y from where the blue part
    stands, so position alone cannot separate them -- the first run handed the hand to the blue
    part and called the result a localization error. A candidate must be chromatic AND resting on
    the bench before it is allowed to represent a part.
    """
    want = {RED: 'red', BLUE: 'blue'}
    rows, found = [], {}
    for body, cls in want.items():
        cands = [c for c in per['comps'] if c['cls'] == cls and c['on_bench']]
        cands.sort(key=lambda c: -c['n_pixels'])
        c = cands[0] if len(cands) == 1 else None
        found[body] = c
        tp = np.asarray(truth[body], float)
        rows.append({'truth': body, 'class': cls, 'n_candidates': len(cands),
                     'xy_error_m': (float(np.linalg.norm(np.array(c['centroid'])[:2] - tp[:2]))
                                    if c else None),
                     'n_pixels': (c['n_pixels'] if c else 0),
                     'found': (c['centroid'] if c else None),
                     'truth_point': tp.tolist(),
                     'z_max': (c['z_max'] if c else None),
                     'bbox_extent_m': (c['bbox_extent_m'] if c else None)})
    return rows, found


def localize_ok(rows):
    return (all(r['n_candidates'] == 1 for r in rows)
            and all(r['n_pixels'] >= MIN_PART_PIXELS for r in rows)
            and all(r['xy_error_m'] is not None and r['xy_error_m'] < LOC_TOL for r in rows))


def chroma_ok(pairs):
    return (len(pairs) == 2
            and all(chroma_class(c) == ('red' if k == RED else 'blue') for k, c in pairs))


# =============================================================================================
def judge(res, model, d, per, seg, bench_z, g, conv, ranked, truth, landed, blue_pose):
    """Every check, with its predicate factored out so a probe can call it. Returns handles."""
    _top, roi = bench_geometry(model, d)
    gsd = float(np.median(per['depth'][per['ok']])) / per['fx']
    err, det = ranked[0][0], ranked[0][2]
    rival_s, rival_c, _rd = ranked[1]
    tol_cal = float(np.max(g)) + 0.005

    # --- colour readback -------------------------------------------------------------------
    def readback(mdl):
        out = {}
        for bname, want in ((RED, RED_RGBA), (BLUE, BLUE_RGBA)):
            gid = _geom_of_body(mdl, bname)
            got = [round(float(v), 3) for v in mdl.geom_rgba[gid]]
            out[bname] = got == [round(v, 3) for v in want]
        return all(out.values()), out
    ok1, rb = readback(model)
    res.add(KEYS['colour'], 'PASS' if ok1 else 'FAIL',
            f'a_payload rgba matches: {rb[RED]}, a_payload_blue: {rb[BLUE]}. Read back off the '
            f'COMPILED model, because a colour set on the wrong object is accepted, changes '
            f'nothing, and still renders a plausible image.')

    # --- render ----------------------------------------------------------------------------
    res.add(KEYS['render'], 'PASS' if (per['rgb'].shape == (H, W, 3)
                                       and per['depth'].shape == (H, W)
                                       and np.all(np.isfinite(per['depth']))) else 'FAIL',
            f"colour {per['rgb'].shape} dtype {per['rgb'].dtype}; depth {per['depth'].shape} in "
            f"METRES, range {np.min(per['depth']):.3f}..{np.max(per['depth']):.3f} m, all finite; "
            f"fx=fy={per['fx']:.1f} px from fovy="
            f"{model.cam_fovy[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, CAM)]:.1f} deg")

    # --- placement -------------------------------------------------------------------------
    placement_ok = bool(np.allclose(landed, blue_pose, atol=1e-9))
    res.add(KEYS['placement'], 'PASS' if placement_ok else 'FAIL',
            f'the blue part was placed at {np.round(blue_pose, 5)} and the COMPILED model holds it '
            f'at {np.round(landed, 5)} (asserted to 1e-9, through the free joint\'s qpos). The '
            f'bench top {bench_z:.4f} m is read off a_table\'s own geom position + half-height, '
            f'not typed -- an earlier version carried BENCH_TOP = 0.24 with a comment claiming it '
            f'came from the geometry, and the geometry says 0.20.')

    # --- calibration -----------------------------------------------------------------------
    res.add(KEYS['calibration'], 'PASS' if err < tol_cal else 'FAIL',
            f'forward=axis{conv["forward"]} fwd_sign={conv["fwd_sign"]:+.0f} '
            f'up_sign={conv["up_sign"]:+.0f} h_sign={conv["v_sign"]:+.0f}; score {err * 1000:.2f} '
            f'mm (tolerance {tol_cal * 1000:.0f} mm) = the red part\'s own {det.get("red_n")} '
            f'labelled pixels missing its own pose by {det.get("red_xy", 0) * 1000:.2f} mm in x,y '
            f'and {det.get("red_z", 0) * 1000:.2f} mm at the top, plus the bench\'s '
            f'{det.get("bench_n")} labelled pixels missing the bench\'s own footprint by '
            f'{det.get("bench", 0) * 1000:.2f} mm. A mirrored cloud still looks plausible, so the '
            f'label set is the calibration board and the pipeline never sees it.')

    observable = rival_s >= 2.0 * LOC_TOL
    res.add(KEYS['observable'], 'PASS' if observable else 'FAIL',
            f'24 candidates ranked. Winner {err * 1000:.2f} mm; the next best is '
            f'{rival_s * 1000:.2f} mm ({rival_c}) -- a margin of {rival_s - err:.3f} m, i.e. '
            f'{(rival_s - err) / LOC_TOL:.1f}x the {LOC_TOL * 1000:.0f} mm localization tolerance. '
            f'This is not automatic: fitted on the parts alone, a HORIZONTALLY mirrored convention '
            f'scored 9.005 mm against the winner\'s 9.005 mm, because both parts sit on the image '
            f'centre column (cols 154..165 of W=320, centre 160) and a horizontal mirror maps each '
            f'part\'s own pixels back onto itself. The bench is 70 mm off the optical axis and '
            f'breaks that tie. A calibration target on the optical axis is not a target.')

    # --- coordinate validation: the cloud has the part's own size --------------------------
    rows, found = localize(per, truth, g)

    def footprint_ok(rows_):
        bad = []
        for r_ in rows_:
            e = r_['bbox_extent_m']
            if e is None:
                return False, f'{r_["truth"]} was not found at all'
            for k, nm in ((0, 'x'), (1, 'y')):
                if e[k] > 2.0 * g[k] + 4.0 * gsd:
                    bad.append(f'{r_["truth"]} {nm} extent {e[k] * 1000:.1f} mm exceeds '
                               f'{2 * g[k] * 1000:.1f} + {4 * gsd * 1000:.1f} mm')
        return not bad, '; '.join(bad)
    fok, fwhy = footprint_ok(rows)
    res.add(KEYS['footprint'], 'PASS' if fok else 'FAIL',
            '; '.join(f'{r["truth"]}: world x extent {(r["bbox_extent_m"] or [0, 0, 0])[0] * 1000:.1f}'
                      f' mm, y {(r["bbox_extent_m"] or [0, 0, 0])[1] * 1000:.1f} mm, against the '
                      f'part\'s own {2 * g[0] * 1000:.1f} x {2 * g[1] * 1000:.1f} mm'
                      for r in rows)
            + f'. Tolerance 4 pixels of ground sample distance ({4 * gsd * 1000:.1f} mm at '
              f'{float(np.median(per["depth"][per["ok"]])):.3f} m). A cloud scaled or scrambled by '
              f'a wrong range model would not have the part\'s own size.'
            + (f' VIOLATIONS: {fwhy}' if fwhy else ''))

    # --- visibility ------------------------------------------------------------------------
    vis = {b: int((seg == _geom_of_body(model, b)).sum()) for b in (RED, BLUE)}
    vok = all(n >= MIN_PART_PIXELS for n in vis.values())
    res.add(KEYS['visibility'], 'PASS' if vok else 'FAIL',
            'labelled pixels: ' + '; '.join(f'{k} {n}' for k, n in vis.items())
            + f' (a part needs at least {MIN_PART_PIXELS}). Measured with the segmentation '
              f'renderer, which is an ORACLE used only for this audit and for calibration -- '
              f'reported separately so that an occlusion failure cannot be misread as a detector '
              f'failure. It replaces a baked-in comment claiming the arm\'s hand hid the second '
              f'part: the hand never hid it, the part was parked at the world origin.')

    # --- bench separation ------------------------------------------------------------------
    near = per['ok'] & (per['depth'] > 0.01)
    res.add(KEYS['bench'], 'PASS' if near.sum() > 1000 else 'FAIL',
            f'{int(near.sum())} valid depth pixels; the bench reads '
            f'{float(np.median(per["depth"][near])):.4f} m. Segmentation keeps only points above '
            f'{bench_z:.4f} m inside the bench footprint x[{roi[0]:.3f},{roi[1]:.3f}] '
            f'y[{roi[2]:.3f},{roi[3]:.3f}]. The plane is removed by a height threshold plus a '
            f'footprint, not by RANSAC.')

    # --- clutter rejected -------------------------------------------------------------------
    off = [c for c in per['comps'] if not c['on_bench']]
    res.add(KEYS['clutter'], 'PASS' if off else 'FAIL',
            f'{len(per["comps"])} components above the bench; rejected by the "rests on the bench" '
            f'test: ' + '; '.join(f'#{i} {c["n_pixels"]}px top {c["z_max"]:.4f} m '
                                  f'({c["z_max"] - bench_z:.4f} m above the bench, limit '
                                  f'{2 * g[2] + ON_BENCH_TOL:.4f}) rgb {np.round(c["rgb"], 1)} '
                                  f'-> {c["cls"]}'
                                  for i, c in enumerate(off))
            + '. The arm\'s hand is a real occluder-shaped object that is NOT a part, and a matcher '
              'that cannot say so is measuring proximity rather than detecting parts. Note the two '
              'discriminators are independent: here the hand is grey, so the chroma test alone '
              'would exclude it, and the on-bench test is the second line of defence against a '
              'chromatic object that is not resting on the bench.')

    # --- localization ------------------------------------------------------------------------
    worst = max((r['xy_error_m'] for r in rows if r['xy_error_m'] is not None), default=float('nan'))
    res.add(KEYS['localization'], 'PASS' if localize_ok(rows) else 'FAIL',
            'the two parts matched to: ' + '; '.join(
                f'{r["truth"]} ({r["class"]}): ' +
                (f'x,y error {r["xy_error_m"] * 1000:.1f} mm over {r["n_pixels"]} px, from '
                 f'{r["n_candidates"]} candidate(s)' if r['n_pixels']
                 else f'NOT FOUND ({r["n_candidates"]} eligible candidates)')
                for r in rows)
            + f' (tolerance {LOC_TOL * 1000:.0f} mm; each part must show at least '
              f'{MIN_PART_PIXELS} px and have exactly ONE eligible candidate). The parts stand '
              f'{PART_CLEARANCE:.2f} m apart face-to-face, so "separated, not piled up" is the '
              f'placement, not a hope. z is not compared to the body origin: depth renders the TOP '
              f'surface, so the comparable z is bench_z + 2*half = {bench_z + 2 * g[2]:.4f} m and '
              f'the measured tops are '
            + '; '.join(f'{r["truth"]} ' +
                        (f'{(r["z_max"] - (bench_z + 2 * g[2])) * 1000:+.1f} mm'
                         if r['z_max'] is not None else 'n/a') for r in rows) + ' from it.')

    # --- chroma ------------------------------------------------------------------------------
    pairs = [(r['truth'], np.array(found[r['truth']]['rgb'])) for r in rows if found[r['truth']]]
    res.add(KEYS['chroma'], 'PASS' if chroma_ok(pairs) else 'FAIL',
            '; '.join(f'{k}: mean RGB {np.round(c, 1)} -> {chroma_class(c)}' for k, c in pairs)
            + f'. Each class must beat the other channels by {CHROMA_RATIO}x. The arm\'s hand '
              f'measures [93.5 93.7 93.8] and the old ordering-only rule calls that '
              f'"{order_only_class(HAND_RGB)}" -- blue beats red by 0.3 of a level. That is the '
              f'rule this version replaced.')

    # --- stream ------------------------------------------------------------------------------
    frame_s = per['render_s'] + per['total_s']
    res.add(KEYS['stream'], 'PASS' if frame_s > 0 else 'FAIL',
            f"one RGB-D frame costs {frame_s * 1000:.1f} ms here at {W}x{H} = {1.0 / frame_s:.1f} "
            f"Hz: {per['render_s'] * 1000:.1f} ms of it is the two render passes (colour and "
            f"depth) and {per['total_s'] * 1000:.1f} ms is the perception step (unproject + "
            f"segment + classify). The render dominates, and it is the simulator's offscreen GL in "
            f"a headless container, not a camera property -- so this is a compute latency on this "
            f"machine, not a sensor latency. The observation carries frame={CAM!r} (a NAMED camera "
            f"in the model, so the frame is resolvable rather than implied), t_wall, age computed "
            f"by the consumer, and fx=fy={per['fx']:.1f} px. No real sensor is modelled.")

    return {'rows': rows, 'found': found, 'pairs': pairs, 'footprint_ok': footprint_ok,
            'tol_cal': tol_cal, 'observable': observable}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='p3-vision-02')
    args = ap.parse_args(argv)

    model, bench_z, blue_pose = build_scene()
    d = mujoco.MjData(model)
    prepare(model, d, poses={BLUE: blue_pose})
    landed = pose_of(model, d, BLUE)
    if not np.allclose(landed, blue_pose, atol=1e-9):
        raise SystemExit(f'the blue part did not land where it was placed: {landed} vs {blue_pose}')
    g = part_size(model)
    truth = {RED: pose_of(model, d, RED), BLUE: landed}

    view = View(model)
    t0 = time.perf_counter()
    rgb = view.capture(d, 'rgb')
    depth = view.capture(d, 'depth')
    render_s = time.perf_counter() - t0
    seg = view.capture(d, 'seg')

    def fresh(dd):
        return view.capture(dd, 'rgb'), view.capture(dd, 'depth')

    err, conv, ranked = calibrate(model, d, depth, seg, bench_z, g, truth[RED])
    per = analyze(model, d, rgb, depth, bench_z, g, conv)
    per['render_s'] = render_s

    res = Results()
    h = judge(res, model, d, per, seg, bench_z, g, conv, ranked, truth, landed, blue_pose)
    rows, found, pairs = h['rows'], h['found'], h['pairs']
    tol_cal = h['tol_cal']

    # =============================== falsifiability ==========================================
    probes, coverage = [], []

    def run(key, breaks, what, fn):
        try:
            fired = bool(fn())
        except Exception as exc:
            fired, what = False, f'{what} -- probe raised {exc!r}'
        probes.append({'probe': key, 'breaks': breaks, 'what': what,
                       'made_the_check_fail': fired})
        if fired:
            coverage.extend(breaks)

    def p_readback():
        m2, _bz, _bp = build_scene()
        gid = _geom_of_body(m2, RED)
        m2.geom_rgba[gid] = BLUE_RGBA          # the blue colour, on the WRONG object
        got = [round(float(v), 3) for v in m2.geom_rgba[gid]]
        return got != [round(v, 3) for v in RED_RGBA]

    def p_blue_parked():
        dd = mujoco.MjData(model)
        prepare(model, dd)                      # keyframe only -> the blue part back at the ORIGIN
        return not np.allclose(pose_of(model, dd, BLUE), blue_pose, atol=1e-9)

    def p_rival_convention():
        s, _det = calibration_error(model, d, depth, ranked[1][1], seg, bench_z, g, truth[RED])
        return not (s < tol_cal)

    def p_depth_scale():
        p2 = analyze(model, d, rgb, depth * 1.5, bench_z, g, conv)
        r2, _f2 = localize(p2, truth, g)
        return not h['footprint_ok'](r2)[0]

    def p_parts_away():
        dd = mujoco.MjData(model)
        prepare(model, dd, poses={RED: RED_AWAY, BLUE: RED_AWAY})
        r2, _f2 = localize(analyze(model, dd, *fresh(dd), bench_z, g, conv), truth, g)
        return not localize_ok(r2)

    def p_off_bench():
        dd = mujoco.MjData(model)
        prepare(model, dd, poses={BLUE: blue_pose + np.array([0.0, 0.0, 0.30])})
        r2, _f2 = localize(analyze(model, dd, *fresh(dd), bench_z, g, conv), truth, g)
        return not localize_ok(r2)

    def p_no_on_bench():
        real = globals()['rests_on_bench']
        try:
            globals()['rests_on_bench'] = lambda *a, **k: True
            dd = mujoco.MjData(model)
            prepare(model, dd, poses={BLUE: blue_pose + np.array([0.0, 0.0, 0.30])})
            r2, _f2 = localize(analyze(model, dd, *fresh(dd), bench_z, g, conv), truth, g)
            return not localize_ok(r2)
        finally:
            globals()['rests_on_bench'] = real

    def p_v1_rule_set():
        """Restore v1's WHOLE rule set: ordering-only colours AND no on-bench test.

        Either one alone is NOT enough to break the localization, and that is the point. The hand
        is grey, so the chroma test excludes it; the hand floats 0.4022 m above the bench, so the
        on-bench test excludes it. The two discriminators are independent, and v1 had neither --
        which is how it matched a 1215 px hand to the blue part and called the result a 46.9 mm
        localization error. Disabling one only is masked by the other.
        """
        real_c = globals()['chroma_class']
        real_r = globals()['rests_on_bench']
        try:
            globals()['chroma_class'] = order_only_class
            globals()['rests_on_bench'] = lambda *a, **k: True
            p2 = analyze(model, d, rgb, depth, bench_z, g, conv)
            r2, _f2 = localize(p2, truth, g)
            return not localize_ok(r2)
        finally:
            globals()['chroma_class'] = real_c
            globals()['rests_on_bench'] = real_r

    def p_grey_as_part():
        return not chroma_ok([(RED, pairs[0][1]), (BLUE, HAND_RGB)])

    run('colour-readback', ['colour'],
        'write the blue rgba onto the red part\'s geom; the readback must notice', p_readback)
    run('blue-parked', ['placement'],
        'restore the keyframe without placing the part; the placement assertion must notice',
        p_blue_parked)
    run('rival-convention', ['calibration', 'observable'],
        'score the genuinely different runner-up convention; the calibration must go red',
        p_rival_convention)
    run('depth-scale', ['footprint'],
        'scale every depth by 1.5 (a wrong range model); the footprint must stop matching',
        p_depth_scale)
    run('parts-away', ['visibility', 'localization'],
        'move both parts out of the arena through their qpos; nothing must be found', p_parts_away)
    run('off-bench', ['clutter', 'localization'],
        'lift the blue part 0.30 m clear of the bench; it must not be counted', p_off_bench)
    run('on-bench-test-disabled', ['localization'],
        'disable the "rests on the bench" predicate and lift the part; it must then be matched',
        p_no_on_bench)
    run('v1-rule-set', ['localization'],
        'restore v1\'s whole rule set (ordering-only colours and no on-bench test); the hand must '
        'then be matched to the blue part', p_v1_rule_set)
    run('grey-as-a-part', ['chroma'],
        'offer the arm\'s own grey as the blue part\'s colour; the chroma check must go red',
        p_grey_as_part)

    falsifiable = sorted(k for k in KEYS if k not in STRUCTURAL)
    covered = sorted(set(coverage))
    fired = [p for p in probes if p['made_the_check_fail']]
    judged_names = {r['name'] for r in res.rows}          # BEFORE this row is added
    contract_ok = (len(KEYS) == len(judged_names) and set(KEYS.values()) == judged_names)
    all_fired = len(fired) == len(probes)
    res.add('every check has been shown able to fail, or is declared structural with a reason',
            'PASS' if (contract_ok and all_fired and covered == falsifiable) else 'FAIL',
            f'the key table and the judged rows agree on {len(KEYS)} checks: {contract_ok}. '
            f'{len(fired)}/{len(probes)} probes fired. Falsifiable checks: {len(falsifiable)}; '
            f'covered by a firing probe: {len(covered)}. '
            + ('All covered. ' if covered == falsifiable
               else f'UNCOVERED: {sorted(set(falsifiable) - set(covered))}; MISSING ROWS: '
                    f'{sorted(set(covered) - set(falsifiable))}. ')
            + 'Declared structural, with the reason: '
            + '; '.join(f'"{KEYS[k]}" -- {v}' for k, v in STRUCTURAL.items())
            + '. Probes: ' + '; '.join(f'{p["probe"]}={"fired" if p["made_the_check_fail"] else "NO"}'
                                       for p in probes))

    # =============================== report ==================================================
    print('=' * 100)
    print('P3 VISION -- RGB-D over the bench, part localization, coordinate validation')
    print('=' * 100)
    print(f'  world     {WORLD.name} (sha256 {hashlib.sha256(WORLD.read_bytes()).hexdigest()[:16]})'
          f' + probe additions: {CAM!r}, {BLUE}, part geom colours')
    print(f'  bench top {bench_z:.4f} m DERIVED from a_table geom')
    print(f'  parts     {RED} {np.round(truth[RED], 4)}   {BLUE} {np.round(truth[BLUE], 4)}'
          f'   size {np.round(g, 4)}   gap {PART_CLEARANCE} m')
    print(f'  convention {conv}  score {err * 1000:.2f} mm  (next best '
          f'{ranked[1][0] * 1000:.2f} mm)')
    print(f'  render {render_s * 1000:.1f} ms, perception {per["total_s"] * 1000:.1f} ms/frame '
          f'at {W}x{H}')
    print()
    for r in res.rows:
        print(f"{'ok  ' if r['status'] == 'PASS' else 'FAIL'} [{r['status']}] {r['name']}")
        print(f'            {r["detail"]}')
    c = res.counts()
    print(f"\n{c['pass']}/{c['checks']} checks PASS -- verdict {res.verdict()}")

    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=True)
    (out / 'report.json').write_text(json.dumps({
        'world': WORLD.name,
        'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        'probe_additions': [CAM, BLUE, f'{BLUE}_geom', 'a_payload geom colour', 'cam fovy 45'],
        'supersedes': ('reports/p3-vision-01, whose blue part sat at the world origin because a '
                       '<keyframe> pads a later free joint with zero qpos, whose truth was a typed '
                       'constant, whose colour check passed on a grey blob, whose falsifiability '
                       'probe could not fire, and whose convention fit was a tie'),
        'resolution': [W, H],
        'camera_pos': [float(v) for v in d.cam_xpos[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, CAM)]],
        'camera_convention': conv,
        'calibration_score_m': err,
        'calibration_runner_up_m': ranked[1][0],
        'calibration_targets': ['a_payload (on the optical axis)', 'a_table (70 mm off the axis)'],
        'calibration_ranked_top6': [{'score_m': s, 'conv': cv, 'detail': dt}
                                    for s, cv, dt in ranked[:6]],
        'bench_top_m': bench_z,
        'part_size_m': [float(v) for v in g],
        'part_clearance_m': PART_CLEARANCE,
        'render_ms': render_s * 1000,
        'perception_ms': per['total_s'] * 1000,
        'components': [{'n_pixels': c_['n_pixels'], 'centroid': c_['centroid'],
                        'z_min': c_['z_min'], 'z_max': c_['z_max'],
                        'bbox_extent_m': c_['bbox_extent_m'],
                        'rgb': [float(v) for v in c_['rgb']], 'class': c_['cls'],
                        'on_bench': bool(c_['on_bench'])} for c_ in per['comps']],
        'localization': rows,
        'verdict': res.verdict(), 'counts': res.counts(), 'checks': res.rows,
        'structural_not_probed': {KEYS[k]: v for k, v in STRUCTURAL.items()},
        'falsifiability': probes,
    }, indent=1), encoding='utf-8')
    view.close()
    print(f'\nwrote reports/{args.run_id}/report.json')
    return 0 if res.verdict() == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
