"""Generate independent v5 navigation commissioning inputs, not a V1 PASS.

Static map: initialized fixed geometry at lidar height, surveyed operator prior,
NOT SLAM. It is not a 3D collision model. Footprint conservatively covers the
mechanism's declared candidate transport posture; actuator range is not a proof.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
import yaml
from nav_geometry_inventory import classify, bounds, robot_bounds
from make_map_n import pgm_bytes

ROOT = Path(__file__).resolve().parents[1]
POSTURE = {'c_deck_slide': [-.01, .01], 'c_deck_slide_y': [-.01, .01],
           'c_deck_yaw': [-.01, .01], 'c_pusher_slide_joint': [-.001, .391],
           'c_pusher_lift_joint': [-.001, .002]}


def transport_bounds(low, high):
    # All future pusher extensions, plus measured-posture guarded deck motion.
    # Rotational point displacement <= radius*|angle|; 3D radius from the entire
    # initialized self assembly plus the maximum pusher translation is an upper bound.
    low, high = np.asarray(low, float), np.asarray(high, float)
    if low.shape != (3,) or high.shape != (3,) or not np.all(np.isfinite([low, high])) or np.any(high < low):
        raise ValueError('invalid assembly envelope')
    radius = np.linalg.norm(np.maximum(abs(low), abs(high))) + .392
    angular = radius * .01
    expansion = np.array([.402 + angular, .01 + angular, .003 + angular])
    return low - expansion, high + expansion


def posture_check(model, data):
    failures = []
    for name, (lo, hi) in POSTURE.items():
        joint = model.joint(name).id
        value = float(data.qpos[model.jnt_qposadr[joint]])
        if not np.isfinite(value) or not lo <= value <= hi:
            failures.append(dict(joint=name, value=value, allowed=[lo, hi]))
    return dict(allowed=not failures, reason_code=None if not failures else 'TRANSPORT_POSTURE_UNCONFIRMED', failures=failures)


def map_grid(model, data, groups, start, height, resolution=.05):
    geoms = []
    for geom in range(model.ngeom):
        name = model.geom(geom).name or '@geom%d' % geom
        if name not in groups['static']: continue
        lo, hi = bounds(data.geom_xpos[geom], data.geom_xmat[geom].reshape(3, 3), model.geom_size[geom], model.geom_type[geom])
        if lo[2] <= height <= hi[2]: geoms.append((name, lo, hi))
    if not geoms: raise ValueError('no stationary scan-height geometry')
    low = np.floor(np.min([entry[1][:2] for entry in geoms], axis=0) / resolution) * resolution - 1.
    high = np.ceil(np.max([entry[2][:2] for entry in geoms], axis=0) / resolution) * resolution + 1.
    width, rows = np.ceil((high - low) / resolution).astype(int)
    if not 0 < width * rows <= 1000000: raise ValueError('unbounded map size')
    yy, xx = np.meshgrid(low[1] + (np.arange(rows)[::-1] + .5) * resolution,
                         low[0] + (np.arange(width) + .5) * resolution, indexing='ij')
    grid = np.zeros((rows, width), np.uint8)
    for name, lo, hi in geoms:
        # Axis-aligned world bounds are conservative; report this approximation.
        grid[(xx >= lo[0]) & (xx <= hi[0]) & (yy >= lo[1]) & (yy <= hi[1])] = 100
    col, row = np.floor((np.asarray(start) - low) / resolution).astype(int)
    row = rows - 1 - row
    if not (0 <= row < rows and 0 <= col < width) or grid[row, col]: raise ValueError('declared start not free in static prior')
    reachable = np.zeros_like(grid, bool); todo = [(row, col)]; reachable[row, col] = True
    while todo:
        r, c = todo.pop()
        for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < width and not grid[nr, nc] and not reachable[nr, nc]:
                reachable[nr, nc] = True; todo.append((nr, nc))
    grid[~reachable] = 100
    return dict(grid=bytearray(grid.ravel()), width=int(width), height=int(rows), origin=low.tolist(), resolution=resolution), [name for name, _, _ in geoms]


def generate():
    world = ROOT / 'assets/world_p5_candidate_v5.xml'
    model = mujoco.MjModel.from_xml_path(str(world)); data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0); mujoco.mj_forward(model, data)
    # Known spawn from the asset, never runtime localization truth.
    start = data.body('n_base_link').xpos[:2].tolist()
    lidar_height = float(data.site('n_lidar_site').xpos[2])
    groups = classify(model, data, ['n_base_link', 'n2_base_link', 'h_LINK_BASE', 'a_link0'])
    built, mapped = map_grid(model, data, groups, start, lidar_height)
    low, high = transport_bounds(*robot_bounds(model, data, 'n_base_link'))
    points = [[low[0], low[1]], [high[0], low[1]], [high[0], high[1]], [low[0], high[1]]]
    points = [[float(value) for value in point] for point in points]
    old = ROOT / 'config/nav2_n_probe.yaml'
    params = yaml.safe_load(old.read_text())
    params['amcl']['ros__parameters']['initial_pose'] = dict(x=start[0], y=start[1], z=0., yaw=0.)
    for costmap in ('local_costmap', 'global_costmap'):
        section = params[costmap][costmap]['ros__parameters']
        section.pop('robot_radius', None); section['footprint'] = json.dumps(points)
    local = params['local_costmap']['local_costmap']['ros__parameters']
    local['width'] = 8; local['height'] = 8
    params['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] = .20
    monitor = params['collision_monitor']['ros__parameters']
    # This installed profile consumes the costmap's live footprint. Do not
    # invent a PolygonStop key from a different Nav2 release.
    if monitor['polygons'] != ['FootprintApproach'] or monitor['FootprintApproach']['footprint_topic'] != 'local_costmap/published_footprint':
        raise ValueError('unsupported collision monitor footprint binding')
    cfg = yaml.safe_load((ROOT / 'config/n_probe.yaml').read_text())
    cfg['world'] = 'assets/world_p5_candidate_v5.xml'
    cfg['sim']['realtime_factor'] = .25
    profile = dict(scope='EMPTY_SHARED_WORLD_NAV2_COMMISSIONING_NOT_ORDER', start=start,
                   goal=[start[0] + .6, start[1], 0.], footprint=points,
                   transport_posture=POSTURE, initialized_posture=posture_check(model, data),
                   cargo_envelope='NOT_COMMISSIONED: empty first, loaded requires catalog containment',
                   map_source='INITIAL_FIXED_GEOMETRY_SURVEY_NOT_SLAM', mapped_geoms=mapped,
                   static_map_approximation='scan-height AABB, not 3D collision avoidance',
                   world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
                   parent_nav_params_sha256=hashlib.sha256(old.read_bytes()).hexdigest(),
                   generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    metadata = dict(image='joint_world_v5.pgm', resolution=built['resolution'], origin=[*built['origin'], 0.],
                    negate=0, occupied_thresh=.65, free_thresh=.196)
    return {'assets/maps/joint_world_v5.pgm': pgm_bytes(built),
            'assets/maps/joint_world_v5.yaml': yaml.safe_dump(metadata).encode(),
            'config/nav2_joint_world_v5.yaml': yaml.safe_dump(params, sort_keys=False).encode(),
            'config/joint_world_v5.yaml': yaml.safe_dump(cfg, sort_keys=False).encode(),
            'config/joint_world_v5.profile.json': (json.dumps(profile, indent=2) + '\n').encode()}


def main():
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--check', action='store_true'); args = parser.parse_args()
    files = generate(); differences = []
    for name, raw in files.items():
        path = ROOT / name
        if args.check:
            if not path.is_file() or path.read_bytes() != raw: differences.append(name)
        else:
            path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(raw)
    print(json.dumps(dict(check=args.check, files=list(files), differences=differences)))
    return int(bool(differences))


if __name__ == '__main__': raise SystemExit(main())
