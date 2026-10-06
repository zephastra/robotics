"""Generate independent v7 navigation commissioning inputs, not a V1 PASS.

Static map: initialized fixed geometry at lidar height, surveyed operator prior,
NOT SLAM. It is not a 3D collision model. Footprint covers the initialized
transport configuration (deck parked at home, +2.21 m ahead of the wheels, as
qualified by the G2 straight-line drive); the declared transport posture keeps
the deck slide parked, so the +1.5 m deck stroke is NOT part of the driving
footprint and any slide beyond the declared band is TRANSPORT_POSTURE_UNCONFIRMED.
Retainer shoe open/closed footprint effect was measured by an offline FK sweep
in a throwaway MjData: the assembly AABB is identical at both range extremes,
so the shoe contributes zero footprint expansion; both extremes and the closed
band are recorded in the profile instead.
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
WORLD = ROOT / 'assets/world_p5_candidate_v7_hinged_retainer.xml'
OLD_PARAMS = ROOT / 'config/nav2_joint_world_v5.yaml'
OLD_CFG = ROOT / 'config/n_probe.yaml'
POSTURE = {'c_deck_slide': [-.01, .01], 'c_deck_slide_y': [-.01, .01],
           'c_deck_yaw': [-.01, .01], 'c_pusher_slide_joint': [-.001, .391],
           'c_pusher_lift_joint': [-.001, .002],
           'c_retainer_-1_joint': [-.08, .05], 'c_retainer_1_joint': [-.08, .05]}


def transport_bounds(low, high):
    # Deck is declared parked at the qualified drive configuration; pusher
    # stroke 0.39 (+0.002 guard) is the only mechanism reach added on top of
    # the initialized self assembly. Rotational point displacement is bounded
    # by radius*|angle| with the declared 0.01 rad deck slack. The deck slide
    # stroke (ctrl range [-0.5, 1.5], joint unlimited) is deliberately NOT
    # included: sliding the deck while driving is outside the declared
    # transport posture and must fail the posture gate, not widen the footprint.
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


def retainer_sweep(model):
    # Offline FK sweep in throwaway MjData: assembly AABB at both shoe range
    # extremes. Read-only kinematics; no dynamics, no runtime sim.
    extremes = []
    for name in ('c_retainer_-1_joint', 'c_retainer_1_joint'):
        jid = model.joint(name).id
        extremes.append((name, float(model.jnt_range[jid][0])))
        extremes.append((name, float(model.jnt_range[jid][1])))
    bounds_at = {}
    for name, value in extremes:
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = value
        mujoco.mj_forward(model, data)
        lo, hi = robot_bounds(model, data, 'n_base_link')
        bounds_at['%s@%r' % (name, value)] = [lo.tolist(), hi.tolist()]
    return bounds_at


def cargo_envelope(model, data):
    lo_all, hi_all = None, None
    for geom in range(model.ngeom):
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom]))
        if body != 'c_payload': continue
        lo, hi = bounds(data.geom_xpos[geom], data.geom_xmat[geom].reshape(3, 3), model.geom_size[geom], model.geom_type[geom])
        lo_all = lo if lo_all is None else np.minimum(lo_all, lo)
        hi_all = hi if hi_all is None else np.maximum(hi_all, hi)
    if lo_all is None: raise ValueError('no cargo geoms under c_payload')
    return dict(note='tray initialized on the band front at home keyframe, NOT on the deck; deck loading happens later in the chain',
                half_extents_m=[float(x) for x in (hi_all - lo_all) / 2.],
                world_aabb=[[float(x) for x in lo_all], [float(x) for x in hi_all]])


def generate():
    model = mujoco.MjModel.from_xml_path(str(WORLD)); data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0); mujoco.mj_forward(model, data)
    # Known spawn from the asset, never runtime localization truth.
    start = data.body('n_base_link').xpos[:2].tolist()
    lidar_height = float(data.site('n_lidar_site').xpos[2])
    groups = classify(model, data, ['n_base_link', 'n2_base_link', 'h_LINK_BASE', 'a_link0'])
    built, mapped = map_grid(model, data, groups, start, lidar_height)
    low, high = transport_bounds(*robot_bounds(model, data, 'n_base_link'))
    points = [[low[0], low[1]], [high[0], low[1]], [high[0], high[1]], [low[0], high[1]]]
    points = [[float(value) for value in point] for point in points]
    params = yaml.safe_load(OLD_PARAMS.read_text())
    params['amcl']['ros__parameters']['initial_pose'] = dict(x=start[0], y=start[1], z=0., yaw=0.)
    for costmap in ('local_costmap', 'global_costmap'):
        section = params[costmap][costmap]['ros__parameters']
        section.pop('robot_radius', None); section['footprint'] = json.dumps(points)
    local = params['local_costmap']['local_costmap']['ros__parameters']
    local['width'] = 8; local['height'] = 8
    params['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] = .20
    follow = params['controller_server']['ros__parameters']['FollowPath']
    # G5 leg audit: the 0.95 m corridor cannot contain an in-place rotation of
    # the 3.565 x 0.681 m footprint; the straight leg starts yaw-aligned. If
    # the parent params ever drop this key, fail loudly instead of inheriting.
    if 'use_rotate_to_heading' not in follow: raise ValueError('parent lacks use_rotate_to_heading')
    follow['use_rotate_to_heading'] = False
    monitor = params['collision_monitor']['ros__parameters']
    # This installed profile consumes the costmap's live footprint. Do not
    # invent a PolygonStop key from a different Nav2 release.
    if monitor['polygons'] != ['FootprintApproach'] or monitor['FootprintApproach']['footprint_topic'] != 'local_costmap/published_footprint':
        raise ValueError('unsupported collision monitor footprint binding')
    cfg = yaml.safe_load(OLD_CFG.read_text())
    cfg['world'] = 'assets/world_p5_candidate_v7_hinged_retainer.xml'
    cfg['sim']['realtime_factor'] = .25
    profile = dict(scope='V7_CANDIDATE_TRANSPORT_IDENTITY_NOT_ORDER', start=start,
                   goal=[start[0] + .6, start[1], 0.], footprint=points,
                   transport_posture=POSTURE, initialized_posture=posture_check(model, data),
                   initialized_posture_note='allowed=false at scene start is CORRECT, not a defect: shoes are OPEN at home (ctrl 1.5708) and the tray is on the band; transport posture is reached only after deck loading and shoe closing in the chain. The posture gate is a transport-time gate.',
                   retention_identity=dict(joints=['c_retainer_-1_joint', 'c_retainer_1_joint'],
                                           closed_rad=-.08, open_rad=1.5707963267948966,
                                           home_ctrl_rad=1.5707963267948966,
                                           home_state='OPEN: geometric proof, shoe tip 25 mm clear of tray wall at open extreme',
                                           closed_band_rad=[-.08, .05],
                                           closed_band_derivation='WORKING-REGION band, not a single-scene steady state: G1 measured closed settle -0.000636 rad (probe_tray_retainer, declared PRELOAD_ANGLE -0.025), first v7 nav run measured -0.033530 rad symmetric under the same command with tray on deck (p4-nav2-v7-02) -- the working point varies with load/contact. Band spans the closed side: lower = joint hard limit -0.08, upper = +0.05 (31x below OPEN 1.5708; any real opening runs toward +1.5708). History: G5 mint v1 [-0.08,-0.079] (hard limit mistaken for working point) and v2 centre -0.000636 (single-fact) were each caught fail-closed by runs 01/02.',
                                           footprint_effect='NONE: offline FK sweep, assembly AABB identical at both range extremes',
                                           fk_sweep_bounds=retainer_sweep(model)),
                   footprint_basis=dict(home_assembly_aabb=[[float(x) for x in robot_bounds(model, data, 'n_base_link')[0]],
                                                            [float(x) for x in robot_bounds(model, data, 'n_base_link')[1]]],
                                        deck_offset_note='deck frame parks 2.21 m ahead of the wheels at home; this offset is inside the home AABB and IS covered',
                                        deck_slide_stroke_excluded=[-.5, 1.5],
                                        deck_slide_exclusion_reason='declared transport posture keeps the deck parked; sliding while driving is unqualified'),
                   planner_constraints=dict(rotate_to_heading_disabled=True,
                                            reason='0.95 m corridor cannot contain an in-place rotation of the 3.565 x 0.681 m footprint (G5 leg audit); the straight leg starts yaw-aligned',
                                            spin_recovery_risk='stock BT recovery may still request Spin; declared risk for the first run, run evidence will decide'),
                   cargo_envelope=cargo_envelope(model, data),
                   map_source='INITIAL_FIXED_GEOMETRY_SURVEY_NOT_SLAM', mapped_geoms=mapped,
                   static_map_approximation='scan-height AABB, not 3D collision avoidance',
                   world_sha256=hashlib.sha256(WORLD.read_bytes()).hexdigest(),
                   parent_nav_params_sha256=hashlib.sha256(OLD_PARAMS.read_bytes()).hexdigest(),
                   generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    metadata = dict(image='joint_world_v7.pgm', resolution=built['resolution'], origin=[*built['origin'], 0.],
                    negate=0, occupied_thresh=.65, free_thresh=.196)
    profile_bytes = (json.dumps(profile, indent=2) + '\n').encode()
    # Loaded-profile companion: the acceptance window is centred on the MEASURED
    # deck centre in the base frame; margins (x +/-0.28, y +/-0.125) are the
    # commissioned v5 contract geometry for the same deck design; the mechanism
    # allowance 0.0422 m is the G4-measured v7 unload envelope, not a reused v5
    # number; cargo radius is the measured tray half-width incl. handles.
    deck_lo, deck_hi = None, None
    base_pos = data.body('n_base_link').xpos
    for geom in range(model.ngeom):
        if int(model.geom_bodyid[geom]) != model.body('c_deck').id: continue
        lo, hi = bounds(data.geom_xpos[geom], data.geom_xmat[geom].reshape(3, 3), model.geom_size[geom], model.geom_type[geom])
        deck_lo = lo if deck_lo is None else np.minimum(deck_lo, lo)
        deck_hi = hi if deck_hi is None else np.maximum(deck_hi, hi)
    deck_centre_x = float((deck_lo[0] + deck_hi[0]) / 2. - base_pos[0])
    cargo_radius = float(cargo_envelope(model, data)['half_extents_m'][1])
    allowance = .0422
    y_half = cargo_radius + .125 + allowance
    loaded = dict(scope='DECLARED_LOADED_NAV_CANDIDATE_NOT_PHYSICAL_ACCEPTANCE_V7',
                  world_sha256=profile['world_sha256'],
                  parent_profile_sha256=hashlib.sha256(profile_bytes).hexdigest(),
                  generator_sha256=profile['generator_sha256'],
                  footprint=[[low[0], -y_half], [high[0], -y_half], [high[0], y_half], [low[0], y_half]],
                  cargo_radius_m=cargo_radius, mechanism_allowance_m=allowance,
                  deck_centre_base_x_m=deck_centre_x,
                  observation_contract=dict(centre_low=[deck_centre_x - .28, -.125],
                                            centre_high=[deck_centre_x + .28, .125],
                                            max_sim_age_s=.5, max_wall_age_s=.5),
                  catalog=['50mm red box: half-size .02/.015/.025 m', 'blue cylinder: radius .015, half-height .025 m'],
                  runtime_gate='RGBD measured tray pose + verified quantity + ideal deck contact; NOT_WIRED',
                  retention='DECLARED_INIT_ONLY: shoes closed at scene start by the candidate probe; NOT runtime loading evidence',
                  loaded_navigation='NOT_RUN', full_order='NOT_RUN', v1_complete=False)
    return {'assets/maps/joint_world_v7.pgm': pgm_bytes(built),
            'assets/maps/joint_world_v7.yaml': yaml.safe_dump(metadata).encode(),
            'config/nav2_joint_world_v7.yaml': yaml.safe_dump(params, sort_keys=False).encode(),
            'config/joint_world_v7.yaml': yaml.safe_dump(cfg, sort_keys=False).encode(),
            'config/joint_world_v7.profile.json': profile_bytes,
            'config/joint_world_v7_loaded.profile.json': (json.dumps(loaded, indent=2) + '\n').encode()}


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
