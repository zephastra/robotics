"""Static loaded catalog candidate. NOT proof that runtime cargo stays in bounds."""
import argparse
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
import yaml
from w4_plant import collision_radius
from loaded_nav_envelope import combined_envelope

ROOT=Path(__file__).resolve().parents[1]


def generate():
    empty_path=ROOT/'config/joint_world_v5.profile.json'
    empty=json.loads(empty_path.read_text()); world=ROOT/'assets/world_p5_candidate_v5.xml'
    m=mujoco.MjModel.from_xml_path(str(world));d=mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
    if hashlib.sha256(world.read_bytes()).hexdigest()!=empty['world_sha256']:
        raise ValueError('empty profile world mismatch')
    base=d.body('n_base_link').xpos
    rotation=d.body('n_base_link').xmat.reshape(3,3)
    # Initialized support fixture only, before ANY physics. A declared allowed
    # cargo centre region, not a runtime body reading or a guarantee of retention.
    crowns=np.array([(d.geom_xpos[g]-base)@rotation for g in range(m.ngeom)
        if (m.geom(g).name or '').startswith('c_deck_roller')])
    if not len(crowns):raise ValueError('no deck support catalog')
    low=crowns[:,:2].min(axis=0);high=crowns[:,:2].max(axis=0)
    radius=collision_radius(m,m.body('c_payload').id)
    floor=m.geom('c_tray_floor').id
    # Catalog limits for the existing 50mm red boxes / 30mm blue cylinders.
    # Any other part must be rejected or separately commissioned.
    part_radius=float(np.linalg.norm([.02,.015,.025]))
    floor_radius=float(np.linalg.norm(abs(m.geom_pos[floor])+m.geom_size[floor]))
    loaded_radius=max(radius,floor_radius+2*part_radius)
    # Deck translation <= sqrt(2)*.01m. Any point's yaw displacement <= R*.01.
    allowance=float(np.sqrt(2)*.01+(np.max(np.linalg.norm(crowns[:,:2],axis=1))+loaded_radius)*.01)
    footprint=combined_envelope(empty['footprint'],low,high,loaded_radius,allowance)
    profile=dict(scope='DECLARED_LOADED_NAV_CANDIDATE_NOT_PHYSICAL_ACCEPTANCE',
        world_sha256=empty['world_sha256'],parent_profile_sha256=hashlib.sha256(empty_path.read_bytes()).hexdigest(),
        generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        footprint=footprint,cargo_radius_m=loaded_radius,mechanism_allowance_m=allowance,
        observation_contract=dict(centre_low=low.tolist(),centre_high=high.tolist(),max_sim_age_s=.5,max_wall_age_s=.5),
        catalog=['50mm red box: half-size .02/.015/.025 m','blue cylinder: radius .015, half-height .025 m'],
        runtime_gate='RGBD measured tray pose + verified quantity + ideal deck contact; NOT_WIRED',
        retention='NOT_RUN',loaded_navigation='NOT_RUN',full_order='NOT_RUN',v1_complete=False)
    params=yaml.safe_load((ROOT/'config/nav2_joint_world_v5.yaml').read_text())
    for name in ('local_costmap','global_costmap'):
        params[name][name]['ros__parameters']['footprint']=json.dumps(footprint)
    return {'config/nav2_joint_world_v5_loaded.yaml':yaml.safe_dump(params,sort_keys=False).encode(),
        'config/joint_world_v5_loaded.profile.json':(json.dumps(profile,indent=2)+'\n').encode()}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');a=p.parse_args()
    mismatch=[]
    for relative,raw in generate().items():
        path=ROOT/relative
        if a.check:
            if not path.exists() or path.read_bytes()!=raw:mismatch.append(relative)
        else:path.write_bytes(raw)
    print(json.dumps(dict(check=a.check,differences=mismatch)))
    return bool(mismatch)


if __name__=='__main__':raise SystemExit(main())
