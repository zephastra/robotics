"""Zero-physics projected-footprint diagnostic, NOT a collision acceptance."""
import argparse
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from nav_geometry_inventory import classify,bounds

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-id',required=True)
    parser.add_argument('--centered-load',action='store_true')
    args=parser.parse_args();out=ROOT/'reports'/args.run_id;out.mkdir(exist_ok=False)
    world=ROOT/'assets/world_p5_candidate_v5.xml'
    model=mujoco.MjModel.from_xml_path(str(world));data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,0);mujoco.mj_forward(model,data)
    groups=classify(model,data,['n_base_link','n2_base_link','h_LINK_BASE','a_link0'])
    empty=json.loads((ROOT/'config/joint_world_v5.profile.json').read_text())
    loaded=json.loads((ROOT/('config/joint_world_v5_centered.profile.json' if args.centered_load
                             else 'config/joint_world_v5_loaded.profile.json')).read_text())
    start=np.asarray(empty['start']);height=float(data.site('n_lidar_site').xpos[2])
    overlaps={}
    for key,profile in [('empty',empty),('loaded',loaded)]:
        points=np.asarray(profile['footprint'])+start
        low,high=points.min(axis=0),points.max(axis=0)
        result=[]
        for name in groups['static']:
            geom=model.geom(name).id
            lo,hi=bounds(data.geom_xpos[geom],data.geom_xmat[geom].reshape(3,3),
                model.geom_size[geom],model.geom_type[geom])
            if lo[2]<=height<=hi[2] and np.all(hi[:2]>=low) and np.all(lo[:2]<=high):
                result.append(dict(name=name,lo=lo.tolist(),hi=hi.tolist(),
                    overlap_width=(np.minimum(hi[:2],high)-np.maximum(lo[:2],low)).tolist()))
        overlaps[key]=result
    report=dict(scope='INITIAL_STATIC_SCAN_HEIGHT_AABB_OVERLAP_ZERO_PHYSICS',physics_steps=0,
        world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),start=start.tolist(),
        lidar_height_m=height,projected_overlap=overlaps,
        result='DIAGNOSTIC_ONLY',nav2='NOT_RUN',full_order='NOT_RUN',v1_complete=False,
        limitations=['static initialized AABB, not live costmap or exact collision',
                     'no inflation/localization drift, no runtime moving geometry',
                     'must not remove obstacles or shrink a safety envelope from this report alone'])
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
