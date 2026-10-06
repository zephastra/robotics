"""Independent conservative CONTROL candidate; no acceptance limits changed.

Evidence motivating this experiment: c_deck_yaw crosses +/-0.01rad and tray
slips 29.8mm on the original real loaded Nav2 run. Lower command acceleration
is a hypothesis, NOT proof of retention or a derived mechanical guarantee.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]
PARENT='config/nav2_joint_world_v5_centered.yaml'
OUT='config/nav2_joint_world_v5_retained.yaml'


def candidate(parent):
    params=copy.deepcopy(parent)
    controller=params['controller_server']['ros__parameters']
    path=controller['FollowPath']
    path.update(desired_linear_vel=.15,regulated_linear_scaling_min_speed=.05,
        rotate_to_heading_angular_vel=.15,max_angular_accel=.01)
    # Controller convergence must be tighter than SkillAdapter's ORIGINAL
    # 0.12m approach acceptance. Independent arrival judges remain unchanged.
    for name in controller['goal_checker_plugins']:
        controller[name]['xy_goal_tolerance']=.08
    smoother=params['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[.15,0.,.15],min_velocity=[-.15,0.,-.15],
        max_accel=[.05,0.,.01],max_decel=[-.05,0.,-.01])
    return params


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');a=p.parse_args()
    raw=(ROOT/PARENT).read_bytes()
    output=yaml.safe_dump(candidate(yaml.safe_load(raw)),sort_keys=False).encode()
    metadata=dict(scope='UNQUALIFIED_LOADED_MOTION_CONTROL_CANDIDATE',
        parent=PARENT,parent_sha256=hashlib.sha256(raw).hexdigest(),
        generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        params=OUT,params_sha256=hashlib.sha256(output).hexdigest(),
        acceptance_changes=[],qualified=False,
        hypothesis='lower linear/angular command acceleration reduces slip and deck yaw; MUST be physically tested',
        limitations=['not an emergency brake guarantee','no 3D swept-volume guard','not a full order'])
    files={OUT:output,'config/joint_world_v5_retained.profile.json':(json.dumps(metadata,indent=2)+'\n').encode()}
    mismatches=[]
    for name,content in files.items():
        path=ROOT/name
        if a.check:
            if not path.exists() or path.read_bytes()!=content:mismatches.append(name)
        else:path.write_bytes(content)
    print(json.dumps(dict(check=a.check,mismatches=mismatches,files=list(files))))
    return int(bool(mismatches))


if __name__=='__main__':raise SystemExit(main())
