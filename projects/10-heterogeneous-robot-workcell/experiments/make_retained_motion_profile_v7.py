"""v7 independent conservative CONTROL candidates; no acceptance limits changed.

Two single-variable lineages on the v7 nav2 identity (parent config/nav2_joint_world_v7.yaml):

1. retained  (original lineage): lower LINEAR+ANGULAR command acceleration.
   Tested in run-04 (p4-nav2-v7-04-retained) and REFUTED: tray ratchet slip
   worsened 5.216 -> 11.446 mm.
2. turn      (--angular-only lineage): lower ANGULAR-axis limits ONLY, linear
   speed unchanged at the base values. Motivated by the D1/D2 attribution
   (p4-g6-yaw-d1-seg1 / p4-g6-yaw-d2-seg8): straight legs up to 4.63 m with
   stop-go segmentation produce ~0 yaw and ~0.1 mm slip; only the Nav2
   angular command stream excites deck yaw (-0.011011 rad in run-05) and
   tray ratchet slip (25.3 mm). Prediction under test: lowering ONLY the
   angular axis returns yaw inside the band and cuts slip.

Scene stays the v7 base loaded transport identity (shoes closed at start).
Both outputs are UNQUALIFIED control candidates; the qualified mechanism is
mirrored from experiments/make_retained_motion_profile.py (v5).
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]
PARENT='config/nav2_joint_world_v7.yaml'
OUT_RETAINED='config/nav2_joint_world_v7_retained.yaml'
OUT_TURN='config/nav2_joint_world_v7_turn.yaml'


def candidate_retained(parent):
    params=copy.deepcopy(parent)
    controller=params['controller_server']['ros__parameters']
    path=controller['FollowPath']
    path.update(desired_linear_vel=.15,regulated_linear_scaling_min_speed=.05,
        rotate_to_heading_angular_vel=.15,max_angular_accel=.01)
    # Same values as v5: controller convergence tighter than the ORIGINAL
    # 0.12m approach acceptance. Independent arrival judges remain unchanged.
    # use_rotate_to_heading stays false from the v7 parent (corridor constraint).
    for name in controller['goal_checker_plugins']:
        controller[name]['xy_goal_tolerance']=.08
    smoother=params['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[.15,0.,.15],min_velocity=[-.15,0.,-.15],
        max_accel=[.05,0.,.01],max_decel=[-.05,0.,-.01])
    return params


def candidate_turn(parent):
    # Single-variable lineage vs base: ONLY the angular axis is lowered.
    # Linear values are the untouched v7 parent values (desired_linear_vel 0.2,
    # regulated min 0.15, smoother linear 1.0/-1.0, goal tolerance 0.15).
    params=copy.deepcopy(parent)
    controller=params['controller_server']['ros__parameters']
    controller['FollowPath']['max_angular_accel']=.01
    smoother=params['velocity_smoother']['ros__parameters']
    smoother['max_velocity'][2]=.3
    smoother['min_velocity'][2]=-.3
    smoother['max_accel'][2]=.1
    smoother['max_decel'][2]=-.1
    return params


def metadata(parent_raw, out_name, output, generator_path, hypothesis):
    return dict(scope='UNQUALIFIED_LOADED_MOTION_CONTROL_CANDIDATE',
        parent=PARENT,parent_sha256=hashlib.sha256(parent_raw).hexdigest(),
        generator_sha256=hashlib.sha256(generator_path.read_bytes()).hexdigest(),
        params=out_name,params_sha256=hashlib.sha256(output).hexdigest(),
        acceptance_changes=[],qualified=False,hypothesis=hypothesis,
        limitations=['not an emergency brake guarantee','no 3D swept-volume guard','not a full order',
            'v7: use_rotate_to_heading remains false (corridor cannot contain in-place rotation)',
            'v7: scene is the base loaded transport identity; centered load is v5 lineage'])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');a=p.parse_args()
    raw=(ROOT/PARENT).read_bytes()
    gen=Path(__file__).resolve()
    out_retained=yaml.safe_dump(candidate_retained(yaml.safe_load(raw)),sort_keys=False).encode()
    out_turn=yaml.safe_dump(candidate_turn(yaml.safe_load(raw)),sort_keys=False).encode()
    meta_retained=metadata(raw,OUT_RETAINED,out_retained,gen,
        'lower linear+angular command acceleration reduces slip and deck yaw; TESTED run-04 and REFUTED (slip worsened)')
    meta_turn=metadata(raw,OUT_TURN,out_turn,gen,
        'angular-axis-only lowering returns deck yaw inside the band and cuts tray ratchet slip; MUST be physically tested')
    files={OUT_RETAINED:out_retained,'config/joint_world_v7_retained.profile.json':(json.dumps(meta_retained,indent=2)+'\n').encode(),
           OUT_TURN:out_turn,'config/joint_world_v7_turn.profile.json':(json.dumps(meta_turn,indent=2)+'\n').encode()}
    mismatches=[]
    for name,content in files.items():
        path=ROOT/name
        if a.check:
            if not path.exists() or path.read_bytes()!=content:mismatches.append(name)
        else:path.write_bytes(content)
    print(json.dumps(dict(check=a.check,mismatches=mismatches,files=sorted(files))))
    return int(bool(mismatches))


if __name__=='__main__':raise SystemExit(main())
