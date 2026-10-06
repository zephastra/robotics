"""Independent NARROWER admissible load-centre region, not a smaller cargo.

Original wide-load profile is preserved. Full catalog rotation-invariant radius
and mechanism uncertainty remain unchanged. Motion is REFUSED outside the new
region including measurement uncertainty. No acceptance threshold is changed.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import yaml
from loaded_nav_envelope import combined_envelope

ROOT=Path(__file__).resolve().parents[1]
ARTIFACTS=('config/joint_world_v5_centered.profile.json','config/nav2_joint_world_v5_centered.yaml')


def generate():
    parent=ROOT/'config/joint_world_v5_loaded.profile.json'
    loaded=json.loads(parent.read_text());empty=json.loads((ROOT/'config/joint_world_v5.profile.json').read_text())
    result=copy.deepcopy(loaded)
    result['scope']='RESTRICTED_CENTRED_LOAD_CANDIDATE_NOT_FULL_ORDER'
    result['parent_loaded_profile_sha256']=hashlib.sha256(parent.read_bytes()).hexdigest()
    result['generator_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    contract=result['observation_contract'];contract['centre_low'][1]=-.06;contract['centre_high'][1]=.06
    result['footprint']=combined_envelope(empty['footprint'],contract['centre_low'],contract['centre_high'],
                                          loaded['cargo_radius_m'],loaded['mechanism_allowance_m'])
    result['runtime_gate']='vehicle RGBD/quantity/ideal deck contact; wired only in initialized probe_vehicle_nav2'
    result['restriction']='centre y +/-60mm INCLUDING measurement uncertainty; not broad-load acceptance'
    config=yaml.safe_load((ROOT/'config/nav2_joint_world_v5_loaded.yaml').read_text())
    for name in ('global_costmap','local_costmap'):
        config[name][name]['ros__parameters']['footprint']=str(result['footprint'])
    return {ARTIFACTS[0]:json.dumps(result,indent=2)+'\n',
            ARTIFACTS[1]:yaml.safe_dump(config,sort_keys=False)}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--check',action='store_true')
    args=parser.parse_args();expected=generate()
    if args.check:
        bad=[name for name,data in expected.items() if not (ROOT/name).exists() or (ROOT/name).read_text()!=data]
        print('centered profile check:',bad or 'PASS');return int(bool(bad))
    for name,data in expected.items():(ROOT/name).write_text(data)
    print('generated',*expected);return 0


if __name__=='__main__':raise SystemExit(main())
