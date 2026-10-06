"""Independent ZERO-STEP snapshot judgment; cannot drive or resume the live world."""
import argparse
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]


def inspect(source):
    model=mujoco.MjModel.from_binary_path(str(source/'loading_model.mjb'))
    data=mujoco.MjData(model)
    with np.load(source/'loading_state.npz',allow_pickle=False) as saved:
        flag=int(saved['flag']);state=saved['state']
        if state.shape!=(mujoco.mj_stateSize(model,flag),) or not np.all(np.isfinite(state)):
            raise ValueError('invalid diagnostic snapshot')
        mujoco.mj_setState(model,data,state,flag)
    mujoco.mj_forward(model,data)
    contacts=[]
    for index,c in enumerate(data.contact[:data.ncon]):
        names=[model.geom(g).name or '' for g in (c.geom1,c.geom2)]
        if not any(n.startswith(('n_','c_deck','c_pusher')) for n in names):continue
        force=np.zeros(6);mujoco.mj_contactForce(model,data,index,force)
        contacts.append(dict(geom_ids=[int(c.geom1),int(c.geom2)],geoms=names,
            bodies=[model.body(model.geom_bodyid[g]).name for g in (c.geom1,c.geom2)],
            penetration_m=float(c.dist),force_diagnostic=force.tolist()))
    bodies={name:data.body(name).xpos.tolist() for name in
            ('n_base_link','n2_base_link','h_LINK_BASE','h_LINK_FOOT_L','h_LINK_FOOT_R','c_deck')}
    return dict(scope='INDEPENDENT_RECORDED_LOADING_SNAPSHOT_ZERO_PHYSICS_STEPS',
        sim_s=float(data.time),model_sha256=hashlib.sha256((source/'loading_model.mjb').read_bytes()).hexdigest(),
        state_sha256=hashlib.sha256((source/'loading_state.npz').read_bytes()).hexdigest(),
        positions_judge_only=bodies,contacts=contacts,physics_steps=0,
        note='forward/force diagnostic only; not new physical execution, stop, Nav2 or order proof')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run',required=True);parser.add_argument('--run-id',required=True)
    args=parser.parse_args()
    if any(Path(x).name!=x for x in (args.source_run,args.run_id)):parser.error('bare run IDs required')
    out=ROOT/'reports'/args.run_id;out.mkdir(parents=True,exist_ok=False)
    report=inspect(ROOT/'reports'/args.source_run)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
