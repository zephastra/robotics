"""Reanalyse recorded receiver RGB-D; NEVER overwrite the execution verdict."""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'experiments')]
from workcell.tray import derive_cells
from receiver_rgbd_reader import locate
from tray_rgbd_reader import read_cloud,pv


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id',required=True)
    ap.add_argument('--recording',required=True)
    args=ap.parse_args()
    directory=(ROOT/'reports'/args.recording).resolve()
    if directory.parent != (ROOT/'reports').resolve():raise ValueError('invalid recording directory')
    source=directory/'receiver_rgbd.npz'
    frame=np.load(source,allow_pickle=False)
    rgb,depth,cloud,valid=[frame[k] for k in ('rgb','depth','world','valid')]
    pv.H,pv.W=depth.shape
    roi=tuple(frame['roi']);floor_z=float(frame['floor_z'])
    # Catalog only, no recorded object's true position/orientation is loaded.
    model=mujoco.MjModel.from_xml_path(str(ROOT/'assets/world_p5_candidate_v4.xml'))
    data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,0);mujoco.mj_forward(model,data)
    template=derive_cells(model,data)
    f,r=model.geom('c_tray_floor').id,model.geom('c_tray_wall_xp').id
    template['rim_height_m']=float(model.geom_pos[r][2]+model.geom_size[r][2]
                                  -model.geom_pos[f][2]-model.geom_size[f][2])
    localized=locate(template,cloud,rgb,valid,roi,floor_z)
    sizes={'red':np.array([.02,.015,.025]),'blue':np.array([.015,.015,.025])}
    observed=read_cloud(rgb,depth,cloud,valid,localized['cells'],sizes) if localized[
        'status']=='RESOLVED' else dict(status='UNKNOWN',counts=None,parts=[])
    missing=locate(template,cloud,rgb,np.zeros_like(valid),roi,floor_z)
    hidden=valid.copy();hidden[cloud[...,1]<-.0767]=False
    occluded=locate(template,cloud,rgb,hidden,roi,floor_z)
    checks=dict(visual_localized=localized['status']=='RESOLVED',
        independent_count=observed['status']=='RESOLVED' and observed['counts']=={'red':2,'blue':1},
        three_cells=sorted(p['cell'] for p in observed['parts'])==[0,1,2],
        missing_unknown=missing['status']=='UNKNOWN',partial_tray_unknown=occluded['status']=='UNKNOWN')
    out=ROOT/'reports'/args.run_id;out.mkdir(exist_ok=False)
    report=dict(scope='RECORDED_SENSOR_REANALYSIS_NOT_EXECUTION',recording=args.recording,
        recording_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        localization={k:v for k,v in localized.items() if k!='cells'},observed=observed,
        missing=missing,partial={k:v for k,v in occluded.items() if k!='cells'})
    acceptance=dict(scope=report['scope'],checks={k:'PASS' if v else 'FAIL' for k,v in checks.items()},
        diagnostic_result='PASS' if all(checks.values()) else 'FAIL',
        original_execution_verdict_unchanged=True,full_order='NOT_RUN',v1_complete=False)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (out/'acceptance.json').write_text(json.dumps(acceptance,indent=2)+'\n')
    print(json.dumps(acceptance,indent=2))
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
