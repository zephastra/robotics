"""Two-stock-body initialization calibration diagnostic; no H or loading claim.

Use the same loading-probe stock initial poses. Record the existing calibration
without changing its algorithm: net chassis drift can contaminate wheel signs.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'experiments')]
from w4_plant import LogisticsPlant


class RecordedPlant(LogisticsPlant):
    def __init__(self,*args,**kwargs):
        self.calibration_trace=[]
        super().__init__(*args,**kwargs)

    def _tick(self,command):
        before=self.chassis_x()
        super()._tick(command)
        b=self.model.body('n_base_link').id
        r=self.data.xmat[b].reshape(3,3)
        self.calibration_trace.append(dict(x=self.chassis_x(),dx=self.chassis_x()-before,
            yaw=float(np.arctan2(r[1,0],r[0,0])),
            control={s:float(self.data.ctrl[a]) for s,a in self.wheel_actuators.items()}))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--handover',action='store_true')
    args=parser.parse_args()
    if Path(args.run_id).name!=args.run_id:parser.error('bare run ID required')
    out=ROOT/'reports'/args.run_id;out.mkdir(parents=True,exist_ok=False)
    world=ROOT/'assets/world_p5_candidate_v5.xml'
    m0=mujoco.MjModel.from_xml_path(str(world));d0=mujoco.MjData(m0)
    mujoco.mj_resetDataKeyframe(m0,d0,0);mujoco.mj_forward(m0,d0)
    table=d0.body('a_table').xpos.copy()
    loading_x=float(d0.geom_xpos[m0.geom('c_fixed_roller_1_2').id][0])
    spec=mujoco.MjSpec.from_file(str(world))
    positions={ 'p5_red_second':table+np.array([.12,.04,.035]),
                'p5_blue_cylinder':table+np.array([-.06,.14,.035]) }
    for name,kind in (('p5_red_second',mujoco.mjtGeom.mjGEOM_BOX),
                      ('p5_blue_cylinder',mujoco.mjtGeom.mjGEOM_CYLINDER)):
        body=spec.worldbody.add_body(name=name,pos=positions[name].tolist())
        body.add_freejoint(name=name+'_free')
        body.add_geom(name=name+'_geom',type=kind,
            size=[.02,.015,.025] if name=='p5_red_second' else [.015,.025,0],
            mass=.03,friction=[.6,.005,.0001])
    model=spec.compile();data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,0)
    for name,pos in positions.items():
        adr=int(model.jnt_qposadr[model.joint(name+'_free').id])
        data.qpos[adr:adr+7]=[*pos,1,0,0,0]
    if not args.handover:
        bid=model.body('c_payload').id
        adr=int(model.jnt_qposadr[model.body_jntadr[bid]])
        data.qpos[adr]=loading_x
    mujoco.mj_forward(model,data)
    plant=RecordedPlant(model=model,data=data,world=world)
    report=dict(scope='INITIALIZATION_CALIBRATION_ONLY_NO_CONTROLLER_CHANGE',
        handover_initial_tray=args.handover,world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        wheel_signs=plant._wheel_signs(),trace=plant.calibration_trace,
        calibration_qpos_writes=plant.qpos_writes,runtime_qpos_writes=0,
        omitted_render_cameras='no mass or collisions; same physical stock initialization',
        full_order='NOT_RUN',v1_complete=False)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
