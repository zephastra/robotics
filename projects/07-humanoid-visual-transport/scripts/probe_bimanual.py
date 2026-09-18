"""Ground-truth-assisted mechanics diagnostic; NOT visual autonomous control."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import sys
import mujoco
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from humanoid007.runtime import Runtime,OPEN,GRASP,verify_assets


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--duration',type=float,default=20)
    parser.add_argument('--stand-only',action='store_true')
    args=parser.parse_args()
    verify_assets()
    r=Runtime()
    scene=json.loads((ROOT/'config/scene.json').read_text())
    center=np.array(scene['source'])
    goals={s:center+np.array([0,sign*scene['handle_y'],0]) for s,sign in [('left',1),('right',-1)]}
    out=ROOT/'reports'/('mechanics-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    out.mkdir(parents=True)
    records=[];arms=None;status='DURATION_REACHED'
    while r.d.time<args.duration:
        now=r.d.time
        if not args.stand_only and now>4 and r.tick%25==0:
            height=.12*np.clip((now-12)/4,0,1)
            arms=r.arm_ik({s:g+np.array([0,0,height]) for s,g in goals.items()},arms)
        fraction=np.clip((now-8)/3,0,1) if not args.stand_only else 0
        hands={s:OPEN+fraction*(GRASP-OPEN) for s in goals}
        head=r.gaze(center) if r.tick%25==0 else r.head_target
        r.step(r.hold_command(np.zeros(3)),arms,hands,head)
        if r.tick%250==0:
            row=dict(**r.snapshot(),payload=r.d.body('payload').xpos.tolist())
            records.append(row)
            print(round(now,2),row['base'],row['payload'],flush=True)
            if row['tilt_deg']>45 or row['base'][2]<.65:
                status='BODY_FALL';break
    with mujoco.Renderer(r.m,720,960) as renderer:
        camera=mujoco.MjvCamera();camera.lookat[:]=[.2,0,.9]
        camera.distance,camera.azimuth,camera.elevation=2.5,135,-15
        renderer.update_scene(r.d,camera=camera)
        Image.fromarray(renderer.render()).save(out/'final.png')
        renderer.update_scene(r.d,camera='eyes')
        Image.fromarray(renderer.render()).save(out/'eyes.png')
    (out/'report.json').write_text(json.dumps(dict(status=status,scope='mechanics diagnostic, not task success',records=records),indent=2))
    print('Report:',out)
    return 1 if status=='BODY_FALL' else 0


if __name__=='__main__': raise SystemExit(main())
