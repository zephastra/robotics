"""Headless integration experiment; failures are saved and never hidden."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from humanoid006.runtime import Runtime, verify_assets
from humanoid006.task import Mission, observe


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--lift-only',action='store_true')
    p.add_argument('--snapshot',action='store_true')
    args=p.parse_args()
    manifest=verify_assets()
    r=Runtime()
    mission=Mission(r,args.lift_only)
    out=ROOT/'reports'/('mission-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    out.mkdir(parents=True)
    records=[]; last=''; started=time.monotonic()
    while r.d.time<120 and not mission.result:
        if r.tick%25==0:
            mission.update()
            obs=observe(r)
            if mission.phase!=last:
                print(f'{r.d.time:.2f} {mission.phase} {mission.reason} payload={np.round(obs["payload"],3)} contacts={obs["forces"]}',flush=True)
                last=mission.phase
            if r.tick%250==0:
                records.append(dict(**r.snapshot(),phase=mission.phase,payload=obs['payload'].tolist(),forces=obs['forces']))
        if not mission.result:
            r.step(r.hold_command(mission.goal,precise=mission.phase=='CARRY'),mission.arm,mission.hand)
    if not mission.result: mission.finish('TIMEOUT','DURATION_LIMIT')
    report=dict(status=mission.result,reason=mission.reason,events=mission.events,records=records,
                final=r.snapshot(),max_lift_m=mission.max_lift,wall_seconds=time.monotonic()-started,
                manifest=manifest)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if args.snapshot:
        import mujoco
        from PIL import Image
        with mujoco.Renderer(r.m,720,960) as renderer:
            camera=mujoco.MjvCamera();camera.lookat[:]=[r.d.qpos[0]+.3,r.d.qpos[1]-.1,1.]
            camera.distance,camera.azimuth,camera.elevation=2.4,130,-15
            renderer.update_scene(r.d,camera=camera)
            Image.fromarray(renderer.render()).save(out/'final.png')
    print('Report:',out,flush=True)
    raise SystemExit(0 if mission.result=='COMPLETED' else 1)


if __name__=='__main__':main()
