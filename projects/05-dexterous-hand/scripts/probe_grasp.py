"""Developer grasp calibration. Physics-only probe, not an acceptance test."""
from pathlib import Path
import argparse
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pose', default='-.2684,.7374,1.0586,1.0175,0,.8488,.9083,.7839,.2684,.7374,1.0586,1.0175,1.1135,.0672,1.544,.0487')
    parser.add_argument('--snapshot', action='store_true')
    args = parser.parse_args()
    m = mujoco.MjModel.from_xml_path(str(ROOT / 'assets/scene.xml'))
    d = mujoco.MjData(m)
    close = np.fromstring(args.pose, sep=',')
    opened = np.zeros(16)
    opened[12] = .3
    for i in range(16):
        j = m.actuator_trnid[i, 0]
        d.qpos[m.jnt_qposadr[j]] = opened[i]
    mujoco.mj_forward(m, d)
    for t in range(5000):
        now = t * m.opt.timestep
        d.ctrl[:16] = opened + np.clip((now - 1) / 2, 0, 1) * (close - opened)
        d.ctrl[16:] = [0, np.clip((now - 4) / 2, 0, 1) * .12]
        mujoco.mj_step(m, d)
        if t % 500 == 0:
            contacts = []
            oid = m.geom('object_geom').id
            for c in d.contact:
                if oid in (c.geom1, c.geom2):
                    gid = c.geom2 if c.geom1 == oid else c.geom1
                    contacts.append(m.body(m.geom_bodyid[gid]).name or m.geom(gid).name)
            print(round(now, 2), np.round(d.body('object').xpos, 3), contacts)
    if args.snapshot:
        with mujoco.Renderer(m, 720, 960) as renderer:
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [.05, 0, .22]
            camera.distance = .65
            camera.azimuth = 120
            camera.elevation = -25
            renderer.update_scene(d, camera=camera)
            from PIL import Image
            out = ROOT / 'reports/probe.png'
            out.parent.mkdir(exist_ok=True)
            Image.fromarray(renderer.render()).save(out)
            print(out)


if __name__ == '__main__':
    main()
