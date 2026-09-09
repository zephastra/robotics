from pathlib import Path
import hashlib
import json
import mujoco
import numpy as np
from .core import FINGERS, OPEN

ROOT = Path(__file__).resolve().parents[2]


def verify_assets():
    path = ROOT / 'assets/manifest.json'
    if not path.exists():
        raise RuntimeError('Assets missing: run bash scripts/setup.sh')
    manifest = json.loads(path.read_text())
    for name, expected in manifest['sha256'].items():
        p = ROOT / name
        if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Asset checksum mismatch: ' + name)
    return manifest


def load(seed=0, offset=0., mass=.05):
    m = mujoco.MjModel.from_xml_path(str(ROOT / 'assets/scene.xml'))
    m.body_mass[m.body('object').id] = mass
    m.body_inertia[m.body('object').id] = 2 / 5 * mass * .035 ** 2
    d = mujoco.MjData(m)
    mujoco.mj_setConst(m, d)
    rng = np.random.default_rng(seed)
    # Trial initialization only; never rewrite the object's pose while running.
    adr = m.jnt_qposadr[m.joint('object_free').id]
    d.qpos[adr:adr+2] += rng.uniform(-offset, offset, size=2)
    for i in range(16):
        d.qpos[m.jnt_qposadr[m.actuator_trnid[i, 0]]] = OPEN[i]
    d.ctrl[:] = np.r_[OPEN, 0., 0.]
    mujoco.mj_forward(m, d)
    return m, d


def observe(m, d):
    oid = m.geom('object_geom').id
    finger_forces = dict.fromkeys(FINGERS, 0.)
    source = tray = False
    for c in d.contact:
        if oid not in (c.geom1, c.geom2):
            continue
        gid = c.geom2 if c.geom1 == oid else c.geom1
        source |= gid == m.geom('source_pedestal').id
        tray |= gid == m.geom('target_tray').id
    for i in range(d.ncon):
        c = d.contact[i]
        if oid not in (c.geom1, c.geom2):
            continue
        gid = c.geom2 if c.geom1 == oid else c.geom1
        name = m.body(m.geom_bodyid[gid]).name
        for prefix in FINGERS:
            if name.startswith(prefix + '_'):
                force = np.zeros(6)
                mujoco.mj_contactForce(m, d, i, force)
                finger_forces[prefix] += max(0., float(force[0]))
    obj = d.body('object').xpos.copy()
    velocity = np.zeros(6)
    mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, m.body('object').id, velocity, 0)
    return dict(object=obj.tolist(), relative_object=(obj-d.body('palm').xpos).tolist(),
                speed=float(np.linalg.norm(velocity[3:])), finger_forces=finger_forces,
                source_contact=bool(source), tray_contact=bool(tray))
