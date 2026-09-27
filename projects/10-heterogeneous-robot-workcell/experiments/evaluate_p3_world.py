"""The P3 world gate: is the formal cell a valid base for two independent AMRs?

WHAT THIS IS FOR
----------------
`P3-WORLD-01` asks for "the formal scene, naming, TF, clock, and robot INSTANCE ISOLATION".
`build_p3_world.py` produces the scene by handing `merge_world` a five-role table. This file
decides whether the result is usable -- and, per this project's rule, every check is shown able to
fail before its PASS is worth anything.

THE FOUR THINGS P3-WORLD-01 NAMES, AND HOW EACH IS MEASURED
-----------------------------------------------------------
  naming   every named entity belongs to exactly one instance, or is DECLARED cell furniture.
           A name belonging to nobody is a frame nobody can address.
  TF       each instance's bodies form ONE tree rooted in the world; no instance is parented under
           another. Structural half of "no duplicate TF publishers"; the live half (one writer per
           edge) needs ROS and belongs to `P3-NAV-02`.
  clock    one time base -- a single timestep/integrator across sources, and exactly one keyframe.
  instances  perturbing one instance moves no body of any other, for ALL 20 ordered pairs.

TWO THINGS THIS FILE DELIBERATELY DOES *NOT* CLAIM TO CHECK
-----------------------------------------------------------
1. **Duplicate names across instances.** It cannot happen: MuJoCo refuses to compile a model with
   two bodies of the same name, so the model either loads or it does not. A "shared name" branch
   here would be unreachable code that reports PASS forever -- the exact defect this project keeps
   finding. The guarantee is at compile time, and it is stated rather than re-checked.

2. **That moving AMR 1 moves nothing of C.** It DOES move C's deck, because the V1 assembly mounts
   that deck on this chassis -- that is the whole content of `D041`. So the isolation check carries
   the exception EXPLICITLY and, following P1's lesson, also requires the exception to be USED:
   if the deck is ever un-mounted, this check goes red instead of quietly going quiet.

WHAT IS NOT ESTABLISHED
-----------------------
The cell is not the final V1 layout: the two AMRs sit at two stations derived from footprints, not
in one shared driving area. Instance 2 carries no deck (see `build_p3_world.py`). Nothing here says
anything about ROS-level TF broadcast or a live `/clock`.
"""
from __future__ import annotations

import hashlib
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import build_p3_world as bpw  # noqa: E402

bpw.install()                     # hand merge_world the P3 role table before anything reads it
import merge_world as mw  # noqa: E402
import roller_rig as rig  # noqa: E402

WORLD = mw.ASSETS / 'world_p3_cell.xml'
SETTLE_SECONDS = 3.0
LEAK_TOL = 1e-9
PERTURB = 0.05

#: Named cell-level furniture: legitimately owned by nobody. A name that matches neither an
#: instance prefix nor this list is an entity that LOST its owner -- which is a real failure.
CELL_LEVEL = ('cell_ground', 'cell_wall_north', 'cell_wall_south', 'cell_wall_east',
              'cell_wall_west', 'world')

#: The declared assembly: C's deck rides N's chassis (D041). The exception to isolation.
DECK_ON = ('c', 'n')


class Results:
    def __init__(self):
        self.rows = []

    def add(self, name, status, detail):
        self.rows.append({'name': name, 'status': status, 'detail': detail})

    def verdict(self):
        return 'PASS' if all(r['status'] == 'PASS' for r in self.rows) else 'FAIL'

    def counts(self):
        c = {'checks': len(self.rows)}
        for s in ('PASS', 'FAIL', 'NOT_RUN'):
            c[s.lower()] = sum(1 for r in self.rows if r['status'] == s)
        return c


class World:
    def __init__(self, path=WORLD):
        self.path = pathlib.Path(path)
        self.model = mujoco.MjModel.from_xml_path(str(self.path))
        self.home = (np.array(self.model.key_qpos[0]) if self.model.nkey
                     else np.zeros(self.model.nq))
        self.keys = tuple(r.key for r in bpw.P3_ROLES)

    def name_of(self, objtype, i):
        return mujoco.mj_id2name(self.model, objtype, i) or ''

    def instance_of(self, name):
        """Owner of `name`, or None. Longest prefix wins so `n2_` beats `n_`."""
        best = None
        for k in self.keys:
            if name.startswith(f'{k}_') and (best is None or len(k) > len(best)):
                best = k
        return best

    def bodies_of(self, key):
        return [b for b in range(self.model.nbody)
                if self.instance_of(self.name_of(mujoco.mjtObj.mjOBJ_BODY, b)) == key]

    def dofs_of(self, key):
        out = []
        for j in range(self.model.njnt):
            b = int(self.model.jnt_bodyid[j])
            if self.instance_of(self.name_of(mujoco.mjtObj.mjOBJ_BODY, b)) == key:
                out.append((int(self.model.jnt_qposadr[j]), int(self.model.jnt_type[j])))
        return out

    def relocated_children(self):
        """C's bodies whose ancestry reaches instance n -- i.e. the ones riding the chassis.

        NOT "whose parent is not a c-body": C's fixture contributes 39 world-rooted bodies, so
        that predicate names the FIXTURE as relocated and the DECK as not, and the leak check
        then measures exactly the wrong set. Measured, not guessed.
        """
        mine = set(self.bodies_of(DECK_ON[0]))
        out = []
        for b in mine:
            x, hops = int(self.model.body_parentid[b]), 0
            while x != 0 and hops < 64:
                if self.instance_of(self.name_of(mujoco.mjtObj.mjOBJ_BODY, x)) == DECK_ON[1]:
                    out.append(b)
                    break
                x, hops = int(self.model.body_parentid[x]), hops + 1
        return sorted(out)


def _perturb(m, home, key, world):
    d = mujoco.MjData(m)
    d.qpos[:] = home
    mujoco.mj_forward(m, d)
    for adr, jtype in world.dofs_of(key):
        if jtype == mujoco.mjtJoint.mjJNT_FREE:
            d.qpos[adr:adr + 3] += PERTURB
            d.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]
        else:
            d.qpos[adr] += PERTURB
    mujoco.mj_forward(m, d)
    return d


def measure_leak(m, home, src, dst_bodies, world):
    d0 = mujoco.MjData(m)
    d0.qpos[:] = home
    mujoco.mj_forward(m, d0)
    if not dst_bodies:
        return 0.0
    before = np.array([d0.xpos[b] for b in dst_bodies])
    d1 = _perturb(m, home, src, world)
    after = np.array([d1.xpos[b] for b in dst_bodies])
    return float(np.max(np.abs(after - before)))


# ----------------------------------------------------------------------------- naming
def check_loads(world, res):
    n = world.model
    res.add('the P3 cell loads', 'PASS',
            f'{world.path.name} compiles: nq={n.nq} nv={n.nv} nu={n.nu} nbody={n.nbody} '
            f'ngeom={n.ngeom} njnt={n.njnt} nkey={n.nkey}')


def check_instances_present(world, res):
    m = world.model
    parts, bad = [], []
    for k in world.keys:
        nb, nj = len(world.bodies_of(k)), len(world.dofs_of(k))
        na = sum(1 for a in range(m.nu)
                 if world.instance_of(world.name_of(mujoco.mjtObj.mjOBJ_ACTUATOR, a)) == k)
        parts.append(f'{k}: {nb}b/{nj}j/{na}a')
        if nb == 0 or nj == 0 or na == 0:
            bad.append(k)
    res.add('every instance is present with its own bodies, joints and actuators',
            'FAIL' if bad else 'PASS',
            ('MISSING: ' + ', '.join(bad)) if bad else '; '.join(parts))


def check_naming(world, res, *, extra_unowned=None):
    """Every named entity is owned by exactly one instance, or is declared cell furniture.

    Duplicate names are NOT checked: MuJoCo refuses to compile them, so they are impossible by
    construction and a branch for them would be unreachable code.
    """
    m = world.model
    per = {k: 0 for k in world.keys}
    unowned = []
    counts = ((mujoco.mjtObj.mjOBJ_BODY, m.nbody, 'body'),
              (mujoco.mjtObj.mjOBJ_JOINT, m.njnt, 'joint'),
              (mujoco.mjtObj.mjOBJ_GEOM, m.ngeom, 'geom'),
              (mujoco.mjtObj.mjOBJ_ACTUATOR, m.nu, 'actuator'),
              (mujoco.mjtObj.mjOBJ_SITE, m.nsite, 'site'))
    for objtype, n, label in counts:
        for i in range(n):
            nm = world.name_of(objtype, i)
            if not nm:
                continue
            k = world.instance_of(nm)
            if k is not None:
                per[k] += 1
            elif nm not in CELL_LEVEL:
                unowned.append(f'{label}:{nm}')
    detail = '; '.join(f'{k}:{v}' for k, v in per.items())
    detail += f' | declared cell furniture: {[c for c in CELL_LEVEL if c != "world"]}'
    if unowned:
        detail += f' | UNOWNED (lost its prefix): {unowned[:6]}{" ..." if len(unowned) > 6 else ""}'
    res.add('every name belongs to exactly one instance or is declared cell furniture',
            'FAIL' if unowned else 'PASS', detail)


def check_tf_tree(world, res):
    m = world.model
    bad, roots = [], {}
    for k in world.keys:
        mine = set(world.bodies_of(k))
        r = [b for b in mine if int(m.body_parentid[b]) == 0]
        for b in mine:
            p = int(m.body_parentid[b])
            if p != 0 and p not in mine:
                pn = world.name_of(mujoco.mjtObj.mjOBJ_BODY, p)
                pk = world.instance_of(pn)
                if (world.instance_of(world.name_of(mujoco.mjtObj.mjOBJ_BODY, b)),
                        pk) == DECK_ON:
                    continue                        # the declared assembly, checked elsewhere
                bad.append(f'{world.name_of(mujoco.mjtObj.mjOBJ_BODY, b)!r} hangs under {pn!r}')
        roots[k] = len(r)
        if len(r) == 0:
            bad.append(f'{k}: no world-rooted body at all')
    res.add('no instance hangs under another, and every instance reaches the world',
            'FAIL' if bad else 'PASS',
            ('; '.join(bad[:4]) + ' | ') if bad else ''
            + 'world-rooted subtrees per instance: '
            + ', '.join(f'{k}:{roots[k]}' for k in world.keys)
            + ' (an instance may own SEVERAL world-rooted subtrees -- a fixture row, a table and '
              'an arm are not one chain; what would be wrong is any of them hanging under another '
              'instance)')


# ----------------------------------------------------------------------------- clock
def check_time_base(world, res, *, sources=None):
    m = world.model
    src = sources if sources is not None else mw.sources_agree_on_stepping()
    agree = (len({round(float(v[0]), 9) for v in src.values()}) == 1
             and len({int(v[1]) for v in src.values()}) == 1)
    one_key = m.nkey == 1
    res.add('the instances share one time base',
            'PASS' if (agree and one_key) else 'FAIL',
            f'sources step at {sorted(set((round(float(v[0]), 9), int(v[1])) for v in src.values()))} '
            f'-> agree={agree}; home keyframes in the merged world: {m.nkey} (want exactly 1)')


# ----------------------------------------------------------------------------- isolation
def check_isolation(world, res):
    m, home = world.model, world.home
    leaks, moved_nothing = [], []
    relocated = set(world.relocated_children())
    reloc_names = [world.name_of(mujoco.mjtObj.mjOBJ_BODY, b) for b in relocated]
    exception_leak, exception_used = 0.0, False

    for a in world.keys:
        for b in world.keys:
            if a == b:
                continue
            dst = world.bodies_of(b)
            keep = [x for x in dst if x not in relocated]
            leak = measure_leak(m, home, a, keep, world)
            if (a, b) == DECK_ON:
                exception_leak = leak
                exception_used = len(relocated) == 0 or leak <= LEAK_TOL
            if leak > LEAK_TOL:
                leaks.append(f'{a}->{b}:{leak:.2e}')
        if measure_leak(m, home, a, world.bodies_of(a), world) <= LEAK_TOL:
            moved_nothing.append(a)

    # and the declared exception must actually be exercised
    d1 = _perturb(m, home, DECK_ON[1], world)
    d0 = mujoco.MjData(m)
    d0.qpos[:] = home
    mujoco.mj_forward(m, d0)
    rode = [world.name_of(mujoco.mjtObj.mjOBJ_BODY, b) for b in relocated
            if np.max(np.abs(d1.xpos[b] - d0.xpos[b])) > LEAK_TOL]
    exception_live = (len(rode) == len(relocated)) and len(relocated) > 0

    ok = (not leaks) and (not moved_nothing) and exception_live
    detail = (f'of {len(world.keys) * (len(world.keys) - 1)} ordered pairs, those excluding the '
              f'declared assembly leak {("NOTHING (leaks " + str(leaks) + ")") if leaks else "0"}; '
              f'{len(world.keys) - len(moved_nothing)}/{len(world.keys)} instances move their own '
              f'bodies. Declared assembly {DECK_ON[0]}-on-{DECK_ON[1]}: {len(relocated)} relocated '
              f'bodies all rode ({rode == reloc_names}), so the exception is USED -- if the deck '
              f'were ever un-mounted this goes red instead of quietly going quiet.')
    res.add('moving one instance moves no other instance (bar the declared assembly)',
            'PASS' if ok else 'FAIL', detail)


def check_no_cross_contact(world, res):
    m = world.model
    d = mujoco.MjData(m)
    d.qpos[:] = world.home
    mujoco.mj_forward(m, d)
    pairs = set()
    for c in range(d.ncon):
        i1 = world.instance_of(world.name_of(mujoco.mjtObj.mjOBJ_BODY,
                                             int(m.geom_bodyid[d.contact[c].geom1])))
        i2 = world.instance_of(world.name_of(mujoco.mjtObj.mjOBJ_BODY,
                                             int(m.geom_bodyid[d.contact[c].geom2])))
        if i1 and i2 and i1 != i2:
            pairs.add(tuple(sorted((i1, i2))))
    res.add('no instance touches another at home',
            'FAIL' if pairs else 'PASS',
            f'cross-instance contacts: {sorted(pairs) or "none"} (total contacts {d.ncon})')


def check_cell_enclosed(world, res):
    """★ The boundary must actually CLOSE the cell, and each wall must sit where its name says.

    Added 2026-09-23. Every other row in this gate passed on a world whose WEST and SOUTH sides
    had no wall at all: those rows count the boundary, they do not test it. A check that counts
    objects is not a check that the objects do their job -- and the consequence is not cosmetic,
    because a lidar in an open room reports the world outside it and a planner routes through the
    gap. It is also the reason P3's navigation task could not have been run on this world.

    Measured from the COMPILED model, not from the numbers the generator passed in: a generator
    that reports its own intent is exactly how the two placement offsets stayed wrong.
    """
    m = world.model
    T = mw.CELL_WALL_THICKNESS
    walls, ground = [], None
    for i in range(m.ngeom):
        n = world.name_of(mujoco.mjtObj.mjOBJ_GEOM, i)
        if n == 'cell_ground':
            ground = i
        elif n.startswith('cell_wall_'):
            walls.append(i)
    if ground is None or not walls:
        res.add('the cell boundary encloses the cell, and each wall is where its name says',
                'FAIL', f'ground={ground!r}, {len(walls)} cell_wall_* geoms')
        return
    p = np.array(m.geom_pos[ground], float)
    s = np.array(m.geom_size[ground], float)
    x0, x1, y0, y1 = p[0] - s[0], p[0] + s[0], p[1] - s[1], p[1] + s[1]

    def extents(i):
        pp = np.array(m.geom_pos[i], float)
        ss = np.array(m.geom_size[i], float)
        return pp - ss, pp + ss

    SIDES = (('west', 'cell_wall_west', 0, x0, (y0, y1)),
             ('east', 'cell_wall_east', 0, x1, (y0, y1)),
             ('south', 'cell_wall_south', 1, y0, (x0, x1)),
             ('north', 'cell_wall_north', 1, y1, (x0, x1)))
    bad, rows = [], []
    for side, want_name, axis, bound, band in SIDES:
        perp = 1 - axis
        pieces = []
        for i in walls:
            lo, hi = extents(i)
            if min(hi[axis], bound + T) - max(lo[axis], bound - T) <= 0:
                continue
            a, b = max(lo[perp], band[0]), min(hi[perp], band[1])
            if b > a:
                pieces.append((a, b))
        pieces.sort()
        covered, cur = 0.0, None
        for a, b in pieces:                    # UNION: an overlap must not double-count
            if cur is None or a > cur:
                covered += b - a
            elif b > cur:
                covered += b - cur
            cur = b if cur is None else max(cur, b)
        need = band[1] - band[0]
        frac = covered / need if need else 0.0
        rows.append(f'{side} {frac * 100:.1f}%')
        if frac < 0.99:
            bad.append(f'{side} is OPEN: {covered:.3f} of {need:.3f} m walled')
        named = [i for i in walls if world.name_of(mujoco.mjtObj.mjOBJ_GEOM, i) == want_name]
        if not named:
            bad.append(f'{want_name} does not exist')
            continue
        lo, hi = extents(named[0])
        along_x = (hi[0] - lo[0]) > (hi[1] - lo[1])
        want_along_x = (axis == 1)
        if along_x != want_along_x:
            bad.append(f'{want_name} runs along {"x" if along_x else "y"} but its name says '
                       f'{"x" if want_along_x else "y"}')
        centre = 0.5 * (lo[axis] + hi[axis])
        if abs(centre - bound) > 2.0 * T:
            bad.append(f'{want_name} sits at {centre:.3f} on axis {axis}, not on the {side} '
                       f'bound {bound:.3f}')
    detail = 'coverage: ' + ', '.join(rows)
    if bad:
        detail = '; '.join(bad[:4]) + ' | ' + detail
    res.add('the cell boundary encloses the cell, and each wall is where its name says',
            'FAIL' if bad else 'PASS', detail)


def check_settles(world, res, *, qvel0=None, hold=True):
    """Settle for SETTLE_SECONDS and report the worst |qvel| PER INSTANCE.

    The humanoid is NOT quasi-static: 25 pure-torque actuators with zero command sink it 1.582 m
    in 3 s. The P1 gate therefore evaluates H's own stand law every step, and so does this -- the
    law is imported, not re-derived. Without it this row would report a known, expected humanoid
    drift as a failure of the CELL, which is a different claim.
    """
    m = world.model
    d = mujoco.MjData(m)
    d.qpos[:] = world.home
    if qvel0 is not None:
        d.qvel[:] = qvel0
    mujoco.mj_forward(m, d)
    home = np.array(world.home)
    for _ in range(int(SETTLE_SECONDS / m.opt.timestep)):
        if hold:
            d.ctrl[:] = mw.home_hold_ctrl(m, home, d.qpos, d.qvel)
        mujoco.mj_step(m, d)
    per, worst, where = {}, 0.0, None
    for k in world.keys:
        v = max((float(np.max(np.abs(d.qvel[int(m.jnt_dofadr[j]):int(m.jnt_dofadr[j])
                                           + (6 if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE
                                              else 1)])))
                 for j in range(m.njnt)
                 if world.instance_of(world.name_of(mujoco.mjtObj.mjOBJ_BODY, m.jnt_bodyid[j])) == k),
                default=0.0)
        per[k] = v
        if v > worst:
            worst, where = v, k
    finite = bool(np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel)))
    res.add('the cell settles',
            'PASS' if (finite and worst < 0.10) else 'FAIL',
            f'after {SETTLE_SECONDS} s with the imported hold law: worst per-instance |qvel| is '
            f'{where} at {worst:.4f} (limit 0.10); ' +
            ', '.join(f'{k}:{v:.4f}' for k, v in per.items()) + f'; finite={finite}')


def check_deck_reproduces(world, res):
    m = world.model
    d = mujoco.MjData(m)
    d.qpos[:] = world.home
    mujoco.mj_forward(m, d)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'c_deck')
    got = float(d.xpos[bid][0]) - (mw.station_layout()['c'][0] + rig.DECK_X0)
    parent = world.name_of(mujoco.mjtObj.mjOBJ_BODY, int(m.body_parentid[bid]))
    tol = 1e-4
    ok = abs(got) < tol and parent == f'n_{mw.CHASSIS_MOUNT_BODY}'
    res.add("the deck is still on instance 1 and still reproduces C's geometry at home",
            'PASS' if ok else 'FAIL',
            f'c_deck parent {parent!r}; deck world x - (C station + DECK_X0) = {got:+.3e} '
            f'(tol {tol:g}). Rebuilding in five roles must not silently undo the V1 assembly.')


# ----------------------------------------------------------------------------- falsifiability
def falsifiability():
    """Each check must be driven to a non-PASS state by a probe. Returns the evidence rows."""
    out = []

    def run(label, what, fn):
        try:
            fired = bool(fn())
        except Exception as exc:
            fired, what = False, f'{what} -- probe raised {exc!r}'
        out.append({'check': label, 'probe': what, 'made_the_check_fail': fired})

    # naming: strip a body's instance prefix so it belongs to nobody
    def p_naming():
        spec = mujoco.MjSpec.from_file(str(WORLD))
        spec.body('n2_base_link').name = 'base_link'
        m = spec.compile()
        w = _wrap(m)
        r = Results()
        check_naming(w, r)
        return r.rows[0]['status'] != 'PASS'

    # tf-tree: build a model where a `n2_`-named body hangs under instance 1
    def p_tf():
        spec = mujoco.MjSpec()
        spec.worldbody.add_geom(name='cell_ground', type=mujoco.mjtGeom.mjGEOM_PLANE,
                                size=[2, 2, 0.1])
        n = spec.worldbody.add_body(name='n_base_link', pos=[0, 0, 0.1])
        n.add_geom(name='n_g', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.1, 0.1, 0.1])
        n2 = n.add_body(name='n2_orphan', pos=[0.5, 0, 0])
        n2.add_geom(name='n2_g', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.1, 0.1, 0.1])
        m = spec.compile()
        w = _wrap(m, keys=('n', 'n2'))
        r = Results()
        check_tf_tree(w, r)
        return r.rows[0]['status'] != 'PASS'

    # clock: a model with two keyframes
    def p_clock():
        xml = ('<mujoco><worldbody><body name="n_a" pos="0 0 0.1">'
               '<joint name="n_a_j" type="hinge" axis="0 0 1"/>'
               '<geom name="n_a_g" type="box" size=".1 .1 .1"/></body></worldbody>'
               '<keyframe><key name="k1" qpos="0"/><key name="k2" qpos="0"/></keyframe></mujoco>')
        m = mujoco.MjModel.from_xml_string(xml)
        w = _wrap(m, keys=('n',))
        r = Results()
        check_time_base(w, r, sources={'n': (m.opt.timestep, int(m.opt.integrator))})
        return r.rows[0]['status'] != 'PASS'

    # isolation: two bodies REALLY coupled by an equality constraint must read a non-zero leak
    def p_isolation():
        """A REAL coupling, built with parent-child so forward kinematics can see it.

        An equality constraint was tried first and made this probe read 0: `mj_forward` does not
        resolve equalities, they act through the solver at step time. So the probe could not fail,
        which is the very thing the probe exists to rule out.
        """
        spec = mujoco.MjSpec()
        spec.worldbody.add_geom(name='cell_ground', type=mujoco.mjtGeom.mjGEOM_PLANE,
                                size=[2, 2, 0.1])
        a = spec.worldbody.add_body(name='n_a', pos=[0, 0, 0.2])
        a.add_freejoint(name='n_a_free')
        a.add_geom(name='n_a_g', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.05, 0.05, 0.05])
        b = a.add_body(name='n2_b', pos=[0.5, 0, 0])          # n2 rides n: a REAL cross-instance link
        b.add_geom(name='n2_b_g', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.05, 0.05, 0.05])
        m = spec.compile()
        w = _wrap(m, keys=('n', 'n2'))
        leak = measure_leak(m, np.zeros(m.nq), 'n', w.bodies_of('n2'), w)
        return leak > LEAK_TOL

    # cross-contact: shift instance 2 onto instance 1
    def p_contact():
        """Two overlapping boxes owned by different instances: the check must see the contact."""
        spec = mujoco.MjSpec()
        spec.worldbody.add_geom(name='cell_ground', type=mujoco.mjtGeom.mjGEOM_PLANE,
                                size=[2, 2, 0.1])
        a = spec.worldbody.add_body(name='n_a', pos=[0, 0, 0.5])
        a.add_freejoint(name='n_a_free')          # DYNAMIC: see the note below
        ga = a.add_geom(name='n_a_g', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.2, 0.2, 0.2])
        b = spec.worldbody.add_body(name='n2_b', pos=[0.1, 0, 0.5])      # overlapping on purpose
        b.add_freejoint(name='n2_b_free')         # DYNAMIC: static-static pairs never collide
        gb = b.add_geom(name='n2_b_g', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.2, 0.2, 0.2])
        # EXPLICIT: these are not guaranteed to be 1, and with them at 0 the boxes never touch,
        # so the probe would silently prove nothing.
        for g in (ga, gb):
            g.contype = 1
            g.conaffinity = 1
        m = spec.compile()
        # SELF-CHECK inside the probe: if the two boxes do not in fact touch, this probe proves
        # nothing and must say so rather than report a quiet "no".
        _d = mujoco.MjData(m)
        mujoco.mj_forward(m, _d)
        assert _d.ncon > 0, ('the probe bodies are not actually in contact (ncon=0), so this '
                             'probe cannot falsify anything')
        w = _wrap(m, keys=('n', 'n2'))
        r = Results()
        check_no_cross_contact(w, r)
        return r.rows[0]['status'] != 'PASS'

    # settles: launch the whole cell
    def p_settles():
        w = World()
        r = Results()
        check_settles(w, r, qvel0=np.full(w.model.nv, 5.0))
        return r.rows[0]['status'] != 'PASS'

    run('naming', 'strip the `n2_` prefix off a body so it belongs to nobody', p_naming)
    run('tf-tree', 'hang a `n2_`-named body under instance 1', p_tf)
    run('clock', 'give the model two keyframes', p_clock)
    run('isolation', 'weld two instances together with an equality constraint', p_isolation)
    run('cross-contact', 'two overlapping DYNAMIC bodies owned by different instances', p_contact)
    def p_walls():
        """Open one wall. A check that counts walls cannot notice; this row must."""
        spec = mujoco.MjSpec.from_file(str(WORLD))
        target = None
        for g in spec.worldbody.geoms:
            if g.name == 'cell_wall_west':
                target = g
        assert target is not None, 'the probe needs cell_wall_west to exist'
        target.pos = [float(target.pos[0]), 40.0, float(target.pos[2])]
        r = Results()
        check_cell_enclosed(_wrap(spec.compile()), r)
        return r.rows[0]['status'] != 'PASS'

    run('cell-enclosed', 'move the west wall 40 m off the room', p_walls)

    run('settles', 'launch every DOF at 5 m/s', p_settles)
    return out


def _wrap(m, keys=None):
    w = World.__new__(World)
    w.path = WORLD
    w.model = m
    w.home = np.array(m.key_qpos[0]) if m.nkey else np.zeros(m.nq)
    w.keys = keys or tuple(r.key for r in bpw.P3_ROLES)
    return w


def main():
    world = World()
    res = Results()
    check_loads(world, res)
    check_instances_present(world, res)
    check_naming(world, res)
    check_tf_tree(world, res)
    check_time_base(world, res)
    check_isolation(world, res)
    check_no_cross_contact(world, res)
    check_cell_enclosed(world, res)
    check_settles(world, res)
    check_deck_reproduces(world, res)
    probes = falsifiability()
    n_ok = sum(1 for p in probes if p['made_the_check_fail'])
    res.add('every check has been shown able to fail',
            'PASS' if n_ok == len(probes) else 'FAIL',
            f'{n_ok}/{len(probes)}: ' + '; '.join(
                f"{p['check']}={'yes' if p['made_the_check_fail'] else 'NO'}" for p in probes))

    print('=' * 100)
    print('P3 WORLD GATE -- the formal cell as a base for two independent AMRs')
    print('=' * 100)
    print(f'  world     {WORLD.name}  sha256 {hashlib.sha256(WORLD.read_bytes()).hexdigest()[:16]}')
    print(f'  instances {", ".join(world.keys)}')
    print()
    for r in res.rows:
        print(f"{'ok  ' if r['status'] == 'PASS' else 'FAIL'} [{r['status']}] {r['name']}")
        print(f'            {r["detail"]}')
    c = res.counts()
    print(f"\n{c['pass']}/{c['checks']} checks PASS -- verdict {res.verdict()}")
    print('\nNOT established: the final V1 layout (two derived stations, not one shared driving '
          'area); instance 2 carries no deck; ROS-level TF broadcast and a live /clock.')

    out = ROOT / 'reports' / 'p3-world-01'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'gate.json').write_text(json.dumps({
        'world': WORLD.name,
        'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        'instances': list(world.keys),
        'verdict': res.verdict(),
        'counts': res.counts(),
        'checks': res.rows,
        'falsifiability': probes,
    }, indent=1), encoding='utf-8')
    print(f'\nwrote reports/p3-world-01/gate.json')
    return 0 if res.verdict() == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
