"""Diagnose the parking creep, and keep the experiment valid AFTER the fix.

Two hypotheses, and they need different fixes:
  H1 residual momentum -- the vehicle was still rolling when the wheels were cut, and it coasts
     because a rigid-body wheel has no rolling resistance. The speed decays; a rate servo kills it.
  H2 a persistent force -- something pushes it (a compressed spring, a contact).

★ The state under test is frozen **while still moving**. The first version drove with
`drive_to`, which now brakes on its own, so both arms of the A/B started from a standstill and the
comparison read "0.00 vs 0.00" -- an experiment that had stopped being able to fail. The frozen
state therefore comes from a bare drive loop that deliberately does NOT settle.

And the creep is reported as DISTANCE, not only speed: at 33.8 mm/s with a ~0.9 s decay constant
the coast carries the vehicle about 30 mm, against a dock tolerance of 3 mm and a guide channel
that has 1 mm of clearance per side. That is why it matters.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import w4_plant as wp  # noqa: E402


def pairs_at(model, data, body_id):
    """Every contact pair touching `body_id` or its subtree, with the normal force."""
    out = []
    subtree = {body_id}
    for b in range(model.nbody):
        if model.body_parentid[b] in subtree:
            subtree.add(b)
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom[0]), int(c.geom[1])
        b1, b2 = int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])
        if b1 not in subtree and b2 not in subtree:
            continue
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1) or f'geom{g1}'
        m = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2) or f'geom{g2}'
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        out.append((n, m, float(c.dist), abs(float(force[0]))))
    return sorted(out, key=lambda r: r[2])


def joints_of(model, body_name):
    row = {}
    body = model.body(body_name).id
    for j in range(model.njnt):
        if int(model.jnt_bodyid[j]) != body:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        row[name] = (int(model.jnt_type[j]), float(model.jnt_stiffness[j]),
                     float(model.jnt_range[j][0]), float(model.jnt_range[j][1]))
    return row


def drive_bare(plant, target_x, *, fast_mps, slow_mps, slow_from_m, stop_short_m):
    """Drive with the plant's own rate servo but WITHOUT its settle, and stop `stop_short_m`
    short of the target so the frozen state is a MOVING one.

    Two speeds, because that is how the real chain arrives: the lane at `DRIVE_SPEED_MPS`, then the
    last `slow_from_m` at `DOCK_SPEED_MPS`. A single slow speed down the whole lane never reaches
    the dock inside the timeout, and a fixture that freezes 1.7 m short is not testing the dock.
    """
    model, data = plant.model, plant.data
    signs = plant._wheel_signs()
    dt = float(model.opt.timestep)
    for _ in range(int(90.0 / dt)):
        error = float(target_x) - plant.chassis_x()
        if abs(error) <= stop_short_m:
            break
        speed = slow_mps if abs(error) <= slow_from_m else fast_mps
        command = float(np.clip(error / 0.4, -speed, speed))
        plant._tick(lambda s: plant._wheel_rate_torque(s, (command / 0.04) * signs[s]))
    return plant.chassis_x(), plant.chassis_speed()


def main():
    plant = wp.LogisticsPlant()
    model, data = plant.model, plant.data
    station = plant.stations['station_b']
    chassis = model.body('n_base_link').id
    dt = float(model.opt.timestep)

    print('=== the driven joints, verbatim from the compiled world ===')
    for side, act in plant.wheel_actuators.items():
        jdof = int(model.jnt_dofadr[model.joint(f'n_wheel_{side}_joint').id])
        print(f'  wheel {side}: damping {float(model.dof_damping[jdof])} '
              f'frictionloss {float(model.dof_frictionloss[jdof])} '
              f'armature {float(model.dof_armature[jdof])} '
              f'forcerange {list(model.actuator_forcerange[act])}')
    print('  -> a small joint damping and no friction loss: cutting the torque leaves a wheel that')
    print('     free-wheels, so the vehicle coasts. It decays, but with a long time constant.')
    print('  datum stop joints:', joints_of(model, 'w2_datum_stop'))
    print('  deck joints:', {k: v[0] for k, v in joints_of(model, 'c_deck').items()})

    print()
    print('=== freeze a MOVING state just short of the receiver dock ===')
    x0, v0 = drive_bare(plant, station['dock_x_m'], fast_mps=wp.DRIVE_SPEED_MPS,
                        slow_mps=wp.DOCK_SPEED_MPS, slow_from_m=0.60, stop_short_m=0.04)
    print(f'  frozen at x {x0:.6f}, speed {v0 * 1000:+.2f} mm/s, '
          f'{station["dock_x_m"] - x0:+.4f} m short of the dock target')
    snapshot = (np.array(data.qpos), np.array(data.qvel), float(data.time))

    print()
    print('=== what could be pushing it: contacts and loaded springs ===')
    for n, m, dist, fn in pairs_at(model, data, chassis)[:6]:
        print(f'    {n:26s} {m:26s} dist {dist:+.6f} fn {fn:9.4f}')
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'w2_datum_slide')
    addr = int(model.jnt_qposadr[jid]) if jid >= 0 else None
    print('  w2_datum_slide qpos:', None if addr is None else round(float(data.qpos[addr]), 6),
          '(0.0 = the datum stop is NOT compressed, so it is not pushing)')
    print('  c_deck_slide qpos:', round(plant._hold_deck_slide(), 6))

    def replay(label, torque_of_side):
        data.qpos[:], data.qvel[:], data.time = snapshot[0], snapshot[1], snapshot[2]
        mujoco.mj_forward(model, data)
        start = plant.chassis_x()
        series = []
        for i in range(int(2.0 / dt)):
            plant._tick(torque_of_side)
            if i % 20 == 0:
                series.append((i * dt, plant.chassis_speed()))
        travel = abs(plant.chassis_x() - start)
        window = max(abs(v) for t, v in series if t >= 0.25)
        print(f'  {label}')
        for t, v in series[::5]:
            print(f'    t {t:5.2f}   v {v * 1000:+8.2f} mm/s')
        print(f'    -> max |v| over the contract window (t>=0.25 s): {window * 1000:8.2f} mm/s')
        print(f'    -> COASTED {travel * 1000:7.2f} mm in 2.0 s')
        return window, travel

    print()
    print('=== A/B from that ONE frozen, still-moving state ===')

    def zero(side):
        return 0.0

    def servo(side):
        return plant._wheel_rate_torque(side, 0.0)

    a, a_travel = replay('control: wheels at ZERO torque (the previous behaviour)', zero)
    b, b_travel = replay('candidate: rate servo with a zero setpoint (the brake)', servo)

    print()
    print('=== verdict ===')
    print(f'  zero torque : held {a * 1000:8.2f} mm/s, coasted {a_travel * 1000:7.2f} mm')
    print(f'  the brake   : held {b * 1000:8.2f} mm/s, coasted {b_travel * 1000:7.2f} mm')
    print(f'  contract stopped_speed_mps {wp.STOPPED_SPEED_MPS * 1000:.1f} mm/s; '
          f'zero torque passes {a <= wp.STOPPED_SPEED_MPS}; brake passes '
          f'{b <= wp.STOPPED_SPEED_MPS}')
    print(f'  the dock tolerance is 0.003 m and the guide channel has 1 mm per side, so '
          f'{a_travel * 1000:.0f} mm of coast is the difference between docking and not')


if __name__ == '__main__':
    main()
