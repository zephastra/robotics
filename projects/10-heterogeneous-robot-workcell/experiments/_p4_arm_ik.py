# -*- coding: utf-8 -*-
"""Arm IK solved on a SCRATCH MjData, so the probe never touches the simulation's own state.

WHY THIS FILE EXISTS
--------------------
`probe_p4_reach.ik` reads `d.site_xpos[sid]` from the rig it is handed. That is correct inside
`probe_p4_reach`, where the rig IS the solver's data. But when it is called with one rig while
`qpos` was written to a DIFFERENT rig's data, it evaluates its error at a pose it never set --
the solver reports "solved" for a configuration it never visited. The earlier round's
"-0.67 rad elbow recovers everything" result came from exactly that, and its giveaway was that the
shoulder did not move even though the arm chain cannot move the shoulder.

So the rule here is: **one data object, owned by this solver, seeded from the caller's qpos.**

WHAT IT SOLVES
--------------
Both hands onto their tray handles, as functions of the pelvis target. The arm has 5 dof per side;
the task is 3 (position only), so the system is redundant and the solution is whatever the damped
least squares walks to from the seed. That is stated rather than hidden: the pose is not unique,
and the criterion on it is the hand-to-handle DISTANCE, not the joint angles.

Not touching the tray: the handles are read from the tray's own current position, which the legs
cannot move. If a future run carries the tray with the hands, the handle position becomes a
function of the arm pose too, and this becomes a fixed point rather than a solve.
"""
import numpy as np


class ArmIK:
    """Damped-least-squares arm IK against its own scratch data."""

    def __init__(self, model, arm_joints, limits, site_of_side, *, lam=0.02, step_cap=0.15):
        import mujoco
        self._mj = mujoco
        self.m = model
        self.scratch = mujoco.MjData(model)
        self.arm_joints = tuple(arm_joints)
        self.limits = limits
        self.site_of_side = site_of_side
        self.lam = float(lam)
        self.step_cap = float(step_cap)

        self.qadrs = np.array([int(model.jnt_qposadr[model.joint('h_' + n).id])
                               for n in self.arm_joints])
        self.dofs = np.array([int(model.jnt_dofadr[model.joint('h_' + n).id])
                              for n in self.arm_joints])
        self.lows = np.array([float(limits[n][0]) for n in self.arm_joints])
        self.highs = np.array([float(limits[n][1]) for n in self.arm_joints])
        self.sites = {}
        for side, name in site_of_side.items():
            self.sites[side] = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name))
            if self.sites[side] < 0:
                raise RuntimeError('the world has no site %s' % name)

        # bookkeeping, so the caller can report what the solve actually did rather than assert it
        self.calls = 0
        self.iterations = 0
        self.world_calls = 0
        self.clamp_hits = 0

    def hand(self, side, q):
        """Where the hand is, at `q`, on the scratch data. One forward, no solve."""
        d = self.scratch
        d.qpos[:] = q
        self._mj.mj_forward(self.m, d)
        return np.array(d.site_xpos[self.sites[side]], dtype=float)

    def solve(self, q_seed, targets, *, iters=300, tol=1e-6):
        """Put each hand on its target. Returns (q, worst_residual, per_side_residual).

        `targets` maps side -> the world point the hand must reach. The solve is joint-space
        sequential: one side, then the other, both on the same scratch data, so the second side
        sees the first side's result. Two passes, because the two arms do not interact through the
        tray (it is a separate free body) but they DO share the torso, and a single pass would
        leave the first arm's correction unverified.
        """
        q = np.array(q_seed, dtype=float)
        per = {}
        for _pass in range(2):
            for side in sorted(targets):
                res, q = self._solve_one(q, side, np.asarray(targets[side], dtype=float), iters, tol)
                per[side] = res
        worst = max(per.values()) if per else float('inf')
        return q, worst, per

    def _solve_one(self, q_seed, side, target, iters, tol):
        m, d = self.m, self.scratch
        sid = self.sites[side]
        q = np.array(q_seed, dtype=float)
        jacp = np.zeros((3, m.nv))
        best_q, best = np.array(q), float('inf')
        self.calls += 1
        for _ in range(iters):
            d.qpos[:] = q
            self._mj.mj_forward(m, d)
            self.world_calls += 1
            err = target - np.asarray(d.site_xpos[sid], dtype=float)
            r = float(np.linalg.norm(err))
            if r < best:
                best_q, best = np.array(q), r
            if r < tol:
                break
            self.iterations += 1
            jacp[:] = 0.0
            self._mj.mj_jacSite(m, d, jacp, None, sid)
            J = jacp[:, self.dofs]
            dq = J.T @ np.linalg.solve(J @ J.T + (self.lam ** 2) * np.eye(3), err)
            biggest = float(np.max(np.abs(dq)))
            if biggest > self.step_cap:
                dq = dq * (self.step_cap / biggest)
            new = q[self.qadrs] + dq
            clipped = np.clip(new, self.lows, self.highs)
            self.clamp_hits += int(np.count_nonzero(np.abs(clipped - new) > 1e-12))
            q[self.qadrs] = clipped
        return best, best_q
