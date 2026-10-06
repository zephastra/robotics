"""Shared-zone interlock: two actors may use the same floor, but not at the same time.

`docs/MASTER_PLAN.md` section 2, step 3:

    人形退出共享操作区并证明确已清空，固定臂获得区内操作许可。

The clause that makes this a real object is **证明确已清空 -- PROVE it is clear**. A handshake that
relays "the humanoid says it has left" is not a proof; the proof is a measurement of the geometry
that would collide. So this module measures, and it refuses on the measurement.

WHY THIS IS NOT THE COMMAND GATE
`safety.CommandGate` decides what the actuator is *told* -- TTL, silence, velocity -- and it has no
notion of where any body is. This asks a different question, *who may occupy the region*, so it is a
different object. Neither substitutes for the other, and a report needs both.

THE MEASUREMENT, AND THE TWO TRAPS IT AVOIDS
The distance is taken over **collision-enabled geoms only**. That is not tidiness:

* This project already measured (`D109`, the H3 audit) that `mj_geomDistance` answers about
  GEOMETRY, and that a visual-only geom (`contype = 0`, `conaffinity = 0`) produces phantom
  near-collisions -- the same function reported 272 self-penetrating pairs for a robot standing
  still, including a body against *itself*. A zone that fired on visual meshes would refuse forever.
  Measured here too: over all geoms the arm/humanoid minimum is 0.255643 m and the closest pair is
  a different pair from the collision-enabled answer of 0.266779 m.
* `distmax` is a **RANGE LIMIT, not a ceiling**. When a pair's true distance is at or beyond it the
  return value is not a distance, so such pairs are counted as `n_at_budget` and reported as
  beyond-budget rather than quoted as a number.

WHAT THIS DOES NOT DO
It does not model the arm's swept volume. The margin is a declared distance between the two actors'
collision geometry, measured, not a claim about trajectories. It also does not stop anyone: it
answers a permission question. Enforcement is the caller's, and the evidence has to show the caller
acting on it.
"""
import math
from .schema import SchemaRefused

#: Every reason this module can refuse with. Closed and declared, so a report can count them and so
#: a new refusal cannot appear that nothing is able to label.
INTERLOCK_REFUSALS = frozenset({
    'REFUSED_NOT_CLEAR',        # the other actor is inside the required margin, as measured
    'REFUSED_ALREADY_HELD',     # another actor already holds the zone
    'REFUSED_NOT_THE_HOLDER',   # a release/permit from someone who does not hold it
    'REFUSED_NO_SUCH_ACTOR',
    'REFUSED_BAD_MARGIN',
    'REFUSED_NO_EVIDENCE',      # no collision-enabled pairs is UNKNOWN, not distant
})

#: The actors allowed to hold the zone. Declared, so "who may ask" has an answer.
ACTORS = ('arm', 'humanoid')


def collision_enabled(model, geom):
    """Whether the solver would ever generate a contact for this geom."""
    return int(model.geom_contype[geom]) != 0 or int(model.geom_conaffinity[geom]) != 0


def geom_sets(model, prefix_by_actor):
    """{actor: {'all': [...], 'live': [...]}} by BODY-NAME prefix.

    By prefix rather than by a hand-written list: a list is correct only until someone adds a link.
    """
    out = {actor: {'all': [], 'live': []} for actor in prefix_by_actor}
    for g in range(model.ngeom):
        name = None
        import mujoco
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) or ''
        for actor, prefix in prefix_by_actor.items():
            if name.startswith(prefix):
                out[actor]['all'].append(g)
                if collision_enabled(model, g):
                    out[actor]['live'].append(g)
                break
    return out


def closest_pair(model, data, geoms_a, geoms_b, *, budget_m):
    """(distance | None, pair | None, n_at_budget) over the geoms given.

    A pair at or beyond `budget_m` is NOT returned as a distance -- `distmax` is a range limit, so
    quoting its return value would be quoting the budget back as if it were a measurement.
    A negative distance means penetration; it is returned, not clamped to zero, because
    "0.000000" and "-0.008" are different findings.
    """
    import mujoco
    best, who, at_budget = None, None, 0
    for ga in geoms_a:
        for gb in geoms_b:
            d = float(mujoco.mj_geomDistance(model, data, ga, gb, budget_m, None))
            if d >= budget_m:
                at_budget += 1
                continue
            if best is None or d < best:
                best, who = d, (ga, gb)
    return best, who, at_budget


class SharedZone:
    """The contested region between two actors, DERIVED from the model on every call.

    `bounds_x()` is recomputed from the current state, so it cannot go stale the way a typed
    constant does -- which is the failure this project has now recorded four times.
    """

    def __init__(self, model, data, *, prefix_by_actor, margin_m, budget_m):
        self.model = model
        self.data = data
        self.prefix_by_actor = dict(prefix_by_actor)
        self.margin_m = float(margin_m)
        self.budget_m = float(budget_m)
        self.geoms = geom_sets(model, self.prefix_by_actor)

    def actors(self):
        return tuple(sorted(self.prefix_by_actor))

    def extent_x(self, actor):
        """(lo, hi) of this actor's COLLISION-ENABLED geometry, in world x. Conservative bound."""
        import numpy as np
        gids = self.geoms[actor]['live']
        if not gids:
            return None
        lo, hi = np.inf, -np.inf
        for g in gids:
            c = float(self.data.geom_xpos[g][0])
            r = float(np.max(self.model.geom_size[g][:3]))
            lo, hi = min(lo, c - r), max(hi, c + r)
        return (lo, hi)

    def separation(self, a, b):
        d, pair, n = closest_pair(self.model, self.data, self.geoms[a]['live'],
                                  self.geoms[b]['live'], budget_m=self.budget_m)
        return {'distance_m': d, 'pair': pair, 'n_at_budget': n,
                'names': self._pair_names(pair)}

    def _pair_names(self, pair):
        import mujoco
        if pair is None:
            return None
        out = []
        for g in pair:
            b = int(self.model.geom_bodyid[g])
            out.append('%s/geom%d'
                       % (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b) or '?', g))
        return ' vs '.join(out)

    def bounds_x(self):
        """The x-interval in which either actor is within `margin_m` of the other.

        Both extents padded by the margin, then intersected -- the region where an intrusion is
        possible. Empty (lo > hi) means the zone does not exist in this configuration, and that is
        reported rather than hidden.
        """
        ex = [self.extent_x(a) for a in self.actors()]
        ex = [e for e in ex if e is not None]
        if len(ex) < 2:
            return None
        lo = max(e[0] - self.margin_m for e in ex)
        hi = min(e[1] + self.margin_m for e in ex)
        return {'lo': lo, 'hi': hi, 'empty': lo > hi, 'margin_m': self.margin_m}


class Interlock:
    """Who may occupy the shared region. Granted on a measurement, never on a claim."""

    def __init__(self, *, model, data, margin_m, prefix_by_actor=None, budget_m=1.0):
        if (not math.isfinite(margin_m) or margin_m <= 0
                or not math.isfinite(budget_m) or budget_m < margin_m):
            raise SchemaRefused('REFUSED_BAD_MARGIN', repr(margin_m), field='margin_m')
        self.zone = SharedZone(model, data,
                               prefix_by_actor=prefix_by_actor or {'arm': 'a_',
                                                                   'humanoid': 'h_'},
                               margin_m=float(margin_m), budget_m=float(budget_m))
        self.holder = None
        self.counters = {'granted': 0, 'refused': 0, 'released': 0}
        self.reasons = {}
        self.history = []

    def _refuse(self, reason, who):
        self.counters['refused'] += 1
        self.reasons[reason] = self.reasons.get(reason, 0) + 1
        self.history.append({'who': who, 'granted': False, 'reason': reason})
        return False, reason

    def request(self, who):
        """Ask to occupy the zone. Granted only if the OTHER actor is measurably clear.

        The order matters and is the whole point of the clause: the humanoid leaves, THAT is
        measured, and only then can the arm be granted. So a request is refused unless the other
        actor is at least `margin_m` away -- a claim of having left is not consulted, because this
        object never receives one.
        """
        if who not in self.zone.actors():
            return self._refuse('REFUSED_NO_SUCH_ACTOR', who)
        if self.holder is not None and self.holder != who:
            return self._refuse('REFUSED_ALREADY_HELD', who)
        if (len(self.zone.actors()) != 2
                or any(not self.zone.geoms[a]['live'] for a in self.zone.actors())):
            return self._refuse('REFUSED_NO_EVIDENCE', who)
        other = next((a for a in self.zone.actors() if a != who), None)
        sep = self.separation(who, other) if other else {'distance_m': None}
        d = sep['distance_m']
        if d is not None and not math.isfinite(d):
            return self._refuse('REFUSED_NO_EVIDENCE', who)
        if d is None:
            # every pair beyond budget: clear by a wide margin, which is still a measurement
            self.holder = who
            self.counters['granted'] += 1
            self.history.append({'who': who, 'granted': True,
                                 'reason': 'GRANTED_BEYOND_BUDGET',
                                 'distance_m': None, 'n_at_budget': sep['n_at_budget']})
            return True, 'GRANTED_BEYOND_BUDGET'
        if d < self.zone.margin_m:
            return self._refuse('REFUSED_NOT_CLEAR', who)
        self.holder = who
        self.counters['granted'] += 1
        self.history.append({'who': who, 'granted': True, 'reason': 'GRANTED',
                             'distance_m': d, 'pair': sep['names']})
        return True, 'GRANTED'

    def release(self, who):
        if who != self.holder:
            return self._refuse('REFUSED_NOT_THE_HOLDER', who)
        self.holder = None
        self.counters['released'] += 1
        self.history.append({'who': who, 'granted': None, 'reason': 'RELEASED'})
        return True, 'RELEASED'

    def separation(self, a, b):
        return self.zone.separation(a, b)

    def snapshot(self):
        """Everything a report needs, read from the current state rather than remembered."""
        ex = {a: self.zone.extent_x(a) for a in self.zone.actors()}
        acts = self.zone.actors()
        return {
            'margin_m': self.zone.margin_m,
            'budget_m': self.zone.budget_m,
            'holder': self.holder,
            'counters': dict(self.counters),
            'reasons': dict(self.reasons),
            'extent_x': {a: (None if e is None else [e[0], e[1]]) for a, e in ex.items()},
            'bounds_x': self.zone.bounds_x(),
            'separation': (self.separation(*acts) if len(acts) == 2 else None),
            'history': list(self.history),
        }
