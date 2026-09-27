"""The simulator's command gate: it decides what the wheels are actually told, and why.

The ROS side may ask; the simulator may refuse. Every refusal is counted under a reason code,
because the failure mode this guards against is a fallback that nobody can see -- the same
shape as 009's lifecycle manager whose "service_timeout" was never set, where the log line
said "send failed" and the cause was a reply that arrived late.

Three rules, all from the P1-N scope:

  * a command carries the sim time it was issued and a validity window (`ttl_s`); a command
    that is older than its window is refused, not clamped,
  * silence -- no accepted command for `silence_s` of sim time -- falls back to zero, so a
    dead ROS process stops the robot instead of leaving it driving on the last command,
  * anything malformed is refused with a reason and never extends the lease.
"""
from . import Refused, decode


class CommandGate:
    """Holds the currently valid velocity command and reports how it got there."""

    def __init__(self, *, ttl_s, silence_s, v_max, w_max):
        self.ttl_s = float(ttl_s)
        self.silence_s = float(silence_s)
        self.v_max = float(v_max)
        self.w_max = float(w_max)
        self.active = None              # the last accepted command
        self.last_accepted_sim_time = None
        self.highest_seq = -1
        self.counters = {'accepted': 0, 'refused': 0}
        self.reasons = {}

    def _refuse(self, reason):
        self.counters['refused'] += 1
        self.reasons[reason] = self.reasons.get(reason, 0) + 1

    def offer(self, raw, now_sim):
        """Try to accept a datagram. Returns (accepted, reason).

        A refused datagram leaves the active command untouched: it neither extends nor
        cancels it. The lease is governed by `ttl_s` alone.
        """
        try:
            payload = decode('command', raw)
            if abs(payload['v']) > self.v_max:
                raise Refused('REFUSED_OUT_OF_RANGE', f"v={payload['v']} > {self.v_max}")
            if abs(payload['w']) > self.w_max:
                raise Refused('REFUSED_OUT_OF_RANGE', f"w={payload['w']} > {self.w_max}")
            if payload['seq'] <= self.highest_seq:
                raise Refused('REFUSED_STALE_SEQ',
                              f"{payload['seq']} <= {self.highest_seq}")
            if payload['issued_sim_time'] > now_sim + 0.05:
                raise Refused('REFUSED_FROM_THE_FUTURE',
                              f"{payload['issued_sim_time']} > {now_sim}")
            if now_sim - payload['issued_sim_time'] > payload['ttl_s']:
                raise Refused('REFUSED_ALREADY_EXPIRED',
                              f"age {now_sim - payload['issued_sim_time']:.3f}s")
        except Refused as exc:
            self._refuse(exc.reason)
            return False, exc.reason
        self.highest_seq = payload['seq']
        self.active = payload
        self.last_accepted_sim_time = now_sim
        self.counters['accepted'] += 1
        return True, 'ACCEPTED'

    def step(self, now_sim):
        """What the wheels are told this step: (v, w, mode, reason)."""
        if self.active is None:
            return 0.0, 0.0, 'SILENT', 'NO_COMMAND_EVER'
        if (self.last_accepted_sim_time is not None
                and now_sim - self.last_accepted_sim_time > self.silence_s):
            return 0.0, 0.0, 'SILENT', 'SILENT_SINCE_LAST_COMMAND'
        age = now_sim - self.active['issued_sim_time']
        if age > self.active['ttl_s']:
            return 0.0, 0.0, 'EXPIRED', 'TTL_EXPIRED'
        return self.active['v'], self.active['w'], 'HOLDING', 'HELD'

    def summary(self, now_sim):
        v, w, mode, reason = self.step(now_sim)
        age = 0.0
        if self.active is not None:
            age = now_sim - self.active['issued_sim_time']
        return {
            'mode': mode,
            'reason': reason,
            'age_s': round(age, 6),
            'v': v,
            'w': w,
            'accepted': self.counters['accepted'],
            'refused': self.counters['refused'],
            'reasons': dict(sorted(self.reasons.items())),
        }
