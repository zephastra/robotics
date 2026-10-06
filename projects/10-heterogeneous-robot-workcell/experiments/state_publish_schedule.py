"""Publish NEW physical states on sim cadence or bounded wall cadence.

Rendering can make simulation time advance slowly. Never manufacture a clock
tick, repeat a sim timestamp, advance physics, or loosen any bridge watchdog.
"""
import math


class StatePublishSchedule:
    def __init__(self,sim_period_s=.05,wall_period_s=.25):
        if not all(math.isfinite(v) and v>0 for v in (sim_period_s,wall_period_s)):
            raise ValueError('finite positive publish periods required')
        self.sim_period_s=sim_period_s;self.wall_period_s=wall_period_s
        self.last_sim=None;self.last_wall=None

    def due(self,sim_s,wall_s):
        if not all(math.isfinite(v) for v in (sim_s,wall_s)):
            raise ValueError('finite clocks required')
        if self.last_sim is not None:
            if sim_s<self.last_sim or wall_s<self.last_wall:
                raise RuntimeError('PUBLISH_CLOCK_REVERSED')
            if sim_s==self.last_sim:return False
            if (sim_s-self.last_sim<self.sim_period_s
                    and wall_s-self.last_wall<self.wall_period_s):return False
        self.last_sim,self.last_wall=sim_s,wall_s
        return True
