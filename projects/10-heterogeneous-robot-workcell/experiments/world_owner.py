"""One final command writer/time owner for the candidate shared MuJoCo world.

Freeze is explicitly a SIMULATION fallback, not a mechanical stop. This owner
cannot resume from a frozen moving state. Recovery integration is not claimed.
Controllers provide control vectors; they must not write/step MjData themselves.
"""
import threading
import weakref

import mujoco
import numpy as np


class WorldSafetyHold(RuntimeError):
    pass


class WorldOwner:
    _owners = weakref.WeakValueDictionary()
    _registry_lock = threading.Lock()

    def __init__(self, model, data, permit):
        if permit is None:
            raise ValueError('shared world requires a runtime permit')
        if data.qpos.shape != (model.nq,) or data.ctrl.shape != (model.nu,):
            raise ValueError('world state dimensions do not match model')
        with self._registry_lock:
            if id(data) in self._owners:
                raise ValueError('MjData already has a final writer')
            self._owners[id(data)] = self
        self.model, self.data, self.permit = model, data, permit
        self.lock = threading.RLock()
        self.frozen = None
        self.steps = 0
        self.rejected_steps = 0

    def attach(self, plant):
        if plant.model is not self.model or plant.data is not self.data:
            raise ValueError('controller must share the exact model and data')
        if getattr(plant, 'step_owner', None) not in (None, self):
            raise ValueError('controller already belongs to another writer')
        plant.step_owner = self

    def _hold(self, verdict):
        if self.frozen is None:
            self.frozen = dict(verdict, simulation_frozen=True,
                hold_mode='SIM_FROZEN_NO_PHYSICAL_STOP_PROOF',
                stopped_confirmed=False, frozen_at_sim_s=float(self.data.time))
        self.rejected_steps += 1
        raise WorldSafetyHold(self.frozen['reason_code'])

    def commit(self, produce_control, *, advance=False):
        with self.lock:
            if self.frozen is not None:
                self._hold(self.frozen)
            permission = self.permit.check()
            if not permission['allowed']:
                self._hold(permission)
            try:
                control = np.asarray(produce_control(), dtype=float).copy()
                if control.shape != (self.model.nu,) or not np.all(np.isfinite(control)):
                    raise ValueError('invalid control vector')
            except Exception:
                self._hold(self.permit.stop('CONTROL_UNAVAILABLE', 'control production failed'))
            # A slow callback can outlive its wall-time permit. Check at COMMIT too.
            permission = self.permit.check()
            if not permission['allowed']:
                self._hold(permission)
            self.data.ctrl[:] = control
            if advance:
                mujoco.mj_step(self.model, self.data)
                self.steps += 1
            return float(self.data.time)

    def step(self, produce_control):
        return self.commit(produce_control,advance=True)
