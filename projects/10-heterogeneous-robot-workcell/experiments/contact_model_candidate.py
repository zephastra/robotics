"""Explicit init-only multi-contact candidate for installed MuJoCo 3.3.6.

No geometry, friction coefficient, timestep, actuator, qpos or equality writes.
Not qualified by this function: raw contacts/stopping must be revalidated.
"""
import mujoco
import math


def deck_contact_time_constant(model, value):
    """Init-only solver compliance candidate; never fake support or constraints."""
    if not math.isfinite(value) or not 2*float(model.opt.timestep)<=value<=.02:
        raise ValueError('contact time constant must be finite, >=2 steps and <=0.02 s')
    changed=[]
    for index in range(model.ngeom):
        name=model.geom(index).name or ''
        if name=='c_tray_floor' or name.startswith('c_deck_roller'):
            before=model.geom_solref[index].copy()
            if before[0]<=0:raise ValueError('direct-format contact needs separate qualification')
            model.geom_solref[index,0]=value
            changed.append(dict(geom=name,before=before.tolist(),after=model.geom_solref[index].tolist()))
    if not changed:raise ValueError('deck contact candidate has no target geoms')
    return dict(candidate='INIT_ONLY_DECK_SOLREF_TIME_CONSTANT',changes=changed,
        timestep_changes=[],friction_changes=[],geometry_changes=[],qualified=False)


def enable_multiccd(model):
    bit=getattr(mujoco.mjtEnableBit,'mjENBL_MULTICCD',None)
    if bit is None:raise ValueError('unsupported MuJoCo multiccd flag API; requalify instead of guessing')
    before=int(model.opt.enableflags)
    model.opt.enableflags=before|int(bit)
    return dict(candidate='INIT_ONLY_MULTICCD',mujoco_version=mujoco.__version__,
        enableflags_before=before,enableflags_after=int(model.opt.enableflags),
        friction_changes=[],geometry_changes=[],timestep_changes=[],qualified=False)
