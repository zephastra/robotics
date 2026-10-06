"""Zero-step contact flag inventory; no MjData or physical writer."""
import json
from pathlib import Path
import mujoco
root=Path(__file__).resolve().parents[1]
model=mujoco.MjModel.from_xml_path(str(root/'assets/world_p5_candidate_v5.xml'))
print(json.dumps(dict(version=mujoco.__version__,physics_steps=0,
    integrator=int(model.opt.integrator),solver=int(model.opt.solver),
    iterations=int(model.opt.iterations),timestep=float(model.opt.timestep),
    contacts={name:dict(solref=model.geom_solref[model.geom(name).id].tolist(),
        solimp=model.geom_solimp[model.geom(name).id].tolist(),
        condim=int(model.geom_condim[model.geom(name).id]))
        for name in ('c_tray_floor','c_deck_roller_1','c_deck_roller_idler_1_side1')},
    enableflags=int(model.opt.enableflags),disableflags=int(model.opt.disableflags),
    enable_bits={k:int(v) for k,v in mujoco.mjtEnableBit.__members__.items()},
    disable_bits={k:int(v) for k,v in mujoco.mjtDisableBit.__members__.items()}),indent=2))
