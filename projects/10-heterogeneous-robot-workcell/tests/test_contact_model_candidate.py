from pathlib import Path
import sys
from types import SimpleNamespace
import mujoco
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from contact_model_candidate import enable_multiccd
from contact_model_candidate import deck_contact_time_constant
import numpy as np
import pytest


def test_multiccd_changes_only_its_enable_bit():
    opt=SimpleNamespace(enableflags=int(mujoco.mjtEnableBit.mjENBL_ENERGY),
        disableflags=0,timestep=.002,cone=0,impratio=1.)
    model=SimpleNamespace(opt=opt)
    original=dict(vars(opt));report=enable_multiccd(model)
    assert opt.enableflags==original['enableflags']|int(mujoco.mjtEnableBit.mjENBL_MULTICCD)
    assert {k:v for k,v in vars(opt).items() if k!='enableflags'}=={k:v for k,v in original.items() if k!='enableflags'}
    assert report['qualified'] is False
    assert report['geometry_changes']==report['friction_changes']==report['timestep_changes']==[]


def test_candidate_never_steps_or_initializes_or_attaches_cargo():
    source=(Path(__file__).resolve().parents[1]/'experiments/contact_model_candidate.py').read_text()
    assert 'mj_step(' not in source and 'MjData(' not in source and 'qpos[' not in source


def test_probe_candidates_are_opt_in_and_enabled_before_owner():
    root=Path(__file__).resolve().parents[1]/'experiments'
    for name in ('probe_vehicle_nav2.py','probe_candidate_multi_loading.py'):
        source=(root/name).read_text()
        assert "'--multi-ccd',action='store_true'" in source
        call=source.index("report['contact_candidate']=enable_multiccd(model)")
        boundary=('WorldOwner(model, data' if name=='probe_vehicle_nav2.py'
                  else 'data=mujoco.MjData(model)')
        assert source.index('if args.multi_ccd:')<call<source.index(boundary)


def test_deck_solver_candidate_changes_only_declared_time_constants():
    names=['c_tray_floor','c_deck_roller_0','other_geom']
    model=SimpleNamespace(ngeom=3,opt=SimpleNamespace(timestep=.002),
        geom=lambda index:SimpleNamespace(name=names[index]),
        geom_solref=np.array([[.02,1.],[.02,1.],[.02,1.]]))
    result=deck_contact_time_constant(model,.01)
    assert model.geom_solref.tolist()==[[.01,1.],[.01,1.],[.02,1.]]
    assert result['qualified'] is False
    assert result['friction_changes']==result['geometry_changes']==result['timestep_changes']==[]


@pytest.mark.parametrize('value',[float('nan'),.003,.021])
def test_deck_solver_candidate_refuses_unsafe_time_constant(value):
    model=SimpleNamespace(opt=SimpleNamespace(timestep=.002))
    with pytest.raises(ValueError):deck_contact_time_constant(model,value)
