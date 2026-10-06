"""Math only, not physical docking or permissible envelope proof."""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from dock_measurements import source_residual


def test_aligned_junction_zero():
    assert source_residual([5.,0.],[5.08,0.],.08,np.eye(3))==dict(lateral_m=0.,longitudinal_m=0.,yaw_rad=0.)


def test_actual_base_displacement_cannot_hide_behind_cached_crowns():
    result=source_residual([5.,0.],[5.11,.12],.08,np.eye(3))
    assert result['longitudinal_m']==pytest.approx(-.03)
    assert result['lateral_m']==pytest.approx(.12)


def test_rotation_moves_the_actual_first_crown_and_is_measured():
    yaw=-.2;rotation=np.array([[np.cos(yaw),-np.sin(yaw),0],
        [np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    deck=np.array([4.,0.])+rotation[:2,:2]@np.array([1.08,0.])
    result=source_residual([5.,0.],deck,.08,rotation)
    assert result['lateral_m']<-.2
    assert result['yaw_rad']==pytest.approx(yaw)


@pytest.mark.parametrize('rotation',[np.zeros((3,3)),np.diag([-1,1,1]),np.eye(3)*np.nan])
def test_invalid_sensor_geometry_refused(rotation):
    with pytest.raises(ValueError):source_residual([5.,0.],[5.08,0.],.08,rotation)


def test_actual_plant_method_reads_live_crown_not_cached_slide(monkeypatch):
    """Fake instrument wiring, not a new physical docking run."""
    from types import SimpleNamespace
    import w4_plant
    monkeypatch.setattr(w4_plant.mujoco, 'mj_name2id', lambda *args: 0)
    crowns = {'c_deck_frame': np.zeros(3),
              'c_fixed_roller_0_0': np.array([4., 0., .81]),
              'c_deck_roller_0': np.array([5.11, .12, .81])}
    plant = SimpleNamespace(source_band=[4., 5.], pitch=.08,
                            data=SimpleNamespace(geom_xmat=np.eye(3).reshape(1, 9)),
                            model=None, _geom=lambda name: crowns[name],
                            deck_row=[5.08])
    first = w4_plant.LogisticsPlant.dock_residuals(plant, 'station_c')
    assert first['longitudinal_m'] == pytest.approx(-.03)
    assert first['lateral_m'] == pytest.approx(.12)
    crowns['c_deck_roller_0'] = np.array([5.08, -.04, .81])
    second = w4_plant.LogisticsPlant.dock_residuals(plant, 'station_c')
    assert second['longitudinal_m'] == pytest.approx(0.)
    assert second['lateral_m'] == pytest.approx(-.04)
    assert plant.deck_row == [5.08]  # cached nominal crown stayed unchanged
