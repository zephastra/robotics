import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
import pcl_plane_reader as reader


def mock_backend(monkeypatch,tmp_path,stdout):
    (tmp_path/'runtime').mkdir()
    (tmp_path/'runtime/pcl_receiver_plane').write_bytes(b'fake executable for contract test only')
    monkeypatch.setattr(reader,'ROOT',tmp_path)
    monkeypatch.setattr(reader.subprocess,'run',lambda *a,**kw:SimpleNamespace(stdout=stdout))


def inputs():
    return np.array([[[0.,0.,.82]]]),np.array([[True]]),(-1.,1.,-1.,1.),.82


def test_unavailable_pcl_is_unknown(monkeypatch,tmp_path):
    monkeypatch.setattr(reader,'ROOT',tmp_path)
    assert reader.estimate(*inputs())['status']=='UNKNOWN'


def test_native_out_of_calibration_result_is_rejected(monkeypatch,tmp_path):
    mock_backend(monkeypatch,tmp_path,'{"status":"RESOLVED","floor_z_m":0.9}')
    assert reader.estimate(*inputs())['status']=='UNKNOWN'


def test_malformed_native_result_is_unknown(monkeypatch,tmp_path):
    mock_backend(monkeypatch,tmp_path,'not json')
    assert reader.estimate(*inputs())['status']=='UNKNOWN'


def test_valid_native_result_includes_binary_identity(monkeypatch,tmp_path):
    mock_backend(monkeypatch,tmp_path,'{"status":"RESOLVED","floor_z_m":0.82}')
    result=reader.estimate(*inputs())
    assert result['status']=='RESOLVED' and len(result['binary_sha256'])==64


def test_late_native_result_does_not_authorize_movement(monkeypatch,tmp_path):
    mock_backend(monkeypatch,tmp_path,'{"status":"RESOLVED","floor_z_m":0.82}')
    readings=iter([0.,1.])
    monkeypatch.setattr(reader.time,'monotonic',lambda:next(readings))
    assert reader.estimate(*inputs())['reason']=='STALE_PCL_PLANE'


def test_wrong_native_schema_is_unknown(monkeypatch,tmp_path):
    mock_backend(monkeypatch,tmp_path,'[]')
    assert reader.estimate(*inputs())['reason']=='INVALID_PCL_RESULT'
