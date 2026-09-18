from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

from humanoid007 import app


def test_parallel_reports_remain_unique_with_identical_timestamps(tmp_path,monkeypatch):
    fixed=SimpleNamespace(strftime=lambda _: 'same-timestamp')
    monkeypatch.setattr(app,'datetime',SimpleNamespace(now=lambda _:fixed))
    with ThreadPoolExecutor(max_workers=8) as workers:
        paths=list(workers.map(lambda _:app.new_report_directory(tmp_path/'reports'),range(32)))
    assert len(set(paths))==32
    assert all(p.is_dir() and p.parent==tmp_path/'reports' for p in paths)
