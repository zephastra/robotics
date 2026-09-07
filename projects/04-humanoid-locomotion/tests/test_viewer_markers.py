"""Keep viewer decoration independent of the blocking text-overlay API."""
from contextlib import nullcontext
from types import SimpleNamespace

import numpy as np
import pytest

from humanoid004.app import add_markers


@pytest.mark.parametrize('goal', [None, {'name': 'A', 'x': 1., 'y': 2.}])
def test_markers_never_call_text_overlay(monkeypatch, goal):
    import humanoid004.app as app

    geometry_calls = []
    monkeypatch.setattr(app.mujoco, 'mjv_initGeom', lambda *args: geometry_calls.append(args))

    class Viewer:
        user_scn = SimpleNamespace(ngeom=0, geoms=[object()])
        cam = SimpleNamespace(lookat=np.zeros(3))

        def lock(self):
            return nullcontext()

        def set_texts(self, *args):
            pytest.fail('Text overlays must not block the simulation/input loop')

    viewer = Viewer()
    add_markers(viewer, goal, [3., 4., 0.], 'MANUAL', 2., [0., 0., 0.], 0.)
    np.testing.assert_allclose(viewer.cam.lookat, [3., 4., .65])
    assert viewer.user_scn.ngeom == int(goal is not None)
    assert len(geometry_calls) == int(goal is not None)
