from nav_msgs.msg import OccupancyGrid
import pytest
from mobile_manipulator.map_visual import map_markers


def test_map_geometry_matches_grid_and_compresses_runs():
    grid=OccupancyGrid(); grid.header.frame_id='map'
    grid.info.width=4; grid.info.height=1; grid.info.resolution=.05
    grid.info.origin.orientation.w=1.; grid.data=[0,0,100,-1]
    marker=map_markers(grid).markers[0]
    assert len(marker.points)==18  # Three same-colour runs, not four cubes.
    assert marker.header.frame_id=='map'
    assert marker.points[1].x==pytest.approx(.1)
    assert marker.colors[0].r>.7 and marker.colors[6].r<.1
