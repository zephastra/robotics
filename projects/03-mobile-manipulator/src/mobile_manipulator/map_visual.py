"""Texture-free display of the actual OccupancyGrid for RViz compatibility."""
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def map_markers(grid):
    marker=Marker(); marker.header.frame_id=grid.header.frame_id
    marker.ns='occupancy_grid'; marker.id=0; marker.type=Marker.TRIANGLE_LIST
    marker.action=Marker.ADD; marker.pose=grid.info.origin
    marker.scale.x=marker.scale.y=marker.scale.z=1.
    marker.color.a=1.
    palette=[ColorRGBA(r=.78,g=.78,b=.78,a=1.),ColorRGBA(r=.06,g=.06,b=.06,a=1.),ColorRGBA(r=.35,g=.35,b=.35,a=1.)]
    w=grid.info.width; h=grid.info.height; resolution=grid.info.resolution
    def category(value): return 1 if value>=65 else (0 if 0<=value<=25 else 2)
    for row in range(h):
        col=0
        while col<w:
            start=col; kind=category(grid.data[row*w+col]); col+=1
            while col<w and category(grid.data[row*w+col])==kind: col+=1
            x0,x1=start*resolution,col*resolution; y0,y1=row*resolution,(row+1)*resolution
            for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y0),(x1,y1),(x0,y1)):
                marker.points.append(Point(x=x,y=y,z=-.01)); marker.colors.append(palette[kind])
    return MarkerArray(markers=[marker])
