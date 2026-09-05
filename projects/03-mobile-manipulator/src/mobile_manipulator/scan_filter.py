"""Mask known robot-body returns; never clear free space through the mast."""
import math


def filter_self_returns(ranges, angle_min, angle_increment, tool=None):
    output=list(ranges)
    for i,r in enumerate(ranges):
        if not math.isfinite(r): continue
        angle=angle_min+i*angle_increment
        x=.14+r*math.cos(angle); y=r*math.sin(angle)
        mast=-.39<x<-.29 and -.27<y<-.17
        column=abs(x)<.08 and abs(y)<.08
        palm=(tool is not None and abs(tool[2]+.1-.60)<.045 and
              math.hypot(x-tool[0],y-tool[1])<.105)
        if mast or column or palm: output[i]=float('nan')
    return output
