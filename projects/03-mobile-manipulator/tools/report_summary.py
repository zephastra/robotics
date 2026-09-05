#!/usr/bin/env python3
"""Summarize recorded simulator evidence without claiming real-world accuracy."""
import json
import math
from pathlib import Path
import sys

root=Path(__file__).resolve().parents[1]
if len(sys.argv)>1:
    path=Path(sys.argv[1])
else:
    files=list((root/'reports').glob('*/mission.json'))
    if not files: raise SystemExit('No mission report found.')
    path=max(files,key=lambda p:p.stat().st_mtime)
data=json.loads(path.read_text())
summary={'report':str(path),'status':data['status'],'checks':data['checks']}
if 'wall_seconds' in data: summary['wall_seconds']=data['wall_seconds']
elif data.get('events'): summary['last_event_wall_seconds']=data['events'][-1]['wall_elapsed']
if 'error' in data: summary['error']=data['error']
if data.get('final_physics',{}).get('cargo') and data.get('final_physics',{}).get('base'):
    state=data['final_physics']
    summary['box_xy_error_m']=math.dist(state['cargo'][:2],[2.2,3.])
    summary['home_xy_error_m']=math.dist(state['base'][:2],[0.,0.])
    summary['holder']=state['holder']
print(json.dumps(summary,indent=2))
