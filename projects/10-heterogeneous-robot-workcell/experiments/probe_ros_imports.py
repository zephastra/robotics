"""No nodes, discovery or motion: inspect ROS ABI/package availability only."""
import importlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

result = {'python':sys.version, 'modules':{}, 'packages':{}}
for name in ('rclpy','sensor_msgs.msg','nav_msgs.msg','geometry_msgs.msg','rosgraph_msgs.msg','tf2_ros','nav2_msgs.action'):
    try:
        module=importlib.import_module(name)
        result['modules'][name]={'status':'IMPORT_OK','path':module.__file__}
    except Exception as exc:
        result['modules'][name]={'status':'ERROR','error':str(exc)}
for name in ('nav2_bringup','slam_toolbox'):
    path=Path('/opt/nav2')/name/'share'/name/'package.xml'
    result['packages'][name]=ET.parse(path).getroot().findtext('version')
result['navigation_acceptance']='NOT_RUN: import success is not navigation'
out=Path(__file__).resolve().parents[1]/'reports/p1-env-ros-01'
out.mkdir(parents=True,exist_ok=False)
(out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
raise SystemExit(0 if all(v['status']=='IMPORT_OK' for v in result['modules'].values()) else 1)
