#!/usr/bin/env python3
"""Generate all original robot/workcell assets without external project paths."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
ORANGE = '0.9 0.38 0.08 1'
GREY = '0.35 0.38 0.42 1'
OBSTACLES = [
    ('west',-2.0,1.5,0.15,7.0,1.6), ('east',4.5,1.5,0.15,7.0,1.6),
    ('south',1.25,-2.0,6.65,0.15,1.6), ('north',1.25,5.0,6.65,0.15,1.6),
    ('source_table',2.2,0,0.4,0.9,0.65),
    ('destination_table',2.2,3,0.4,0.9,0.65),
    ('machine',0.25,1.6,0.6,0.6,0.85),
]


def inertial(mass):
    return f'<inertial><mass>{mass}</mass><inertia><ixx>{mass*.03}</ixx><iyy>{mass*.03}</iyy><izz>{mass*.03}</izz></inertia></inertial>'


def box(size, color, pose='0 0 0 0 0 0', collision=True):
    geom = f'<geometry><box><size>{size}</size></box></geometry>'
    visual = f'<visual name="visual"><pose>{pose}</pose>{geom}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>'
    return visual + (f'<collision name="collision"><pose>{pose}</pose>{geom}</collision>' if collision else '')


def link(name, parent, pose, body, mass=1.0):
    rel = f' relative_to="{parent}"' if parent else ''
    return f'<link name="{name}"><pose{rel}>{pose}</pose>{inertial(mass)}{body}</link>'


def joint(name, parent, child, kind, axis='0 0 1', lo=-3.14, hi=3.14, velocity=.6):
    ax = '' if kind == 'fixed' else f'<axis><xyz>{axis}</xyz><limit><lower>{lo}</lower><upper>{hi}</upper><effort>1000</effort><velocity>{velocity}</velocity></limit><dynamics><damping>0.1</damping></dynamics></axis>'
    return f'<joint name="{name}" type="{kind}"><pose relative_to="{child}">0 0 0 0 0 0</pose><parent>{parent}</parent><child>{child}</child>{ax}</joint>'


def controller(name):
    if name == 'lift':
        # Force-controlled vertical axis with nominal gravity compensation.
        # Unlike ideal velocity servos this explicitly lifts the arm's mass.
        return '<plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController"><joint_name>lift</joint_name><topic>/mm/joint/lift</topic><p_gain>500</p_gain><i_gain>30</i_gain><d_gain>70</d_gain><i_max>10</i_max><i_min>-10</i_min><cmd_offset>31.392</cmd_offset><cmd_max>100</cmd_max><cmd_min>-100</cmd_min></plugin>'
    return f'<plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController"><joint_name>{name}</joint_name><topic>/mm/joint/{name}</topic><p_gain>6</p_gain><use_velocity_commands>true</use_velocity_commands><cmd_max>0.5</cmd_max><cmd_min>-0.5</cmd_min></plugin>'


def robot():
    links = [link('base_link','', '0 0 0 0 0 0',box('0.70 0.58 0.24',ORANGE,'0 0 0.24 0 0 0'),45)]
    joints = []
    for side,y in [('left',.34),('right',-.34)]:
        geom = '<geometry><cylinder><radius>0.12</radius><length>0.055</length></cylinder></geometry>'
        body = f'<collision name="collision"><pose>0 0 0 1.57079632679 0 0</pose>{geom}<surface><friction><ode><mu>1.1</mu><mu2>1.1</mu2></ode></friction></surface></collision><visual name="visual"><pose>0 0 0 1.57079632679 0 0</pose>{geom}<material><ambient>0.08 0.08 0.08 1</ambient></material></visual>'
        links.append(link(side+'_wheel','base_link',f'0 {y} 0.12 0 0 0',body,1))
        joints.append(joint(side+'_wheel_joint','base_link',side+'_wheel','revolute','0 1 0',-1e12,1e12,velocity=8.0))
    for name,x in [('front',.28),('rear',-.28)]:
        body = '<collision name="collision"><geometry><sphere><radius>0.06</radius></sphere></geometry><surface><friction><ode><mu>0.01</mu><mu2>0.01</mu2></ode></friction></surface></collision><visual name="visual"><geometry><sphere><radius>0.06</radius></sphere></geometry></visual>'
        links.append(link(name+'_caster','base_link',f'{x} 0 0.06 0 0 0',body,.2))
        joints.append(joint(name+'_caster_joint','base_link',name+'_caster','fixed'))
    links += [link('tray','base_link','-0.24 0 0.41 0 0 0',box('0.28 0.32 0.04','0.25 0.65 0.25 1'),.5),
              link('column','base_link','0 0 0.48 0 0 0',box('0.12 0.12 0.30',GREY),2),
              link('carriage','base_link','0 0 0.65 0 0 0',box('0.16 0.16 0.10',GREY),1),
              link('upper_arm','carriage','0 0 0 0 0 0',box('0.42 0.065 0.065',ORANGE,'0.21 0 0 0 0 0'),1),
              link('forearm','upper_arm','0.42 0 0 0 0 0',box('0.40 0.055 0.055','0.8 0.8 0.8 1','0.2 0 0.065 0 0 0'),.7),
              link('tool','forearm','0.40 0 -0.30 0 0 0',box('0.075 0.15 0.06',GREY,'0 0 0.10 0 0 0')+'<visual name="shaft"><pose>0 0 0.23 0 0 0</pose><geometry><cylinder><radius>0.025</radius><length>0.27</length></cylinder></geometry></visual>',.4)]
    joints += [joint('tray_joint','base_link','tray','fixed'),joint('column_joint','base_link','column','fixed'),
               joint('lift','base_link','carriage','prismatic','0 0 1',0,.55),
               joint('shoulder','carriage','upper_arm','revolute'),joint('elbow','upper_arm','forearm','revolute'),
               joint('wrist','forearm','tool','revolute')]
    for name,sign in [('left',1),('right',-1)]:
        # Fingers are visual guides. Holding physics uses a proximity-gated fixed
        # constraint, explicitly documented as an assisted simulation grasp.
        links.append(link('finger_'+name+'_link','tool','0 0 0 0 0 0',box('0.08 0.012 0.11',GREY,f'0 {sign*.006} 0.015 0 0 0',False),.05))
        joints.append(joint('finger_'+name,'tool','finger_'+name+'_link','prismatic',f'0 {sign} 0',.050,.095))
    lidar = '<sensor name="lidar" type="gpu_lidar"><gz_frame_id>laser_link</gz_frame_id><topic>/mm/scan</topic><update_rate>10</update_rate><always_on>true</always_on><lidar><scan><horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.14159265359</min_angle><max_angle>3.14159265359</max_angle></horizontal></scan><range><min>0.12</min><max>12</max></range></lidar></sensor>'
    links.append(link('laser_link','base_link','0.14 0 0.60 0 0 0',lidar,.05))
    joints.append(joint('laser_joint','base_link','laser_link','fixed'))
    camera = '<sensor name="camera" type="camera"><pose>0 0 0 0 1.57079632679 0</pose><gz_frame_id>camera_link</gz_frame_id><topic>/mm/camera</topic><update_rate>8</update_rate><always_on>true</always_on><camera><horizontal_fov>1.4</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image><clip><near>0.05</near><far>8</far></clip></camera></sensor>'
    links.append(link('camera_link','base_link','0.3 0 1.8 0 0 0',camera+box('.12 .08 .06',GREY),.1))
    joints.append(joint('camera_joint','base_link','camera_link','fixed'))
    links.append(link('camera_mast','base_link','-0.34 -0.22 1.03 0 0 0',box('.04 .04 1.45',GREY),.3))
    joints.append(joint('mast_joint','base_link','camera_mast','fixed'))
    plugins = '<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive"><left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint><wheel_separation>0.68</wheel_separation><wheel_radius>0.12</wheel_radius><topic>/mm/cmd_vel</topic><odom_topic>/mm/odom</odom_topic><frame_id>odom</frame_id><child_frame_id>base_link</child_frame_id><odom_publish_frequency>30</odom_publish_frequency><max_linear_acceleration>0.4</max_linear_acceleration><max_angular_acceleration>0.7</max_angular_acceleration></plugin>'
    plugins += '<plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"><topic>/mm/joint_states</topic></plugin>'
    plugins += ''.join(controller(n) for n in ('shoulder','elbow','lift','wrist','finger_left','finger_right'))
    plugins += '<plugin filename="libguarded_grasp.so" name="mm003::GuardedGrasp"/>'
    return '<model name="mobile_manipulator"><self_collide>false</self_collide>'+''.join(links+joints)+plugins+'</model>'


def generate():
    for folder in ['worlds','maps','description','config']:
        (ROOT/folder).mkdir(exist_ok=True)
    objects = ''
    for n,x,y,sx,sy,h in OBSTACLES:
        color = '0.32 0.48 0.35 1' if n=='source_table' else '0.42 0.38 0.32 1'
        objects += f'<model name="{n}"><static>true</static><pose>{x} {y} {h/2} 0 0 0</pose><link name="body">{box(f"{sx} {sy} {h}",color)}</link></model>'
    objects += '<model name="placement_marker"><static>true</static><pose>2.2 3 .651 0 0 0</pose><link name="body">'+box('.16 .16 .002','0.95 0.04 0.04 1',collision=False)+'</link></model>'
    objects += '<model name="home_marker"><static>true</static><pose>.70 0 .002 0 0 0</pose><link name="body">'+box('.25 .12 .002','0.95 0.04 0.95 1',collision=False)+'</link></model>'
    cargo = '<model name="material_box"><pose>2.2 0 0.70 0 0 0</pose><link name="body">'+inertial(.15)+box('.10 .10 .10','0.04 0.18 0.95 1')+'</link></model>'
    ground = '<model name="floor"><static>true</static><link name="body"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>14 14</size></plane></geometry></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>14 14</size></plane></geometry><material><ambient>0.72 0.72 0.72 1</ambient><diffuse>0.72 0.72 0.72 1</diffuse></material></visual></link></model>'
    plugins = ''.join(f'<plugin filename="gz-sim-{lib}-system" name="gz::sim::systems::{name}"/>' for lib,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')])
    sdf = '<?xml version="1.0"?><sdf version="1.9"><world name="material_lab"><physics name="physics" type="ode"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>'+plugins+'<light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.3 0.1 -1</direction><cast_shadows>false</cast_shadows></light>'+ground+objects+cargo+robot()+'</world></sdf>'
    (ROOT/'worlds/workcell.sdf').write_text(sdf,encoding='utf-8')
    # URDF for RViz: copy the original geometry and kinematic tree from SDF.
    model = ET.fromstring(sdf).find('world/model[@name="mobile_manipulator"]')
    urdf = ET.Element('robot',name='mobile_manipulator')
    for sl in model.findall('link'):
        ul = ET.SubElement(urdf,'link',name=sl.attrib['name'])
        for vis in sl.findall('visual'):
            uv=ET.SubElement(ul,'visual'); pv=vis.findtext('pose','0 0 0 0 0 0').split()
            ET.SubElement(uv,'origin',xyz=' '.join(pv[:3]),rpy=' '.join(pv[3:]))
            geom=ET.SubElement(uv,'geometry'); sg=vis.find('geometry')[0]
            attrs = {'size':sg.findtext('size')} if sg.tag=='box' else ({'radius':sg.findtext('radius'),'length':sg.findtext('length')} if sg.tag=='cylinder' else {'radius':sg.findtext('radius')})
            ET.SubElement(geom,sg.tag,attrs)
            color=vis.findtext('material/ambient','0.6 0.6 0.6 1')
            mat=ET.SubElement(uv,'material',name=sl.attrib['name']+vis.attrib['name']); ET.SubElement(mat,'color',rgba=color)
    for sj in model.findall('joint'):
        kind=sj.attrib['type']; child=sj.findtext('child'); parent=sj.findtext('parent')
        uj=ET.SubElement(urdf,'joint',name=sj.attrib['name'],type=kind)
        ET.SubElement(uj,'parent',link=parent); ET.SubElement(uj,'child',link=child)
        pos=model.find(f'link[@name="{child}"]/pose').text.split()
        ET.SubElement(uj,'origin',xyz=' '.join(pos[:3]),rpy=' '.join(pos[3:]))
        if kind!='fixed':
            ET.SubElement(uj,'axis',xyz=sj.findtext('axis/xyz'))
            ET.SubElement(uj,'limit',lower=sj.findtext('axis/limit/lower'),upper=sj.findtext('axis/limit/upper'),effort='1000',velocity=sj.findtext('axis/limit/velocity'))
    ET.indent(urdf)
    ET.ElementTree(urdf).write(ROOT/'description/robot.urdf',encoding='unicode',xml_declaration=True)
    # Conservative static map includes whole workbenches, not movable cargo.
    res=.05; ox,oy=-2.5,-2.5; width,height=150,160
    pixels=bytearray([254])*(width*height)
    for _,x,y,sx,sy,_ in OBSTACLES:
        for row in range(height):
            wy=oy+(height-row-.5)*res
            for col in range(width):
                wx=ox+(col+.5)*res
                if abs(wx-x)<=sx/2 and abs(wy-y)<=sy/2: pixels[row*width+col]=0
    (ROOT/'maps/lab.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+pixels)
    (ROOT/'maps/lab.yaml').write_text(yaml.safe_dump(dict(image='lab.pgm',resolution=res,origin=[ox,oy,0.0],negate=0,occupied_thresh=.65,free_thresh=.196)))
    bridge=[]
    for ros,gz,rt,gt,d in [('/clock','/clock','rosgraph_msgs/msg/Clock','Clock','GZ_TO_ROS'),('/odom','/mm/odom','nav_msgs/msg/Odometry','Odometry','GZ_TO_ROS'),('/scan','/mm/scan','sensor_msgs/msg/LaserScan','LaserScan','GZ_TO_ROS'),('/cmd_vel','/mm/cmd_vel','geometry_msgs/msg/Twist','Twist','ROS_TO_GZ'),('/joint_states','/mm/joint_states','sensor_msgs/msg/JointState','Model','GZ_TO_ROS'),('/camera/image_raw','/mm/camera','sensor_msgs/msg/Image','Image','GZ_TO_ROS'),('/camera/camera_info','/mm/camera/camera_info','sensor_msgs/msg/CameraInfo','CameraInfo','GZ_TO_ROS'),('/mm/physics','/mm/physics','std_msgs/msg/String','StringMsg','GZ_TO_ROS'),('/mm/grasp_command','/mm/grasp_command','std_msgs/msg/String','StringMsg','ROS_TO_GZ')]:
        if ros == '/camera/camera_info': gz = '/mm/camera_info'
        if ros == '/scan': ros = '/scan_raw'
        bridge.append(dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=rt,gz_type_name='gz.msgs.'+gt,direction=d))
    for n in ['shoulder','elbow','lift','wrist','finger_left','finger_right']:
        bridge.append(dict(ros_topic_name='/mm/joint/'+n,gz_topic_name='/mm/joint/'+n,ros_type_name='std_msgs/msg/Float64',gz_type_name='gz.msgs.Double',direction='ROS_TO_GZ'))
    (ROOT/'config/bridge.yaml').write_text(yaml.safe_dump(bridge,sort_keys=False))
    print('Generated independent workcell, robot URDF, map and bridge configuration.')


if __name__=='__main__': generate()
