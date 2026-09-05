"""Sequential, bounded pick/transport/place mission with evidence-based checks."""
import json
import math
import os
from pathlib import Path
import time
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from .kinematics import inverse, forward, JOINT_NAMES, TRAY
from .vision import detect_box

ROOT = Path(__file__).resolve().parents[2]


class Mission(Node):
    def __init__(self):
        super().__init__('mm_mission', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.cfg = yaml.safe_load((ROOT/'config/mission.yaml').read_text())
        self.report_dir = Path(os.environ.get('MM_REPORT_DIR', ROOT/'reports'/time.strftime('%Y%m%d-%H%M%S')))
        self.report_dir.mkdir(parents=True, exist_ok=True)
        self.report = {'status':'RUNNING', 'events':[], 'checks':{}, 'vision':[],
                       'grasp_model':'proximity-gated fixed constraint; not friction grasp'}
        self.start_wall = time.monotonic(); self.cancelled = False; self.driving = False
        self.phase = 'STARTUP'; self.joints = {}; self.physics = {}; self.last_physics = 0.
        self.rgb = None; self.image_stamp = 0; self.info = None; self.image_wall = 0.; self.image_count = 0
        self.cv = CvBridge(); self.goal = None
        self.create_subscription(JointState, '/joint_states', self.joint_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/mm/physics', self.physics_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', lambda m: setattr(self, 'info', m), qos_profile_sensor_data)
        self.jpub = {n:self.create_publisher(Float64, '/mm/joint/'+n, 10) for n in JOINT_NAMES}
        self.grasp_pub = self.create_publisher(String, '/mm/grasp_command', 10)
        self.permit = self.create_publisher(String, '/mm/drive_permit', 10)
        self.status = self.create_publisher(String, '/mm/status', 10)
        self.create_service(Trigger, '/mission/cancel', self.cancel_cb)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf_buffer=Buffer(); self.tf_listener=TransformListener(self.tf_buffer,self)
        self.create_timer(.2, self.heartbeat, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))

    def joint_cb(self, m): self.joints = dict(zip(m.name, m.position))
    def physics_cb(self, m):
        self.physics = json.loads(m.data); self.last_physics = time.monotonic()
    def image_cb(self, m):
        self.rgb = self.cv.imgmsg_to_cv2(m, desired_encoding='rgb8')
        self.image_stamp = m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        self.image_count += 1
        self.image_wall = time.monotonic()
    def cancel_cb(self, req, res):
        self.cancelled = True; self.driving = False
        res.success = True; res.message = 'Cancellation requested; base stops, held cargo stays attached.'
        return res
    def heartbeat(self):
        fresh = time.monotonic()-self.last_physics < 1.
        allow = self.driving and fresh and self.physics.get('holder') != 'tool' and not self.cancelled
        self.permit.publish(String(data='drive' if allow else 'stop'))
        self.status.publish(String(data=json.dumps({'phase':self.phase,'status':self.report['status']})))
    def spin(self):
        rclpy.spin_once(self, timeout_sec=.02)
        if self.cancelled: raise RuntimeError('Mission cancelled by user')
        if time.monotonic()-self.start_wall > self.cfg['wall_timeout']: raise TimeoutError('Mission wall-clock deadline')
    def until(self, predicate, timeout, label):
        deadline = time.monotonic()+timeout
        while not predicate():
            self.spin()
            if time.monotonic() > deadline: raise TimeoutError(label)
    def settle(self, sim_seconds):
        start = self.get_clock().now().nanoseconds
        self.until(lambda:(self.get_clock().now().nanoseconds-start)/1e9 >= sim_seconds,
                   max(20.,sim_seconds*15), 'Simulation stopped advancing')
    def event(self, name):
        self.phase = name
        self.report['events'].append({'phase':name,'wall_elapsed':round(time.monotonic()-self.start_wall,2),
                                      'physics':dict(self.physics)})
        self.get_logger().info('PHASE: '+name); self.save()
    def save(self):
        (self.report_dir/'mission.json').write_text(json.dumps(self.report, indent=2))
    def check(self, name, okay):
        self.report['checks'][name] = bool(okay); self.save()
        if not okay: raise RuntimeError('Physical acceptance failed: '+name)
    def joints_to(self, targets):
        if self.driving: raise RuntimeError('Arm movement forbidden during driving')
        self.until(lambda:self.physics and self.physics['base_speed'] < .02, 15., 'Base not stopped')
        # Refresh absolute setpoints while waiting for measured feedback. A
        # one-shot command may be lost during initial DDS/bridge discovery.
        deadline=time.monotonic()+45.; last_publish=0.
        while not all(abs(self.joints.get(n,999)-v)<.012 for n,v in targets.items()):
            self.spin()
            if time.monotonic()-last_publish>.15:
                for n,v in targets.items(): self.jpub[n].publish(Float64(data=float(v)))
                last_publish=time.monotonic()
            if time.monotonic()>deadline: raise TimeoutError('Joint target timeout '+str(targets))
        for n,v in targets.items(): self.jpub[n].publish(Float64(data=float(v)))
        self.settle(.25)
    def arm(self, x,y,z): self.joints_to(dict(zip(JOINT_NAMES[:4],inverse(x,y,z))))
    def fingers(self, closed): self.joints_to({n:.052 if closed else .085 for n in JOINT_NAMES[4:]})
    def raise_arm(self):
        p = forward([self.joints[n] for n in JOINT_NAMES[:4]])
        self.arm(p[0],p[1],.85)
    def stow(self):
        self.raise_arm(); self.arm(TRAY[0],TRAY[1],.85)
    def command(self, command, holder):
        stamp = self.physics.get('sim_time',0)
        self.grasp_pub.publish(String(data=command))
        self.until(lambda:self.physics.get('sim_time',0)>stamp+.15 and
                   (self.physics.get('holder')==holder or bool(self.physics.get('rejection'))), 10., 'Grasp command timeout')
        if self.physics.get('holder') != holder:
            raise RuntimeError('Grasp rejected: '+self.physics.get('rejection','unknown'))
        self.settle(.3)
    def navigate(self, target, label):
        self.event(label); self.stow()
        self.until(self.nav.server_is_ready, 90., 'Nav2 action unavailable')
        request = NavigateToPose.Goal(); request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x=float(target[0]); request.pose.pose.position.y=float(target[1])
        request.pose.pose.orientation.z=math.sin(target[2]/2); request.pose.pose.orientation.w=math.cos(target[2]/2)
        self.driving=True
        try:
            f=self.nav.send_goal_async(request); self.until(f.done, 15., 'Nav2 goal response timeout')
            self.goal=f.result()
            if not self.goal.accepted: raise RuntimeError('Nav2 rejected goal')
            result=self.goal.get_result_async()
            self.until(result.done, float(self.cfg['navigation_timeout']), 'Navigation timeout: '+label)
            if result.result().status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError('Navigation failed: '+str(result.result()))
            self.goal=None
        finally:
            self.driving=False; self.heartbeat()
        self.settle(1.0)
    def observe(self, color, height, label):
        self.event(label); samples=[]; frame_stamps=[]; last=-1; deadline=time.monotonic()+self.cfg['vision_timeout']
        last_error='No fresh camera/calibration'
        while len(samples)<4:
            self.spin()
            if time.monotonic()>deadline:
                if self.rgb is not None: cv2.imwrite(str(self.report_dir/(label+'_failed.png')),cv2.cvtColor(self.rgb,cv2.COLOR_RGB2BGR))
                raise RuntimeError(f'Vision failed: {last_error}; samples={len(samples)}, images={self.image_count}, age={time.monotonic()-self.image_wall:.2f}s')
            if self.info is None or self.rgb is None or self.image_count==last or time.monotonic()-self.image_wall>1.: continue
            last=self.image_count
            try:
                x,y,pixels=detect_box(self.rgb,self.info.k[0],self.info.k[4],self.info.k[2],self.info.k[5],height,color)
                if color != 'magenta': inverse(x,y,height-.05 if color=='blue' else height+.05)
                samples.append((x,y,pixels['yaw']) if color=='magenta' else (x,y))
                frame_stamps.append(self.image_stamp)
            except ValueError as e:
                last_error=str(e); samples=[]; frame_stamps=[]
        if np.max(np.ptp(samples,axis=0))>.015: raise RuntimeError('Vision target unstable')
        sample=np.median(samples,axis=0); x,y=sample[:2]
        cv2.imwrite(str(self.report_dir/(label+'.png')), cv2.cvtColor(self.rgb,cv2.COLOR_RGB2BGR))
        self.report['vision'].append({'label':label,'base_xy':[float(x),float(y)],'pixels':pixels,'color':color,
                                     'frame_stamps_ns':frame_stamps,
                                     'method':'RGB marker + calibrated camera ray / known plane intersection'})
        self.save()
        return tuple(float(v) for v in sample)
    def align_home(self):
        # The asymmetric rectangular marker is at (0.70, 0) relative to the
        # desired base pose. Its principal axis gives relative heading, modulo
        # pi; Nav2's coarse return must already face within 90 degrees of home.
        for attempt in range(4):
            x,y,yaw=self.observe('magenta',.003,'LOCATE_HOME_MARKER_'+str(attempt))
            dx=x-.70*math.cos(yaw); dy=y-.70*math.sin(yaw)
            if math.hypot(dx,dy)<.04 and abs(yaw)<.045:
                self.check('home_visually_aligned',True); return
            if attempt==3: raise RuntimeError('Visual home alignment did not converge')
            self.until(lambda:self.tf_buffer.can_transform('map','base_link',rclpy.time.Time()),10.,'Home alignment TF unavailable')
            tf=self.tf_buffer.lookup_transform('map','base_link',rclpy.time.Time()).transform
            q=tf.rotation
            heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            target=[tf.translation.x+math.cos(heading)*dx-math.sin(heading)*dy,
                    tf.translation.y+math.sin(heading)*dx+math.cos(heading)*dy,heading+yaw]
            self.navigate(target,'ALIGN_HOME_'+str(attempt))
    def pick_at(self, x,y,z, label):
        self.event(label); self.fingers(False); self.arm(x,y,.85); self.arm(x,y,z)
        self.fingers(True); before=self.physics['cargo'][2]
        self.command('grip','tool'); self.raise_arm()
        self.check(label+'_lifted', self.physics['cargo'][2]>before+.09 and self.physics['holder']=='tool')
    def run(self):
        self.until(lambda:all(n in self.joints for n in JOINT_NAMES) and bool(self.physics) and self.info is not None,
                   90., 'Missing joint states, grasp plugin or camera calibration')
        self.settle(1.)
        self.event('GRASP_GUARD_SELF_TEST')
        self.grasp_pub.publish(String(data='grip')); self.settle(.5)
        self.check('distant_grasp_rejected',self.physics['holder']=='none' and self.physics['rejection']=='outside_capture_volume')
        self.fingers(False); self.stow()
        self.navigate(self.cfg['source']['navigation'],'NAVIGATE_SOURCE')
        xy=self.observe('blue',self.cfg['source']['object_top_z'],'LOCATE_SOURCE_BOX')
        self.pick_at(*xy,self.cfg['source']['object_top_z']-.05,'PICK_SOURCE')
        self.event('LOAD_TRAY'); self.arm(TRAY[0],TRAY[1],.85); self.arm(*TRAY)
        self.command('release','none'); self.fingers(False); self.settle(.5)
        cargo=self.physics['cargo']; tray=self.physics['tray']
        self.check('loaded_on_tray',math.dist(cargo[:3],[tray[0],tray[1],tray[2]+.07])<.05)
        self.command('secure','tray'); self.stow()
        self.navigate(self.cfg['destination']['navigation'],'TRANSPORT_TO_DESTINATION')
        self.check('cargo_arrived_on_tray',self.physics['holder']=='tray' and
                   math.dist(self.physics['cargo'][:2],self.physics['tray'][:2])<.05)
        place=self.observe('red',.652,'LOCATE_PLACE_MARKER')
        self.event('UNLOAD_TRAY'); self.arm(*TRAY); self.command('unsecure','none')
        self.pick_at(*TRAY,'PICK_FROM_TRAY')
        self.event('PLACE_ON_DESTINATION'); self.arm(*place,.85); self.arm(*place,.705)
        self.command('release','none'); self.fingers(False); self.raise_arm(); self.settle(1.)
        cargo=self.physics['cargo']; target=self.cfg['destination']['place_center']
        self.check('placed_at_destination',self.physics['holder']=='none' and math.dist(cargo[:2],target[:2])<.07 and abs(cargo[2]-target[2])<.025)
        self.navigate(self.cfg['home'],'RETURN_HOME')
        self.align_home()
        self.check('returned_home',math.dist(self.physics['base'][:2],self.cfg['home'][:2])<.15)
        self.report['final_physics']=dict(self.physics)
        self.report['status']='COMPLETED'; self.event('COMPLETED')


def main():
    rclpy.init(); node=Mission(); code=0
    try: node.run()
    except (Exception, KeyboardInterrupt) as exc:
        code=1; node.driving=False
        node.report['status']='CANCELLED' if node.cancelled or isinstance(exc,(KeyboardInterrupt,ExternalShutdownException)) else 'FAILED'
        node.report['error']=str(exc); node.get_logger().error(str(exc)); node.save()
        node.report['final_physics']=dict(node.physics); node.save()
        if node.goal is not None and rclpy.ok(): node.goal.cancel_goal_async()
    finally:
        node.driving=False
        node.report['wall_seconds']=round(time.monotonic()-node.start_wall,2); node.save()
        if rclpy.ok():
            node.heartbeat()
            for _ in range(10):
                if rclpy.ok(): rclpy.spin_once(node, timeout_sec=.02)
        print('REPORT: '+str(node.report_dir/'mission.json'), flush=True)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    raise SystemExit(code)


if __name__=='__main__': main()
