#!/usr/bin/env python3
"""ROS actuator compatibility and CPU torpedo/dropper simulation.

Consumes the plant's integrated time. Flight coefficients and crate contact are
approximations; geometry/provenance are in config/talos_tasks.yaml.
"""
import json
import math
import time
import itertools
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32, Float64, Float64MultiArray, String
from std_srvs.srv import Trigger, SetBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from riptide_msgs2.msg import ActuatorStatus, KillSwitchReport
from payload_model import advance, advance_rotating, crossing, torpedo_contact, launch_speed, support_extent, payload_mounts
from ament_index_python.packages import get_package_share_directory
from claw_world import ClawWorld, quaternion
from magnet_light_model import MagnetLights
from run_score import RunScore, CourseJudge


def rotation(rpy):
    r,p,y=rpy;cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def quat_rotation(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def frame(p,r):
    t=np.eye(4);t[:3,:3]=r;t[:3,3]=p;return t


class TaskSimulator(Node):
    def __init__(self):
        super().__init__('task_simulator')
        def config(name):return yaml.safe_load(Path(self.declare_parameter(name,'').value).read_text())
        self.robot=self.declare_parameter('robot','talos').value
        self.cfg=config('task_config');vehicle=config('vehicle_config');self.hydro=config('hydrodynamics_config')
        mapping=config('mapping_config');data=mapping['/'+self.robot+'/riptide_mapping2']['ros__parameters']['init_data']
        self.base=np.asarray(vehicle['base_link'],float)
        self.mounts={kind:payload_mounts(vehicle,self.cfg,kind) for kind in ('torpedo','dropper')}
        self.frames={};visiting=set()
        def resolve(key):
            if key in ('map','world'):return np.eye(4)
            if key.endswith('_frame'):key=key[:-6]
            if key in self.frames:return self.frames[key]
            if key in visiting:raise ValueError('Cycle in task mapping')
            visiting.add(key);a=data[key];p=a['pose']
            t=resolve(a['parent'])@frame([p.get(c,0) for c in ('x','y','z')],rotation([0,0,math.radians(p.get('yaw',0))]))
            self.frames[key]=t;visiting.remove(key);return t
        resolve('torpedo')
        self.bins={key:(resolve(key),data[key].get('class','')) for key in ('bin_vinyl1','bin_vinyl2','bin_vinyl3','bin_vinyl4')}
        self.magnet_lights=None
        if 'magnet_lights' in self.cfg and 'magnet' in vehicle:
            for key in self.cfg['magnet_lights']['targets']:resolve(key)
            self.magnet_lights=MagnetLights(self.cfg['magnet_lights'],self.frames,vehicle)
        origin=mapping['/**/zed_faker']['ros__parameters'].get('map_origin_pool',[0,0,0])
        self.map_to_pool=frame([origin[0],origin[1],0],rotation([0,0,math.radians(origin[2])]))
        self.claw=None
        if 'claw' in self.cfg and 'claw' in vehicle:
            for key in ('table','helmet','warning',*self.cfg['claw']['props']):resolve(key)
            pose=self.cfg['claw'].get('pose',vehicle['claw']['pose'])
            mount=frame(np.asarray(pose[:3])-self.base,rotation(pose[3:]))
            folder=Path(get_package_share_directory('c_simulator'))/'collision_files/tasks'
            self.claw=ClawWorld(self.cfg['claw'],self.frames,mount,folder,self.map_to_pool,self.cfg.get('table_collision'))
            self.tool_tf=StaticTransformBroadcaster(self)
            t=TransformStamped();t.header.stamp=self.get_clock().now().to_msg()
            t.header.frame_id=self.robot+'/base_link';t.child_frame_id='simulator/'+self.robot+'/claw_tool'
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,mount[:3,3])
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,quaternion(mount[:3,:3]))
            self.tool_tf.sendTransform(t)
        for kind in ('torpedo','dropper'):
            c=self.cfg[kind]
            for k in ('mass','radius','added_mass','displaced_volume','drag_axial','drag_lateral','spring_energy','cooldown','length'):
                if not math.isfinite(c[k]) or c[k]<0:raise ValueError('Invalid payload '+k)
            if c['mass']<=0 or c['displaced_volume']<=0 or c['radius']<=0 or c['length']<2*c['radius'] or not 0<c['count']<=255:raise ValueError('Invalid payload size/count')
        self.truth=None;self.truth_wall=0.;self.time=0.;self.last_time=None
        self.run=RunScore();self.judge=None;self.run_message='Ready to start a run'
        if 'scoring' in self.cfg and 'octagon' in self.cfg:
            for key in ('gate','gate_repair','slalom_front','slalom_middle','slalom_back',
                        'table','octagon','compass','hammer_and_wrench','buoy','sos'):resolve(key)
            # Reuse the plant's collision envelope, translated COM -> base_link.
            collision=Path(get_package_share_directory('c_simulator'))/'collision_files/robots'/('puddles_placeholder.urdf' if self.robot=='puddles' else self.robot+'.urdf')
            vertices=[]
            for element in ET.parse(collision).findall('.//collision'):
                box=element.find('geometry/box')
                if box is None:continue
                size=np.fromstring(box.get('size'),sep=' ')/2
                origin=element.find('origin')
                xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
                rpy=np.fromstring(origin.get('rpy','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
                vertices.extend(rotation(rpy)@(size*np.array(sign))+xyz+np.asarray(vehicle['com'])-self.base
                                for sign in itertools.product((-1,1),repeat=3))
            if self.magnet_lights:vertices.append(self.magnet_lights.tip)
            geometry=dict(self.cfg['scoring'],surface_z=self.cfg['octagon']['surface_z'],
                          octagon_apothem=self.cfg['octagon']['apothem']-self.cfg['octagon']['pipe_radius'])
            self.judge=CourseJudge(self.run,geometry,self.frames,vertices)
        self.armed=False;self.killed=True;self.busy_until=0.;self.busy_kind=''
        self.available={k:self.cfg[k]['count'] for k in ('torpedo','dropper')}
        self.payloads=[];self.next_id=0;self.score={'success':0,'wrong_target':0,'blocked':0,'miss':0}
        self.markers=self.create_publisher(MarkerArray,'simulator/projectiles',10)
        self.events=self.create_publisher(String,'simulator/task_events',10)
        self.scores=self.create_publisher(String,'simulator/task_score',10)
        self.run_pub=self.create_publisher(String,'simulator/run_score',10)
        self.create_subscription(String,'simulator/run_command',self.run_command,10)
        self.status_pub=self.create_publisher(ActuatorStatus,'state/actuator/status',10)
        self.busy_pub=self.create_publisher(Bool,'state/actuator/busy',10)
        self.feedback=self.create_publisher(Bool,'state/actuator/cmd_status',10)
        self.object_pub=self.create_publisher(MarkerArray,'simulator/task_objects',10)
        self.claw_pub=self.create_publisher(Float64MultiArray,'simulator/claw_joints',10)
        self.light_pub=self.create_publisher(MarkerArray,'simulator/magnet_lights',10)
        self.create_service(Trigger,'simulator/reset_tasks',self.reset_tasks_service)
        self.create_subscription(Empty,'simulator/reset_tasks',lambda _:self.reply(self.reset_tasks()),10)
        self.create_service(Trigger,'simulator/reset_magnet_lights',self.reset_magnet_lights)
        self.create_subscription(Bool,'command/actuator/claw',lambda m:self.reply(self.claw_command(m.data)),10)
        self.create_service(SetBool,'command/actuator/claw',self.claw_service)
        self.create_subscription(Float32,'command/actuator/claw_move_s',self.claw_timed,10)
        self.create_service(Trigger,'simulator/reset_table',self.reset_table)
        self.create_subscription(Odometry,'simulator/ground_truth',self.pose,10)
        self.create_subscription(Float64,'simulator/time',self.clock,10)
        self.create_subscription(KillSwitchReport,'command/software_kill',self.kill,10)
        self.create_subscription(Bool,'command/actuator/arm',lambda m:self.reply(self.arm(m.data)),10)
        self.create_service(SetBool,'command/actuator/arm',self.arm_service)
        self.create_subscription(Empty,'command/actuator/notify_reload',lambda _:self.reply(self.reload()),10)
        self.create_service(Trigger,'command/actuator/notify_reload',self.reload_service)
        for kind in ('torpedo','dropper'):
            self.create_subscription(Empty,'command/actuator/'+kind,lambda _,k=kind:self.reply(self.fire(k)),10)
            self.create_service(Trigger,'command/actuator/'+kind,lambda req,res,k=kind:self.fire_service(req,res,k))
            self.create_subscription(Empty,'command/simple_'+kind+'_fire',lambda _,k=kind:self.simple_fire(k),10)
        self.create_timer(.02,self.publish)
        self.get_logger().info('Payload flight and rigid claw/table contacts enabled; physical parameters are unvalidated priors')

    def pose(self,msg):self.truth=msg;self.truth_wall=time.monotonic()
    def claw_service(self,req,res):res.success,res.message=self.claw_command(req.data);return res
    def claw_timed(self,msg):
        if not math.isfinite(msg.data):self.reply((False,'Claw duration must be finite'));return
        self.reply(self.claw_command(msg.data>0,abs(msg.data)))
    def claw_command(self,opened,duration=None):
        if self.claw is None:return False,'No claw configured'
        if self.killed or not self.armed:return False,'Actuators are disarmed or vehicle is killed'
        if self.truth is None or time.monotonic()-self.truth_wall>1:return False,'No fresh vehicle ground truth'
        if duration==0:self.claw.stop()
        else:self.claw.command(opened,duration)
        return True,'Claw opening' if opened else 'Claw closing'
    def reset_table(self,req,res):
        if self.run.running:res.success=False;res.message='Stop/reset the scored run before resetting table props';return res
        if self.claw:self.claw.reset()
        res.success=self.claw is not None;res.message='Table props reset';return res
    def reset_tasks_service(self,req,res):res.success,res.message=self.reset_tasks();return res
    def reset_tasks(self):
        self.payloads.clear();self.reload();self.busy_kind=''
        self.score={k:0 for k in self.score}
        if self.claw:self.claw.reset()
        if self.magnet_lights:self.magnet_lights.reset()
        self.run.reset()
        self.update_role()
        if self.judge:self.judge.reset()
        self.run_message='Tasks and run reset'
        msg=String();msg.data=json.dumps(dict(kind='tasks',result='reset',target='',time=self.time))
        self.events.publish(msg)
        # Publish the cleared scene immediately, including while physics is paused.
        self.publish()
        return True,'All tasks reset; ammunition reloaded and actuators disarmed'
    def reset_magnet_lights(self,req,res):
        if self.run.running:res.success=False;res.message='Stop/reset the scored run before resetting lights';return res
        if self.magnet_lights:self.magnet_lights.reset()
        res.success=self.magnet_lights is not None;res.message='Magnet lights reset';return res
    def run_command(self,msg):
        try:
            command=json.loads(msg.data)
            action=command['action']
            if action=='start':
                if self.judge is None:raise ValueError('Course scoring is not configured')
                if self.run.running:raise ValueError('Stop the current run first')
                role=command.get('role','repair')
                if role not in ('repair','rescue'):raise ValueError('Select the known role: repair or rescue')
                self.reset_tasks()
                self.run.start(self.time,role,command.get('heading_coin',False),command.get('role_coin',False))
                self.run_message='Run started; pass the gate first'
            elif action=='stop':
                if self.judge:self.judge.finish_gate_attempt()
                self.run.stop(self.time);self.run_message='Run stopped'
            elif action=='adjustment':
                value=float(command['points'])
                if not math.isfinite(value):raise ValueError('Adjustment must be finite')
                self.run.adjustment=value;self.run_message='Manual adjustment updated'
            elif action=='pinger_select':
                self.run.select_pinger(command['task'],command.get('random',False))
                self.run_message='Pinger scoring selection recorded (no acoustic simulation)'
            elif action=='pinger_switch':
                self.run.switch_pinger();self.run_message='Pinger scoring target switched'
            else:raise ValueError('Unknown run command')
            self.update_role()
            self.publish()
        except (ValueError,KeyError,TypeError) as error:
            self.run_message='Command rejected: '+str(error)
            self.get_logger().warning(self.run_message)
    def update_role(self):
        for kind in ('torpedo','dropper'):self.cfg[kind]['target_class']=self.run.target_class
    def kill(self,msg):
        if msg.kill_switch_id==1:
            self.killed=msg.switch_asserting_kill
            if self.killed:self.armed=False
    def arm(self,value):
        if value and self.killed:return False,'Vehicle is killed; enable it first'
        self.armed=value;return True,'Armed' if value else 'Disarmed'
    def reload(self):
        self.armed=False;self.busy_until=0.
        self.available={k:self.cfg[k]['count'] for k in self.available}
        return True,'Reloaded and disarmed'
    def reply(self,result):
        msg=Bool();msg.data=result[0];self.feedback.publish(msg)
        if not result[0]:self.get_logger().warning(result[1])
    def arm_service(self,req,res):res.success,res.message=self.arm(req.data);return res
    def reload_service(self,req,res):res.success,res.message=self.reload();return res
    def fire_service(self,req,res,kind):res.success,res.message=self.fire(kind);return res
    def simple_fire(self,kind):
        result=self.arm(True)
        self.reply(self.fire(kind) if result[0] else result)
    def slot_state(self,kind,index):
        c=self.cfg[kind];t=self.truth.pose.pose;r=quat_rotation(t.orientation)
        slot=self.mounts[kind][index];mount=slot[:3,3]
        orientation=r@slot[:3,:3];axis=orientation[:,0]
        position=np.array([t.position.x,t.position.y,t.position.z])+r@mount
        return position,axis,mount,orientation
    def fire(self,kind):
        if not self.armed or self.killed:return False,'Actuators are disarmed or vehicle is killed'
        if self.time<self.busy_until:return False,'Actuator is busy'
        if self.available[kind]<=0:return False,'No '+kind+' ammunition; reload first'
        if self.truth is None or time.monotonic()-self.truth_wall>1:return False,'No fresh vehicle ground truth'
        c=self.cfg[kind];t=self.truth.pose.pose;r=quat_rotation(t.orientation)
        index=c['count']-self.available[kind]
        position,axis,mount,orientation=self.slot_state(kind,index)
        twist=self.truth.twist.twist
        v=np.array([twist.linear.x,twist.linear.y,twist.linear.z]);w=np.array([twist.angular.x,twist.angular.y,twist.angular.z])
        velocity=r@(v+np.cross(w,mount))+axis*launch_speed(c,self.hydro['water_density'])
        angular_velocity=r@w
        if 'center_of_mass' in c:
            velocity+=np.cross(angular_velocity,axis*c['center_of_mass'])
        self.payloads.append(dict(id=self.next_id,kind=kind,position=position,velocity=velocity,axis=axis,orientation=orientation,slot=index,
                                  angular_velocity=angular_velocity,age=0.,active=True,scored=False,entered=set()))
        self.payloads=self.payloads[-64:];self.next_id+=1;self.available[kind]-=1
        self.busy_until=self.time+c['cooldown'];self.busy_kind=kind
        board=self.frames['torpedo'];tip=position+axis*c['length']/2
        self.run.release_payload(kind,self.next_id-1,abs(float((board[:3,:3].T@(tip-board[:3,3]))[0])))
        self.event(self.payloads[-1],'released','',False)
        return True,kind+' released'

    def event(self,p,result,target,score=True):
        if score:
            if p['scored']:return
            p['scored']=True;self.score[result]+=1
            cls=self.bins[target][1] if target in self.bins else ''
            size=''
            if p['kind']=='torpedo':
                hole=next((h for h in self.cfg['torpedo']['holes'] if h['name']==target),None)
                if hole:cls=hole['class'];size=hole['name'].rsplit('_',1)[-1]
            self.run.payload_result(p['kind'],p['id'],result,target,cls,size)
        msg=String();msg.data=json.dumps(dict(id=p['id'],kind=p['kind'],result=result,target=target,time=self.time,slot=p['slot']))
        self.events.publish(msg)
    def clock(self,msg):
        current=msg.data
        if not math.isfinite(current):return
        if self.last_time is not None and current<self.last_time:
            self.reset_tasks()
        dt=max(0,current-(self.last_time if self.last_time is not None else current))
        course_dt=dt
        self.time=current;self.last_time=current
        while dt>1e-9:
            step=min(dt,.002);self.step(step,current-dt);dt-=step
        if self.judge and self.truth is not None and time.monotonic()-self.truth_wall<=1:
            p=self.truth.pose.pose
            body=frame([p.position.x,p.position.y,p.position.z],quat_rotation(p.orientation))
            if self.claw and self.run.eligible:self.run.basket_contents=self.claw.basket_contents()
            self.judge.update(body,course_dt,self.claw.held if self.claw else None)
            self.update_role()
        elif self.judge:
            self.judge.previous=None
            self.judge.surface_dwell=self.judge.facing_dwell=0.
    def step(self,dt,t):
        hydro=self.hydro
        if self.magnet_lights:
            body=None
            if self.truth is not None and time.monotonic()-self.truth_wall<=1:
                p=self.truth.pose.pose
                body=frame([p.position.x,p.position.y,p.position.z],quat_rotation(p.orientation))
            for key in self.magnet_lights.step(dt,body):
                self.run.light(key)
                msg=String();msg.data=json.dumps(dict(kind='magnet',result='activated',target=key,time=t+dt))
                self.events.publish(msg)
        water=np.asarray(hydro['current_velocity'])+np.asarray(hydro['current_oscillation_amplitude'])*math.sin(2*math.pi*hydro['current_oscillation_frequency']*t)
        if self.claw and self.truth is not None:
            pose=self.truth.pose.pose; r=quat_rotation(pose.orientation)
            body=frame([pose.position.x,pose.position.y,pose.position.z],r)
            twist=self.truth.twist.twist
            v=r@np.array([twist.linear.x,twist.linear.y,twist.linear.z])
            w=r@np.array([twist.angular.x,twist.angular.y,twist.angular.z])
            self.claw.step(dt,body,v,w,water,hydro['water_density'],self.armed and not self.killed and time.monotonic()-self.truth_wall<1)
            for key,result,target in self.claw.events:
                if result in self.score:self.score[result]+=1
                prop=self.cfg['claw']['props'][key]
                role='repair' if prop['basket']=='warning' else 'rescue'
                self.run.object_event(key,result,target,prop['basket'],role)
                msg=String();msg.data=json.dumps(dict(id=key,kind='claw',result=result,target=target,time=self.time))
                self.events.publish(msg)
            self.claw.events.clear()
        crate=self.cfg['crate'];height=crate['outer_height']-crate['base_thickness']
        inner=crate['inner_width']/2-crate['liner_thickness'];outer=crate['outer_width']/2
        for p in self.payloads:
            if not p['active']:continue
            c=self.cfg[p['kind']];old=p['position'].copy();p['age']+=dt
            if 'center_of_mass' in c:
                new,velocity,p['orientation'],p['angular_velocity']=advance_rotating(
                    old,p['velocity'],p['orientation'],p['angular_velocity'],c,water,hydro['water_density'],dt)
                p['axis']=p['orientation'][:,0]
            else:
                new,velocity=advance(old,p['velocity'],p['axis'],c,water,hydro['water_density'],dt)
            if p['kind']=='torpedo':
                target=self.frames['torpedo'];r=target[:3,:3];origin=target[:3,3]
                hit=torpedo_contact(r.T@(old-origin),r.T@(new-origin),r.T@p['axis'],c)
                if hit:
                    result,name,cls,point=hit
                    if result=='blocked':
                        new=r@point+origin;velocity[:]=0;p['active']=False;self.event(p,'blocked',name)
                    else:self.event(p,'success' if cls==c['target_class'] else 'wrong_target',name)
            for key,(target,cls) in self.bins.items():
                r=target[:3,:3];origin=target[:3,3];a=r.T@(old-origin);b=r.T@(new-origin)
                extent=support_extent(r.T@p['axis'],c)
                # Top entry must clear the inner walls with the entire marker.
                top=crossing(a,b,2,height+extent[2])
                if top is not None and b[2]<a[2] and np.all(np.abs(top[:2])+extent[:2]<=inner):
                    p['entered'].add(key)
                # Walls are conservative solid contact proxies for the lattice.
                for axis in (0,1):
                    for sign in (-1,1):
                        surface=sign*(inner-extent[axis] if key in p['entered'] else outer+extent[axis])
                        hit=crossing(a,b,axis,surface)
                        if hit is not None and -extent[2]<hit[2]<height+extent[2] and abs(hit[1-axis])<outer+extent[1-axis]:
                            b=hit;velocity*=.15
                            normal=r[:,axis];velocity-=normal*np.dot(velocity,normal)
                            new=r@b+origin
                floor=crossing(a,b,2,extent[2])
                if floor is not None and b[2]<a[2] and max(abs(floor[0]),abs(floor[1]))<=outer:
                    new=r@floor+origin;velocity[:]=0;p['active']=False
                    landed_inside=np.all(np.abs(floor[:2])+extent[:2]<=inner)
                    if p['kind']=='dropper' and key in p['entered'] and landed_inside:
                        self.event(p,'success' if cls==c['target_class'] else 'wrong_target',key)
                    else:self.event(p,'blocked',key)
                elif top is not None and b[2]<a[2] and not np.all(np.abs(top[:2])+extent[:2]<=inner) and np.all(np.abs(top[:2])<=outer+extent[:2]):
                    new=r@top+origin;velocity[:]=0;p['active']=False;self.event(p,'blocked',key+'_rim')
            local=self.map_to_pool@np.r_[new,1.]
            vertical=support_extent(p['axis'],c)[2]
            if new[2]<-2.1336+vertical:
                new[2]=-2.1336+vertical;velocity[:]=0;p['active']=False;self.event(p,'miss','pool_floor')
            if local[0]<0 or local[0]>50 or local[1]<0 or local[1]>22.86 or p['age']>30:
                velocity[:]=0;p['active']=False;self.event(p,'miss','pool_wall_or_timeout')
            p['position']=new;p['velocity']=velocity
    def publish(self):
        run=String();snapshot=self.run.snapshot(self.time);snapshot['message']=self.run_message
        run.data=json.dumps(snapshot);self.run_pub.publish(run)
        if self.magnet_lights:
            lights=MarkerArray()
            for index,(key,state) in enumerate(self.magnet_lights.states.items()):
                m=Marker();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
                m.ns=key;m.id=index;m.type=Marker.SPHERE;m.action=Marker.ADD
                t=self.magnet_lights.faces[key]
                m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,t[:3,3])
                m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=map(float,quaternion(t[:3,:3]))
                m.scale.x=.002;m.scale.y=m.scale.z=.044
                m.color.r=float(state=='red');m.color.g=float(state=='green');m.color.a=1.
                lights.markers.append(m)
            self.light_pub.publish(lights)
        status=ActuatorStatus();status.actuators_armed=self.armed
        status.claw_state=ActuatorStatus.CLAW_UNKNOWN;status.rack_state=ActuatorStatus.RACK_UNKNOWN
        claw_busy=False
        if self.claw:
            joints=self.claw.joints();error=abs(sum(joints)/2-self.claw.target)
            claw_busy=self.armed and error>.002 and self.claw.held is None
            status.claw_state=(ActuatorStatus.CLAW_DISARMED if not self.armed else
                ActuatorStatus.CLAW_OPENING if claw_busy and self.claw.direction>0 else
                ActuatorStatus.CLAW_CLOSING if claw_busy else
                ActuatorStatus.CLAW_OPENED if sum(joints)/2>self.claw.travel-.002 else ActuatorStatus.CLAW_BOTTLE_CLOSED)
            joint_msg=Float64MultiArray();joint_msg.data=[float(j) for j in joints];self.claw_pub.publish(joint_msg)
            objects=MarkerArray()
            for index,key in enumerate(self.claw.props):
                t=self.claw.prop_pose(key);m=Marker();m.header.frame_id='map'
                if key==self.claw.held:
                    t=np.linalg.inv(self.claw.body)@t;m.header.frame_id=self.robot+'/base_link'
                m.header.stamp=self.get_clock().now().to_msg();m.ns=key;m.id=index;m.type=Marker.MESH_RESOURCE;m.action=Marker.ADD
                m.mesh_resource='package://riptide_meshes/meshes/'+self.cfg['claw']['props'][key]['mesh']+'/model.dae'
                m.mesh_use_embedded_materials=True;m.scale.x=m.scale.y=m.scale.z=1.;m.color.a=1.
                m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,t[:3,3])
                m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=map(float,quaternion(t[:3,:3]))
                objects.markers.append(m)
            self.object_pub.publish(objects)
        status.torpedo_available_count=self.available['torpedo'];status.dropper_available_count=self.available['dropper']
        busy=self.time<self.busy_until
        status.torpedo_state=(ActuatorStatus.TORPEDO_DISARMED if not self.armed else ActuatorStatus.TORPEDO_FIRING if busy and self.busy_kind=='torpedo' else ActuatorStatus.TORPEDO_CHARGED if self.available['torpedo'] else ActuatorStatus.TORPEDO_FIRED)
        status.dropper_state=(ActuatorStatus.DROPPER_DISARMED if not self.armed else ActuatorStatus.DROPPER_DROPPING if busy and self.busy_kind=='dropper' else ActuatorStatus.DROPPER_READY if self.available['dropper'] else ActuatorStatus.DROPPER_DROPPED)
        # Contact calculations can produce numpy.bool_; ROS requires a Python bool.
        self.status_pub.publish(status);b=Bool();b.data=bool(busy or claw_busy);self.busy_pub.publish(b)
        score=String();score.data=json.dumps(self.score);self.scores.publish(score)
        array=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;array.markers.append(clear)
        for p in self.payloads:
            array.markers.append(self.marker(p['kind'],p['id'],p['position'],p['orientation'],False))
        if self.truth is not None:
            for kind,c in ((k,self.cfg[k]) for k in ('torpedo','dropper')):
                for index in range(c['count']-self.available[kind],c['count']):
                    position,axis,mount,orientation=self.slot_state(kind,index)
                    array.markers.append(self.marker(kind,index,position,orientation,True))
        self.markers.publish(array)

    def marker(self,kind,index,position,orientation,loaded):
        c=self.cfg[kind];m=Marker();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.ns=kind+('_loaded' if loaded else '');m.id=index;m.type=Marker.MESH_RESOURCE;m.action=Marker.ADD
        m.mesh_resource='package://camera_faker/models/payloads/projectile.obj'
        m.mesh_use_embedded_materials=False
        m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,position)
        # Matrix -> quaternion preserves fin roll as well as the launch direction.
        r=orientation;trace=np.trace(r)
        if trace>0:
            s=2*np.sqrt(trace+1);q=np.array([(r[2,1]-r[1,2])/s,(r[0,2]-r[2,0])/s,(r[1,0]-r[0,1])/s,s/4])
        else:
            i=int(np.argmax(np.diag(r)));j=(i+1)%3;k=(i+2)%3
            s=2*np.sqrt(1+r[i,i]-r[j,j]-r[k,k]);q=np.zeros(4)
            q[i]=s/4;q[j]=(r[j,i]+r[i,j])/s;q[k]=(r[k,i]+r[i,k])/s;q[3]=(r[k,j]-r[j,k])/s
        q/=np.linalg.norm(q)
        m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=map(float,q)
        m.scale.x=float(c['length']);m.scale.y=m.scale.z=float(2*c['radius'])
        m.color.r=.65;m.color.g=.025;m.color.b=.035;m.color.a=1.
        return m


def main():
    rclpy.init();node=TaskSimulator()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node.claw:node.claw.close()
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
