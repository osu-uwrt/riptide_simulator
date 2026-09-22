#!/usr/bin/env python3
"""Drive the actual Fossen plant into the table using thruster commands."""
import os,signal,subprocess,time,tempfile,xacro
from pathlib import Path
import numpy as np,yaml,rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory as share
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from std_msgs.msg import Float32MultiArray,Float64MultiArray,Float64
from std_srvs.srv import SetBool
from robot_localization.srv import SetPose
from riptide_msgs2.msg import KillSwitchReport
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from claw_world import rotation,quaternion
assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0')
rclpy.init();node=Node('claw_table_contact_check');latest={}
broadcaster=StaticTransformBroadcaster(node);transforms=[]
for child in ('world','odom'):
 t=TransformStamped();t.header.frame_id='map';t.child_frame_id=child;t.transform.rotation.w=1.;transforms.append(t)
broadcaster.sendTransform(transforms)
node.create_subscription(Float64,'/talos/simulator/time',lambda m:latest.update(clock=m.data),10)
node.create_subscription(Odometry,'/talos/simulator/ground_truth',lambda m:latest.update(truth=m),10)
node.create_subscription(Float64MultiArray,'/talos/simulator/claw_joints',lambda m:latest.update(jaws=m.data),10)
forces=node.create_publisher(Float32MultiArray,'/talos/thruster_forces',10)
kill=node.create_publisher(KillSwitchReport,'/talos/command/software_kill',10)
pose_client=node.create_client(SetPose,'/talos/set_sim_pose')
arm=node.create_client(SetBool,'/talos/command/actuator/arm')
claw=node.create_client(SetBool,'/talos/command/actuator/claw')
package=Path(share('c_simulator'));vehicle=yaml.safe_load((Path(share('riptide_descriptions2'))/'config/talos.yaml').read_text())
task=yaml.safe_load((package/'config/talos_tasks.yaml').read_text())
mapping=yaml.safe_load((package/'config/simulation.yaml').read_text())['/talos/riptide_mapping2']['ros__parameters']['init_data']
def frame(key):
 if key in ('map','world'):return np.eye(4)
 key=key.removesuffix('_frame');a=mapping[key];p=a['pose'];yaw=np.deg2rad(p.get('yaw',0));R=rotation([0,0,np.sin(yaw/2),np.cos(yaw/2)])
 t=np.eye(4);t[:3,:3]=R;t[:3,3]=[p.get(k,0) for k in ('x','y','z')];return frame(a['parent'])@t
table=frame('table');Rdesired=table[:3,:3];mount=np.array(task['claw']['pose'][:3])-vehicle['base_link']
center=(table@np.array([.05,0,0,1]))[:3];desired=center+[0,0,.04]-Rdesired@mount
matrix=[]
import pybullet as pb
for t in vehicle['thrusters']:
 p=t['pose'];direction=rotation(pb.getQuaternionFromEuler(p[3:]))[:,0]
 matrix.append(np.r_[direction,np.cross(np.array(p[:3])-vehicle['com'],direction)])
allocation=np.linalg.pinv(np.array(matrix).T)
vertices=[]
for filename in ('claw_pad.obj','claw_pad_right.obj'):
 vertices.append(np.array([[float(x) for x in line.split()[1:4]] for line in (package/'collision_files/tasks'/filename).read_text().splitlines() if line.startswith('v ')]))
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
def call(client,request):
 assert client.wait_for_service(timeout_sec=8)
 future=client.call_async(request);end=time.monotonic()+5
 while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
 assert future.done();return future.result()
def press(seconds,vertical):
 end=latest['clock']+seconds;deadline=time.monotonic()+seconds*8+5;start=time.monotonic();lowest=10.;highest_speed=0.;gap=np.array(latest['jaws']);last_bottom=10.
 while latest['clock']<end:
  assert time.monotonic()<deadline,'Physics stopped advancing'
  m=latest['truth'];p=m.pose.pose;tw=m.twist.twist
  R=rotation([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]);xyz=np.array([p.position.x,p.position.y,p.position.z]);omega=np.array([tw.angular.x,tw.angular.y,tw.angular.z]);v=np.array([tw.linear.x,tw.linear.y,tw.linear.z])
  error=Rdesired.T@R;angle=.5*np.array([error[2,1]-error[1,2],error[0,2]-error[2,0],error[1,0]-error[0,1]])
  force=30*(desired-xyz)-18*(R@v);force[2]=vertical
  torque=np.clip(-90*angle-15*omega,-14,14)
  command=Float32MultiArray();command.data=np.clip(allocation@np.r_[R.T@force,torque],-25,25).astype(float).tolist();forces.publish(command)
  points=[]
  for vs,sign,jaw in zip(vertices,(1,-1),latest['jaws']):
   ps=(R@(vs+mount+[0,sign*jaw,0]).T).T+xyz
   local=(table[:3,:3].T@(ps-table[:3,3]).T).T;inside=np.all(np.abs(local[:,:2])<.317,axis=1)
   if inside.any():points.extend(local[inside,2])
  assert points,'Claw left the test tabletop'
  last_bottom=min(points);lowest=min(lowest,last_bottom);highest_speed=max(highest_speed,np.linalg.norm(v)+np.linalg.norm(omega))
  assert last_bottom>-.003,('claw penetrated tabletop',last_bottom)
  assert np.max(np.abs(np.array(latest['jaws'])-gap))<1e-6,'Table impact back-drove the jaws'
  assert highest_speed<3.,('Unstable vehicle response',highest_speed)
  spin(.02)
 return lowest,last_bottom,highest_speed,seconds/(time.monotonic()-start)
log=open('/tmp/claw-contact-live-node.log','w')
temporary=tempfile.TemporaryDirectory(prefix='claw-contact-ros-')
description=xacro.process_file(str(Path(share('riptide_descriptions2'))/'robots/talos.xacro')).toxml()
parameters=Path(temporary.name)/'robot.yaml';parameters.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':description}}}))
processes=[]
for command in (
 ['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(parameters)],
 ['ros2','run','joint_state_publisher','joint_state_publisher','--ros-args','--params-file',str(parameters)],
 ['ros2','launch','c_simulator','physics_simulator.launch.py','sensor_noise:=false']):
 processes.append(subprocess.Popen(command,stdout=log,stderr=log,start_new_session=True))
try:
 deadline=time.monotonic()+10
 while ('truth' not in latest or 'jaws' not in latest) and time.monotonic()<deadline:spin(.05)
 assert 'truth' in latest and 'jaws' in latest
 request=SetPose.Request();request.pose.header.frame_id='map';target=request.pose.pose.pose;target.position.x,target.position.y,target.position.z=map(float,desired);q=quaternion(Rdesired);target.orientation.x,target.orientation.y,target.orientation.z,target.orientation.w=map(float,q);call(pose_client,request)
 report=KillSwitchReport();report.kill_switch_id=1;report.switch_asserting_kill=False
 for _ in range(10):kill.publish(report);spin(.03)
 req=SetBool.Request();req.data=True;assert call(arm,req).success
 result=press(4.,-30);print('open impact',result,flush=True)
 assert result[0]<.004,'Open claw never reached the table'
 result=press(1.8,30);print('release',result,flush=True);assert result[1]>.02,'Claw failed to leave the table'
 req.data=False;assert call(claw,req).success
 # Keep driving up while closing, so the gap is settled before the next impact.
 end=latest['clock']+2.5;deadline=time.monotonic()+25
 while latest['clock']<end:
  assert time.monotonic()<deadline,'Physics stopped while closing the claw'
  forces.publish(Float32MultiArray(data=[0.]*8));spin(.02)
 assert max(latest['jaws'])<.001,'Claw did not finish closing'
 result=press(4.,-30);print('closed impact',result,flush=True)
 assert result[0]<.004,'Closed claw never reached the table'
 assert press(1.8,30)[1]>.02
 print('PASS: open/closed claw impacts stop the live vehicle, preserve jaw gap, and release upward',flush=True)
finally:
 for process in processes:
  if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
 for process in processes:
  try:process.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
 temporary.cleanup();log.close();node.destroy_node();rclpy.try_shutdown()
