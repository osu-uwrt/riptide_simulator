#!/usr/bin/env python3
"""Display-backed plant, task and camera launch check in an isolated ROS domain."""
import os, signal, subprocess, tempfile, time
from pathlib import Path
import rclpy, yaml, xacro
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory as share
from geometry_msgs.msg import TransformStamped, Pose
from sensor_msgs.msg import Image
from riptide_msgs2.msg import ActuatorStatus, KillSwitchReport
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from std_msgs.msg import Float32, Float64MultiArray

assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated ROS domain'
rclpy.init()
n=Node('pool_live_physics_check')
tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,n)
b=StaticTransformBroadcaster(n)
frames=[]
for child in ('world','odom'):
    t=TransformStamped(); t.header.stamp=n.get_clock().now().to_msg(); t.header.frame_id='map'; t.child_frame_id=child; t.transform.rotation.w=1.; frames.append(t)
b.sendTransform(frames)
counts={'physics':0,'ffc':0,'dfc':0}
subs=[]
def count(key, msg):counts[key]+=1
subs.append(n.create_subscription(Pose,'/talos/simulator/state',lambda m:count('physics',m),10))
for name in ('ffc','dfc'):
    subs.append(n.create_subscription(Image,f'/talos/{name}/zed_node/rgb/image_rect_color',lambda m,k=name:count(k,m),10))
latest={}
subs.append(n.create_subscription(ActuatorStatus,'/talos/state/actuator/status',lambda m:latest.update(status=m),10))
subs.append(n.create_subscription(MarkerArray,'/talos/simulator/projectiles',lambda m:latest.update(markers=m),10))
subs.append(n.create_subscription(MarkerArray,'/talos/simulator/task_objects',lambda m:latest.update(objects=m),10))
subs.append(n.create_subscription(Float64MultiArray,'/talos/simulator/claw_joints',lambda m:latest.update(joints=m.data),10))
kill=n.create_publisher(KillSwitchReport,'/talos/command/software_kill',10)
arm=n.create_client(SetBool,'/talos/command/actuator/arm')
fire=n.create_client(Trigger,'/talos/command/actuator/torpedo')
claw=n.create_client(SetBool,'/talos/command/actuator/claw')
reset_table=n.create_client(Trigger,'/talos/simulator/reset_table')
timed_claw=n.create_publisher(Float32,'/talos/command/actuator/claw_move_s',10)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def call(client,request):
    f=client.call_async(request)
    end=time.monotonic()+3
    while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
    assert f.done() and f.result().success, f.result()
processes=[]
with tempfile.TemporaryDirectory() as tmp:
    description=xacro.process_file(str(Path(share('riptide_descriptions2'))/'robots/talos.xacro')).toxml()
    config=Path(tmp)/'rsp.yaml';config.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':description}}}))
    log=open('/tmp/pool-live-physics.log','w')
    try:
        for command in (
            ['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(config)],
            ['ros2','run','joint_state_publisher','joint_state_publisher','--ros-args','--params-file',str(config)],
            ['ros2','launch','c_simulator','physics_simulator.launch.py','robot:=talos'],
            ['ros2','launch','camera_faker','zedfaker.launch.py','robot:=talos','headless:=true'],
        ):
            processes.append(subprocess.Popen(command,stdout=log,stderr=log,start_new_session=True))
        deadline=time.monotonic()+9
        while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
        print(counts)
        assert all(v>10 for v in counts.values()), 'Missing live physics/camera data'
        assert 'status' in latest and latest['status'].torpedo_available_count==2
        report=KillSwitchReport();report.kill_switch_id=1;report.switch_asserting_kill=False
        for _ in range(10):kill.publish(report);spin(.03)
        req=SetBool.Request();req.data=True;call(arm,req);call(fire,Trigger.Request());spin(.5)
        assert latest['status'].torpedo_available_count==1
        assert any(m.action==Marker.ADD and m.ns=="torpedo" for m in latest['markers'].markers)
        assert {m.ns for m in latest['objects'].markers}=={'pill','bandage','nut_and_bolt','plug'}
        assert tf_buffer.can_transform('talos/base_link','simulator/talos/claw_tool',rclpy.time.Time())
        assert abs(sum(latest['joints']))<.004,'Claw did not start closed'
        req=SetBool.Request();req.data=True;call(claw,req);spin(2.4)
        opened=sum(latest['joints'])
        req=SetBool.Request();req.data=False;call(claw,req);spin(.7)
        closed=sum(latest['joints']);assert closed<opened-.025,(opened,closed)
        command=Float32();command.data=.3;timed_claw.publish(command);spin(.5)
        partial=sum(latest['joints']);assert partial>closed+.008
        spin(.3);assert abs(sum(latest['joints'])-partial)<.004,'Timed claw did not stop'
        req=SetBool.Request();req.data=False;call(claw,req);spin(.1)
        command=Float32();command.data=0.;timed_claw.publish(command);spin(.1)
        stopped=sum(latest['joints']);spin(.3)
        assert abs(sum(latest['joints'])-stopped)<.004,'Explicit claw stop did not hold'
        call(reset_table,Trigger.Request());spin(.4)
        assert abs(sum(latest['joints']))<.004,'Reset did not close claw'
        assert len(latest['objects'].markers)==4
        assert all(p.poll() is None for p in processes), 'Integration process exited'
        print('Plant + cameras + payload firing + four dynamic props + claw tool TF, service/timed motion/stop/reset: passed')
    finally:
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        log.close()
n.destroy_node();rclpy.shutdown()
