#!/usr/bin/env python3
"""Isolated actuator service, geometry, scoring and reset regression."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory as share
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger, SetBool
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from riptide_msgs2.msg import KillSwitchReport, ActuatorStatus
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from task_simulator import rotation, frame, quat_rotation
from payload_model import payload_mounts


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0')
    rclpy.init();node=Node('task_regression')
    package=Path(share('c_simulator'));vehicle_path=Path(share('riptide_descriptions2'))/'config/talos.yaml'
    task_path=package/'config/talos_tasks.yaml';mapping_path=package/'config/simulation.yaml'
    cfg=yaml.safe_load(task_path.read_text());vehicle=yaml.safe_load(vehicle_path.read_text())
    data=yaml.safe_load(mapping_path.read_text())['/talos/riptide_mapping2']['ros__parameters']['init_data']
    def resolve(key):
        if key in ('map','world'):return np.eye(4)
        if key.endswith('_frame'):key=key[:-6]
        a=data[key];p=a['pose'];return resolve(a['parent'])@frame([p.get(c,0) for c in ('x','y','z')],rotation([0,0,math.radians(p.get('yaw',0))]))
    pose_pub=node.create_publisher(Odometry,'/talos/simulator/ground_truth',10)
    clock=node.create_publisher(Float64,'/talos/simulator/time',10)
    kill=node.create_publisher(KillSwitchReport,'/talos/command/software_kill',10)
    events=[];states=[];visible=[]
    subscriptions=[node.create_subscription(String,'/talos/simulator/task_events',lambda m:events.append(json.loads(m.data)),100),node.create_subscription(ActuatorStatus,'/talos/state/actuator/status',states.append,10)]
    clients={k:node.create_client(SetBool if k=='arm' else Trigger,'/talos/command/actuator/'+k) for k in ('arm','torpedo','dropper','notify_reload')}
    subscriptions.append(node.create_subscription(MarkerArray,'/talos/simulator/projectiles',lambda m:visible.append(m),10))
    def markers(ns):
        return [m for m in visible[-1].markers if m.action==Marker.ADD and m.ns==ns]
    current=0.;pose=Odometry();pose.pose.pose.orientation.w=1.;pose.pose.pose.position.z=-1.
    def spin(seconds,simulate=True,killed=False):
        nonlocal current
        # Keep simulation at most real time and drain callbacks between ticks.
        # Advancing on every callback can outrun physics and leave clock ticks
        # queued across a supposedly paused release-pose assertion.
        for _ in range(math.ceil(seconds/.01)):
            deadline=time.monotonic()+.01
            report=KillSwitchReport();report.kill_switch_id=1;report.switch_asserting_kill=killed;kill.publish(report)
            pose.header.stamp=node.get_clock().now().to_msg();pose_pub.publish(pose)
            if simulate:current+=.01
            msg=Float64();msg.data=current;clock.publish(msg)
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=max(0.,deadline-time.monotonic()))
    def call(key,value=None):
        req=SetBool.Request() if key=='arm' else Trigger.Request()
        if key=='arm':req.data=value
        future=clients[key].call_async(req);deadline=time.monotonic()+2
        while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.005)
        assert future.done(),'Actuator service timed out';return future.result()
    def spawn_pose(kind,position,rotation_matrix):
        c=cfg[kind];r=rotation_matrix
        mount=payload_mounts(vehicle,cfg,kind)[0][:3,3]
        base=position-r@mount
        pose.pose.pose.position.x,pose.pose.pose.position.y,pose.pose.pose.position.z=map(float,base)
        yaw=math.atan2(r[1,0],r[0,0]);pose.pose.pose.orientation.z=math.sin(yaw/2);pose.pose.pose.orientation.w=math.cos(yaw/2)
        spin(.1)
    command=['ros2','run','c_simulator','task_simulator.py','--ros-args','-r','__ns:=/talos']
    for k,v in {'robot':'talos','vehicle_config':vehicle_path,'task_config':task_path,'mapping_config':mapping_path,'hydrodynamics_config':package/'config/talos_hydrodynamics.yaml'}.items():command+=['-p',f'{k}:={v}']
    with open('/tmp/ros-task-smoke-node.log','w') as log:
        process=subprocess.Popen(command,stdout=log,stderr=log,start_new_session=True)
        try:
            assert clients['arm'].wait_for_service(timeout_sec=8)
            spin(.15,killed=True);assert not call('arm',True).success
            spin(.1);assert not call('torpedo').success
            assert len(markers('torpedo_loaded'))==2 and len(markers('dropper_loaded'))==2
            for kind in ('torpedo','dropper'):
                for m in markers(kind+'_loaded'):
                    expected=payload_mounts(vehicle,cfg,kind)[m.id][:3,3]+[0,0,-1]
                    np.testing.assert_allclose([m.pose.position.x,m.pose.position.y,m.pose.position.z],expected,atol=1e-7)
                    assert m.mesh_resource.endswith('projectile.obj') and m.type==Marker.MESH_RESOURCE
            assert markers('torpedo_loaded')[0].scale==markers('dropper_loaded')[0].scale
            target=resolve('torpedo');cfg_t=cfg['torpedo']
            for hole in (cfg_t['holes'][0],cfg_t['holes'][2]):
                assert call('notify_reload').success
                yz=(np.array(hole['uv'])-.5)*2*cfg_t['panel_half_size']
                spawn_pose('torpedo',(target@np.r_[.30,yz,1])[:3],target[:3,:3]@rotation([0,0,math.pi]))
                assert call('arm',True).success;events.clear()
                spin(.08,simulate=False)
                loaded=markers('torpedo_loaded')[0]
                assert call('torpedo').success
                assert not call('torpedo').success,'Ignored cooldown'
                spin(.08,simulate=False)
                assert len(markers('torpedo_loaded'))==1 and len(markers('dropper_loaded'))==2
                released=markers('torpedo')[-1]
                assert released.pose==loaded.pose, 'Payload teleported instead of leaving its loaded mount'
                spin(.6)
                expected='success' if hole['class']=='fire' else 'wrong_target'
                assert any(e['result']==expected and e['target']==hole['name'] for e in events),events
                flown=markers('torpedo')[-1]
                assert quat_rotation(flown.pose.orientation)[2,0]<-.05, 'Torpedo marker did not pitch nose down'
                assert flown.pose.position.z<loaded.pose.position.z-.01, 'Torpedo did not sink after launch'
            assert call('notify_reload').success
            spawn_pose('torpedo',(target@np.array([.3,0,0,1]))[:3],target[:3,:3]@rotation([0,0,math.pi]))
            assert call('arm',True).success;events.clear();assert call('torpedo').success;spin(.6)
            assert any(e['result']=='blocked' for e in events),events
            # Drain in-flight clock messages before comparing a frozen release
            # pose; otherwise a queued physics tick can advance the new round.
            spin(.08,simulate=False)
            loaded=markers('torpedo_loaded')[0]
            assert loaded.id==1 and call('torpedo').success
            spin(.08,simulate=False)
            assert not markers('torpedo_loaded')
            assert markers('torpedo')[-1].pose==loaded.pose, (markers('torpedo')[-1].pose,loaded.pose)
            assert not call('torpedo').success
            for key in ('bin_vinyl2','bin_vinyl1'):
                assert call('notify_reload').success
                crate=resolve(key);spawn_pose('dropper',(crate@np.array([0,0,.42,1]))[:3],crate[:3,:3])
                assert call('arm',True).success;events.clear()
                spin(.08,simulate=False)
                loaded=markers('dropper_loaded')[0]
                assert call('dropper').success;spin(.08,simulate=False)
                assert len(markers('dropper_loaded'))==1 and len(markers('torpedo_loaded'))==2
                assert markers('dropper')[-1].pose==loaded.pose, (markers('dropper')[-1].pose,loaded.pose)
                spin(1.4)
                expected='success' if data[key]['class']=='fire' else 'wrong_target'
                assert any(e['result']==expected and e['target']==key for e in events),events
            # No ammunition after two firings; reloading disarms, time reset clears counts.
            spin(.6);spin(.08,simulate=False);loaded=markers('dropper_loaded')[0]
            assert loaded.id==1 and call('dropper').success
            spin(.08,simulate=False)
            assert not markers('dropper_loaded') and markers('dropper')[-1].pose==loaded.pose
            spin(.6);assert not call('dropper').success
            crate=resolve('bin_vinyl2')
            for x,result in ((cfg['crate']['outer_width']/2,'blocked'),(.65,'miss')):
                assert call('notify_reload').success
                spawn_pose('dropper',(crate@np.array([x,0,.42,1]))[:3],crate[:3,:3])
                assert call('arm',True).success;events.clear();assert call('dropper').success;spin(1.8)
                assert any(e['result']==result for e in events),events
            current=0;spin(.1);assert states[-1].torpedo_available_count==2 and not states[-1].actuators_armed
            assert len(markers('torpedo_loaded'))==2 and len(markers('dropper_loaded'))==2
            assert not markers('torpedo') and not markers('dropper')
            print('Passed four robot-config payload mounts, continuous release, loaded counts, kill/arm, cooldown, ammo, torpedo openings/wrong target/vinyl block, crate entry/landing/rim/miss, reload and reset')
        finally:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
