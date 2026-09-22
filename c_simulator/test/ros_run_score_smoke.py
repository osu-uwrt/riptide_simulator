#!/usr/bin/env python3
"""Live ROS scoring, geometry, payloads and manual run controls; isolated domain."""
import argparse
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
from std_msgs.msg import Empty, Float64, String
from std_srvs.srv import SetBool, Trigger
from nav_msgs.msg import Odometry
from riptide_msgs2.msg import KillSwitchReport
from ament_index_python.packages import get_package_share_directory as share
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from magnet_light_model import pose, MagnetLights
from claw_world import quaternion
from payload_model import payload_mounts


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path)
    args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0')
    package=Path(share('c_simulator'));vehicle_path=Path(share('riptide_descriptions2'))/'config/talos.yaml'
    from riptide_sim_config import resolve
    resolved=resolve()
    task_path=resolved/'task.yaml';mapping_path=resolved/'mapping.yaml'
    cfg=yaml.safe_load(task_path.read_text());vehicle=yaml.safe_load(vehicle_path.read_text())
    data=yaml.safe_load(mapping_path.read_text())['/talos/riptide_mapping2']['ros__parameters']['init_data']
    def resolve(key):
        if key in ('map','world'):return np.eye(4)
        key=key.removesuffix('_frame');e=data[key];p=e['pose']
        return resolve(e['parent'])@pose([p.get(c,0) for c in ('x','y','z')]+[0,0,math.radians(p.get('yaw',0))])
    rclpy.init();node=Node('run_score_smoke');latest={}
    sub=node.create_subscription(String,'/talos/simulator/run_score',lambda m:latest.update(json.loads(m.data)),10)
    commands=node.create_publisher(String,'/talos/simulator/run_command',10)
    truth=node.create_publisher(Odometry,'/talos/simulator/ground_truth',10)
    clock=node.create_publisher(Float64,'/talos/simulator/time',10)
    kill=node.create_publisher(KillSwitchReport,'/talos/command/software_kill',10)
    reset=node.create_publisher(Empty,'/talos/simulator/reset_tasks',10)
    arm=node.create_client(SetBool,'/talos/command/actuator/arm')
    drop=node.create_client(Trigger,'/talos/command/actuator/dropper')
    body=pose([0,0,-1,0,0,0]);current=0.
    def spin(seconds,advance=True):
        nonlocal current
        for _ in range(math.ceil(seconds/.01)):
            until=time.monotonic()+.01
            m=Odometry();m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,body[:3,3])
            m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=map(float,quaternion(body[:3,:3]))
            truth.publish(m)
            k=KillSwitchReport();k.kill_switch_id=1;k.switch_asserting_kill=False;kill.publish(k)
            if advance:current+=.01
            t=Float64();t.data=current;clock.publish(t)
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=max(0,until-time.monotonic()))
    def command(action,**kw):
        m=String();m.data=json.dumps(dict(action=action,**kw));commands.publish(m);spin(.2,False)
    def points(key):return next(r['points'] for r in latest['rows'] if r['key']==key)
    def call(client,request):
        f=client.call_async(request);end=time.monotonic()+3
        while not f.done() and time.monotonic()<end:spin(.01,False)
        assert f.done() and f.result().success, f.result()
    launch=['ros2','run','c_simulator','task_simulator.py','--ros-args','-r','__ns:=/talos']
    for k,v in {'robot':'talos','vehicle_config':vehicle_path,'task_config':task_path,'mapping_config':mapping_path,'hydrodynamics_config':package/'config/talos_hydrodynamics.yaml'}.items():launch+=['-p',f'{k}:={v}']
    with open('/tmp/ros-run-score-node.log','w') as log:
        process=subprocess.Popen(launch,stdout=log,stderr=log,start_new_session=True)
        try:
            assert arm.wait_for_service(timeout_sec=8)
            spin(.3,False)
            command('start',role='repair')
            assert latest['running'] and latest['total']==0
            command('pinger_select',task='restore',random=True)
            gate=resolve('gate')
            for x in np.linspace(1,-1,35):
                body=gate@pose([float(x),.75,0,0,0,math.pi]);spin(.02)
            assert latest['gate_passed'] and latest['role']=='rescue',latest
            assert latest['target_class']=='blood'
            command('start',role='repair')
            assert 'rejected' in latest['message'] and latest['role']=='rescue'
            before=latest['elapsed'];spin(.2,False)
            assert abs(latest['elapsed']-before)<.02
            # A real drop into a blood bin proves the role reaches payload scoring.
            crate=resolve('bin_vinyl1');body=crate.copy()
            body[:3,3]=(crate@np.array([0,0,.42,1]))[:3]-body[:3,:3]@payload_mounts(vehicle,cfg,'dropper')[0][:3,3]
            spin(.15)
            req=SetBool.Request();req.data=True;call(arm,req);call(drop,Trigger.Request())
            spin(2.)
            assert points('bins')==800,latest
            magnet=MagnetLights(cfg['magnet_lights'],{k:resolve(k) for k in cfg['magnet_lights']['targets']},vehicle)
            body=np.eye(4);body[:3,3]=magnet.sensors['magnet_target1']-magnet.tip
            spin(.65);assert points('lights')==500,latest
            # Surface fully inside the octagon, facing the rescue sign.
            center=resolve('octagon')[:3,3]
            direction=resolve('buoy')[:3,3]-center
            heading=math.atan2(direction[1],direction[0])
            body=pose([center[0],center[1],-1.,0,0,heading]);spin(.1)
            for z in np.linspace(-1.,-.1,20):
                body=pose([center[0],center[1],z,0,0,heading]);spin(.02)
            spin(1.3)
            assert points('surface')==800 and points('facing')==400,latest
            assert points('pinger_first')==500,latest
            command('stop');elapsed=latest['elapsed'];total=latest['total']
            spin(.3);assert latest['elapsed']==elapsed and latest['total']==total
            command('adjustment',points=123.5);assert latest['total']==total+123.5
            if args.capture:
                capture=subprocess.Popen(['ros2','launch','camera_faker','pool_viewer.launch.py','headless:=true','show_scorecard:=true','initial_focus:=octagon','camera_scale:=0.25','exit_after_frames:=45','screenshot_path:='+str(args.capture)],stdout=log,stderr=log,start_new_session=True)
                try:
                    end=time.monotonic()+25
                    while capture.poll() is None and time.monotonic()<end:spin(.1,False)
                    assert capture.poll()==0 and args.capture.exists()
                finally:
                    if capture.poll() is None:os.killpg(capture.pid,signal.SIGINT);capture.wait(timeout=5)
            reset.publish(Empty());spin(.2,False)
            assert latest['total']==0 and latest['elapsed']==0 and not latest['running']
            print('Passed live start/stop/reset, paused timer, gate-derived rescue role, real bin/magnet scoring, octagon surfacing/facing, future pinger hook and manual adjustment')
        finally:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
