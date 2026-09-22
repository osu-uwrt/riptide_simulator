#!/usr/bin/env python3
"""Isolated reset-all service/topic regression; requires a nonzero ROS_DOMAIN_ID."""
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
from std_msgs.msg import Empty, Float64, String
from std_srvs.srv import Trigger, SetBool
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from riptide_msgs2.msg import KillSwitchReport, ActuatorStatus
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from riptide_sim_config.geometry import rotation, frame
from magnet_light_model import MagnetLights


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0')
    rclpy.init();node=Node('task_reset_regression')
    package=Path(share('c_simulator'));vehicle_path=Path(share('riptide_descriptions2'))/'config/talos.yaml'
    from riptide_sim_config import resolve
    resolved=resolve()
    task_path=resolved/'task.yaml';mapping_path=resolved/'mapping.yaml'
    cfg=yaml.safe_load(task_path.read_text());vehicle=yaml.safe_load(vehicle_path.read_text())
    data=yaml.safe_load(mapping_path.read_text())['/talos/riptide_mapping2']['ros__parameters']['init_data']
    def resolve(key):
        if key in ('map','world'):return np.eye(4)
        if key.endswith('_frame'):key=key[:-6]
        a=data[key];p=a['pose'];return resolve(a['parent'])@frame([p.get(c,0) for c in ('x','y','z')],rotation([0,0,math.radians(p.get('yaw',0))]))
    pose_pub=node.create_publisher(Odometry,'/talos/simulator/ground_truth',10)
    clock=node.create_publisher(Float64,'/talos/simulator/time',10)
    kill=node.create_publisher(KillSwitchReport,'/talos/command/software_kill',10)
    events=[];states=[];visible=[];lights=[];scores=[];objects=[]
    subscriptions=[node.create_subscription(String,'/talos/simulator/task_events',lambda m:events.append(json.loads(m.data)),100),node.create_subscription(ActuatorStatus,'/talos/state/actuator/status',states.append,10)]
    clients={k:node.create_client(SetBool if k=='arm' else Trigger,'/talos/command/actuator/'+k) for k in ('arm','torpedo','dropper','notify_reload')}
    clients['reset_tasks']=node.create_client(Trigger,'/talos/simulator/reset_tasks')
    reset_pub=node.create_publisher(Empty,'/talos/simulator/reset_tasks',10)
    subscriptions.extend([
        node.create_subscription(MarkerArray,'/talos/simulator/magnet_lights',lights.append,10),
        node.create_subscription(String,'/talos/simulator/task_score',lambda m:scores.append(json.loads(m.data)),10),
        node.create_subscription(MarkerArray,'/talos/simulator/task_objects',objects.append,10)])
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
    command=['ros2','run','c_simulator','task_simulator.py','--ros-args','-r','__ns:=/talos']
    for k,v in {'robot':'talos','vehicle_config':vehicle_path,'task_config':task_path,'mapping_config':mapping_path,'hydrodynamics_config':package/'config/talos_hydrodynamics.yaml'}.items():command+=['-p',f'{k}:={v}']
    with open('/tmp/ros-task-reset-smoke-node.log','w') as log:
        process=subprocess.Popen(command,stdout=log,stderr=log,start_new_session=True)
        try:
            assert clients['arm'].wait_for_service(timeout_sec=8)
            assert clients['reset_tasks'].wait_for_service(timeout_sec=8)
            spin(.2)
            assert call('arm',True).success
            assert call('torpedo').success
            spin(max(.65,cfg['torpedo']['cooldown']+.1))
            assert call('dropper').success
            spin(2.)
            # Both reset interfaces clear an attempted course while physics is paused.
            magnet=MagnetLights(cfg['magnet_lights'],{k:resolve(k) for k in cfg['magnet_lights']['targets']},vehicle)
            near=magnet.sensors['magnet_target1']-magnet.tip
            pose.pose.pose.position.x,pose.pose.pose.position.y,pose.pose.pose.position.z=map(float,near)
            pose.pose.pose.orientation.z=0.;pose.pose.pose.orientation.w=1.
            spin(.65)
            assert any(m.ns=='magnet_target1' and m.color.g==1 for m in lights[-1].markers)
            assert markers('torpedo') and markers('dropper') and any(scores[-1].values())
            def check_reset():
                spin(.15,simulate=False)
                assert not markers('torpedo') and not markers('dropper')
                assert visible[-1].markers[0].action==Marker.DELETEALL
                assert len(markers('torpedo_loaded'))==cfg['torpedo']['count']
                assert len(markers('dropper_loaded'))==cfg['dropper']['count']
                assert not states[-1].actuators_armed
                assert states[-1].torpedo_available_count==cfg['torpedo']['count']
                assert states[-1].dropper_available_count==cfg['dropper']['count']
                assert not any(scores[-1].values())
                assert all(m.color.r==1 and m.color.g==0 for m in lights[-1].markers)
                for m in objects[-1].markers:
                    assert m.header.frame_id=='map'
                    np.testing.assert_allclose([m.pose.position.x,m.pose.position.y,m.pose.position.z],resolve(m.ns)[:3,3],atol=1e-7)
                assert any(e['kind']=='tasks' and e['result']=='reset' for e in events)
            spin(.08,simulate=False)
            assert call('reset_tasks').success
            check_reset()
            # The viewer uses the topic; reset also works during a release cooldown.
            assert call('arm',True).success
            assert call('torpedo').success
            spin(max(.65,cfg['torpedo']['cooldown']+.1))
            assert call('dropper').success
            spin(.08,simulate=False)
            assert markers('torpedo') and markers('dropper')
            assert any(m.color.g==1 for m in lights[-1].markers)
            reset_pub.publish(Empty())
            check_reset()
            # Reset is repeatable, and the existing clock-rewind path still works.
            assert call('reset_tasks').success
            check_reset()
            assert call('arm',True).success
            assert call('torpedo').success
            current=0;spin(.1);assert states[-1].torpedo_available_count==2 and not states[-1].actuators_armed
            assert len(markers('torpedo_loaded'))==2 and len(markers('dropper_loaded'))==2
            assert not markers('torpedo') and not markers('dropper')
            print('Passed reset-all service/topic while paused, payload cleanup, ammunition, disarm, magnet/table/score reset, repeated reset and clock rewind')
        finally:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
