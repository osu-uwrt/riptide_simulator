#!/usr/bin/env python3
"""Live robot/year matrix and staged zero-camera fixture; isolated ROS domain."""
from pathlib import Path
import os, signal, subprocess, tempfile, time, shutil
import yaml
from riptide_sim_config import resolve
from ament_index_python.packages import get_package_share_directory
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def check(robot,year,scenario='default',resolved=None):
    node=rclpy.create_node('matrix_check');seen={}
    subscriptions=[node.create_subscription(Odometry,f'/{robot}/simulator/ground_truth',lambda m:seen.update(truth=m),10),node.create_subscription(String,f'/{robot}/simulator/run_score',lambda m:seen.update(score=m),10)]
    name=f'{robot}-{year}-{scenario}'+('-zero' if resolved else '')
    with open('/tmp/reuse-matrix-'+name+'.log','w') as log:
        args=['ros2','launch','c_simulator','full_simulator.launch.py','robot:='+robot,'year:='+year,'scenario:='+scenario,'headless:=true','with_rviz:=false','with_apriltag:=false']
        if resolved:args+=['resolved_config:='+str(resolved)]
        p=subprocess.Popen(args,stdout=log,stderr=log,start_new_session=True)
        try:
            end=time.monotonic()+6
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
            assert 'truth' in seen,(name,'no physics')
            assert ('score' in seen)==(scenario!='empty_pool'),(name,'unexpected scoring state')
            log.flush()
            text=Path(log.name).read_text()
            assert 'process has died' not in text and 'Pool viewer:' not in text,text
            assert 'Renderer:' in text,'No rendered scene'
            print('Passed matrix:',name)
        finally:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    node.destroy_node()

assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0')
rclpy.init()
check('example_auv','2026','empty_pool')
check('talos','example')
with tempfile.TemporaryDirectory(prefix='sim-staged-') as tmp:
    root=Path(tmp)/'share';source=Path(get_package_share_directory('c_simulator'))
    for path in ['robots/example_auv','tasks/example','worlds','scenarios']:
        shutil.copytree(source/path,root/path)
    robot=root/'robots/example_auv/robot.yaml';value=yaml.safe_load(robot.read_text());value['cameras']=[];robot.write_text(yaml.safe_dump(value))
    resolved=resolve('example_auv','example','empty_pool',root=root,output=Path(tmp)/'runs')
    check('example_auv','example','empty_pool',resolved)
rclpy.shutdown()
