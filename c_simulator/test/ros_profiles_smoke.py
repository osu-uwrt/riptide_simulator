#!/usr/bin/env python3
"""End-to-end reuse check; isolated ROS domain and an OpenGL display required."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32MultiArray
from std_srvs.srv import Trigger
from robot_localization.srv import SetPose


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0')
    rclpy.init();node=Node('profile_regression')
    latest={};subscriptions=[]
    namespace='test_sub'
    for key,typ,topic in [('truth',Odometry,'simulator/ground_truth'),('forces',Float32MultiArray,'simulator/actual_thruster_forces'),('score',String,'simulator/run_score'),('beacon',Bool,'simulator/beacon'),('image',Image,'survey/rgb/image_rect_color')]:
        subscriptions.append(node.create_subscription(typ,f'/{namespace}/{topic}',lambda msg,k=key:latest.update({k:msg}),10))
    command=node.create_publisher(String,f'/{namespace}/simulator/run_command',10)
    beacon=node.create_publisher(Bool,f'/{namespace}/command/beacon',10)
    reset=node.create_client(Trigger,f'/{namespace}/simulator/reset_tasks')
    pose=node.create_client(SetPose,f'/{namespace}/set_sim_pose')
    sync=node.create_client(Trigger,f'/{namespace}/sync_sim_to_estimate')
    estimate=node.create_publisher(Odometry,f'/{namespace}/odometry/filtered',10)
    enable=node.create_publisher(Bool,f'/{namespace}/simulator/enable',10)
    thrust=node.create_publisher(Float32MultiArray,f'/{namespace}/thruster_forces',10)
    params=node.create_client(SetParameters,f'/{namespace}/physics_simulator/set_parameters')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    def until(predicate,message,timeout=12):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:spin(.05)
        assert predicate(),message+'; received '+str(list(latest))
    def call(client,req):
        assert client.wait_for_service(timeout_sec=5)
        future=client.call_async(req)
        until(future.done,'Service timeout')
        return future.result()
    def run(action):
        msg=String();msg.data=json.dumps(dict(action=action));command.publish(msg)
    def score():return json.loads(latest['score'].data)
    def speed(value):
        req=SetParameters.Request();req.parameters=[Parameter('real_time_factor',value=value).to_parameter_msg()]
        assert all(r.successful for r in call(params,req).results)
    log=Path('/tmp/riptide-profile-regression.log').open('w')
    process=subprocess.Popen(['ros2','launch','c_simulator','full_simulator.launch.py','robot:=example_auv','year:=example','namespace:='+namespace,'headless:=true','sensor_noise:=false'],stdout=log,stderr=log,start_new_session=True)
    try:
        until(lambda:all(k in latest for k in ('truth','forces','score','image','beacon')),'Missing example streams')
        assert process.poll() is None
        assert len(latest['forces'].data)==4
        assert latest['image'].width==640 and latest['image'].height==480
        assert latest['truth'].child_frame_id=='simulator/test_sub/base_link'
        assert score()['year']=='example' and score()['config_id']
        run('start')
        until(lambda:score()['total']==125,'First region did not score')
        req=SetPose.Request();req.pose.header.frame_id='map';req.pose.pose.pose.position.x=5.;req.pose.pose.pose.position.y=3.;req.pose.pose.pose.position.z=-1.;req.pose.pose.pose.orientation.w=1.
        # Pose-only alignment preserves integrated time; set_sim_pose resets it.
        odom=Odometry();odom.header.frame_id="map";odom.pose=req.pose.pose
        for _ in range(3):estimate.publish(odom);spin(.05)
        assert call(sync,Trigger.Request()).success
        until(lambda:score()['total']==300,'Second region / completion bonus did not score')
        msg=Bool();msg.data=True;beacon.publish(msg)
        until(lambda:latest['beacon'].data,'Beacon did not activate',2)
        until(lambda:not latest['beacon'].data,'Beacon did not expire',2)
        speed(0.);spin(.2)
        elapsed=score()['elapsed'];spin(.3)
        assert score()['elapsed']==elapsed,'Run advanced while paused'
        assert call(reset,Trigger.Request()).success
        until(lambda:score()['total']==0 and not score()['running'],'Paused reset did not clear score')
        speed(1.)
        msg=Bool();msg.data=True;enable.publish(msg)
        force=Float32MultiArray();force.data=[5.,5.,0.,0.]
        for _ in range(15):thrust.publish(force);spin(.025)
        assert max(latest['forces'].data)>1.,'Generic thrust enable failed'
        msg.data=False;enable.publish(msg)
        until(lambda:max(abs(v) for v in latest['forces'].data)<.01,'Generic disable failed')
        print('Passed: independent robot/year, custom namespace, four thrusters, survey camera, two task instances, 300-point score, beacon timeout, simulation-time pause and reset')
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
