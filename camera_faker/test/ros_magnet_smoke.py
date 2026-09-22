#!/usr/bin/env python3
"""Display-backed bin-light integration with real rendered RGB/depth.

Run in an isolated ROS_DOMAIN_ID after building/sourcing the workspace.
No actuator commands are sent. Optional --weights checks the actual detector.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory as share
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'c_simulator/scripts'))
from magnet_light_model import MagnetLights, pose
from claw_world import quaternion


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('/tmp/magnet-validation'))
    parser.add_argument('--weights',type=Path)
    parser.add_argument('--min-confidence',type=float,default=.65)
    parser.add_argument('--elevations',type=float,nargs='+',default=[45.])
    parser.add_argument('--distances',type=float,nargs='+',default=[.25,.4,.6,1.])
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0')
    camera=Path(share('camera_faker'));task=Path(share('c_simulator'))
    vehicle_path=Path(share('riptide_descriptions2'))/'config/talos.yaml'
    vehicle=yaml.safe_load(vehicle_path.read_text())
    cfg=yaml.safe_load((task/'config/talos_tasks.yaml').read_text())
    mapping=task/'config/simulation.yaml'
    data=yaml.safe_load(mapping.read_text())['/talos/riptide_mapping2']['ros__parameters']['init_data']
    def resolve(key):
        if key in ('map','world'):return np.eye(4)
        key=key.removesuffix('_frame');entry=data[key];p=entry['pose']
        return resolve(entry['parent'])@pose([p.get(k,0) for k in ('x','y','z')]+[0,0,math.radians(p.get('yaw',0))])
    model=MagnetLights(cfg['magnet_lights'],{k:resolve(k) for k in cfg['magnet_lights']['targets']},vehicle)
    mount=pose(next(c['pose'] for c in vehicle['cameras'] if c['name']=='ffc'))
    mount[:3,3]-=vehicle['base_link']
    body=np.eye(4);current=0.;latest={};events=[]
    rclpy.init();node=Node('magnet_integration_test');tf=TransformBroadcaster(node)
    truth=node.create_publisher(Odometry,'/talos/simulator/ground_truth',10)
    clock=node.create_publisher(Float64,'/talos/simulator/time',10)
    subscriptions=[]
    for key,topic,typ in [('rgb','ffc/zed_node/rgb/image_rect_color',Image),
                          ('jpeg','ffc/zed_node/rgb/image_rect_color/compressed',CompressedImage),
                          ('depth','ffc/zed_node/depth/depth_registered',Image),
                          ('info','ffc/zed_node/rgb/camera_info',CameraInfo),
                          ('lights','simulator/magnet_lights',MarkerArray)]:
        subscriptions.append(node.create_subscription(typ,'/talos/'+topic,lambda m,k=key:latest.update({k:m}),10))
    subscriptions.append(node.create_subscription(String,'/talos/simulator/task_events',lambda m:events.append(json.loads(m.data)),10))
    reset=node.create_client(Trigger,'/talos/simulator/reset_magnet_lights')
    reload=node.create_client(Trigger,'/talos/command/actuator/notify_reload')

    def emit():
        stamp=node.get_clock().now().to_msg();messages=[]
        optical=pose([0,0,0,-math.pi/2,0,-math.pi/2])
        for parent,child,t in [('map','simulator/talos/base_link',body),
                               ('simulator/talos/base_link','simulator/talos/ffc_camera_link',mount),
                               ('simulator/talos/ffc_camera_link','simulator/talos/ffc_left_camera_optical_frame',optical)]:
            m=TransformStamped();m.header.stamp=stamp;m.header.frame_id=parent;m.child_frame_id=child
            m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=map(float,t[:3,3])
            m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=map(float,quaternion(t[:3,:3]))
            messages.append(m)
        tf.sendTransform(messages)
        m=Odometry();m.header.stamp=stamp;m.header.frame_id='map';m.child_frame_id='talos/base_link'
        m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,body[:3,3])
        m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=map(float,quaternion(body[:3,:3]))
        truth.publish(m);m=Float64();m.data=current;clock.publish(m)

    def spin(seconds,simulate=False):
        nonlocal current
        for _ in range(math.ceil(seconds/.01)):
            end=time.monotonic()+.01
            if simulate:current+=.01
            emit()
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=max(0,end-time.monotonic()))

    def call(client):
        f=client.call_async(Trigger.Request());end=time.monotonic()+3
        while not f.done() and time.monotonic()<end:spin(.01)
        assert f.done() and f.result().success
        spin(.2)

    def state(key):
        m=next(m for m in latest['lights'].markers if m.ns==key)
        return 'green' if m.color.g>m.color.r else 'red'

    def view(key,distance,elevation=45.):
        nonlocal body
        frame=resolve(key);angle=math.radians(elevation)
        outward=frame[:3,:3]@np.array([math.cos(angle),0,math.sin(angle)])
        camera_world=np.eye(4);camera_world[:3,3]=model.faces[key][:3,3]+distance*outward
        camera_world[:3,0]=-outward
        camera_world[:3,1]=np.cross([0,0,1],-outward)
        camera_world[:3,1]/=np.linalg.norm(camera_world[:3,1])
        camera_world[:3,2]=np.cross(camera_world[:3,0],camera_world[:3,1])
        body=camera_world@np.linalg.inv(mount)
        spin(.4)

    def near(key,distance):
        nonlocal body
        body=pose([0,0,0,0,0,.3])
        body[:3,3]=model.sensors[key]+model.faces[key][:3,0]*distance-body[:3,:3]@model.tip
        spin(.15)

    report=[]
    def capture(key,color,distance,elevation=45.):
        view(key,distance,elevation);m=latest['rgb']
        assert (m.width,m.height)==(1920,1200), 'FFC must publish native HD1200'
        rgb=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        filename=args.output/f'{key}-{color}-{distance:.2f}m-{elevation:g}deg.png'
        cv2.imwrite(str(filename),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
        jpeg=filename.with_suffix('.jpg');jpeg.write_bytes(bytes(latest['jpeg'].data))
        # The centered ring must show the physical LED color in rendered RGB.
        info=latest['info'];u,v=round(info.k[2]),round(info.k[5]);radius=round(info.k[0]*.026/distance)
        patch=rgb[max(0,v-radius):v+radius,max(0,u-radius):u+radius].astype(float)
        channel=0 if color=='red' else 1
        assert (patch[:,:,channel]-patch[:,:,1-channel]).max()>75,filename
        depth=latest['depth'];z=np.frombuffer(depth.data,np.float32).reshape(depth.height,depth.step//4)
        assert abs(float(z[v,u])-distance)<.025,(filename,z[v,u],distance)
        report.append({'image':str(filename),'jpeg':str(jpeg),'target':key,'state':color,
                       'distance':distance,'elevation':elevation})

    params={'robot':'talos','headless':True,'point_cloud.enabled':False,
            'depth_model.enabled':False,'vehicle_config':str(vehicle_path),
            'task_config':str(task/'config/talos_tasks.yaml'),
            'ffc.config':str(Path(share('riptide_hardware2'))/'cfg/ffc_config.yaml'),
            'dfc.config':str(Path(share('riptide_hardware2'))/'cfg/dfc_config.yaml'),
            'shader_folder':str(camera/'shaders/pool'),'texture_folder':str(camera/'textures'),
            'riptide_mesh_folder':str(Path(share('riptide_meshes'))/'meshes'),
            'marker_config':str(Path(share('riptide_rviz'))/'config/markers.yaml'),
            'mapping_config':str(mapping),'scene_config':str(camera/'config/scene_info.yaml')}
    params_file=args.output/'viewer.yaml';params_file.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    command=['ros2','run','c_simulator','task_simulator.py','--ros-args','-r','__ns:=/talos']
    for k,v in {'robot':'talos','vehicle_config':vehicle_path,'task_config':task/'config/talos_tasks.yaml',
                'mapping_config':mapping,'hydrodynamics_config':task/'config/talos_hydrodynamics.yaml'}.items():command+=['-p',f'{k}:={v}']
    processes=[]
    with (args.output/'task.log').open('w') as task_log,(args.output/'viewer.log').open('w') as viewer_log:
        try:
            processes.append(subprocess.Popen(command,stdout=task_log,stderr=task_log,start_new_session=True))
            processes.append(subprocess.Popen([str(camera.parents[1]/'lib/camera_faker/pool_viewer'),'--ros-args','-r','__ns:=/talos','--params-file',str(params_file)],stdout=viewer_log,stderr=viewer_log,start_new_session=True))
            view('magnet_target1',.4);spin(3)
            assert all(p.poll() is None for p in processes)
            assert all(k in latest for k in ('rgb','jpeg','depth','info','lights')),list(latest)
            assert reset.wait_for_service(timeout_sec=2)
            for key in model.states:
                assert state(key)=='red'
                for elevation in args.elevations:
                    for distance in args.distances:capture(key,'red',distance,elevation)
                near(key,.153);spin(.6,True);spin(.2)
                assert state(key)=='red','Triggered outside six inches'
                near(key,.14);spin(.24,True);spin(.2)
                assert state(key)=='red'
                spin(.3);assert state(key)=='red','Wall time advanced the paused dwell'
                near(key,.2);spin(.02,True);near(key,.14)
                spin(.26,True);spin(.2);assert state(key)=='red','Interrupted dwell was accumulated'
                spin(.24,True);spin(.2);assert state(key)=='green','Did not latch within 0.5 seconds'
                near(key,.4);spin(.6,True);spin(.2);assert state(key)=='green'
                for elevation in args.elevations:
                    for distance in args.distances:capture(key,'green',distance,elevation)
            activated=[e for e in events if e.get('kind')=='magnet' and e.get('result')=='activated']
            assert len(activated)==2,activated
            call(reload);assert all(state(k)=='green' for k in model.states),'Ammo reload reset lights'
            call(reset);assert all(state(k)=='red' for k in model.states)
            near('magnet_target1',.14);spin(.5,True);spin(.2);assert state('magnet_target1')=='green'
            current=0.;spin(.3);assert all(state(k)=='red' for k in model.states),'Course reset did not reset lights'
            print('Passed tip proximity, both tilted lights, dwell/pause/interruption, latch, independent reset, course reset, rendered RGB and cover depth')
        finally:
            for p in processes:
                if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
            for p in processes:
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            node.destroy_node();rclpy.shutdown()
    if args.weights:
        from ultralytics import YOLO
        detector=YOLO(str(args.weights))
        for sample in report:
            # Perception consumes the published JPEG, including compression.
            result=detector.predict(sample['jpeg'],device='cpu',conf=.05,iou=.7,verbose=False)[0]
            sample['detections']=[{'class':detector.names[int(b.cls)],'confidence':float(b.conf),
                                   'xyxy':b.xyxy[0].tolist()} for b in result.boxes]
            print(Path(sample['image']).name,[(d['class'],round(d['confidence'],3)) for d in sample['detections']])
    (args.output/'report.json').write_text(json.dumps(report,indent=2))
    if args.weights:
        for sample in report:
            if sample['state']=='red' and sample['distance']==.4:
                assert any(d['class']=='magnet' and d['confidence']>=args.min_confidence
                           for d in sample['detections']),sample
        print('Passed unchanged-model red-light detection for both targets at 0.4 m')


if __name__=='__main__':main()
