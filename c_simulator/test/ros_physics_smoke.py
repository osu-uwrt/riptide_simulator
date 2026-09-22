#!/usr/bin/env python3
"""Live plant regression. Run built/sourced, in an isolated ROS_DOMAIN_ID.

No hardware nodes are launched. Tests reset, sensor timing, truthful TF,
propulsion delay, software kill, stale-command decay and pool alignment.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory as share
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Float64
from visualization_msgs.msg import MarkerArray
from riptide_msgs2.msg import KillSwitchReport
from robot_localization.srv import SetPose
from tf2_ros import Buffer, TransformListener
import xacro
import yaml


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Choose an isolated ROS domain'
    rclpy.init();node = Node('physics_regression')
    counts = {};latest = {};subs = []
    buffer = Buffer();listener = TransformListener(buffer, node)
    channels = {'imu': (Imu, 'vectornav/imu'), 'dvl': (TwistWithCovarianceStamped, 'dvl_twist'),
                'gyro': (TwistWithCovarianceStamped, 'gyro/twist'),
                'depth': (PoseWithCovarianceStamped, 'depth/pose'),
                'truth': (Odometry, 'simulator/ground_truth'),
                'actuator': (Float32MultiArray, 'simulator/actual_thruster_forces'),
                'time': (Float64, 'simulator/time'),
                'collision': (MarkerArray, 'simulator/collisionMarkers')}
    def collect(key, msg):
        counts[key] = counts.get(key, 0)+1;latest[key] = msg
    for key, (typ, topic) in channels.items():
        subs.append(node.create_subscription(typ, '/talos/'+topic, lambda m,k=key: collect(k,m), 20))
    command = node.create_publisher(Float32MultiArray, '/talos/thruster_forces', 10)
    kill = node.create_publisher(KillSwitchReport, '/talos/command/software_kill', 10)
    reset = node.create_client(SetPose, '/talos/set_sim_pose')
    def spin(duration, thrust=None, killed=False):
        deadline = time.monotonic()+duration;next_command=0
        while time.monotonic()<deadline:
            if thrust is not None and time.monotonic()>=next_command:
                report=KillSwitchReport();report.kill_switch_id=1;report.switch_asserting_kill=killed;kill.publish(report)
                force=Float32MultiArray();force.data=list(map(float,thrust));command.publish(force);next_command=time.monotonic()+.02
            rclpy.spin_once(node,timeout_sec=.005)
    processes=[]
    with tempfile.TemporaryDirectory(prefix='riptide-plant-') as tmp:
        config=Path(tmp)/'robot.yaml'
        description=xacro.process_file(str(Path(share('riptide_descriptions2'))/'robots/talos.xacro')).toxml()
        config.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':description}}}))
        log_path=Path('/tmp/riptide-physics-regression.log')
        with log_path.open('w+') as log:
            try:
                for cmd in (
                    ['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(config)],
                    # Sensor mounts are zero-range revolute joints in the vehicle
                    # description; match navigation.launch.py's joint publisher.
                    ['ros2','run','joint_state_publisher','joint_state_publisher','--ros-args','--params-file',str(config)],
                    ['ros2','launch','c_simulator','physics_simulator.launch.py','robot:=talos','sensor_noise:=false'],
                ):
                    processes.append(subprocess.Popen(cmd,stdout=log,stderr=log,start_new_session=True))
                assert reset.wait_for_service(timeout_sec=8), 'No simulator reset service'
                deadline = time.monotonic()+5
                while not all(k in latest for k in ('imu', 'dvl', 'gyro', 'depth', 'truth')) and time.monotonic()<deadline:
                    spin(.1)
                assert all(k in latest for k in ('imu', 'dvl', 'gyro', 'depth', 'truth')), f'Missing sensor streams: {list(latest)}'
                req=SetPose.Request();req.pose.header.frame_id='map';req.pose.pose.pose.position.x=10.;req.pose.pose.pose.position.y=-10.;req.pose.pose.pose.position.z=-1.;req.pose.pose.pose.orientation.w=1.
                future=reset.call_async(req);start=time.monotonic()
                while not future.done() and time.monotonic()-start<1:rclpy.spin_once(node,timeout_sec=.01)
                assert future.done(), 'Reset blocked waiting for an EKF'
                spin(.08)
                truth=latest['truth'];pose=truth.pose.pose
                assert abs(pose.position.x-10)<.01 and abs(pose.position.y+10)<.01 and abs(pose.position.z+1)<.01
                assert truth.header.frame_id=='map' and truth.child_frame_id=='simulator/talos/base_link'
                # TF and odometry arrive independently; compare the same
                # publication time so a pre-reset TF cannot fail this check.
                stamp=rclpy.time.Time.from_msg(truth.header.stamp)
                deadline=time.monotonic()+1
                while not buffer.can_transform('map','simulator/talos/base_link',stamp) and time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.01)
                t=buffer.lookup_transform('map','simulator/talos/base_link',stamp).transform
                assert abs(t.translation.z-pose.position.z)<.01
                assert buffer.can_transform('simulator/talos/base_link',
                                            'simulator/talos/ffc_left_camera_optical_frame',
                                            rclpy.time.Time()), 'Missing namespaced ground-truth camera TF'
                assert not buffer.can_transform('simulator/talos/base_link',
                                                'talos/ffc_left_camera_optical_frame',
                                                rclpy.time.Time()), 'Physics claimed the estimated camera optical frame'
                before=counts.copy();spin(2)
                observed={k:(counts[k]-before.get(k,0))/2 for k in ('imu','dvl','gyro','depth','truth')}
                print('Sensor/ground-truth rates:',observed)
                for key, target in [('imu',50),('dvl',8),('gyro',500),('depth',20),('truth',100)]:
                    assert abs(observed[key]-target)<max(2,target*.15), (key,observed[key])
                covariance=latest['dvl'].twist.covariance
                assert all(covariance[i]>0 for i in (0,7,14)) and all(covariance[i]==0 for i in (21,28,35))
                gyro=latest['gyro']
                assert gyro.header.frame_id=='talos/fog_link' and gyro.twist.covariance[35]>0
                assert abs(gyro.twist.twist.angular.z-latest['truth'].twist.twist.angular.z)<.002
                markers=latest['collision'].markers
                floors=[m for m in markers if abs(m.scale.x-50)<1e-5 and abs(m.scale.y-22.86)<1e-5 and abs(m.scale.z-1)<1e-5]
                assert floors, f'Missing pool floor collision box: {[(m.scale.x,m.scale.y,m.scale.z) for m in markers]}'
                floor=floors[0].pose.position
                np.testing.assert_allclose([floor.x,floor.y,floor.z],[11.43,-5.4864,-2.6336],atol=1e-4)
                force=np.zeros(8);force[0]=18
                spin(.08,force);assert latest['actuator'].data[0]<1, 'No actuator transport delay'
                spin(.6,force);assert 16<latest['actuator'].data[0]<=18.01
                spin(.1,force,killed=True);spin(.8)
                assert max(abs(x) for x in latest['actuator'].data)<.01, 'Kill did not clear propulsion'
                spin(.65,force);assert latest['actuator'].data[0]>15
                spin(1.2)
                assert max(abs(x) for x in latest['actuator'].data)<.01, 'Command watchdog did not expire'
                bad=Float32MultiArray();bad.data=[float('nan')]*8;command.publish(bad);spin(.1)
                pose=latest['truth'].pose.pose;q=pose.orientation
                assert all(math.isfinite(x) for x in [pose.position.x,pose.position.y,pose.position.z,q.x,q.y,q.z,q.w])
                assert abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<1e-9
                # Place the chassis partially into the floor: contact must
                # resolve penetration, not let the vehicle fall through.
                req.pose.pose.pose.position.z=-2.1
                future=reset.call_async(req)
                deadline=time.monotonic()+1
                while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
                assert future.done()
                spin(.2)
                assert -2.05<latest['truth'].pose.pose.position.z<-1.7, 'Floor penetration not resolved'
                print('Reset without EKF, frame alignment, covariance, motor lag, kill, watchdog and nonfinite-input checks passed')
            except Exception:
                log.flush();log.seek(0);print(log.read()[-10000:]);raise
            finally:
                for p in processes:
                    if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
                for p in processes:
                    try:p.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
