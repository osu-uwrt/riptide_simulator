#!/usr/bin/env python3
"""Check one-shot origin alignment against the real EKF in an isolated domain."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory as share
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from transforms3d.quaternions import quat2mat
import xacro
import yaml


def matrix(t):
    result = np.eye(4)
    q = t.rotation
    result[:3, :3] = quat2mat([q.w, q.x, q.y, q.z])
    result[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return result


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated ROS domain'
    rclpy.init()
    node = rclpy.create_node('origin_alignment_regression')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    broadcaster = StaticTransformBroadcaster(node)
    truth = []
    subscription = node.create_subscription(
        Odometry, '/talos/simulator/ground_truth', lambda m: truth.append(m), 10)
    sim_reset = node.create_client(SetPose, '/talos/set_sim_pose')
    ekf_reset = node.create_client(SetPose, '/talos/set_pose')
    processes = []

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)

    def wait_for(predicate, message, seconds=8.):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            spin(.05)
            if predicate():
                return
        raise AssertionError(message)

    def call(client, request):
        assert client.wait_for_service(timeout_sec=5.), 'Reset service missing'
        future = client.call_async(request)
        wait_for(future.done, 'Reset service timed out')
        assert future.exception() is None

    def pose_request(x, y, z, yaw):
        request = SetPose.Request()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = node.get_clock().now().to_msg()
        pose = request.pose.pose.pose
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w, pose.orientation.z = math.cos(yaw / 2), math.sin(yaw / 2)
        for i in range(6):
            request.pose.pose.covariance[7 * i] = 1e-6
        return request

    # The headless plant publishes base_link; the viewer's simulator/origin is
    # base_link * (-vehicle.base_link), identical to the URDF's origin joint.
    vehicle = yaml.safe_load((Path(share('riptide_descriptions2')) / 'config/talos.yaml').read_text())
    base_to_origin = np.eye(4)
    base_to_origin[:3, 3] = -np.array(vehicle['base_link'])

    def errors():
        try:
            estimate = buffer.lookup_transform('map', 'talos/origin', rclpy.time.Time())
            actual = buffer.lookup_transform('map', 'simulator/talos/base_link', rclpy.time.Time())
            stamp = min(rclpy.time.Time.from_msg(t.header.stamp) for t in (estimate, actual))
            estimate = matrix(buffer.lookup_transform('map', 'talos/origin', stamp).transform)
            actual = matrix(buffer.lookup_transform('map', 'simulator/talos/base_link', stamp).transform) @ base_to_origin
            delta = np.linalg.inv(actual) @ estimate
            angle = math.acos(np.clip((np.trace(delta[:3, :3]) - 1) / 2, -1, 1))
            return np.linalg.norm(delta[:3, 3]), math.degrees(angle)
        except Exception:
            return math.inf, math.inf

    def aligned():
        distance, angle = errors()
        return distance < .025 and angle < 1.

    with tempfile.TemporaryDirectory(prefix='talos-origin-') as tmp:
        config = Path(tmp) / 'robot.yaml'
        model = xacro.process_file(str(Path(share('riptide_descriptions2')) / 'robots/talos.xacro')).toxml()
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': model}}}))
        with open('/tmp/talos-origin-alignment.log', 'w') as log:
            def start(command):
                process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
                processes.append(process)
                return process

            def start_ekf():
                return start(['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args',
                              '-r', '__node:=ekf_localization_node', '-r', '__ns:=/talos',
                              '--params-file', str(Path(share('riptide_hardware2')) / 'cfg/talos_ekf.yaml')])

            try:
                for package in ('robot_state_publisher', 'joint_state_publisher'):
                    start(['ros2', 'run', package, package, '--ros-args', '--params-file', str(config)])
                start(['ros2', 'launch', 'c_simulator', 'physics_simulator.launch.py',
                       'sensor_noise:=false', 'collisions:=false', 'with_tasks:=false'])
                wait_for(lambda: bool(truth), 'Physics did not start without an EKF')
                # Start navigation late, then delay its world transform too.
                start_ekf()
                spin(1.)
                t = TransformStamped()
                t.header.frame_id, t.child_frame_id = 'map', 'odom'
                t.transform.translation.x, t.transform.translation.y = 2., -3.
                t.transform.translation.z = .2
                t.transform.rotation.w, t.transform.rotation.z = math.cos(.3), math.sin(.3)
                broadcaster.sendTransform(t)
                wait_for(aligned, 'Startup did not align origins with a nonidentity map/odom transform')
                print('Delayed startup origin error (m, deg):', errors(), flush=True)

                # Introduce a deliberate estimator-only offset. A continuous
                # truth injection would erase it and conceal navigation drift.
                pose = truth[-1].pose.pose
                request = pose_request(pose.position.x + .5, pose.position.y, pose.position.z, 0.)
                request.pose.pose.pose.orientation = pose.orientation
                call(ekf_reset, request)
                spin(.7)
                assert errors()[0] > .4, 'Simulator continuously overwrote the real EKF'

                call(sim_reset, pose_request(5., -8., -1., .8))
                wait_for(aligned, 'Explicit simulator reset did not realign origins')
                print('Explicit reset origin error (m, deg):', errors(), flush=True)
                assert all(p.poll() is None for p in processes), 'A test process exited'
                log.flush()
                text = Path(log.name).read_text()
                assert text.count('Initialized EKF from simulator base_link') == 2, text[-4000:]
                print('Startup readiness, world-frame conversion, one-shot initialization and reset passed', flush=True)
            finally:
                for process in processes:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                for process in processes:
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
