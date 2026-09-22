#!/usr/bin/env python3
"""Exercise the real bringup/controller/EKF against independent simulator truth.

Run in an isolated ROS_DOMAIN_ID after building/sourcing the workspace. Noise is
enabled by default. Holds, drives a closed XY path while turning, then holds
again. No truth is fed into the estimator or controller after the initial reset.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Float64
from riptide_msgs2.msg import ControllerCommand, KillSwitchReport
from robot_localization.srv import SetPose
from tf2_ros import Buffer, TransformListener
from transforms3d.quaternions import quat2mat
from transforms3d.euler import euler2mat
from transforms3d.axangles import mat2axangle
import yaml


def vector(v):
    return np.array([v.x, v.y, v.z])


def rotation(q):
    return quat2mat([q.w, q.x, q.y, q.z])


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=120.)
    parser.add_argument('--output', default='/tmp/riptide-navigation-drift.json')
    parser.add_argument('--report-only', action='store_true')
    parser.add_argument('--no-noise', action='store_true')
    parser.add_argument('--passive', action='store_true',
                        help='Let the unpowered vehicle float/tilt to test drift at rest')
    parser.add_argument('--distance', type=float, default=0.,
                        help='Drive this many meters with a test-only truth-feedback driver; no production controllers')
    parser.add_argument('--heading', type=float, default=0., help='Initial heading in degrees')
    parser.add_argument('--flip-axis', choices=['roll', 'pitch'],
                        help='Perform a full flip before the distance test')
    parser.add_argument('--launch-arg', action='append', default=[],
                        help='Additional simulation launch argument, e.g. sensor_config:=/tmp/sensors.yaml')
    parser.add_argument('--with-viewer', action='store_true',
                        help='Also run camera rendering and RViz to exercise desktop load')
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated ROS domain'
    assert args.duration >= 40
    assert args.distance >= 0 and not (args.passive and args.distance)
    assert not args.flip_axis or args.distance > 0, '--flip-axis requires --distance'
    rclpy.init(args=[])
    node = rclpy.create_node('navigation_drift_regression')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    samples = {key: [] for key in ('truth', 'ekf', 'dvl', 'gyro', 'imu')}
    subs = []
    for key, typ, topic in (
            ('truth', Odometry, 'simulator/ground_truth'),
            ('ekf', Odometry, 'odometry/filtered'),
            ('dvl', TwistWithCovarianceStamped, 'dvl_twist'),
            ('gyro', TwistWithCovarianceStamped, 'gyro/twist'),
            ('imu', Imu, 'vectornav/imu')):
        subs.append(node.create_subscription(
            typ, '/talos/' + topic, lambda m, k=key: samples[k].append(m), 100))
    linear = node.create_publisher(ControllerCommand, '/talos/controller/linear', 10)
    angular = node.create_publisher(ControllerCommand, '/talos/controller/angular', 10)
    kill = node.create_publisher(KillSwitchReport, '/talos/command/software_kill', 10)
    thrust = node.create_publisher(Float32MultiArray, '/talos/thruster_forces', 10)
    simulation_times = []
    subs.append(node.create_subscription(Float64, '/talos/simulator/time',
                lambda m: simulation_times.append((node.get_clock().now().nanoseconds * 1e-9, m.data)), 100))
    heading = math.radians(args.heading)
    desired_rotation = euler2mat(0, 0, heading)
    if args.distance:
        vehicle = yaml.safe_load((Path(get_package_share_directory('riptide_descriptions2')) / 'config/talos.yaml').read_text())
        columns = []
        for thruster in vehicle['thrusters']:
            p = thruster['pose']
            direction = euler2mat(*p[3:])[:, 0]
            columns.append(np.r_[direction, np.cross(np.array(p[:3])-vehicle['com'], direction)])
        allocation = np.linalg.pinv(np.array(columns).T)
    reset = node.create_client(SetPose, '/talos/set_sim_pose')
    origin = np.array([5., -8., -1.])
    output = Path(args.output)
    log = output.with_suffix('.log').open('w')
    simulation_args = ['robot:=talos', 'with_camera_faker:=' + str(args.with_viewer),
                       'with_rviz:=' + str(args.with_viewer),
                       'sensor_noise:=' + ('false' if args.no_noise else 'true')] + args.launch_arg
    if args.passive or args.distance:
        # These comparisons need only navigation and physics. In particular,
        # do not launch controllers that persist learned gains to shared files.
        commands = [
            # Navigation on the simulator's /clock, as simulation.launch.py runs it.
            ['ros2', 'launch', 'c_simulator', 'sim_navigation.launch.py', 'robot:=talos'],
            ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '0', '0', '0', '0', '0', '0', 'map', 'world'],
            ['ros2', 'launch', 'c_simulator', 'full_simulator.launch.py'] + simulation_args,
        ]
    else:
        commands = [['ros2', 'launch', 'riptide_bringup2', 'simulation.launch.py'] + simulation_args]
    processes = [subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
                 for command in commands]
    try:
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            assert all(p.poll() is None for p in processes), 'Bringup exited; see ' + str(log.name)
            if (all(samples[k] for k in ('truth', 'ekf', 'dvl'))
                    and reset.service_is_ready() and (args.passive or args.distance or linear.get_subscription_count())
                    and buffer.can_transform('map', 'odom', rclpy.time.Time())):
                break
        else:
            raise AssertionError('Navigation/controller did not start; see ' + str(log.name))
        request = SetPose.Request()
        request.pose.header.frame_id = 'map'
        p = request.pose.pose.pose.position
        p.x, p.y, p.z = origin.tolist()
        request.pose.pose.pose.orientation.w = math.cos(heading / 2)
        request.pose.pose.pose.orientation.z = math.sin(heading / 2)
        # The simulator forwards set_pose asynchronously; require both the TF
        # tree and EKF service to be ready, then verify the reset took effect.
        ekf_reset = node.create_client(SetPose, '/talos/set_pose')
        assert ekf_reset.wait_for_service(timeout_sec=5), 'EKF reset service missing'
        for attempt in range(5):
            future = reset.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            assert future.done(), 'Simulator reset timed out'
            until = time.monotonic() + 1
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.01)
            transform = buffer.lookup_transform('map', 'odom', rclpy.time.Time()).transform
            estimate = rotation(transform.rotation) @ vector(samples['ekf'][-1].pose.pose.position) + vector(transform.translation)
            if np.linalg.norm(estimate - vector(samples['truth'][-1].pose.pose.position)) < .1:
                break
        else:
            raise AssertionError('Simulator/EKF reset did not align initial poses')
        for values in samples.values():
            values.clear()
        simulation_times.clear()
        start = time.monotonic()
        next_command = start
        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            if args.distance and not samples['truth']:
                rclpy.spin_once(node, timeout_sec=.005)
                continue
            if now >= next_command and not args.passive:
                assert all(p.poll() is None for p in processes), 'Bringup exited during motion; see ' + str(log.name)
                elapsed = now - start
                if args.distance:
                    # Long translation exposes scale/heading errors that a
                    # stationary run or a small closed loop can conceal.
                    drive_start = .50 if args.flip_axis else .15
                    drive_duration = .80 - drive_start
                    phase = np.clip((elapsed / args.duration - drive_start) / drive_duration, 0., 1.)
                    target = origin + desired_rotation[:, 0] * args.distance * phase
                    state = samples['truth'][-1]
                    rot = rotation(state.pose.pose.orientation)
                    velocity = rot @ vector(state.twist.twist.linear)
                    target_velocity = (desired_rotation[:, 0] * args.distance / (drive_duration * args.duration)
                                       if 0 < phase < 1 else np.zeros(3))
                    force = rot.T @ (35 * (target-vector(state.pose.pose.position)) +
                                     50 * (target_velocity-velocity) + np.array([0., 0., -3.]))
                    target_rotation = desired_rotation
                    if args.flip_axis:
                        angle = np.clip((elapsed / args.duration - .1) / .35, 0., 1.) * 2 * math.pi
                        target_rotation = desired_rotation @ euler2mat(
                            angle if args.flip_axis == 'roll' else 0.,
                            angle if args.flip_axis == 'pitch' else 0., 0.)
                    axis, angle = mat2axangle(rot.T @ target_rotation)
                    torque = 30 * axis * angle - 10 * vector(state.twist.twist.angular)
                    msg = Float32MultiArray()
                    msg.data = np.clip(allocation @ np.r_[force, torque], -20, 20).tolist()
                    report = KillSwitchReport(); report.kill_switch_id = 1; report.switch_asserting_kill = False
                    kill.publish(report); thrust.publish(msg)
                    next_command = now + .02
                    rclpy.spin_once(node, timeout_sec=.005)
                    continue
                # 25% hold, 50% closed path, 25% final hold.
                phase = np.clip((elapsed / args.duration - .25) * 2, 0., 1.) * 2 * math.pi
                target = origin + np.array([.75 * (1 - math.cos(phase)), .75 * math.sin(phase), 0.])
                yaw = .7 * math.sin(phase)
                report = KillSwitchReport()
                report.kill_switch_id = 1
                report.switch_asserting_kill = False
                kill.publish(report)
                lin = ControllerCommand()
                lin.mode = ControllerCommand.POSITION
                lin.setpoint_vect.x, lin.setpoint_vect.y, lin.setpoint_vect.z = target.tolist()
                ang = ControllerCommand()
                ang.mode = ControllerCommand.POSITION
                ang.setpoint_quat.w = math.cos(yaw / 2)
                ang.setpoint_quat.z = math.sin(yaw / 2)
                linear.publish(lin)
                angular.publish(ang)
                next_command = now + .05
            rclpy.spin_once(node, timeout_sec=.005)
        transform = buffer.lookup_transform('map', 'odom', rclpy.time.Time()).transform
        world_rotation, world_offset = rotation(transform.rotation), vector(transform.translation)
        truth = samples['truth']
        times = np.array([stamp(m) for m in truth])
        positions = np.array([vector(m.pose.pose.position) for m in truth])
        errors, yaw_errors, attitude_errors, elapsed = [], [], [], []
        for m in samples['ekf']:
            t = stamp(m)
            if not times[0] <= t <= times[-1]:
                continue
            pos = np.array([np.interp(t, times, positions[:, i]) for i in range(3)])
            errors.append(pos - (world_rotation @ vector(m.pose.pose.position) + world_offset))
            nearest = truth[np.argmin(abs(times - t))]
            delta = (world_rotation @ rotation(m.pose.pose.orientation)).T @ rotation(nearest.pose.pose.orientation)
            yaw_errors.append(math.degrees(math.atan2(delta[1, 0], delta[0, 0])))
            attitude_errors.append(math.degrees(math.acos(np.clip((np.trace(delta)-1)/2, -1., 1.))))
            elapsed.append(t - times[0])
        errors = np.array(errors)
        horizontal = np.linalg.norm(errors[:, :2], axis=1)
        final_hold = np.array(elapsed) > args.duration * .85
        # Independently check the velocity delivered by the DVL, including its
        # mounting rotation and the omega x lever-arm term, against truth.
        mount = buffer.lookup_transform('talos/base_link', 'talos/dvl_link', rclpy.time.Time()).transform
        dvl_errors = []
        for m in samples['dvl']:
            i = np.argmin(abs(times - stamp(m)))
            if abs(times[i] - stamp(m)) > .03:
                continue
            twist = truth[i].twist.twist
            expected = rotation(mount.rotation).T @ (
                vector(twist.linear) + np.cross(vector(twist.angular), vector(mount.translation)))
            dvl_errors.append(vector(m.twist.twist.linear) - expected)
        dvl_errors = np.array(dvl_errors)
        # Compare FOG rates with the true rate at the physical sensor mount.
        angular = np.array([vector(m.twist.twist.angular) for m in truth])
        fog = buffer.lookup_transform('talos/base_link', 'talos/fog_link', rclpy.time.Time()).transform
        gyro_errors = []
        for m in samples['gyro']:
            t = stamp(m)
            if times[0] <= t <= times[-1]:
                omega = np.array([np.interp(t, times, angular[:, i]) for i in range(3)])
                expected = rotation(fog.rotation).T @ omega
                gyro_errors.append(m.twist.twist.angular.z - expected[2])
        # Verify gravity removal against independent rigid-body acceleration at
        # the IMU, including its lever arm. Only use the final settled interval;
        # a gravity mismatch appears even when all sample noise is disabled.
        imu_mount = buffer.lookup_transform('talos/base_link', samples['imu'][-1].header.frame_id, rclpy.time.Time()).transform
        # The default sensor profile must not inject a steadily growing heading
        # error. Check the actual IMU messages separately from the FOG-only EKF.
        imu_heading_times, imu_heading_errors = [], []
        for m in samples['imu']:
            t = stamp(m)
            if times[0] <= t <= times[-1]:
                i = np.argmin(abs(times - t))
                expected = rotation(truth[i].pose.pose.orientation) @ rotation(imu_mount.rotation)
                delta = rotation(m.orientation) @ expected.T
                imu_heading_times.append(t - times[0])
                imu_heading_errors.append(math.atan2(delta[1, 0], delta[0, 0]))
        imu_heading_drift = math.degrees(np.polyfit(
            imu_heading_times, np.unwrap(imu_heading_errors), 1)[0]) * 60
        velocities = np.array([rotation(m.pose.pose.orientation) @ (
            vector(m.twist.twist.linear) + np.cross(vector(m.twist.twist.angular), vector(imu_mount.translation)))
            for m in truth])
        acceleration_z = np.gradient(velocities[:, 2], times)
        ekf_path = Path(get_package_share_directory('riptide_hardware2')) / 'cfg/talos_ekf.yaml'
        gravity = yaml.safe_load(ekf_path.read_text())['/**/ekf_localization_node']['ros__parameters']['gravitational_acceleration']
        gravity_errors = []
        for m in samples['imu']:
            t = stamp(m)
            if times[0] + args.duration * .85 <= t <= times[-1]:
                measured_z = (rotation(m.orientation) @ vector(m.linear_acceleration))[2] - gravity
                gravity_errors.append(measured_z - np.interp(t, times, acceleration_z))
        metrics = {
            'duration_s': args.duration,
            'noise_enabled': not args.no_noise,
            'viewer_enabled': args.with_viewer,
            'passive': args.passive,
            'requested_distance_m': args.distance,
            'heading_deg': args.heading,
            'flip_axis': args.flip_axis,
            'sim_to_wall_time_ratio': ((simulation_times[-1][1]-simulation_times[0][1]) /
                                       (simulation_times[-1][0]-simulation_times[0][0])),
            'message_rates_hz': {k: len(v) / args.duration for k, v in samples.items()},
            'horizontal_error_max_m': float(horizontal.max()),
            'horizontal_error_rms_m': float(np.sqrt(np.mean(horizontal ** 2))),
            'horizontal_error_final_m': float(horizontal[-1]),
            'horizontal_error_final_xyz_m': errors[-1].tolist(),
            'vertical_error_max_m': float(np.abs(errors[:, 2]).max()),
            'yaw_error_rms_deg': float(np.sqrt(np.mean(np.square(yaw_errors)))),
            'yaw_error_final_deg': float(yaw_errors[-1]),
            'attitude_error_max_deg': float(max(attitude_errors)),
            'attitude_error_final_deg': float(attitude_errors[-1]),
            'truth_xy_span_m': np.ptp(positions[:, :2], axis=0).tolist(),
            'final_hold_error_change_m': float(np.linalg.norm(errors[final_hold][-1, :2] - errors[final_hold][0, :2])),
            'dvl_velocity_bias_mps': np.mean(dvl_errors, axis=0).tolist(),
            'dvl_velocity_rms_error_mps': np.sqrt(np.mean(dvl_errors ** 2, axis=0)).tolist(),
            'fog_rate_bias_radps': float(np.mean(gyro_errors)),
            'fog_rate_rms_error_radps': float(np.sqrt(np.mean(np.square(gyro_errors)))),
            'fog_reported_variance_radps2': float(samples['gyro'][-1].twist.covariance[35]),
            'imu_heading_drift_deg_per_min': float(imu_heading_drift),
            'imu_gravity_residual_mps2': float(np.mean(gravity_errors)),
        }
        output.write_text(json.dumps(metrics, indent=2) + '\n')
        np.savez_compressed(output.with_suffix('.npz'), elapsed=elapsed, errors=errors,
                            yaw_errors=yaw_errors, attitude_errors=attitude_errors,
                            truth_time=times, truth_position=positions,
                            truth_orientation=np.array([[m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                                                         m.pose.pose.orientation.y, m.pose.pose.orientation.z] for m in truth]))
        print(json.dumps(metrics, indent=2), flush=True)
        if not args.report_only:
            assert metrics['message_rates_hz']['gyro'] > 40, 'Missing gyro stream'
            if args.distance:
                assert np.linalg.norm(positions[-1, :2]-positions[0, :2]) > args.distance * .8, 'Test driver did not travel requested distance'
            elif not args.passive:
                assert min(metrics['truth_xy_span_m']) > .5, 'Controller did not exercise XY motion'
            assert metrics['horizontal_error_max_m'] < .20, 'Excessive position drift'
            assert metrics['vertical_error_max_m'] < .10, 'Excessive depth error'
            assert metrics['yaw_error_rms_deg'] < 1., 'Heading estimate disagrees with truth'
            assert metrics['final_hold_error_change_m'] < .03, 'Position drifts while holding'
            assert max(metrics['dvl_velocity_rms_error_mps']) < .008, 'DVL disagrees with physical sensor velocity'
            assert abs(metrics['fog_rate_bias_radps']) < .00002, 'FOG has a systematic rate bias'
            if not args.no_noise:
                # Allow timing/interpolation error, but catch covariance that
                # makes the EKF distrust an otherwise precise simulated FOG.
                fog_variance_ratio = (metrics['fog_rate_rms_error_radps'] ** 2 /
                                      metrics['fog_reported_variance_radps2'])
                assert .25 < fog_variance_ratio < 4., 'FOG covariance disagrees with measured noise'
            assert abs(metrics['imu_heading_drift_deg_per_min']) < .15, 'IMU has an imposed heading ramp'
            assert abs(metrics['imu_gravity_residual_mps2']) < .01, 'Simulated gravity disagrees with EKF calibration'
            print('Navigation drift regression passed', flush=True)
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
