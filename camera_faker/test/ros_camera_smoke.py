#!/usr/bin/env python3
"""Display-backed integration check. Run in an isolated ROS_DOMAIN_ID.

Uses a synthetic TF fixture, never c_simulator or hardware commands. Requires the
workspace to be built/sourced and DISPLAY to support OpenGL 3.3.
"""
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from ament_index_python.packages import get_package_share_directory as share
import yaml


def quaternion(roll, pitch, yaw):
    cr, cp, cy = [math.cos(v / 2) for v in (roll, pitch, yaw)]
    sr, sp, sy = [math.sin(v / 2) for v in (roll, pitch, yaw)]
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Set an isolated ROS_DOMAIN_ID'
    rclpy.init()
    node = Node('pool_viewer_camera_test')
    tf = TransformBroadcaster(node)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    records = {name: {} for name in ('ffc', 'dfc')}
    counts = {name: 0 for name in records}
    subscriptions = []
    types = {'rgb/image_rect_color': Image, 'left/image_rect_color': Image,
             'depth/depth_registered': Image, 'rgb/camera_info': CameraInfo,
             'left/camera_info': CameraInfo, 'depth/camera_info': CameraInfo,
             'rgb/image_rect_color/compressed': CompressedImage,
             'left/image_rect_color/compressed': CompressedImage,
             'point_cloud/cloud_registered': PointCloud2}

    def collect(name, channel, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        records[name].setdefault(stamp, {})[channel] = msg
        if channel == 'rgb/image_rect_color':
            counts[name] += 1
        # Only retain a few observations; RGB/depth frames are large.
        while len(records[name]) > 12:
            records[name].pop(next(iter(records[name])))

    for name in records:
        for channel, typ in types.items():
            subscriptions.append(node.create_subscription(
                typ, f'/talos/{name}/zed_node/{channel}',
                lambda msg, n=name, c=channel: collect(n, c, msg), 10))
    package = Path(share('camera_faker'))
    descriptions = Path(share('riptide_descriptions2'))
    hardware = Path(share('riptide_hardware2'))
    vehicle = yaml.safe_load((descriptions / 'config/talos.yaml').read_text())
    mounts = {c['name']: c['pose'] for c in vehicle['cameras']}
    fixture_xyz, fixture_yaw = (10, -10, -.6), 0.
    estimate_offset = np.array([1., -.5, .25])

    def emit():
        messages = []
        for parent, child, xyz, rpy in (
            ('map', 'simulator/talos/base_link', fixture_xyz, (0, 0, fixture_yaw)),
            ('map', 'talos/base_link', np.array(fixture_xyz) + estimate_offset, (0, 0, fixture_yaw)),
            ('talos/base_link', 'talos/ffc_camera_link',
             [mounts['ffc'][i]-vehicle['base_link'][i] for i in range(3)], mounts['ffc'][3:]),
            ('talos/base_link', 'talos/dfc_camera_link',
             [mounts['dfc'][i]-vehicle['base_link'][i] for i in range(3)], mounts['dfc'][3:]),
            ('simulator/talos/base_link', 'simulator/talos/ffc_camera_link',
             [mounts['ffc'][i]-vehicle['base_link'][i] for i in range(3)], mounts['ffc'][3:]),
            ('simulator/talos/ffc_camera_link', 'simulator/talos/ffc_left_camera_optical_frame',
             (0, 0, 0), (-math.pi/2, 0, -math.pi/2)),
        ):
            m = TransformStamped()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id, m.child_frame_id = parent, child
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, xyz)
            m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = quaternion(*rpy)
            messages.append(m)
        tf.sendTransform(messages)

    def spin(seconds, send_tf=False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if send_tf:
                emit()
            rclpy.spin_once(node, timeout_sec=.01)

    parameters = node.create_client(SetParametersAtomically,
                                    '/talos/pool_viewer/set_parameters_atomically')

    def configure(values):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(key, value=value).to_parameter_msg()
                              for key, value in values.items()]
        future = parameters.call_async(request)
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            emit()
            rclpy.spin_once(node, timeout_sec=.01)
        assert future.done(), 'Depth parameter update timed out'
        return future.result().result

    with tempfile.TemporaryDirectory(prefix='riptide-camera-test-') as temp:
        params = {'robot': 'talos', 'headless': True, 'depth_noise': 0., 'depth_model.enabled': False,
                  'show_tf': True,
                  'vehicle_config': str(descriptions / 'config/talos.yaml'),
                  'robot_model': '',
                  'payload_model': str(package / 'models/payloads/projectile.glb'),
                  'launcher_model': str(package / 'models/payloads/launcher.glb'),
                  'task_config': str(package / 'config/talos_tasks.yaml'),
                  'ffc.config': str(hardware / 'cfg/ffc_config.yaml'),
                  'dfc.config': str(hardware / 'cfg/dfc_config.yaml'),
                  'shader_folder': str(package / 'shaders/pool'),
                  'texture_folder': str(package / 'textures'),
                  'riptide_mesh_folder': str(Path(share('riptide_meshes')) / 'meshes'),
                  'marker_config': str(Path(share('riptide_rviz')) / 'config/markers.yaml'),
                  'mapping_config': str(package / 'config/simulation.yaml'),
                  'scene_config': str(package / 'config/scene_info.yaml')}
        config = Path(temp) / 'test.yaml'
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        with (Path(temp) / 'viewer.log').open('w+') as log:
            executable = package.parents[1] / 'lib/camera_faker/pool_viewer'
            process = subprocess.Popen([str(executable), '--ros-args', '-r', '__ns:=/talos',
                                        '--params-file', str(config)], stdout=log, stderr=log)
            try:
                spin(2)
                assert process.poll() is None, 'Viewer failed during startup'
                assert not any(counts.values()), 'Published sensors without physics TF'
                start = time.monotonic()
                spin(4, True)
                original_depth = {}

                def check_camera_tree(name):
                    frame = f'talos/{name}_left_camera_optical_frame'
                    frames = yaml.safe_load(buffer.all_frames_as_yaml())
                    assert frames[frame]['parent'] == f'talos/{name}_camera_link', frames[frame]
                    estimated = buffer.lookup_transform('map', frame, rclpy.time.Time()).transform.translation
                    actual = buffer.lookup_transform('map', 'simulator/' + frame, rclpy.time.Time()).transform.translation
                    np.testing.assert_allclose(
                        [estimated.x-actual.x, estimated.y-actual.y, estimated.z-actual.z],
                        estimate_offset, atol=1e-6)

                for name in records:
                    complete = [m for m in records[name].values() if len(m) == len(types)]
                    assert complete, f'{name}: no observation with synchronized RGB/depth/info/cloud/compressed'
                    m = complete[-1]
                    rgb, depth, info = [m[k] for k in ('rgb/image_rect_color', 'depth/depth_registered', 'rgb/camera_info')]
                    assert rgb.encoding == 'rgb8' and depth.encoding == '32FC1'
                    assert (rgb.width, rgb.height) == (1920, 1200)
                    assert len(rgb.data) == rgb.height*rgb.step
                    assert len(depth.data) == depth.height*depth.step
                    assert (info.width, info.height) == (rgb.width, rgb.height)
                    assert rgb.data == m['left/image_rect_color'].data
                    assert all(v.header.frame_id == f'talos/{name}_left_camera_optical_frame' for v in m.values())
                    check_camera_tree(name)
                    bgr = cv2.imdecode(np.asarray(m['rgb/image_rect_color/compressed'].data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    raw = np.asarray(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, 3)
                    assert np.abs(bgr[:, :, ::-1].astype(float)-raw).mean() < 8, 'Compressed color order mismatch'
                    cloud = m['point_cloud/cloud_registered']
                    assert (cloud.width, cloud.height) == ((rgb.width+7)//8, (rgb.height+7)//8)
                    assert cloud.row_step == cloud.width*cloud.point_step
                    d = np.frombuffer(depth.data, dtype='<f4').reshape(depth.height, depth.width)
                    original_depth[name] = d.copy()
                    cloud_xyz = np.ndarray((cloud.height, cloud.width, 3), dtype='<f4', buffer=bytes(cloud.data), strides=(cloud.row_step, cloud.point_step, 4))
                    np.testing.assert_allclose(cloud_xyz[:, :, 2], d[::8, ::8], atol=1e-6)
                    ys, xs = np.mgrid[0:rgb.height:8, 0:rgb.width:8]
                    np.testing.assert_allclose(cloud_xyz[:, :, 0], (xs-info.k[2])*d[::8, ::8]/info.k[0], atol=1e-5)
                    if name == 'dfc':
                        optical = buffer.lookup_transform('map', 'simulator/talos/dfc_left_camera_optical_frame', rclpy.time.Time())
                        expected = (-2.1336-optical.transform.translation.z)/(-math.sin(mounts['dfc'][4]))
                        actual = float(d[round(info.k[5]), round(info.k[2])])
                        assert abs(actual-expected) < .01, f'DFC depth: {actual} vs floor intersection {expected}'
                    print(f'{name}: all 9 topics synchronized; {counts[name]/(time.monotonic()-start):.1f} RGB Hz')
                assert records['ffc'] != records['dfc']
                # Changing only the estimated pose must move the TF used by all
                # sensor headers, without changing the camera-space observation.
                estimate_offset += np.array([.7, .2, -.1])
                for values in records.values():
                    values.clear()
                spin(1., True)
                for name in records:
                    check_camera_tree(name)
                    complete = [m for m in records[name].values() if len(m) == len(types)]
                    assert complete, f'{name}: missing observations after estimated pose shift'
                    m = complete[-1]
                    assert all(v.header.frame_id == f'talos/{name}_left_camera_optical_frame' for v in m.values())
                    d = np.frombuffer(m['depth/depth_registered'].data, dtype='<f4').reshape(original_depth[name].shape)
                    np.testing.assert_allclose(d, original_depth[name], atol=1e-6)
                print('RGB, depth, camera info and clouds follow estimated camera TF; rendering remains tied to physical pose')
                assert configure({'depth_model.enabled': True,
                                  'depth_model.base_sigma': 0., 'depth_noise': 0.,
                                  'depth_model.bias': .1, 'depth_model.dropout': 0.,
                                  'depth_model.range_dropout': 0., 'depth_model.edge_dropout': 0.,
                                  'depth_model.outliers': 0.}).successful
                records['dfc'].clear(); spin(1., True)
                complete = [m for m in records['dfc'].values() if len(m) == len(types)]
                assert complete, 'No synchronized depth after runtime parameter update'
                m = complete[-1]; depth = m['depth/depth_registered']; info = m['rgb/camera_info']
                d = np.frombuffer(depth.data, dtype='<f4').reshape(depth.height, depth.width)
                assert abs(float(d[round(info.k[5]), round(info.k[2])])-expected-.1) < .01
                cloud = m['point_cloud/cloud_registered']
                xyz = np.ndarray((cloud.height, cloud.width, 3), dtype='<f4', buffer=bytes(cloud.data), strides=(cloud.row_step, cloud.point_step, 4))
                np.testing.assert_allclose(xyz[:, :, 2], d[::8, ::8], atol=1e-6)
                assert not configure({'depth_model.min_range': 9., 'depth_model.max_range': 8.}).successful
                assert configure({'depth_model.dropout': 1.}).successful
                # The worker may finish a frame captured before the parameter
                # response. Wait for the first synchronized frame using the new model.
                records['dfc'].clear()
                deadline=time.monotonic()+4
                complete=[]
                while not complete and time.monotonic()<deadline:
                    spin(.1,True)
                    complete=[m for m in records['dfc'].values() if len(m)==len(types)
                              and np.isnan(np.frombuffer(m['depth/depth_registered'].data,dtype='<f4')).all()]
                assert complete, 'No frame applied the updated dropout model'
                m = complete[-1]
                cloud = m['point_cloud/cloud_registered']
                xyz = np.ndarray((cloud.height, cloud.width, 3), dtype='<f4', buffer=bytes(cloud.data), strides=(cloud.row_step, cloud.point_step, 4))
                assert np.isnan(xyz).all(), 'Cloud did not preserve depth dropouts'
                assert configure({'depth_model.enabled': False}).successful
                print('Passed live depth tuning, bias, dropout/cloud consistency and invalid-range rejection')
                def water_observation(tint):
                    assert configure({'water.tint': tint, 'water.scattering': .5}).successful
                    records['dfc'].clear(); spin(.7, True)
                    frames=[m for m in records['dfc'].values() if len(m)==len(types)]
                    assert frames
                    rgb=frames[-1]['rgb/image_rect_color']; depth=frames[-1]['depth/depth_registered']
                    return (np.asarray(rgb.data,dtype=np.uint8).reshape(rgb.height,rgb.width,3),
                            np.frombuffer(depth.data,dtype='<f4').copy())
                blue,blue_depth=water_observation([.01,.08,.8])
                red,red_depth=water_observation([.8,.08,.01])
                difference=(red.astype(float)-blue).mean(axis=(0,1))
                assert difference[0]>10 and difference[2]<-10, 'Water tint did not reach ROS RGB'
                np.testing.assert_allclose(blue_depth,red_depth,atol=1e-6)
                assert not configure({'water.absorption': [-.1,.1,.1]}).successful
                assert not configure({'water.tint': [.1,.2]}).successful
                assert configure({'water.tint': [.025,.22,.29], 'water.scattering': .1}).successful
                print('Passed live water tint in RGB, unchanged metric depth and invalid optics rejection')
                spin(1.5)
                paused = counts.copy()
                spin(.7)
                assert counts == paused, 'Continued publishing stale vehicle pose'
                spin(1.2, True)
                assert all(counts[k] > paused[k] for k in counts), 'Did not recover when physics TF resumed'
                print('Passed missing/stale TF gating and reconnection; DFC geometry and TF verified')
                # Inspect the actual published CAD-origin transform at opposite
                # headings. The configured CAD base point must remain on the
                # physics base link, rather than orbiting an extra scene offset.
                saved_yaw = fixture_yaw
                for fixture_yaw in (0., math.pi):
                    spin(.4, True)
                    origin = buffer.lookup_transform('map', 'simulator/talos/origin', rclpy.time.Time()).transform
                    local = buffer.lookup_transform('simulator/talos/base_link', 'simulator/talos/origin', rclpy.time.Time()).transform
                    np.testing.assert_allclose([local.translation.x, local.translation.y, local.translation.z],
                                               -np.asarray(vehicle['base_link']), atol=1e-6)
                    c, s = math.cos(fixture_yaw), math.sin(fixture_yaw)
                    rotate = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                    point = np.array([origin.translation.x, origin.translation.y, origin.translation.z]) + rotate @ vehicle['base_link']
                    np.testing.assert_allclose(point, fixture_xyz, atol=1e-6)
                fixture_yaw = saved_yaw
                spin(.2, True)
                print('Passed CAD-origin/base-link coincidence at 0 and 180 degree yaw')
                # Independently project the shared torpedo hole centers into the
                # real FFC render. This catches hidden backing sheets that still
                # write depth after the textured front has been cut out.
                def rotation(rpy):
                    x,y,z,w = quaternion(*rpy)
                    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
                data = yaml.safe_load((package/'config/simulation.yaml').read_text())['/talos/riptide_mapping2']['ros__parameters']['init_data']
                def resolve(key):
                    if key in ('map','world'): return np.eye(4)
                    if key.endswith('_frame'): key=key[:-6]
                    entry=data[key]; p=entry['pose']; t=np.eye(4)
                    t[:3,:3]=rotation([0,0,math.radians(p.get('yaw',0))])
                    t[:3,3]=[p.get(c,0) for c in ('x','y','z')]
                    return resolve(entry['parent'])@t
                panel=resolve('torpedo')
                fixture_yaw=math.atan2(panel[1,0],panel[0,0])+math.pi
                body_rotation=rotation([0,0,fixture_yaw])
                camera_position=panel[:3,3]+2*panel[:3,0]
                fixture_xyz=camera_position-body_rotation@(np.array(mounts['ffc'][:3])-vehicle['base_link'])
                camera_rotation=body_rotation@rotation(mounts['ffc'][3:])
                records['ffc'].clear(); spin(1.2, True)
                complete=[m for m in records['ffc'].values() if len(m)==len(types)]
                assert complete
                m=complete[-1]; depth=m['depth/depth_registered']; info=m['rgb/camera_info']
                d=np.frombuffer(depth.data,dtype='<f4').reshape(depth.height,depth.width)
                def sample(yz):
                    local=camera_rotation.T@((panel@np.r_[0.,yz,1])[:3]-camera_position)
                    x=round(info.k[2]-info.k[0]*local[1]/local[0])
                    y=round(info.k[5]-info.k[4]*local[2]/local[0])
                    return float(d[y,x]),float(local[0])
                task=yaml.safe_load((package/'config/talos_tasks.yaml').read_text())['torpedo']
                for hole in task['holes']:
                    actual,front=sample((np.array(hole['uv'])-.5)*2*task['panel_half_size'])
                    assert not math.isfinite(actual) or actual>front+.15, f"{hole['name']}: solid depth behind opening"
                actual,front=sample([0,0])
                assert abs(actual-front)<.03, 'Missing solid vinyl depth'
                print('Passed rendered depth through all four torpedo openings and solid vinyl')
            except Exception:
                log.flush(); log.seek(0); print(log.read())
                raise
            finally:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
