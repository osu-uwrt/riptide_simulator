#!/usr/bin/env python3
"""Exercise preview/native transitions and synchronized output on an isolated ROS graph."""
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from tf2_ros import TransformBroadcaster


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated ROS_DOMAIN_ID'
    rclpy.init()
    node = Node('camera_demand_check')
    broadcaster = TransformBroadcaster(node)
    log_path = Path('/tmp/riptide-camera-demand.log')
    received = {}
    subscriptions = []
    settings = node.create_client(SetParametersAtomically, '/talos/pool_viewer/set_parameters_atomically')

    def frames():
        stamp = node.get_clock().now().to_msg()
        transforms = []
        for child in ('base_link', 'ffc_camera_link'):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'map'
            t.child_frame_id = 'simulator/talos/' + child
            t.transform.translation.x = 3.
            t.transform.translation.y = -2.
            t.transform.translation.z = -.8
            t.transform.rotation.w = 1.
            transforms.append(t)
        broadcaster.sendTransform(transforms)

    timer = node.create_timer(.02, frames)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            assert process.poll() is None, log_path.read_text()[-4000:]

    def subscribe(camera, suffix, msg_type, key):
        def capture(msg):
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            received.setdefault(key, {})[stamp] = msg
            # Retain only a few full frames to avoid test-induced memory pressure.
            while len(received[key]) > 8:
                del received[key][next(iter(received[key]))]
        subscriptions.append(node.create_subscription(
            msg_type, f'/talos/{camera}/zed_node/{suffix}', capture, 1))

    def clear():
        for sub in subscriptions:
            node.destroy_subscription(sub)
        subscriptions.clear()
        received.clear()

    def reject_runtime_scales(values):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg() for name, value in values.items()]
        future = settings.call_async(request)
        end = time.monotonic() + 3
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
        assert future.done(), 'Camera scale update timed out'
        result = future.result().result
        assert not result.successful, "Startup-only scales accepted a runtime change"

    def check_images(width=1920, height=1200):
        shared = set(received.get('rgb', {})) & set(received.get('depth', {})) & set(received.get('info', {}))
        assert shared, 'RGB/depth/CameraInfo did not share acquisition stamps: ' + str({
            key: [(s, getattr(m, 'width', None)) for s, m in received.get(key, {}).items()]
            for key in ('rgb', 'depth', 'info')})
        for stamp in shared:
            info, depth = (received[k][stamp] for k in ('info', 'depth'))
            assert (info.width, info.height) == (depth.width, depth.height), 'Mixed resolutions in one acquisition'
        stamp = max(shared)
        age = node.get_clock().now().nanoseconds * 1e-9 - (stamp[0] + stamp[1] * 1e-9)
        assert age < 1., f'Camera output is stale by {age:.2f}s'
        info, depth, rgb = (received[k][stamp] for k in ('info', 'depth', 'rgb'))
        assert (info.width, info.height) == (width, height)
        assert (depth.width, depth.height, depth.encoding, depth.step) == (width, height, '32FC1', width * 4)
        assert len(depth.data) == depth.step * depth.height
        decoded = cv2.imdecode(np.frombuffer(rgb.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape == (height, width, 3)
        assert info.header.frame_id == depth.header.frame_id == rgb.header.frame_id
        return stamp

    def check_cloud(width=1920, height=1200):
        shared = set(received.get('cloud', {})) & set(received['depth'])
        assert shared, 'Cloud/depth timestamps do not match'
        stamp = max(shared)
        cloud = received['cloud'][stamp]
        depth = received['depth'][stamp]
        w, h = (width + 7) // 8, (height + 7) // 8
        assert (cloud.width, cloud.height) == (w, h)
        cloud_z = np.ndarray((h, w), dtype='<f4', buffer=bytes(cloud.data),
                             offset=8, strides=(cloud.row_step, cloud.point_step))
        depth_z = np.frombuffer(depth.data, dtype='<f4').reshape(height, width)[::8, ::8]
        np.testing.assert_allclose(cloud_z, depth_z, equal_nan=True)

    with log_path.open('w') as log:
        # Render the complete UI into a hidden window, just like a capture run.
        # Stop before the frame limit so this test need not write screenshots.
        process = subprocess.Popen([
            'ros2', 'launch', 'camera_faker', 'pool_viewer.launch.py',
            'headless:=true', 'exit_after_frames:=100000', 'profile:=true',
            'camera_settings:=' + str(Path(__file__).with_name('camera_scales.yaml')),
        ], stdout=log, stderr=log, start_new_session=True)
        try:
            spin(5)
            text = log_path.read_text()
            for camera in ('ffc', 'dfc'):
                assert f'{camera} render: 480x300 (side preview)' in text, text[-2000:]
            offset = len(text)
            subscribe('dfc', 'rgb/camera_info', CameraInfo, 'info')
            spin(2)
            assert received.get('info'), 'Metadata-only subscriber received nothing'
            assert next(reversed(received['info'].values())).width == 960, 'Startup YAML scale was not loaded'
            assert '1920x1200' not in log_path.read_text()[offset:], 'CameraInfo triggered a full render'
            reject_runtime_scales({'dfc.resolution_scale': 1.})
            reject_runtime_scales({'ffc.resolution_scale': .5})
            reject_runtime_scales({'camera_scale': .5})
            clear()
            for camera in ('dfc', 'ffc'):
                width, height = (960, 600) if camera == 'dfc' else (1920, 1200)
                subscribe(camera, 'rgb/camera_info', CameraInfo, 'info')
                subscribe(camera, 'rgb/image_rect_color/compressed', CompressedImage, 'rgb')
                subscribe(camera, 'depth/depth_registered', Image, 'depth')
                spin(7)
                check_images(width, height)
                assert f'{camera} render: {width}x{height} (sensor/primary)' in log_path.read_text()
                if camera == 'dfc':
                    subscribe(camera, 'point_cloud/cloud_registered', PointCloud2, 'cloud')
                    spin(3)
                    check_cloud(width, height)
                offset = len(log_path.read_text())
                clear()
                spin(2)
                assert f'{camera} render: 480x300 (side preview)' in log_path.read_text()[offset:]
            print('PASS: independent YAML startup scales, runtime changes rejected, preview transitions and synchronized RGB/depth/cloud')
            for line in log_path.read_text().splitlines():
                if 'Viewer ' in line:
                    print(line)
        finally:
            clear()
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
