#!/usr/bin/env python3
"""Check that hidden detection overlays still expire incoming markers.

Run with an isolated ROS_DOMAIN_ID and an OpenGL display after building camera_faker.
"""
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated ROS_DOMAIN_ID'
    rclpy.init()
    node = Node('detection_lifetime_check')
    publisher = node.create_publisher(
        MarkerArray, '/talos/yolo_orientation/visualization_marker_array', 10)
    next_id = 0

    with tempfile.TemporaryDirectory(prefix='riptide-detection-lifetime-') as folder:
        log_path = Path(folder) / 'viewer.log'
        with log_path.open('w') as log:
            process = subprocess.Popen([
                'ros2', 'launch', 'camera_faker', 'pool_viewer.launch.py',
                'headless:=true', 'profile:=true', 'detections:=false', 'use_sim_time:=false',
            ], stdout=log, stderr=log, start_new_session=True)

            def spin(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.01)
                    assert process.poll() is None, log_path.read_text()[-4000:]

            def counts(offset=0):
                return [int(value) for value in re.findall(
                    r'detection markers (\d+)', log_path.read_text()[offset:])]

            def publish_batch():
                nonlocal next_id
                message = MarkerArray()
                for _ in range(100):
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.ns = 'lifetime_stress'
                    marker.id = next_id
                    next_id += 1
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.orientation.w = 1.
                    marker.scale.x = marker.scale.y = marker.scale.z = .1
                    marker.lifetime.nanosec = 500_000_000
                    # Amplify retained memory so RSS growth is also easy to see.
                    marker.text = 'x' * 8192
                    message.markers.append(marker)
                publisher.publish(message)

            try:
                deadline = time.monotonic() + 30
                while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                    spin(.1)
                assert publisher.get_subscription_count() > 0, log_path.read_text()[-4000:]
                spin(3)
                offset = log_path.stat().st_size
                timer = node.create_timer(.05, publish_batch)
                try:
                    spin(12)
                finally:
                    node.destroy_timer(timer)
                active = counts(offset)
                drain_offset = log_path.stat().st_size
                spin(4)
                drained = counts(drain_offset)
                print(f'Published {next_id} unique markers; retained samples: {active}; '
                      f'after expiry: {drained}', flush=True)
                assert active and max(active) > 0, 'No markers observed by the viewer'
                assert max(active) <= 2000, 'Expired markers accumulated while the overlay was hidden'
                assert drained and drained[-1] == 0, 'Expired markers remain after publishing stops'
                print('PASS: hidden overlay expires markers during streaming and after the publisher stops')
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
