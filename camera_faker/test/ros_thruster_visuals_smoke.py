#!/usr/bin/env python3
"""Preview-only ROS/render check. Requires DISPLAY and an isolated ROS_DOMAIN_ID.

Publishes synthetic realized forces and /clock, never hardware motor commands.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/tmp/talos-thruster-animation'))
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise RuntimeError('Use an isolated ROS_DOMAIN_ID')
    args.output.mkdir(parents=True, exist_ok=True)
    # Disable time-varying caustic illumination for comparisons of blade geometry.
    settings = args.output / 'settings.yaml'
    settings.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'lighting.brightness': 0., 'lighting.ambient': 1., 'lighting.glare': 0.,
        'water.scattering': .02, 'water.absorption': [0., 0., 0.],
    }}}))
    rclpy.init()
    node = rclpy.create_node('thruster_animation_check')
    clock = node.create_publisher(Clock, '/clock', 10)

    def capture(name, force, robot='talos'):
        namespace = 'rotorcheck_' + name
        topic = f'/{namespace}/simulator/actual_thruster_forces'
        publisher = node.create_publisher(Float32MultiArray, topic, 10)
        path = args.output / (name + '.png')
        log_path = args.output / (name + '.log')
        command = ['ros2', 'launch', 'camera_faker', 'pool_viewer.launch.py',
                   f'robot:={robot}', f'namespace:={namespace}',
                   'year:=2026' if robot == 'talos' else 'year:=example',
                   'demo:=true', 'headless:=true', 'use_sim_time:=true',
                   'exit_after_frames:=90', f'screenshot_path:={path}',
                   f'camera_settings:={settings}']
        first = None
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            deadline = time.monotonic() + 30
            try:
                while process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.015)
                    count = publisher.get_subscription_count()
                    if robot == 'example_auv':
                        assert count == 0, 'Unconfigured robot subscribed to thruster animation'
                    elif count and first is None:
                        first = time.monotonic()
                    elapsed = 0 if first is None else time.monotonic() - first
                    # Let startup and clock discovery settle, then advance 0.18 s
                    # and hold the clock for the final screenshot.
                    step = min(.18, max(0., elapsed - .7) * .4)
                    stamp = Clock()
                    stamp.clock.sec = 10
                    stamp.clock.nanosec = round(step * 1e9)
                    clock.publish(stamp)
                    if first is not None and elapsed > .3:
                        publisher.publish(Float32MultiArray(data=[float(force)] * 8))
                assert process.poll() == 0, log_path.read_text()
                assert robot == 'example_auv' or first is not None, 'No thruster animation subscriber'
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=10)
                node.destroy_publisher(publisher)
        image = cv2.imread(str(path))
        assert image is not None, path
        return image[235:450, 400:665].astype(np.int16)

    try:
        idle = capture('idle', 0)
        forward = capture('forward', 4)
        reverse = capture('reverse', -4)
        changes = []
        for a, b in ((idle, forward), (idle, reverse), (forward, reverse)):
            changed = int(np.count_nonzero(np.max(np.abs(a - b), axis=2) > 10))
            assert changed > 20, f'Blade poses did not visibly change: {changed} pixels'
            changes.append(changed)
        capture('unconfigured', 0, robot='example_auv')
        print('PASS: realized-force input changes rendered blade poses in both directions; optional robot opt-out')
        print('Changed vehicle pixels:', changes, 'Captures:', args.output)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
