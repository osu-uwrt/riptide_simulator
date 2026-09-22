#!/usr/bin/env python3
"""Display-backed LED command/render check, including an unrelated robot.

Run after building/sourcing the workspace, in an isolated ROS_DOMAIN_ID.
Only preview viewers are started; no hardware or physics nodes are launched.
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
from riptide_msgs2.msg import LedCommand
from std_msgs.msg import ColorRGBA
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/tmp/talos-status-lights'))
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise RuntimeError('Run with an isolated ROS_DOMAIN_ID, e.g. 126')
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('status_light_render_check')

    def capture(name, command=None, robot='talos', config=None):
        namespace = 'ledcheck_' + name
        path = args.output / (name + '.png')
        log_path = args.output / (name + '.log')
        topic = f'/{namespace}/command/led'
        pub = node.create_publisher(type(command), topic, 10) if command is not None else None
        launch = ['ros2', 'launch', 'camera_faker', 'pool_viewer.launch.py',
                  f'robot:={robot}', f'namespace:={namespace}',
                  'year:=2026' if robot == 'talos' else 'year:=example',
                  'demo:=true', 'headless:=true', 'initial_focus:=Vehicle',
                  'exit_after_frames:=75', f'screenshot_path:={path}']
        if config:
            launch.append(f'status_lights_config:={config}')
        with log_path.open('w') as log:
            process = subprocess.Popen(launch, stdout=log, stderr=log, start_new_session=True)
            connected = False
            deadline = time.monotonic() + 30
            try:
                while process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.02)
                    if pub and pub.get_subscription_count():
                        connected = True
                        pub.publish(command)
                    if robot == 'example_auv' and config is None:
                        assert not node.get_subscriptions_info_by_topic(topic), 'Unconfigured robot subscribed to LEDs'
                assert process.poll() == 0, log_path.read_text()
                assert pub is None or connected, 'Configured LED subscriber was not discovered'
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=10)
                if pub:
                    node.destroy_publisher(pub)
        image = cv2.imread(str(path))
        assert image is not None, path
        # Observer viewport only: exclude camera previews and UI swatches.
        return image[120:550, 20:1040].astype(np.int16)

    def magenta(image):
        b, g, r = image.transpose(2, 0, 1)
        return int(np.count_nonzero((r > g + 30) & (b > g + 30)))

    def red(image):
        b, g, r = image.transpose(2, 0, 1)
        return int(np.count_nonzero((r > g + 30) & (r > b + 30)))

    try:
        off = capture('off')
        ccb = capture('ccb', LedCommand(red=255, blue=255, target=LedCommand.TARGET_CCB))
        all_lights = capture('all', LedCommand(red=255, blue=255, target=LedCommand.TARGET_ALL))
        alu = capture('alu', LedCommand(red=255, target=LedCommand.TARGET_ALU))
        assert magenta(ccb) <= magenta(off) + 5, 'CCB command affected ALU lights'
        assert magenta(all_lights) > magenta(off) + 20, 'No magenta light/glow visible'
        assert red(alu) > red(off) + 20, 'No red light/glow visible'
        capture('unconfigured', robot='example_auv')
        generic = args.output / 'generic.yaml'
        generic.write_text(yaml.safe_dump({
            'input': {'type': 'std_msgs/msg/ColorRGBA', 'topic': 'command/led'},
            'lights': [{'id': 'beacon', 'pose': [0, 0, .20, 0, 0, 0],
                        'size': [.10, .08, .02], 'radiance': 240}],
        }))
        color = capture('generic', ColorRGBA(r=1., a=1.), robot='example_auv', config=generic)
        assert red(color) > 20, 'Generic ColorRGBA light did not render'
        print('PASS: off, CCB filtering, ALL/ALU colors, clear hull/glow, opt-out robot and generic ColorRGBA')
        print('Captures:', args.output)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
