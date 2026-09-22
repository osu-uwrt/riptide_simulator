#!/usr/bin/env python3
"""Repeated score snapshots must replace, rather than retain, previous YAML trees.

Run on Linux with an isolated ROS_DOMAIN_ID and an OpenGL display.
"""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated ROS_DOMAIN_ID'
    rclpy.init()
    node = Node('score_memory_check')
    publisher = node.create_publisher(String, '/talos/simulator/run_score', 10)
    # Larger than a normal scorecard to expose retention in a short run.
    message = String(data=json.dumps({
        'total': 0, 'elapsed': 0, 'running': False,
        'rows': [{'label': f'Award {i}: ' + 'x' * 64, 'points': i} for i in range(128)],
    }))
    with tempfile.TemporaryDirectory(prefix='riptide-score-memory-') as folder:
        log_path = Path(folder) / 'viewer.log'
        with log_path.open('w') as log:
            process = subprocess.Popen([
                'ros2', 'launch', 'camera_faker', 'pool_viewer.launch.py',
                'headless:=true', 'use_sim_time:=false',
            ], stdout=log, stderr=log, start_new_session=True)

            def spin(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.01)
                    assert process.poll() is None, log_path.read_text()[-4000:]

            def rss_mib(pid):
                status = Path(f'/proc/{pid}/status').read_text()
                return int(re.search(r'VmRSS:\s+(\d+)', status).group(1)) / 1024

            try:
                deadline = time.monotonic() + 30
                while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                    spin(.1)
                assert publisher.get_subscription_count() > 0, log_path.read_text()[-4000:]
                pid = int(re.search(r'pool_viewer-\d+\]: process started with pid \[(\d+)\]',
                                    log_path.read_text()).group(1))
                timer = node.create_timer(.025, lambda: publisher.publish(message))
                try:
                    spin(4)  # Warm allocator and rendering caches before measuring.
                    before = rss_mib(pid)
                    spin(12)
                finally:
                    node.destroy_timer(timer)
                spin(1)
                after = rss_mib(pid)
                print(f'Viewer RSS after warmup: {before:.1f} MiB; after repeated scores: '
                      f'{after:.1f} MiB; growth: {after - before:.1f} MiB', flush=True)
                assert after - before < 16, 'Viewer retained previous score snapshots'
                print('PASS: repeated score updates keep viewer memory bounded')
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
