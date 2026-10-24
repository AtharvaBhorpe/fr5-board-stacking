"""Check local ROS image transport between camera and motion pixi environments.

Uses synthetic images on an isolated localhost domain. Never contacts the robot.
"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def worker(role, settings):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node('perception_transport_' + role)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    ages = []
    expected_payload = bytes([42]) * (settings['width'] * settings['height'] * 3)
    deadline = time.monotonic() + settings['timeout_seconds']
    try:
        if role == 'subscribe':
            def received(message):
                if (message.header.frame_id != 'transport_test_optical' or
                        message.width != settings['width'] or message.height != settings['height'] or
                        message.encoding != 'rgb8' or message.step != message.width * 3 or
                        message.data.tobytes() != expected_payload):
                    raise ValueError('Synthetic image payload or frame was corrupted.')
                stamp = message.header.stamp
                ages.append((node.get_clock().now().nanoseconds - stamp.sec * 10**9 - stamp.nanosec) / 1e6)

            subscription = node.create_subscription(Image, settings['topic'], received, qos)
            while time.monotonic() < deadline and len(ages) < settings['frames']:
                rclpy.spin_once(node, timeout_sec=0.1)
            if len(ages) < settings['frames']:
                raise RuntimeError(f'Only received {len(ages)} of {settings["frames"]} images.')
            ordered = sorted(ages)
            report = {'frames': len(ages), 'image_size': [settings['width'], settings['height']],
                      'encoding': 'rgb8',
                      'median_age_ms': ordered[len(ordered) // 2],
                      'p95_age_ms': ordered[min(len(ordered) - 1, int(.95 * len(ordered)))],
                      'max_age_ms': max(ages), 'payload_verified': True}
            report['ok'] = min(ages) >= 0 and max(ages) <= settings['max_age_ms']
            print(json.dumps(report))
            return 0 if report['ok'] else 2
        publisher = node.create_publisher(Image, settings['topic'], qos)
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError('No motion-environment subscriber discovered.')
        message = Image(height=settings['height'], width=settings['width'], encoding='rgb8',
                        step=settings['width'] * 3, data=expected_payload)
        message.header.frame_id = 'transport_test_optical'
        for _ in range(settings['frames'] + 5):
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=1 / settings['fps'])
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    import tomllib

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/perception.toml')
    parser.add_argument('--role', choices=['publish', 'subscribe'])
    args = parser.parse_args()
    with open(args.config, 'rb') as source:
        settings = tomllib.load(source)['transport_test']
    if args.role:
        return worker(args.role, settings)
    # Do not inherit the laptop's sourced Jazzy workspace into either Humble process.
    env = os.environ.copy()
    for key in ('PYTHONPATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                'LD_LIBRARY_PATH', 'ROS_DISTRO', 'ROS_VERSION', 'ROS_PYTHON_VERSION', 'RMW_IMPLEMENTATION'):
        env.pop(key, None)
    env.update(ROS_DOMAIN_ID=str(settings['domain_id']), ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(Path(settings['log_directory']).resolve()))
    pixi = str(Path(settings['pixi_executable']).resolve())
    command = [pixi, 'run', '--frozen', '-e']
    script = str(Path(__file__).resolve())
    subscriber = subprocess.Popen(command + ['default', 'python', script, '--config', args.config,
                                             '--role', 'subscribe'], env=env,
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        publisher = subprocess.run(command + ['camera-ros', 'python', script, '--config', args.config,
                                              '--role', 'publish'], env=env, capture_output=True,
                                   text=True, timeout=settings['timeout_seconds'] + 10)
        output, errors = subscriber.communicate(timeout=settings['timeout_seconds'] + 10)
        print(output.strip())
        if publisher.returncode or subscriber.returncode:
            print(publisher.stderr + errors, file=sys.stderr)
            return 2
        return 0
    finally:
        if subscriber.poll() is None:
            subscriber.terminate()
            subscriber.communicate(timeout=5)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.TimeoutExpired) as exc:
        print(json.dumps({'ok': False, 'error': str(exc)}))
        sys.exit(2)
