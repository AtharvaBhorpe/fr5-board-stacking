"""On-demand ROS preview observer. No motion, selection reservation or approval."""
import argparse
import contextlib
import json
from pathlib import Path
import select
import subprocess
import sys
import threading
import time
import uuid


def read_reply(process, timeout):
    ready, _, _ = select.select([process.stdout], [], [], timeout)
    if not ready:
        process.kill()
        raise RuntimeError('GPU worker timed out; restart observer')
    line = process.stdout.readline()
    if not line:
        raise RuntimeError('GPU worker exited; inspect its error output')
    return json.loads(line)


def worker(args):
    import cv2
    import numpy as np
    import tomllib
    from sorting_geometry import analyse
    from inspect_b import Models
    with contextlib.redirect_stdout(sys.stderr):
        b_settings = json.loads(args.b_settings.read_text())
        config = tomllib.loads(Path(b_settings['scene_config']).read_text())
        models = Models(b_settings)
    print(json.dumps({'ready': True}), flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            settings = json.loads(args.settings.read_text())
            image_path = Path(request['image'])
            image = cv2.imread(str(image_path))
            if image is None: raise ValueError('Unreadable snapshot')
            with contextlib.redirect_stdout(sys.stderr):
                report, accepted, masks, roi = models.detect(image, config)
            folder = image_path.parent/'b_result'; folder.mkdir(exist_ok=False)
            for i, mask in enumerate(masks, 1):
                if not cv2.imwrite(str(folder/f'proposal_{i}_mask.png'), mask.astype(np.uint8)*255):
                    raise OSError('Mask save failed')
            for item, mask in accepted:
                item['mask_file'] = f'proposal_{item["proposal_id"]}_mask.png'
            report['source_image'] = str(image_path.resolve())
            (folder/'report.json').write_text(json.dumps(report, allow_nan=False))
            result = analyse(folder, settings)
            result['detector_ms'] = report['detector_ms']
            print(json.dumps({'ok': True, 'result': result}, allow_nan=False), flush=True)
        except Exception as error:
            print(json.dumps({'ok': False, 'error': str(error)}), flush=True)


def serve(args):
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from cv_bridge import CvBridge

    root = Path.cwd()
    gpu_python = root/'.pixi/envs/perception-gpu/bin/python'
    command = [str(gpu_python), str(Path(__file__).resolve()), 'worker',
               '--settings', str(args.settings.resolve()), '--b-settings', str(args.b_settings.resolve())]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               text=True, bufsize=1)
    try:
        startup = read_reply(process, 120)
        if startup.get('ready') is not True: raise RuntimeError(str(startup))
        rclpy.init()
        class Observer(Node):
            def __init__(self):
                super().__init__('sorting_preview_observer')
                self.condition = threading.Condition()
                self.busy = threading.Lock()
                self.latest = None
                self.bridge = CvBridge()
                group = ReentrantCallbackGroup()
                self.subscription = self.create_subscription(Image, '/scene/image_raw',
                    self.image, qos_profile_sensor_data, callback_group=group)
                self.publisher = self.create_publisher(String, '/perception/observations_preview', 1)
                self.service = self.create_service(Trigger, '/perception/observe_preview',
                    self.observe, callback_group=group)
                self.get_logger().info('READY: /perception/observe_preview; all results are previews')

            def image(self, msg):
                with self.condition:
                    self.latest = (time.monotonic(), msg)
                    self.condition.notify_all()

            def observe(self, request, response):
                if not self.busy.acquire(blocking=False):
                    response.success = False; response.message = 'Observer busy'; return response
                try:
                    started = time.monotonic()
                    with self.condition:
                        if not self.condition.wait_for(lambda: self.latest is not None and self.latest[0] >= started, timeout=3):
                            raise ValueError('No new scene image after request')
                        msg = self.latest[1]
                    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                    age = self.get_clock().now().nanoseconds*1e-9-stamp
                    if not -.1 <= age <= 1:
                        raise ValueError(f'Scene image timestamp stale/incompatible: age {age:.3f}s')
                    if msg.header.frame_id != 'scene_camera_optical_frame':
                        raise ValueError('Unexpected scene optical frame')
                    snapshot = uuid.uuid4().hex
                    folder = args.output_dir/snapshot; folder.mkdir(parents=True, exist_ok=False)
                    image_path = folder/'scene.png'
                    if not cv2.imwrite(str(image_path), self.bridge.imgmsg_to_cv2(msg, 'bgr8')):
                        raise OSError('Snapshot save failed')
                    process.stdin.write(json.dumps({'image': str(image_path.resolve())})+'\n')
                    process.stdin.flush()
                    reply = read_reply(process, 30)
                    if not reply.get('ok'): raise ValueError(reply.get('error', 'GPU worker failure'))
                    result = reply['result']
                    result.update(snapshot_id=snapshot, source_timestamp_sec=stamp,
                                  observation_age_s=self.get_clock().now().nanoseconds*1e-9-stamp,
                                  source_frame_id=msg.header.frame_id,
                                  interface_scope='preview only; service success means analysis completed, not pick approval')
                    for board in result['boards']:
                        board['instance_id'] = snapshot+':'+board['candidate_id']
                    text = json.dumps(result, allow_nan=False)
                    (folder/'observations.json').write_text(text)
                    self.publisher.publish(String(data=text))
                    response.success = True; response.message = text
                except Exception as error:
                    response.success = False; response.message = str(error)
                finally:
                    self.busy.release()
                return response
        node = Observer()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        try: executor.spin()
        except KeyboardInterrupt: pass
        finally:
            executor.shutdown(); node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
    finally:
        if process.poll() is None:
            process.terminate()
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: process.kill(); process.wait()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=['worker', 'serve'])
    p.add_argument('--settings', type=Path, required=True)
    p.add_argument('--b-settings', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, default=Path('.local/sorting_live_preview'))
    args = p.parse_args()
    if args.mode == 'worker': worker(args)
    else: serve(args)
