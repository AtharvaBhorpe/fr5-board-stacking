"""Stopped-pose hand-eye capture. Never commands the robot."""
import argparse
from collections import deque
import json
from pathlib import Path
import sys
import tomllib

import cv2
import numpy as np


def matrix(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation).reshape(3)
    return result


def quaternion_matrix(q, p):
    q = np.asarray(q, float)
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > .01:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return matrix(rotation, p)


def separation(a, b):
    delta = np.linalg.inv(a) @ b
    angle = np.linalg.norm(cv2.Rodrigues(delta[:3, :3])[0])
    return np.linalg.norm(a[:3, 3]-b[:3, 3]), float(np.degrees(angle))


def stable(samples, tolerance):
    values = np.asarray(samples, float)
    return bool(values.ndim == 2 and len(values) >= 3
                and np.isfinite(values).all()
                and np.max(np.ptp(values, axis=0)) <= tolerance)


def fit(rows):
    if len(rows) < 12:
        raise ValueError('Need at least 12 training poses')
    a = [np.asarray(row['T_base_flange'], float) for row in rows]
    b = [np.asarray(row['T_camera_target'], float) for row in rows]
    axes = np.array([cv2.Rodrigues(a[0][:3, :3].T @ t[:3, :3])[0].ravel()
                     for t in a[1:]])
    singular = np.linalg.svd(axes, compute_uv=False)
    if singular[0] < .3 or singular[1] < .1 * singular[0]:
        raise ValueError('Insufficient rotation diversity about two axes')
    r, t = cv2.calibrateHandEye(
        [v[:3, :3] for v in a], [v[:3, 3] for v in a],
        [v[:3, :3] for v in b], [v[:3, 3] for v in b],
        method=cv2.CALIB_HAND_EYE_PARK)
    result = matrix(r, t)
    if (not np.isfinite(result).all()
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(r), 1, atol=1e-6)):
        raise ValueError('Invalid hand-eye solution')
    return result


def solve(folder, output):
    rows = [json.loads(p.read_text()) for p in sorted(folder.glob('sample_*.json'))]
    if len(rows) < 20:
        raise ValueError('Collect at least 20 diverse stopped poses')
    signature = lambda v: json.dumps([v['camera_info'], v['board'], v['base_frame'],
                                     v['flange_frame'], v['camera_frame']], sort_keys=True)
    if len({signature(row) for row in rows}) != 1:
        raise ValueError('Mixed intrinsics, board or frames; start a separate session')
    training = [v for i, v in enumerate(rows) if i % 5 != 4]
    held = [v for i, v in enumerate(rows) if i % 5 == 4]
    x = fit(training)
    products = [np.asarray(v['T_base_flange']) @ x @ np.asarray(v['T_camera_target'])
                for v in training]
    u, _, vt = np.linalg.svd(np.mean([v[:3, :3] for v in products], axis=0))
    rotation = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    target = matrix(rotation, np.mean([v[:3, 3] for v in products], axis=0))
    errors = [separation(target, np.asarray(v['T_base_flange']) @ x
                         @ np.asarray(v['T_camera_target'])) for v in held]
    report = {'parent_frame': rows[0]['flange_frame'],
              'child_frame': rows[0]['camera_frame'],
              'T_flange_camera': x.tolist(), 'T_base_target': target.tolist(),
              'training_poses': len(training), 'held_out_poses': len(held),
              'held_out_translation_mm': [float(e[0]*1000) for e in errors],
              'held_out_rotation_deg': [e[1] for e in errors],
              'metric_accuracy_validated': False, 'execution_allowed': False}
    with output.open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(report, indent=2))


def collect(settings_path, folder):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from tf2_ros import Buffer, TransformListener, TransformException
    from cv_bridge import CvBridge
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root / 'src/fr5_perception'))
    from fr5_perception.calibration import observe, board_settings
    settings = json.loads(settings_path.read_text())
    with Path(settings['perception_config']).open('rb') as source:
        config = tomllib.load(source)
    board = board_settings(config, settings['board'])
    geometry = {k: board[k] for k in ('dictionary', 'grid_squares_x',
                'grid_squares_y', 'square_length_m', 'marker_length_m')}
    folder.mkdir(parents=True, exist_ok=False)
    (folder / 'settings.json').write_text(json.dumps(settings, indent=2))
    stamp = lambda msg: msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    class Capture(Node):
        def __init__(self):
            super().__init__('fr5_handeye_capture')
            self.image = self.info = None
            self.joints = deque(maxlen=1000)
            self.accepted = []
            self.bridge = CvBridge()
            self.buffer = Buffer(cache_time=Duration(seconds=20))
            self.listener = TransformListener(self.buffer, self)
            self.create_subscription(Image, settings['image_topic'],
                                     self.on_image, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, settings['info_topic'],
                                     self.on_info, qos_profile_sensor_data)
            self.create_subscription(JointState, settings['joints_topic'],
                                     self.on_joints, qos_profile_sensor_data)
            self.create_timer(2., self.capture)

        def on_image(self, msg):
            self.image = msg

        def on_info(self, msg):
            self.info = msg

        def on_joints(self, msg):
            values = dict(zip(msg.name, msg.position))
            if all(name in values for name in settings['joint_names']):
                self.joints.append((stamp(msg), [values[n] for n in settings['joint_names']]))

        def capture(self):
            try:
                image, info = self.image, self.info
                if image is None or info is None:
                    raise ValueError('Waiting for image and CameraInfo')
                now = self.get_clock().now().nanoseconds / 1e9
                if not 0 <= now-stamp(image) <= .5 or abs(stamp(image)-stamp(info)) > .1:
                    raise ValueError('Image or CameraInfo is stale / unsynchronised')
                if (image.header.frame_id != info.header.frame_id or not image.header.frame_id
                        or (image.width, image.height) != (info.width, info.height)):
                    raise ValueError('Image / CameraInfo frame or dimensions disagree')
                recent = [(t, v) for t, v in self.joints if stamp(image)-1 <= t <= stamp(image)]
                if (len(recent) < 3 or recent[-1][0]-recent[0][0] < .5
                        or stamp(image)-recent[-1][0] > .2
                        or not stable([v for _, v in recent], settings['joint_tolerance_rad'])):
                    raise ValueError('Waiting for at least 0.5 seconds of stopped joint feedback')
                if info.distortion_model != 'plumb_bob' or info.k[0] <= 0:
                    raise ValueError('Need valid plumb_bob RGB CameraInfo; do not substitute SDK distortion')
                bgr = self.bridge.imgmsg_to_cv2(image, 'bgr8')
                objects, pixels, ids = observe(bgr, config, settings['board'])
                k, d = np.array(info.k).reshape(3, 3), np.array(info.d)
                ok, r, t = cv2.solvePnP(objects, pixels, k, d)
                if not ok:
                    raise ValueError('PnP failed')
                rotation = cv2.Rodrigues(r)[0]
                if np.min((objects @ rotation.T + t.reshape(3))[:, 2]) <= 0:
                    raise ValueError('Target is behind camera')
                projected = cv2.projectPoints(objects, r, t, k, d)[0]
                rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - pixels.reshape(-1, 2))**2, axis=1))))
                if not np.isfinite(rms) or rms > settings['max_rms_px']:
                    raise ValueError(f'Reprojection error too high: {rms:.3f} px')
                tf = self.buffer.lookup_transform(settings['base_frame'], settings['flange_frame'],
                                                  Time.from_msg(image.header.stamp)).transform
                a = quaternion_matrix([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w],
                                      [tf.translation.x, tf.translation.y, tf.translation.z])
                if any(separation(a, old)[0] < .02 and separation(a, old)[1] < 5
                       for old in self.accepted):
                    raise ValueError('Duplicate robot pose; waiting for a new stopped pose')
                name = f'sample_{image.header.stamp.sec}_{image.header.stamp.nanosec:09d}'
                row = {'image_stamp_sec': image.header.stamp.sec,
                       'image_stamp_nanosec': image.header.stamp.nanosec,
                       'camera_frame': image.header.frame_id,
                       'base_frame': settings['base_frame'], 'flange_frame': settings['flange_frame'],
                       'board': geometry, 'camera_info': {'k': list(info.k), 'd': list(info.d),
                       'width': info.width, 'height': info.height, 'distortion_model': info.distortion_model},
                       'joint_names': settings['joint_names'], 'joint_positions': recent[-1][1],
                       'joint_stamp': recent[-1][0], 'corners': len(ids), 'rms_px': rms,
                       'T_base_flange': a.tolist(), 'T_camera_target': matrix(rotation, t).tolist()}
                if not cv2.imwrite(str(folder / (name+'.png')), bgr):
                    raise OSError('Image could not be saved')
                with (folder / (name+'.json')).open('x') as stream:
                    json.dump(row, stream, indent=2)
                self.accepted.append(a)
                print(f'ACCEPTED {len(self.accepted)}: {len(ids)} corners, {rms:.3f} px', flush=True)
            except (ValueError, cv2.error, TransformException) as error:
                print(f'WAIT: {error}', flush=True)

    rclpy.init()
    node = Capture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def self_test():
    rng = np.random.default_rng(4)
    x = matrix(cv2.Rodrigues(np.array([.2, -.3, .1]))[0], [.04, .01, .08])
    target = matrix(np.eye(3), [.4, -.2, .1])
    rows = []
    for _ in range(20):
        a = matrix(cv2.Rodrigues(rng.normal(0, .6, 3))[0], rng.uniform(-.4, .4, 3))
        b = np.linalg.inv(x) @ np.linalg.inv(a) @ target
        rows.append({'T_base_flange': a.tolist(), 'T_camera_target': b.tolist()})
    assert np.allclose(fit(rows), x, atol=1e-8)
    assert stable([[0]*6]*3, .003)
    assert not stable([[0]*6, [.1]*6, [0]*6], .003)
    assert not stable([[float('nan')]*6]*3, .003)
    assert np.allclose(quaternion_matrix([0, 0, 0, 1], [1, 2, 3]), matrix(np.eye(3), [1, 2, 3]))
    try:
        fit([rows[0]]*20)
    except ValueError:
        pass
    else:
        raise AssertionError('Repeated poses were accepted')
    print('PASS: transform direction, hand-eye recovery, stationarity, degeneracy')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['self-test', 'collect', 'solve'])
    parser.add_argument('--settings', type=Path, default=Path('config/handeye_session.json'))
    parser.add_argument('--folder', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.mode == 'self-test':
        self_test()
    elif args.mode == 'collect':
        if args.folder is None:
            parser.error('--folder is required')
        collect(args.settings, args.folder)
    else:
        if args.folder is None or args.output is None:
            parser.error('--folder and --output are required')
        solve(args.folder, args.output)
