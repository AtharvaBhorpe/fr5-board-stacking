"""Offline scene registration candidate from an unchanged shared ChArUco target."""
import argparse
import json
from pathlib import Path
import sqlite3
import sys
import tomllib

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def transform(r, t):
    result = np.eye(4)
    result[:3, :3] = r
    result[:3, 3] = np.asarray(t).reshape(3)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settings', required=True, type=Path)
    args = parser.parse_args()
    settings = json.loads(args.settings.read_text())
    if settings.get('same_fixed_target_confirmed') is not True:
        raise ValueError('Confirm target stayed fixed between hand-eye and this recording')
    with Path(settings['perception_config']).open('rb') as stream:
        config = tomllib.load(stream)
    sys.path.insert(0, str(Path(settings['perception_config']).resolve().parents[1] / 'src/fr5_perception'))
    from fr5_perception.calibration import observe
    handeye = json.loads(Path(settings['handeye']).read_text())
    base_target = np.asarray(handeye['T_base_target'], float)
    bag = Path(settings['bag'])
    databases = list(bag.glob('*.db3'))
    if len(databases) != 1:
        raise ValueError('This extractor expects exactly one sqlite3 database')
    connection = sqlite3.connect(databases[0].resolve().as_uri()+'?mode=ro', uri=True)
    topics = {name: (tid, typ) for tid, name, typ in connection.execute('SELECT id,name,type FROM topics')}

    def read(topic, offset=0):
        tid, typ = topics[topic]
        item = connection.execute(
            'SELECT data FROM messages WHERE topic_id=? ORDER BY id LIMIT 1 OFFSET ?',
            (tid, offset)).fetchone()
        if item is None:
            raise ValueError('Missing recorded message: '+topic)
        return deserialize_message(item[0], get_message(typ))

    info = read(settings['info_topic'])
    if info.distortion_model != 'plumb_bob':
        raise ValueError('Expected raw plumb_bob camera model')
    k = np.asarray(info.k).reshape(3, 3)
    d = np.asarray(info.d)
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('Invalid camera intrinsics')
    tid = topics[settings['image_topic']][0]
    count = connection.execute('SELECT count(*) FROM messages WHERE topic_id=?', (tid,)).fetchone()[0]
    if count < 3:
        raise ValueError('Need at least three recorded scene images')
    x0, y0, x1, y1 = settings['grid_roi_px']
    scale = settings['detection_scale']
    if scale < 1 or not (0 <= x0 < x1 <= info.width and 0 <= y0 < y1 <= info.height):
        raise ValueError('Invalid grid detection ROI or scale')
    views, errors = [], []
    bridge = CvBridge()
    for offset in sorted(set(np.linspace(0, count-1, 7).astype(int).tolist())):
        try:
            message = read(settings['image_topic'], offset)
            if (message.width, message.height) != (info.width, info.height):
                raise ValueError('Image and calibration dimensions disagree')
            if message.header.frame_id != settings['scene_frame']:
                raise ValueError('Unexpected image frame')
            if info.header.frame_id not in ('', message.header.frame_id):
                raise ValueError('CameraInfo frame conflicts with image frame')
            image = bridge.imgmsg_to_cv2(message, 'bgr8')
            patch = cv2.resize(image[y0:y1, x0:x1], None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_CUBIC)
            objects, pixels, ids = observe(patch, config, settings['board'])
            # Invert OpenCV resize's pixel-centre mapping before applying the full-image K.
            pixels = (pixels.reshape(-1, 2).astype(float)+.5)/scale-.5+[x0, y0]
            ok, r, t = cv2.solvePnP(objects, pixels, k, d)
            if not ok:
                raise ValueError('PnP failed')
            rotation = cv2.Rodrigues(r)[0]
            if np.min((objects @ rotation.T + t.reshape(3))[:, 2]) <= 0:
                raise ValueError('Target behind camera')
            projected = cv2.projectPoints(objects, r, t, k, d)[0].reshape(-1, 2)
            rms = float(np.sqrt(np.mean(np.sum((projected-pixels)**2, axis=1))))
            if not np.isfinite(rms) or rms > settings['max_rms_px']:
                raise ValueError(f'Reprojection error {rms:.3f} px')
            camera_target = transform(rotation, t)
            views.append({'frame_index': offset, 'corners': len(ids), 'rms_px': rms,
                          'T_camera_target': camera_target.tolist(),
                          'T_base_camera': (base_target @ np.linalg.inv(camera_target)).tolist()})
        except (ValueError, cv2.error) as error:
            errors.append({'frame_index': offset, 'reason': str(error)})
    if len(views) < 3:
        raise ValueError('Fewer than three valid views: '+json.dumps(errors))
    transforms = [np.asarray(v['T_base_camera']) for v in views]
    u, _, vt = np.linalg.svd(np.mean([v[:3, :3] for v in transforms], axis=0))
    rotation = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    mean = transform(rotation, np.mean([v[:3, 3] for v in transforms], axis=0))
    spread = [np.linalg.norm(v[:3, 3]-mean[:3, 3])*1000 for v in transforms]
    report = {'parent_frame': 'base_link', 'child_frame': settings['scene_frame'],
              'T_base_camera': mean.tolist(), 'camera_matrix': k.tolist(),
              'distortion_coefficients': d.tolist(), 'distortion_model': info.distortion_model,
              'image_size': [info.width, info.height], 'settings': settings,
              'camera_info_frame_was_empty': not bool(info.header.frame_id),
              'translation_spread_mm': spread, 'views': views, 'rejected_views': errors,
              'metric_accuracy_validated': False, 'execution_allowed': False,
              'warnings': ['Inherits hand-eye and target-pose errors.',
                           'Upsampling aids detection; it adds no physical image detail.',
                           'Fixed-target repetitions are not independent metric accuracy checks.',
                           'Table plane and board top heights remain unverified.']}
    with Path(settings['output']).open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps({'output': settings['output'], 'accepted_views': len(views),
                      'reprojection_rms_px': [v['rms_px'] for v in views],
                      'max_translation_spread_mm': max(spread),
                      'T_base_camera': mean.tolist(), 'execution_allowed': False}, indent=2))


if __name__ == '__main__':
    main()
