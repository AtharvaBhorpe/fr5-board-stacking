"""ChArUco observations and held-out intrinsic calibration. No robot motion."""

import argparse
import json
from pathlib import Path
import tomllib

import cv2
import numpy as np


def board_settings(config, name=None):
    settings = config['calibration']
    name = name or settings['active_board']
    if name not in settings['boards']:
        raise ValueError(f'Unknown calibration board: {name}')
    return {**settings, **settings['boards'][name]}


def board_from_config(config, name=None):
    settings = board_settings(config, name)
    nx, ny = settings['grid_squares_x'], settings['grid_squares_y']
    square, marker = settings['square_length_m'], settings['marker_length_m']
    if nx < 3 or ny < 3 or not 0 < marker < square:
        raise ValueError('Measure and configure the grid counts, square length and marker length first.')
    name = settings['dictionary']
    if not name.startswith('DICT_') or not hasattr(cv2.aruco, name):
        raise ValueError('Configure a supported marker dictionary.')
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
    return cv2.aruco.CharucoBoard((nx, ny), square, marker, dictionary)


def observe(image, config, name=None):
    if image is None or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('Expected a readable uint8 colour image.')
    board = board_from_config(config, name)
    corners, ids, _, marker_ids = cv2.aruco.CharucoDetector(board).detectBoard(image)
    if marker_ids is not None and len(np.unique(marker_ids)) != len(marker_ids):
        raise ValueError('Duplicate marker IDs: show only one calibration grid.')
    if ids is None or len(ids) < config['calibration']['min_corners']:
        raise ValueError(f'Insufficient grid corners: {0 if ids is None else len(ids)} detected.')
    return board.getChessboardCorners()[ids.flatten()].astype(np.float32), corners.astype(np.float32), ids


def fit_intrinsics(observations, image_size, settings):
    if len(observations) < settings['min_views']:
        raise ValueError(f'Need at least {settings["min_views"]} accepted diverse views; found {len(observations)}.')
    # Hold out every fifth view; its camera intrinsics are never used for fitting.
    held_out = [i for i in range(len(observations)) if i % 5 == 4]
    training = [i for i in range(len(observations)) if i not in held_out]
    objects = [observations[i][0] for i in training]
    pixels = [observations[i][1] for i in training]
    rms, matrix, distortion, rvecs, _ = cv2.calibrateCamera(objects, pixels, image_size, None, None)
    errors = []
    for i in held_out:
        object_points, image_points = observations[i][:2]
        ok, rotation, translation = cv2.solvePnP(object_points, image_points, matrix, distortion)
        if not ok:
            raise ValueError('Held-out pose estimation failed.')
        projected, _ = cv2.projectPoints(object_points, rotation, translation, matrix, distortion)
        errors.append(float(np.sqrt(np.mean(np.sum((projected - image_points) ** 2, axis=2)))))
    normals = np.asarray([cv2.Rodrigues(rvec)[0][:, 2] for rvec in rvecs])
    tilt_spread = float(np.degrees(np.max(np.arccos(np.clip(normals @ normals.T, -1, 1)))))
    all_pixels = np.concatenate([obs[1].reshape(-1, 2) for obs in observations])
    coverage = np.ptp(all_pixels, axis=0) / image_size
    reasons = []
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        reasons.append('Non-finite calibration parameters.')
    if (not np.isfinite([rms, *errors]).all() or
            rms > settings['max_rms_px'] or max(errors) > settings['max_rms_px']):
        reasons.append('Reprojection error exceeds the configured pixel threshold.')
    if tilt_spread < settings['min_tilt_spread_deg']:
        reasons.append('Insufficient out-of-plane tilt diversity; rotation on the table is not enough.')
    if min(coverage) < settings['min_axis_coverage_fraction']:
        reasons.append('Grid observations cover too little of the image.')
    width, height = image_size
    if (not 0.3 * width < matrix[0, 0] < 5 * width or
            not 0.3 * width < matrix[1, 1] < 5 * width or
            not 0 <= matrix[0, 2] <= width or not 0 <= matrix[1, 2] <= height):
        reasons.append('Fitted focal length or principal point is implausible; collect better views.')
    return {'camera_matrix': matrix.tolist(), 'distortion_coefficients': distortion.flatten().tolist(),
            'distortion_model': 'opencv_brown_conrady', 'image_size': list(image_size),
            'training_rms_px': float(rms), 'held_out_rms_px': errors, 'views': len(observations),
            'held_out_views': held_out, 'normal_spread_deg': tilt_spread,
            'axis_coverage_fraction': coverage.tolist(), 'intrinsic_checks_passed': not reasons,
            'warnings': reasons, 'metric_workcell_calibrated': False, 'execution_allowed': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('inspect', 'accept', 'fit'))
    parser.add_argument('--image')
    parser.add_argument('--board', help='Measured board profile; defaults to calibration.active_board.')
    parser.add_argument('--config', default='config/perception.toml')
    args = parser.parse_args()
    try:
        with open(args.config, 'rb') as source:
            config = tomllib.load(source)
        settings = board_settings(config, args.board)
        board_name = args.board or config['calibration']['active_board']
        if args.command in ('inspect', 'accept'):
            if not args.image:
                raise ValueError('--image is required.')
            image = cv2.imread(args.image)
            objects, pixels, ids = observe(image, config, board_name)
            report = {'corners': len(ids), 'image_size': [image.shape[1], image.shape[0]],
                      'board_profile': board_name}
            if args.command == 'accept':
                folder = Path(settings['views_directory'])
                folder.mkdir(parents=True, exist_ok=True)
                number = max([int(p.stem.split('_')[-1]) for p in folder.glob('view_*.png')] + [0]) + 1
                target = folder / f'view_{number:03d}.png'
                if not cv2.imwrite(str(target), image):
                    raise OSError('Saving the accepted view failed.')
                metadata = {'source_image': args.image, 'camera_settings': config['logitech'],
                            'board': {k: settings[k] for k in ('dictionary', 'grid_squares_x', 'grid_squares_y',
                                                              'square_length_m', 'marker_length_m')}, **report}
                target.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
                report['saved'] = str(target)
                report['accepted_views'] = len(list(folder.glob('view_*.png')))
        else:
            observations = []
            size = None
            per_board = {}
            for name in config['calibration']['boards']:
                profile = board_settings(config, name)
                count = 0
                for path in sorted(Path(profile['views_directory']).glob('view_*.png')):
                    metadata_file = path.with_suffix('.json')
                    if metadata_file.exists():
                        metadata = json.loads(metadata_file.read_text())
                        recorded = metadata.get('board', {})
                        for key in ('square_length_m', 'marker_length_m', 'grid_squares_x', 'grid_squares_y'):
                            if key in recorded and recorded[key] != profile[key]:
                                raise ValueError(f'Board geometry changed since capture: {path}')
                    image = cv2.imread(str(path))
                    observation = observe(image, config, name)
                    this_size = (image.shape[1], image.shape[0])
                    if size is not None and this_size != size:
                        raise ValueError('Calibration images have mixed resolutions.')
                    size = this_size
                    observations.append(observation)
                    count += 1
                per_board[name] = count
            report = fit_intrinsics(observations, size, settings)
            report['board_profiles'] = config['calibration']['boards']
            report['views_per_board'] = per_board
            report['camera_settings'] = config['logitech']
            target = Path(settings['output_file'])
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        return 0
    except (OSError, ValueError, KeyError, cv2.error) as exc:
        print(json.dumps({'ok': False, 'error': str(exc)}))
        return 2
