"""Fit saved grid images with upstream ROS camera_calibration, then validate/export.

No camera or robot connection. Does not apply the result to a live driver.
"""

import argparse
import json
from pathlib import Path
import tarfile
import tomllib

import cv2
import numpy as np
import yaml
from camera_calibration.calibrator import ChessboardInfo, MonoCalibrator, Patterns


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--camera', choices=['scene', 'wrist'], default='scene')
    parser.add_argument('--full-grid', action='store_true', help='Require all internal grid corners.')
    parser.add_argument('--config', default='config/perception.toml')
    args = parser.parse_args()
    with open(args.config, 'rb') as source:
        config = tomllib.load(source)
    profile = config['ros_calibration'][args.camera]
    settings = config['calibration']
    geometry = settings['boards'][profile['board']]
    min_corners = ((geometry['grid_squares_x'] - 1) * (geometry['grid_squares_y'] - 1)
                   if args.full_grid else settings['min_corners'])
    board = ChessboardInfo('charuco', geometry['grid_squares_x'], geometry['grid_squares_y'],
                          geometry['square_length_m'], geometry['marker_length_m'],
                          geometry['dictionary'].removeprefix('DICT_').lower())
    calibrator = MonoCalibrator([board], pattern=Patterns.ChArUco, name=profile['camera_name'])
    cv2.setNumThreads(1)
    selected = []
    rejected = {'no_grid': 0, 'insufficient_geometry': 0, 'redundant': 0}
    files = sorted(Path(args.input_dir).glob('*.png'), key=lambda path: path.stat().st_mtime_ns)
    if not files:
        raise ValueError('No recorded PNG images found.')
    size = None
    for path in files:
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f'Cannot read recorded image: {path}')
        current_size = (image.shape[1], image.shape[0])
        if size is not None and size != current_size:
            raise ValueError('Recorded images have mixed resolutions.')
        size = current_size
        ok, corners, ids, _ = calibrator.get_corners(image)
        if not ok:
            rejected['no_grid'] += 1
            continue
        if len(ids) < min_corners or board.charuco_board.checkCharucoCornersCollinear(ids):
            rejected['insufficient_geometry'] += 1
            continue
        parameters = calibrator.get_parameters(corners, ids, board, size)
        if not np.isfinite(parameters).all() or parameters[2] <= 0:
            rejected['insufficient_geometry'] += 1
            continue
        # Saved snapshots have nonuniform intervals; per-video-frame motion limits do not apply.
        if not calibrator.is_good_sample(parameters, corners, ids, None, None):
            rejected['redundant'] += 1
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        calibrator.db.append((parameters, gray))
        selected.append((path, corners, ids, gray, parameters))
    progress = calibrator.compute_goodenough()
    print(json.dumps({'recorded': len(files), 'selected': len(selected), 'rejected': rejected}), flush=True)
    if len(selected) < settings['min_views']:
        raise ValueError(f'Need {settings["min_views"]} diverse views; selected {len(selected)}.')
    held_out = [i for i in range(len(selected)) if i % 5 == 4]
    training = [i for i in range(len(selected)) if i not in held_out]
    calibrator.size = size
    calibrator.cal_fromcorners([(selected[i][1], selected[i][2], board) for i in training])
    matrix, distortion = calibrator.intrinsics, calibrator.distortion
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise ValueError('Non-finite camera parameters.')
    errors, normals = [], []
    pixels = []
    for path, corners, ids, _, _ in selected:
        objects = board.charuco_board.getChessboardCorners()[ids.flatten()]
        ok, rotation, translation = cv2.solvePnP(objects, corners, matrix, distortion)
        if not ok or translation[2, 0] <= 0:
            raise ValueError(f'Invalid validation pose: {path}')
        projected, _ = cv2.projectPoints(objects, rotation, translation, matrix, distortion)
        errors.append(float(np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=2)))))
        normals.append(cv2.Rodrigues(rotation)[0][:, 2])
        pixels.extend(corners.reshape(-1, 2))
    normals = np.asarray(normals)
    spread = float(np.degrees(np.max(np.arccos(np.clip(normals @ normals.T, -1, 1)))))
    coverage = np.ptp(pixels, axis=0) / size
    warnings = []
    if max(errors[i] for i in held_out) > settings['max_rms_px']:
        warnings.append('Held-out reprojection error exceeds the configured limit.')
    if max(errors[i] for i in training) > settings['max_rms_px']:
        warnings.append('A training view exceeds the configured pixel-error limit.')
    if spread < settings['min_tilt_spread_deg'] or min(coverage) < settings['min_axis_coverage_fraction']:
        warnings.append('Insufficient tilt diversity or image coverage.')
    if not (0 < matrix[0, 2] < size[0] and 0 < matrix[1, 2] < size[1]
            and .3 * size[0] < matrix[0, 0] < 5 * size[0]
            and .3 * size[0] < matrix[1, 1] < 5 * size[0]):
        warnings.append('Implausible focal length or principal point.')
    report = {'recorded_images': len(files), 'selected_images': [str(s[0]) for s in selected],
              'board_profile': profile['board'], 'board': geometry, 'rejected': rejected,
              'minimum_corners': min_corners,
              'held_out_indices': held_out, 'per_view_rms_px': errors,
              'training_rms_px': float(np.sqrt(np.mean(np.square([errors[i] for i in training])))),
              'held_out_rms_px': [errors[i] for i in held_out], 'normal_spread_deg': spread,
              'axis_coverage_fraction': coverage.tolist(), 'ros_coverage_progress': progress,
              'camera_matrix': matrix.tolist(), 'distortion_coefficients': distortion.flatten().tolist(),
              'image_size': list(size), 'intrinsic_checks_passed': not warnings, 'warnings': warnings,
              'metric_workcell_calibrated': False, 'execution_allowed': False}
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=False)
    data = calibrator.yaml()
    recovered = yaml.safe_load(data)
    np.testing.assert_allclose(np.reshape(recovered['camera_matrix']['data'], (3, 3)), matrix, atol=1e-4)
    (output / 'camera_info.yaml').write_text(data)
    (output / 'validation.json').write_text(json.dumps(report, indent=2, default=float))
    # Archive only fitting views; held-out images must remain independent of fitting.
    calibrator.db = [(selected[i][4], selected[i][3]) for i in training]
    with tarfile.open(output / 'calibrationdata.tar.gz', 'w:gz') as archive:
        calibrator.do_tarfile_save(archive)
    with tarfile.open(output / 'calibrationdata.tar.gz') as archive:
        assert archive.extractfile('ost.yaml').read().decode() == data
    print(json.dumps({k: v for k, v in report.items() if k not in ('selected_images', 'board')}, default=float), flush=True)
    return 0 if not warnings else 2


if __name__ == '__main__':
    raise SystemExit(main())
