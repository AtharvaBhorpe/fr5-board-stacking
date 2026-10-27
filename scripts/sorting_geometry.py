"""Conservative sorting observations. No robot commands or execution approval."""
import json
from pathlib import Path
import cv2
import numpy as np


def rigid(value):
    t = np.asarray(value, float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1])
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-5)):
        raise ValueError('Invalid rigid transform')
    return t


def rays_to_plane(pixels, calibration, point, normal):
    t = rigid(calibration['T_base_camera'])
    point = np.asarray(point, float)
    normal = np.asarray(normal, float)
    if (point.shape != (3,) or normal.shape != (3,)
            or not np.isfinite(point).all() or not np.isfinite(normal).all()
            or np.linalg.norm(normal) < 1e-8):
        raise ValueError('Invalid surface plane')
    normal = normal / np.linalg.norm(normal)
    k = np.asarray(calibration['camera_matrix'], float)
    d = np.asarray(calibration['distortion_coefficients'], float)
    uv = np.asarray(pixels, float).reshape(-1, 1, 2)
    xy = cv2.undistortPoints(uv, k, d).reshape(-1, 2)
    rays = np.column_stack([xy, np.ones(len(xy))]) @ t[:3, :3].T
    denom = rays @ normal
    if np.any(np.abs(denom) < 1e-8):
        raise ValueError('Ray parallel to surface')
    distance = (point - t[:3, 3]) @ normal / denom
    if np.any(distance <= 0) or not np.isfinite(distance).all():
        raise ValueError('Surface intersection behind camera')
    return t[:3, 3] + distance[:, None] * rays


def surface_pose(corners, normal):
    corners = np.asarray(corners, float)
    edges = np.roll(corners, -1, axis=0) - corners
    lengths = np.linalg.norm(edges, axis=1)
    x = edges[int(np.argmax(lengths))]
    z = np.asarray(normal, float)
    z /= np.linalg.norm(z)
    x = x - z * (x @ z)
    if np.linalg.norm(x) < 1e-8:
        raise ValueError('Degenerate board direction')
    x /= np.linalg.norm(x)
    # Rectangular board orientation has 180-degree symmetry.
    significant = np.flatnonzero(np.abs(x) > 1e-6)
    if x[significant[0]] < 0:
        x = -x
    t = np.eye(4)
    t[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    t[:3, 3] = corners.mean(axis=0)
    return rigid(t), sorted([float((lengths[0]+lengths[2])/2),
                             float((lengths[1]+lengths[3])/2)], reverse=True)


def colour(image, mask, settings):
    interior = cv2.erode(mask.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[interior]
    if len(hsv) < 25:
        return 'unknown', {}
    red = (((hsv[:, 0] <= 12) | (hsv[:, 0] >= 165))
           & (hsv[:, 1] >= 90) & (hsv[:, 2] >= 45))
    white = (hsv[:, 1] <= 80) & (hsv[:, 2] >= 65)
    fractions = {'red_fraction': float(red.mean()), 'white_fraction': float(white.mean())}
    threshold = settings['minimum_colour_fraction']
    name = 'red' if red.mean() >= threshold else 'white' if white.mean() >= threshold else 'unknown'
    return name, fractions


def outline(mask, settings):
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 1:
        raise ValueError('Fragmented/occluded mask: full outline unavailable')
    contour = contours[0]
    perimeter = cv2.arcLength(contour, True)
    quad = cv2.approxPolyDP(contour, settings['outline_epsilon_fraction'] * perimeter, True)
    if len(quad) != 4 or not cv2.isContourConvex(quad):
        raise ValueError('No supported convex four-corner outline; possible merge/occlusion')
    quad = quad.reshape(4, 2).astype(float)
    area = abs(cv2.contourArea(quad.astype(np.float32)))
    if area <= 0 or abs(cv2.contourArea(contour)) / area < settings['minimum_outline_fill']:
        raise ValueError('Outline inconsistent with mask')
    return quad


def classify(dimensions, observed_colour, settings):
    matches = []
    for name, spec in settings['classes'].items():
        if spec['colour'] != observed_colour:
            continue
        target = np.asarray(spec['size_m'][:2])
        error = np.max(np.abs(np.asarray(dimensions) - target) / target)
        if error <= settings['size_relative_tolerance']:
            matches.append(name)
    if len(matches) != 1:
        raise ValueError('Physical size/class ambiguous or inconsistent')
    return matches[0]


def board_observations(image, report, folder, settings, calibration):
    observations = []
    plane = settings['pickup_surface']
    for candidate in report['quad_candidates']:
        item = {'candidate_id': candidate['id'], 'class_id': 'unknown',
                'pose_available': False, 'pick_eligible': False,
                'execution_allowed': False, 'reasons': [],
                'T_base_board': None, 'T_camera_board': None}
        try:
            path = folder / candidate['mask_file']
            mask = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if mask is None or mask.shape != image.shape[:2]:
                raise ValueError('Missing/mismatched instance mask')
            shade, measurements = colour(image, mask > 0, settings)
            item.update(visible_colour=shade, colour_measurements=measurements)
            pixels = outline(mask, settings)
            item['supported_pixel_corners'] = pixels.tolist()
            if plane['point_m'] is None or plane['normal'] is None:
                raise ValueError('Pickup board-top plane is not configured')
            if plane['single_layer_confirmed'] is not True:
                raise ValueError('Unknown instance height: stacked/overlapping scene needs per-instance height')
            xyz = rays_to_plane(pixels, calibration, plane['point_m'], plane['normal'])
            pose, dimensions = surface_pose(xyz, plane['normal'])
            item['class_id'] = classify(dimensions, shade, settings)
            item.update(T_base_board=pose.tolist(),
                        T_camera_board=(np.linalg.inv(rigid(calibration['T_base_camera'])) @ pose).tolist(),
                        centre_base_m=pose[:3, 3].tolist(), measured_size_m=dimensions,
                        yaw_base_rad=float(np.arctan2(pose[1, 0], pose[0, 0])),
                        pose_available=True)
            if not settings['camera_to_base_verified']:
                item['reasons'].append('Camera-to-base transform is a candidate, not independently verified')
            item['reasons'].append('Instance outline and suction suitability require live acceptance tests')
        except (ValueError, KeyError) as error:
            item['reasons'].append(str(error))
        observations.append(item)
    return observations


def bin_observations(image, settings, calibration):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, settings['tag_dictionary']))
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(image)
    ids = [] if ids is None else ids.flatten().tolist()
    result = []
    for name, spec in settings['classes'].items():
        tag = spec['destination_tag_id']
        item = {'class_id': name, 'tag_id': tag, 'pose_available': False,
                'execution_allowed': False, 'reasons': []}
        try:
            if ids.count(tag) != 1:
                raise ValueError('Destination tag missing or duplicated')
            pixels = corners[ids.index(tag)].reshape(4, 2)
            item['pixel_centre'] = pixels.mean(axis=0).tolist()
            side = settings['bins'][str(tag)]['tag_size_m']
            offset = settings['bins'][str(tag)]['T_tag_placement']
            if side is None or not np.isfinite(side) or side <= 0:
                raise ValueError('Measured tag size is missing/invalid')
            object_points = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], float)*side/2
            count, rotations, translations, _ = cv2.solvePnPGeneric(
                object_points, pixels.astype(float), np.asarray(calibration['camera_matrix']),
                np.asarray(calibration['distortion_coefficients']), flags=cv2.SOLVEPNP_IPPE_SQUARE)
            options = []
            for r, t in zip(rotations, translations):
                rotation = cv2.Rodrigues(r)[0]
                if np.min(object_points @ rotation[2] + t[2, 0]) <= 0:
                    continue
                projected = cv2.projectPoints(object_points, r, t,
                    np.asarray(calibration['camera_matrix']), np.asarray(calibration['distortion_coefficients']))[0].reshape(4, 2)
                rms = float(np.sqrt(np.mean(np.sum((projected-pixels)**2, axis=1))))
                pose = np.eye(4); pose[:3, :3] = rotation; pose[:3, 3] = t.ravel()
                options.append((rms, pose))
            if not options:
                raise ValueError('No positive-depth tag pose')
            options.sort(key=lambda pair: pair[0])
            rms, camera_tag = options[0]
            item.update(tag_reprojection_rms_px=rms, T_camera_tag=camera_tag.tolist())
            if rms > settings['maximum_tag_rms_px']:
                raise ValueError('Tag reprojection error exceeds limit')
            if offset is None:
                raise ValueError('Measured tag-to-placement transform is missing')
            camera_place = camera_tag @ rigid(offset)
            item.update(T_camera_placement=camera_place.tolist(),
                        T_base_placement=(rigid(calibration['T_base_camera']) @ camera_place).tolist(),
                        pose_available=True)
            item['reasons'].append('Planar tag ambiguity, bin offset and stack-top height need independent checks')
        except (ValueError, cv2.error) as error:
            item['reasons'].append(str(error))
        result.append(item)
    return result


def analyse(folder, settings):
    folder = Path(folder)
    report = json.loads((folder/'report.json').read_text())
    image = cv2.imread(report['source_image'])
    if image is None:
        raise ValueError('Cannot load source image')
    calibration = json.loads(Path(settings['scene_calibration']).read_text())
    if [image.shape[1], image.shape[0]] != calibration['image_size']:
        raise ValueError('Scene calibration resolution mismatch; never apply Logitech calibration to wrist images')
    result = {'frame_id': 'base_link', 'camera_frame': 'scene_camera_optical_frame',
              'source_image': report['source_image'], 'execution_allowed': False,
              'boards': board_observations(image, report, folder, settings, calibration),
              'bins': bin_observations(image, settings, calibration),
              'limits': ['Offline snapshot IDs are not tracked instance IDs.',
                         'This preview does not solve overlapping board instance segmentation.',
                         'Board poses require a known single-layer board-top plane.',
                         'No suction, freshness or physical accuracy acceptance is implied.']}
    return result


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settings', type=Path, required=True)
    parser.add_argument('--b-output', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    settings = json.loads(args.settings.read_text())
    manifest = json.loads((args.b_output/'manifest.json').read_text())
    args.output_dir.mkdir(parents=True, exist_ok=False)
    count = 0
    for entry in manifest['images']:
        result = analyse(args.b_output/entry['key'], settings)
        (args.output_dir/(entry['key']+'.json')).write_text(json.dumps(result, indent=2, allow_nan=False))
        count += 1
        print(f'{entry["key"]}: {len(result["boards"])} candidate boards, '
              f'{sum(b["pose_available"] for b in result["boards"])} board poses; '
              f'{sum(b["pose_available"] for b in result["bins"])} bin poses', flush=True)
    (args.output_dir/'settings_snapshot.json').write_text(json.dumps(settings, indent=2))
    print('Saved', count, 'preview reports to', args.output_dir)


if __name__ == '__main__':
    main()
