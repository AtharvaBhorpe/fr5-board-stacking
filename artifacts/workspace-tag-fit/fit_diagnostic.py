"""Reproduce the four-reference offline fit; no ROS or operational writes."""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from plane_geometry import pixels_to_plane, validate_transform


def centre_normalized(corners, K, D):
    q = cv2.undistortPoints(np.asarray(corners, dtype=np.float64).reshape(4, 1, 2), K, D).reshape(4, 2)
    h = np.column_stack((q, np.ones(4)))
    p = np.cross(np.cross(h[0], h[2]), np.cross(h[1], h[3]))
    if abs(p[2]) < 1e-12:
        raise ValueError('Degenerate tag diagonals')
    return p[:2] / p[2]


def main():
    capture = ROOT / 'artifacts/workspace-tag-check/20261026-150408-917279/tags.json'
    original = ROOT / 'artifacts/workspace-calibration/20261026-142900-324084/corrected-candidates-143930-499921.json'
    tags = json.loads(capture.read_text())
    prior = json.loads(original.read_text())
    K = np.array(prior['camera_matrix'], dtype=np.float64)
    D = np.array(prior['distortion_coefficients'], dtype=np.float64)
    refs = {0: [-670.179, -591.778], 13: [86, -866], 14: [45, 832], 15: [-625, 675]}
    ids = sorted(refs)
    markers = {i: [m for m in tags['markers'] if m['id'] == i] for i in ids}
    assert all(len(v) == 1 for v in markers.values()), 'Missing or duplicate reference IDs'
    assert tags['image_size_px'] == [1920, 1080]
    xy = np.array([refs[i] for i in ids]) / 1000
    points = np.column_stack((xy, np.full(4, -0.060)))
    normalized = np.array([centre_normalized(markers[i][0]['corners_px'], K, D) for i in ids])
    pixels, _ = cv2.projectPoints(np.column_stack((normalized, np.ones(4))), np.zeros(3), np.zeros(3), K, D)
    pixels = pixels.reshape(4, 2)

    # Check the perspective-centre calculation against a known synthetic square.
    square = np.array([[-.02, .02, 0], [.02, .02, 0], [.02, -.02, 0], [-.02, -.02, 0]])
    rv, tv = np.array([.4, -.2, .1]), np.array([.2, -.1, 1.5])
    test_pixels, _ = cv2.projectPoints(square, rv, tv, K, D)
    np.testing.assert_allclose(centre_normalized(test_pixels, K, D), tv[:2]/tv[2], atol=1e-7)

    initial = cv2.solvePnPGeneric(points, pixels, K, D, flags=cv2.SOLVEPNP_IPPE)
    if not initial[0]:
        raise RuntimeError('No pose candidates')
    results = []
    for branch, (r, t) in enumerate(zip(initial[1], initial[2]), 1):
        for stage in ('IPPE', 'LM_refined'):
            if stage == 'LM_refined':
                r, t = cv2.solvePnPRefineLM(points, pixels, K, D, r.copy(), t.copy())
            R, _ = cv2.Rodrigues(r)
            if not np.isfinite(R).all() or not np.isfinite(t).all() or np.any((R @ points.T + t)[2] <= 0):
                continue
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = R, t.ravel()
            T = validate_transform(np.linalg.inv(T))
            if T[2, 3] <= -.060:
                continue
            proj, _ = cv2.projectPoints(points, r, t, K, D)
            residual = proj.reshape(4, 2)-pixels
            predicted = pixels_to_plane(pixels, K, D, T, -.060)
            errors = np.linalg.norm(predicted[:, :2]-xy, axis=1)*1000
            item = dict(branch=branch, stage=stage, T_base_link_camera=T.tolist(), fit_rms_px=float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))), fit_rms_xy_mm=float(np.sqrt(np.mean(errors**2))), points=[dict(id=i, measured_xyz_m=p.tolist(), predicted_xyz_m=q.tolist(), error_xy_mm=float(e)) for i,p,q,e in zip(ids,points,predicted,errors)])
            results.append(item)
    if not results:
        raise RuntimeError('No physically eligible candidates')
    results.sort(key=lambda a:a['fit_rms_px'])
    report = dict(purpose='four_tag_training_fit_only', capture=str(capture), intrinsics_source=str(original), camera_matrix=K.tolist(), distortion_coefficients=D.tolist(), centre_method='undistorted_diagonal_intersection', reference_method='user_reported_ruler', reference_uncertainty_mm=None, paper_thickness_neglected=True, measured_xyz_m=points.tolist(), tag_ids=ids, image_centres_px=pixels.tolist(), candidates=results, intrinsics_validated_for_current_camera=False, independent_accuracy_validated=False, execution_allowed=False)
    out = Path(__file__).parent / 'report.json'
    with out.open('x') as f:
        json.dump(report, f, indent=2, allow_nan=False)
        f.write('\n')
    print('Synthetic diagonal-centre check: PASS')
    for a in results:
        print(a['branch'],a['stage'], 'fit RMS px',round(a['fit_rms_px'],3),'fit RMS mm',round(a['fit_rms_xy_mm'],2))
    best=results[0]
    for p in best['points']:
        print('Tag',p['id'],'fit error mm',round(p['error_xy_mm'],2))
    print('Camera xyz mm:',np.round(np.array(best['T_base_link_camera'])[:3,3]*1000,2).tolist())
    print('Saved',out)


if __name__ == '__main__':
    main()
