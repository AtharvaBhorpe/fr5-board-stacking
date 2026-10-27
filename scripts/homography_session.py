"""Fit pixel-to-base XY on F1-F5; independently validate on V1-V4. No motion."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

FIT = [f'F{i}' for i in range(1, 6)]
CHECK = [f'V{i}' for i in range(1, 5)]
LABELS = FIT + CHECK


def intrinsics(path):
    # Only K, D and image size are used; the saved extrinsic transform is not used.
    data=json.loads(path.read_text())
    if data['distortion_model']!='plumb_bob':raise ValueError('Expected plumb_bob intrinsics')
    k=np.asarray(data['camera_matrix'],float).reshape(3,3)
    d=np.asarray(data['distortion_coefficients'],float)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[0,0],k[1,1])<=0:
        raise ValueError('Invalid camera intrinsics')
    return data,k,d


def fit_plane(points):
    points = np.asarray(points, float)
    centre = points.mean(axis=0)
    _, s, vt = np.linalg.svd(points-centre, full_matrices=False)
    if len(points) < 3 or s[1] < 1e-5:
        raise ValueError('Reference points are collinear or too close')
    n = vt[-1]
    if n[2] < 0: n = -n
    if abs(n[2]) < .5:
        raise ValueError('Reference surface is not a table-like plane in base_link')
    return centre, n


def estimate(fit_uv, fit_xyz, check_uv, check_xyz, k, d):
    xyz = np.asarray(fit_xyz, float)
    checks = np.asarray(check_xyz, float)
    if (xyz.shape != (5, 3) or checks.shape != (4, 3)
            or not np.isfinite(xyz).all() or not np.isfinite(checks).all()):
        raise ValueError('Need five finite fit points and four finite independent check points')
    def undistort(uv):
        uv = np.asarray(uv, float).reshape(-1, 1, 2)
        if not np.isfinite(uv).all(): raise ValueError('Invalid pixels')
        return cv2.undistortPoints(uv, k, d, P=k).reshape(-1, 2)
    fit_pixels = undistort(fit_uv)
    check_pixels = undistort(check_uv)
    if fit_pixels.shape != (5, 2) or check_pixels.shape != (4, 2):
        raise ValueError('Wrong number of image points')
    if abs(cv2.contourArea(cv2.convexHull(fit_pixels.astype(np.float32)))) < 100:
        raise ValueError('Image reference points are collinear or cover too little area')
    centre, n = fit_plane(xyz)
    h, _ = cv2.findHomography(fit_pixels, xyz[:, :2], method=0)
    if h is None or not np.isfinite(h).all() or np.linalg.matrix_rank(h) < 3:
        raise ValueError('Degenerate homography')
    predicted_xy = cv2.perspectiveTransform(check_pixels.reshape(-1, 1, 2), h).reshape(-1, 2)
    z = centre[2] - ((predicted_xy-centre[:2]) @ n[:2])/n[2]
    predicted = np.column_stack([predicted_xy, z])
    fit_xy = cv2.perspectiveTransform(fit_pixels.reshape(-1, 1, 2), h).reshape(-1, 2)
    return {'H_undistorted_pixels_to_base_xy': h.tolist(),
            'plane_point_m': centre.tolist(), 'plane_normal': n.tolist(),
            'fit_xy_error_mm': (np.linalg.norm(fit_xy-xyz[:, :2], axis=1)*1000).tolist(),
            'fit_plane_residual_mm': (((xyz-centre) @ n)*1000).tolist(),
            'check_predicted_xyz_m': predicted.tolist(),
            'check_xy_error_mm': (np.linalg.norm(predicted_xy-checks[:, :2], axis=1)*1000).tolist(),
            'check_xyz_error_mm': (np.linalg.norm(predicted-checks, axis=1)*1000).tolist()}


def self_test():
    xyz = np.array([[0,0,.1],[.4,0,.1],[.4,.3,.1],[0,.3,.1],[.2,.15,.1]])
    checks = np.array([[.1,.1,.1],[.3,.1,.1],[.1,.2,.1],[.3,.2,.1]])
    uv = xyz[:, :2]*1000+[100,200]
    cv = checks[:, :2]*1000+[100,200]
    k = np.array([[800.,0,320],[0,800,240],[0,0,1]])
    r = estimate(uv, xyz, cv, checks, k, np.zeros(5))
    assert max(r['check_xyz_error_mm']) < .001
    d=np.array([.12,-.04,.001,-.002,.005])
    def distort(pixels):
        xy=(pixels-[320,240])/800
        objects=np.column_stack([xy,np.ones(len(xy))])
        return cv2.projectPoints(objects,np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2)
    distorted=estimate(distort(uv),xyz,distort(cv),checks,k,d)
    assert max(distorted['check_xyz_error_mm']) < .01
    shifted = checks.copy(); shifted[0,0] += .01
    r2 = estimate(uv, xyz, cv, shifted, k, np.zeros(5))
    assert 9.99 < r2['check_xy_error_mm'][0] < 10.01
    np.testing.assert_allclose(r['H_undistorted_pixels_to_base_xy'], r2['H_undistorted_pixels_to_base_xy'])
    try: estimate(np.zeros((5,2)), xyz, cv, checks, k, np.zeros(5))
    except ValueError: pass
    else: raise AssertionError('Degenerate pixels accepted')
    try: estimate(uv, np.zeros((5,3)), cv, checks, k, np.zeros(5))
    except ValueError: pass
    else: raise AssertionError('Degenerate robot points accepted')
    print('PASS: metric XY/Z, lens distortion, independent checks excluded from fitting, 10 mm error detection, degeneracy rejection')


def references(args):
    rows = [json.loads(line) for line in args.points.read_text().splitlines() if line.strip()]
    by_label = {row['label']: row for row in rows}
    if len(by_label) != len(rows): raise ValueError('Duplicate point labels')
    for label in LABELS:
        if label not in by_label: raise ValueError('Missing robot point: '+label)
        row = by_label[label]
        if row['frame_id'] != 'base_link' or row['tip_frame'] != args.tip:
            raise ValueError('Wrong reference frame/tip for '+label)
    return by_label


def mark(args):
    references(args)
    image = cv2.imread(str(args.image))
    if image is None: raise ValueError('Cannot read image')
    camera,_,_ = intrinsics(args.intrinsics)
    if [image.shape[1], image.shape[0]] != camera['image_size']:
        raise ValueError('Image resolution differs from intrinsics')
    output = args.folder/'pixels.json'
    if output.exists(): raise ValueError('pixels.json exists; choose a new session folder')
    scale = min(1.,1280/image.shape[1],800/image.shape[0]); clicks=[]
    def callback(event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks)<len(LABELS):
            clicks.append([(x+.5)/scale-.5,(y+.5)/scale-.5])
    window='F/V points: click tape cross CENTRES'
    cv2.namedWindow(window);cv2.setMouseCallback(window,callback)
    try:
        while True:
            view=cv2.resize(image,None,fx=scale,fy=scale)
            for label,uv in zip(LABELS,clicks):
                pos=tuple(np.rint(np.asarray(uv)*scale).astype(int))
                cv2.circle(view,pos,4,(0,255,0),2)
                cv2.putText(view,label,pos,cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,0),2)
            text='Click '+LABELS[len(clicks)] if len(clicks)<len(LABELS) else 'Enter save; U undo; Esc cancel'
            cv2.putText(view,text,(15,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
            cv2.imshow(window,view);key=cv2.waitKey(30)&255
            if key==27:return
            if key in (ord('u'),ord('U')) and clicks:clicks.pop()
            if key in (10,13) and len(clicks)==len(LABELS):
                args.folder.mkdir(parents=True,exist_ok=True)
                with output.open('x') as stream:
                    json.dump({'image':str(args.image.resolve()),
                               'image_sha256':hashlib.sha256(args.image.read_bytes()).hexdigest(),
                               'pixels':dict(zip(LABELS,clicks))},stream,indent=2)
                print('Saved:',output);return
    finally:cv2.destroyAllWindows()


def solve(args):
    rows=references(args)
    pixels=json.loads((args.folder/'pixels.json').read_text())
    if pixels['image_sha256']!=hashlib.sha256(args.image.read_bytes()).hexdigest():
        raise ValueError('Image changed after annotation')
    c,k,d=intrinsics(args.intrinsics)
    r=estimate([pixels['pixels'][v] for v in FIT],[rows[v]['position_m'] for v in FIT],
               [pixels['pixels'][v] for v in CHECK],[rows[v]['position_m'] for v in CHECK],k,d)
    r.update(frame_id='base_link',reference_tip=args.tip,fit_labels=FIT,check_labels=CHECK,
             image_size=c['image_size'],camera_matrix=k.tolist(),
             distortion_coefficients=d.tolist(),limit_mm=args.limit_mm,
             independent_check_passed=max(r['check_xy_error_mm']+r['check_xyz_error_mm']
                                         +[abs(v) for v in r['fit_plane_residual_mm']])<args.limit_mm,
             reference_contact_and_tcp_verified_by_operator=bool(args.reference_contact_verified and all(
                 rows[v].get('contact_confirmed') is True
                 and rows[v].get('physical_tip_calibration_verified') is True for v in LABELS)),
             reference_input_frames=sorted({rows[v].get('input_pose_frame','unspecified') for v in LABELS}),
             execution_allowed=False,
             image_sha256=pixels['image_sha256'],
             points_sha256=hashlib.sha256(args.points.read_bytes()).hexdigest(),
             intrinsics_sha256=hashlib.sha256(args.intrinsics.read_bytes()).hexdigest(),
             limits=['Only the surveyed plane and camera setup are covered.',
                     'Convert raw pixels with undistortPoints(P=K) BEFORE applying H.',
                     'Z is inferred from surveyed plane, not measured per detected board.',
                     'Stack height, tool geometry and physical accuracy need separate checks.'])
    output=args.folder/'homography_validation.json'
    with output.open('x') as stream:json.dump(r,stream,indent=2,allow_nan=False)
    print(json.dumps(r,indent=2));print('Saved:',output)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['self-test','mark','solve'])
    p.add_argument('--points',type=Path)
    p.add_argument('--image',type=Path)
    p.add_argument('--intrinsics',type=Path,default=Path('config/calibration/logitech_extrinsics_candidate_001.json'))
    p.add_argument('--folder',type=Path)
    p.add_argument('--tip',default='tcp')
    p.add_argument('--limit-mm',type=float,default=5.)
    p.add_argument('--reference-contact-verified',action='store_true')
    a=p.parse_args()
    if a.mode=='self-test':self_test()
    else:
        if a.points is None or a.image is None or a.folder is None:p.error('--points, --image and --folder are required')
        if not np.isfinite(a.limit_mm) or a.limit_mm<=0:p.error('--limit-mm must be positive')
        if a.mode=='mark':mark(a)
        else:solve(a)
