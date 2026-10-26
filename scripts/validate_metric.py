"""Independent camera-coordinate checks; no robot or calibration writes."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np


def fit_plane(points):
    p = np.asarray(points, float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 3 or not np.isfinite(p).all():
        raise ValueError('Need at least three finite 3D plane points')
    centre = p.mean(axis=0)
    _, singular, vt = np.linalg.svd(p-centre, full_matrices=False)
    if singular[1] < 1e-5:
        raise ValueError('Plane points are collinear or too close together')
    normal = vt[-1]
    if normal[2] < 0:
        normal = -normal
    return centre, normal


def intersect(pixels, calibration, centre, normal):
    transform = np.asarray(calibration['T_base_camera'], float)
    r, origin = transform[:3,:3], transform[:3,3]
    if (transform.shape != (4,4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0,0,0,1])
            or not np.allclose(r.T@r,np.eye(3),atol=1e-5)
            or not np.isclose(np.linalg.det(r),1,atol=1e-5)):
        raise ValueError('Invalid camera transform')
    if calibration['distortion_model'] != 'plumb_bob':
        raise ValueError('Expected raw plumb_bob camera model')
    uv = np.asarray(pixels,float).reshape(-1,1,2)
    k = np.asarray(calibration['camera_matrix'],float)
    d = np.asarray(calibration['distortion_coefficients'],float)
    rays = cv2.undistortPoints(uv,k,d).reshape(-1,2)
    rays = np.column_stack([rays,np.ones(len(rays))])@r.T
    denominator = rays@normal
    if np.any(np.abs(denominator)<1e-8):
        raise ValueError('Ray is parallel to plane')
    distance = ((centre-origin)@normal)/denominator
    if np.any(distance<=0) or not np.isfinite(distance).all():
        raise ValueError('Intersection is behind camera or invalid')
    return origin+distance[:,None]*rays


def self_test():
    points=np.array([[0,0,0],[.3,0,0],[0,.2,0],[.3,.2,0]])
    centre,normal=fit_plane(points)
    t=np.diag([1.,-1.,-1.,1.]);t[2,3]=1
    calibration={'T_base_camera':t.tolist(),'camera_matrix':[[800,0,320],[0,800,240],[0,0,1]],
                 'distortion_coefficients':[0]*5,'distortion_model':'plumb_bob'}
    expected=np.array([[.1,.05,0],[-.1,-.1,0]])
    pixels=np.column_stack([expected[:,0]*800+320,-expected[:,1]*800+240])
    assert np.allclose(intersect(pixels,calibration,centre,normal),expected)
    try:
        fit_plane([[0,0,0],[1,0,0],[2,0,0]])
    except ValueError:
        pass
    else:
        raise AssertionError('Collinear plane was accepted')
    print('PASS: plane fitting, camera-to-base direction, ray intersection, degeneracy rejection')


def load(settings_path):
    settings=json.loads(settings_path.read_text())
    rows=[json.loads(v) for v in Path(settings['references']).read_text().splitlines() if v.strip()]
    by_label={v['label']:v for v in rows}
    if len(by_label)!=len(rows):
        raise ValueError('Duplicate reference labels')
    for label in settings['plane_labels']+settings['check_labels']:
        row=by_label[label]
        position=np.asarray(row['position_m'],float)
        if position.shape!=(3,) or not np.isfinite(position).all():
            raise ValueError('Invalid reference coordinate: '+label)
        if row['frame_id']!='base_link' or row['tip_frame']!=settings['reference_tip_frame']:
            raise ValueError('Reference frame or calibrated tip differs: '+label)
    if set(settings['plane_labels'])&set(settings['check_labels']):
        raise ValueError('Fit and check points must be separate')
    return settings,by_label


def mark(settings_path):
    settings,_=load(settings_path)
    calibration=json.loads(Path(settings['calibration']).read_text())
    image=cv2.imread(settings['image'])
    if image is None or [image.shape[1],image.shape[0]]!=calibration['image_size']:
        raise ValueError('Need original-resolution scene image matching calibration')
    if Path(settings['pixels']).exists():
        raise ValueError('Pixel file already exists; choose a new name')
    labels=settings['check_labels']; clicks=[]
    scale=min(1.,1280/image.shape[1],800/image.shape[0])
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and len(clicks)<len(labels):
            clicks.append([x/scale,y/scale])
    cv2.namedWindow('Independent point checks')
    cv2.setMouseCallback('Independent point checks',click)
    try:
        while True:
            preview=cv2.resize(image,None,fx=scale,fy=scale)
            for label,point in zip(labels,clicks):
                p=tuple(np.rint(np.array(point)*scale).astype(int))
                cv2.circle(preview,p,5,(0,255,0),2)
                cv2.putText(preview,label,p,cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,0),2)
            message='Click '+labels[len(clicks)] if len(clicks)<len(labels) else 'Enter: save; U: undo; Esc: cancel'
            cv2.putText(preview,message,(15,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
            cv2.imshow('Independent point checks',preview)
            key=cv2.waitKey(30)&255
            if key==27:
                return
            if key in (ord('u'),ord('U')) and clicks:
                clicks.pop()
            if key in (10,13) and len(clicks)==len(labels):
                data={'image':settings['image'],'image_sha256':hashlib.sha256(Path(settings['image']).read_bytes()).hexdigest(),
                      'pixels':dict(zip(labels,clicks))}
                with Path(settings['pixels']).open('x') as f:json.dump(data,f,indent=2)
                print('Saved',settings['pixels']);return
    finally:
        cv2.destroyAllWindows()


def check(settings_path):
    settings,rows=load(settings_path)
    calibration=json.loads(Path(settings['calibration']).read_text())
    picks=json.loads(Path(settings['pixels']).read_text())
    if picks['image_sha256']!=hashlib.sha256(Path(settings['image']).read_bytes()).hexdigest():
        raise ValueError('Image changed since point annotation')
    plane=np.array([rows[v]['position_m'] for v in settings['plane_labels']])
    centre,normal=fit_plane(plane)
    fit_errors=(plane-centre)@normal
    checks=[]
    for label in settings['check_labels']:
        predicted=intersect([picks['pixels'][label]],calibration,centre,normal)[0]
        reference=np.array(rows[label]['position_m'],float)
        delta=predicted-reference
        checks.append({'label':label,'predicted_m':predicted.tolist(),'reference_m':reference.tolist(),
                       'error_xyz_mm':(delta*1000).tolist(),'xy_error_mm':float(np.linalg.norm(delta[:2])*1000),
                       'error_3d_mm':float(np.linalg.norm(delta)*1000)})
    report={'frame_id':'base_link','plane_point_m':centre.tolist(),'plane_normal':normal.tolist(),
            'plane_fit_residual_mm':(fit_errors*1000).tolist(),'checks':checks,
            'reference_contact_and_tcp_verified_by_operator':settings['reference_contact_and_tcp_verified_by_operator'],
            'calibration_sha256':hashlib.sha256(Path(settings['calibration']).read_bytes()).hexdigest(),
            'image_sha256':picks['image_sha256'],'execution_allowed':False,
            'warnings':['Checks use an independently probed plane; do not refit camera with check points.',
                        'Probe/TCP and manual image-click errors contribute to these results.',
                        'No robot absolute-accuracy certification or automatic execution approval.']}
    with Path(settings['output']).open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['self-test','mark','check'])
    parser.add_argument('--settings',type=Path)
    args=parser.parse_args()
    if args.mode=='self-test':self_test()
    elif args.settings is None:parser.error('--settings is required')
    elif args.mode=='mark':mark(args.settings)
    else:check(args.settings)
