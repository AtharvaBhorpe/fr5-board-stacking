"""Read existing B reports and export raw-image centres and long-axis angles."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def rectangle_geometry(corners):
    p=np.asarray(corners,float)
    if p.shape!=(4,2) or not np.isfinite(p).all():raise ValueError('Need four finite cyclic rectangle corners')
    edges=np.roll(p,-1,axis=0)-p
    lengths=np.linalg.norm(edges,axis=1)
    if np.min(lengths)<=0:raise ValueError('Degenerate rectangle')
    edge=edges[int(np.argmax(lengths))]
    angle=float(np.degrees(np.arctan2(edge[1],edge[0]))%180)
    return {'rectangle_center_px':p.mean(axis=0).tolist(),
            'long_axis_angle_deg':angle,'long_axis_angle_rad':float(np.radians(angle)),
            'long_side_px':float(np.max(lengths)),'short_side_px':float(np.min(lengths)),
            'near_square_orientation_ambiguous':bool(np.min(lengths)/np.max(lengths)>.9)}


def self_test():
    for angle in (0.,30.,90.,150.):
        p=cv2.boxPoints(((200.,150.),(160.,80.),angle))
        r=rectangle_geometry(p)
        np.testing.assert_allclose(r['rectangle_center_px'],[200,150],atol=1e-4)
        difference=abs((r['long_axis_angle_deg']-angle+90)%180-90)
        assert difference<1e-3,(angle,r)
    assert rectangle_geometry(cv2.boxPoints(((100.,100.),(80.,80.),0)))['near_square_orientation_ambiguous']
    try:rectangle_geometry([[0,0]]*4)
    except ValueError:pass
    else:raise AssertionError('Degenerate rectangle accepted')
    print('PASS: pixel centre, long-axis angle, 180-degree symmetry, square ambiguity, degeneracy rejection')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',type=Path)
    p.add_argument('--camera-frame',choices=['scene_camera_optical_frame','wrist_camera_color_optical_frame'])
    p.add_argument('--output',type=Path)
    p.add_argument('--overlay',type=Path)
    p.add_argument('--self-test',action='store_true')
    a=p.parse_args()
    if a.self_test:self_test();return
    if a.report is None or a.camera_frame is None:p.error('--report and --camera-frame are required')
    for path in (a.output,a.overlay):
        if path and path.exists():raise ValueError('Preserve existing output; choose another name: '+str(path))
    report=json.loads(a.report.read_text())
    output={'associated_camera_frame':a.camera_frame,'coordinate_system':'raw_image_pixels',
            'image_size':report['image_size'],'source_image':report.get('source_image'),
            'angle_convention':'0 along image +u/right, 90 along +v/down; clockwise on image; modulo 180 degrees',
            'execution_allowed':False,'objects':[],
            'limits':['Rectangle encloses predicted visible mask; not a verified complete-board outline.',
                      'Merged/occluded instances can produce wrong centres and orientations.',
                      'Image angles are not robot yaw or 3D camera-frame rotation.',
                      'Candidate IDs are local to this report, not persistent tracking IDs.']}
    image=None
    if a.overlay:
        image=cv2.imread(report.get('source_image',''))
        if image is None:raise ValueError('Cannot read source image for overlay')
    for candidate in report['quad_candidates']:
        item={'id':candidate['id'],**rectangle_geometry(candidate['pixel_corners']),
              'mask_centroid_px':candidate['pixel_center'],
              'pixel_corners':candidate['pixel_corners'],
              'text_prompt_label':candidate.get('text_label'),
              'detector_score':candidate.get('detector_score'),
              'instance_isolation_verified':False,'metric_pose':None}
        output['objects'].append(item)
        if image is not None:
            centre=np.asarray(item['rectangle_center_px'])
            direction=np.array([np.cos(item['long_axis_angle_rad']),np.sin(item['long_axis_angle_rad'])])
            start=tuple(np.rint(centre).astype(int))
            end=tuple(np.rint(centre+direction*item['long_side_px']*.35).astype(int))
            corners=np.rint(candidate['pixel_corners']).astype(np.int32)
            cv2.polylines(image,[corners],True,(255,255,0),2)
            cv2.drawMarker(image,start,(0,0,255),cv2.MARKER_CROSS,18,2)
            cv2.arrowedLine(image,start,end,(255,0,0),2)
            cv2.putText(image,f'{item["id"]}: ({centre[0]:.1f},{centre[1]:.1f}), {item["long_axis_angle_deg"]:.1f} deg',
                        start,cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),2)
    text=json.dumps(output,indent=2,allow_nan=False)
    if a.output:
        a.output.parent.mkdir(parents=True,exist_ok=True)
        with a.output.open('x') as stream:stream.write(text+'\n')
    if a.overlay:
        a.overlay.parent.mkdir(parents=True,exist_ok=True)
        if not cv2.imwrite(str(a.overlay),image):raise OSError('Overlay save failed')
    print(text)


if __name__=='__main__':main()
