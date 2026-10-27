"""Click placement centres and long-axis directions; save preview transforms only."""
import argparse
import copy
import json
from pathlib import Path
import cv2
import numpy as np
from sorting_geometry import bin_observations, rigid, rays_to_plane


def placement_offset(camera_tag, centre_pixel, axis_pixel, distance, calibration):
    base_tag = rigid(calibration['T_base_camera']) @ rigid(camera_tag)
    xyz = rays_to_plane([centre_pixel, axis_pixel], calibration,
                         base_tag[:3, 3], base_tag[:3, 2])
    local = (xyz-base_tag[:3, 3]) @ base_tag[:3, :3]
    radial = local[0].copy(); radial[2] = 0
    x = local[1]-local[0]; x[2] = 0
    if np.linalg.norm(radial) < .005 or np.linalg.norm(x) < .005:
        raise ValueError('Click farther from tag centre / farther along long edge')
    x /= np.linalg.norm(x)
    t = np.eye(4)
    t[:3, 0] = x; t[:3, 1] = [-x[1], x[0], 0]; t[:3, 2] = [0, 0, 1]
    t[:3, 3] = radial/np.linalg.norm(radial)*distance
    return rigid(t)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--settings', type=Path, required=True)
    p.add_argument('--image', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--tag-support-coplanar', action='store_true',
                   help='Use only when tags lie on the receiving support plane')
    a = p.parse_args()
    if not a.tag_support_coplanar:
        p.error('This shortcut requires tags on the same plane as their receiving surface')
    if a.output.exists(): raise ValueError('Choose a new output settings filename')
    settings = json.loads(a.settings.read_text())
    calibration = json.loads(Path(settings['scene_calibration']).read_text())
    image = cv2.imread(str(a.image))
    if image is None or [image.shape[1], image.shape[0]] != calibration['image_size']:
        raise ValueError('Need original-resolution Logitech frame')
    probe = copy.deepcopy(settings)
    for b in probe['bins'].values(): b['T_tag_placement'] = None
    tags = {b['tag_id']: b for b in bin_observations(image, probe, calibration)}
    scale = min(1., 1280/image.shape[1], 800/image.shape[0])
    window = 'Bin setup: centre then long-axis direction'
    cv2.namedWindow(window)
    try:
        for name, spec in settings['classes'].items():
            tag_id = spec['destination_tag_id']; observed = tags[tag_id]
            if 'T_camera_tag' not in observed:
                raise ValueError(f'Tag {tag_id}: {observed["reasons"]}')
            if observed['tag_reprojection_rms_px'] > settings['maximum_tag_rms_px']:
                raise ValueError(f'Tag {tag_id} pose too uncertain')
            clicks = []
            def callback(event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 2:
                    clicks.append([(x+.5)/scale-.5, (y+.5)/scale-.5])
            cv2.setMouseCallback(window, callback)
            while True:
                view = cv2.resize(image, None, fx=scale, fy=scale)
                for i, point in enumerate(clicks):
                    pos = tuple(np.rint(np.array(point)*scale).astype(int))
                    cv2.circle(view, pos, 5, (0, 255, 0), 2)
                    cv2.putText(view, str(i+1), pos, cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,0), 2)
                action = 'click desired CENTRE' if not clicks else 'click along required LONG axis' if len(clicks)==1 else 'Enter: accept; U: undo'
                cv2.putText(view, f'{name}, tag {tag_id}: {action}', (15,30), cv2.FONT_HERSHEY_SIMPLEX, .65, (0,255,255), 2)
                cv2.imshow(window, view)
                key = cv2.waitKey(30)&255
                if key == 27: print('Cancelled; no settings saved'); return
                if key in (ord('u'),ord('U')) and clicks: clicks.pop()
                if key in (10,13) and len(clicks)==2:
                    b = settings['bins'][str(tag_id)]
                    t = placement_offset(observed['T_camera_tag'], clicks[0], clicks[1],
                                          b['nominal_centre_distance_m'], calibration)
                    # Offset origin is receiving surface; board frame is its top surface.
                    t[2,3] = spec['size_m'][2]
                    b['T_tag_placement'] = t.tolist()
                    b['stack_count'] = 0
                    b['offset_source'] = 'Clicked direction, nominal measured distance, empty destination; preview only'
                    b['click_pixels'] = clicks
                    break
    finally:
        cv2.destroyAllWindows()
    with a.output.open('x') as stream: json.dump(settings, stream, indent=2)
    print('Saved:', a.output)
    print('Top-surface placement assumes empty bins and tags coplanar with receiving surfaces.')
    print('Verify axes physically; update destination stack height after every placement.')


if __name__ == '__main__': main()
