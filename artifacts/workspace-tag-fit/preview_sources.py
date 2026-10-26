"""Offline candidate-calibration preview. No ROS or robot targets."""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from perceive_scene import detect_source_candidates
from plane_geometry import pixels_to_plane, validate_transform
from classify_board import classify_board


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--fit', type=Path, required=True)
    parser.add_argument('--roi', type=int, nargs=4, action='append', required=True)
    parser.add_argument('--sheet-top-z', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not np.isfinite(args.sheet_top_z):
        raise ValueError('Sheet top height must be finite')
    frame = cv2.imread(str(args.image))
    if frame is None or frame.shape[:2] != (1080, 1920):
        raise ValueError('Expected readable original 1920x1080 image')
    fit = json.loads(args.fit.read_text())
    selected = min(fit['candidates'], key=lambda c:c['fit_rms_px'])
    T = validate_transform(selected['T_base_link_camera'])
    report = dict(image=str(args.image.resolve()), fit=str(args.fit.resolve()),
                  T_base_link_camera=T.tolist(), frame_id='base_link',
                  assumed_sheet_top_z_m=args.sheet_top_z, rois=args.roi,
                  purpose='offline_candidate_calibration_preview',
                  independent_accuracy_validated=False, execution_allowed=False,
                  robot_targets=[], candidates=[], rejected_regions=[])
    overlay = frame.copy()
    for region, roi in enumerate(args.roi, 1):
        x0,y0,x1,y1=roi
        if not (0 <= x0 < x1 <= 1920 and 0 <= y0 < y1 <= 1080):
            raise ValueError('ROI outside image')
        candidates,rejected = detect_source_candidates(frame,roi)
        report['rejected_regions'].extend(rejected)
        cv2.rectangle(overlay,(x0,y0),(x1-1,y1-1),(255,180,0),2)
        for c in candidates:
            c['candidate_id']=f"region-{region}-{c['candidate_id']}"
            corners = pixels_to_plane(c['outline_px'],fit['camera_matrix'],fit['distortion_coefficients'],T,args.sheet_top_z)
            classification = classify_board(corners,c['colour'])
            centre = corners.mean(axis=0)
            lengths=np.linalg.norm(np.roll(corners,-1,axis=0)-corners,axis=1)*1000
            c.update(metric_outline_m=corners.tolist(), edge_lengths_mm=lengths.tolist(), diagnostic_centre_m=centre.tolist(), metric_classification=classification, execution_allowed=False)
            report['candidates'].append(c)
            poly=np.rint(c['outline_px']).astype(np.int32)
            cv2.polylines(overlay,[poly],True,(0,255,255),2)
            anchor=np.rint(poly.mean(axis=0)).astype(int)
            label=f"{classification['variant'] or 'UNMATCHED'}: X {centre[0]*1000:.0f} Y {centre[1]*1000:.0f} mm"
            cv2.putText(overlay,label,tuple(anchor),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),4)
            cv2.putText(overlay,label,tuple(anchor),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1)
            print(c['candidate_id'],c['colour'],'centre mm',np.round(centre*1000,1).tolist(),'classification',classification)
    cv2.putText(overlay,'DIAGNOSTIC ONLY - UNVALIDATED TRANSFORM - NO ROBOT TARGETS',(15,35),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
    args.output.mkdir(parents=True,exist_ok=False)
    if not cv2.imwrite(str(args.output/'preview.png'),overlay):
        raise RuntimeError('Cannot save preview')
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Saved',args.output.resolve())


if __name__ == '__main__':
    main()
