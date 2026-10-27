"""Re-render existing B masks with supported quadrilaterals; no model rerun."""
import argparse
import json
from pathlib import Path
import cv2
from b_live_geometry import measure,draw,self_test


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',type=Path)
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--self-test',action='store_true')
    a=p.parse_args()
    if a.self_test:self_test();return
    if a.report is None or a.output_dir is None:p.error('Need report and output-dir')
    report=json.loads(a.report.read_text());image=cv2.imread(report['source_image'])
    if image is None:raise ValueError('Source image not readable')
    objects=[]
    for c in report['quad_candidates']:
        mask=cv2.imread(str(a.report.parent/c['mask_file']),cv2.IMREAD_GRAYSCALE)
        if mask is None or mask.shape!=image.shape[:2]:raise ValueError('Instance mask missing/mismatched')
        objects.append(measure(c,mask))
    a.output_dir.mkdir(parents=True,exist_ok=False)
    if not cv2.imwrite(str(a.output_dir/'overlay.jpg'),draw(image,objects)):raise OSError('Image save failed')
    result={'coordinate_system':'raw_image_pixels','objects':objects,'execution_allowed':False,
            'legend':'Green: supported four-corner mask outline. Orange: enclosing rectangle fallback.',
            'limits':['Neither outline mode certifies instance separation or physical board identity.',
                      'An image-plane long-axis angle is not robot yaw.']}
    (a.output_dir/'coordinates.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
