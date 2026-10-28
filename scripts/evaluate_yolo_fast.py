"""Check compact-mask inference against real labels, including restored pixel geometry."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import tomllib
import cv2
import numpy as np
from prepare_yolo_pilot import NAMES
from yolo_pixels import YoloModels
from evaluate_yolo_boards import ground_truth,mask_ious,assess
from b_live_geometry import measure,supported_quad,quad_center_and_axis,draw


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--settings',type=Path,required=True)
    p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args()
    if a.output_dir.exists():p.error('Output exists; use a new folder')
    settings=json.loads(a.settings.read_text());config=tomllib.loads(Path(settings['scene_config']).read_text())
    model=YoloModels(settings);a.output_dir.mkdir(parents=True)
    summary={'settings':settings,'splits':{},'execution_allowed':False,
             'timing_scope':'Includes full-resolution diagnostic masks; use ROS replay for live timing'}
    for split in ('val','test'):
        folder=a.output_dir/split;folder.mkdir();rows=[];totals={name:Counter() for name in NAMES}
        for path in sorted((a.dataset/'images'/split).glob('*')):
            image=cv2.imread(str(path));report,accepted,masks,_=model.detect(image,config,include_masks=True)
            classes=np.array([NAMES.index(c) for c in report['prediction_class_names']],int)
            gt_classes,gt_masks=ground_truth(a.dataset/'labels'/split/(path.stem+'.txt'),image.shape)
            checked=assess(mask_ious(gt_masks,masks),gt_classes,classes)
            objects=[measure(c,m) for c,m in accepted];matches={j:i for i,j,_ in checked['matches']};checks=[]
            for obj in objects:
                index=int(obj['candidate_id'].split('_')[-1])-1
                if index not in matches:continue
                reference=matches[index];q=supported_quad(gt_masks[reference])
                if q is None:continue
                centre,angle=quad_center_and_axis(q)
                checks.append({'class_name':obj['class_name'],
                    'centre_error_px':float(np.linalg.norm(centre-obj['centre_px'])),
                    'image_angle_error_deg':float(abs((angle-obj['orientation_image_deg']+90)%180-90))})
            for name,values in checked['per_class'].items():totals[name].update(values)
            checked.update(image=path.name,adapter_boards=len(objects),adapter_uncertain=len(report['uncertain_candidates']),
                           adapter_objects=objects,supported_outline_pixel_checks=checks,
                           unmatched_coordinate_targets=[o['candidate_id'] for o in objects if int(o['candidate_id'].split('_')[-1])-1 not in matches])
            rows.append(checked)
            cv2.imwrite(str(folder/(path.stem+'.jpg')),draw(image.copy(),objects))
            print(json.dumps({'split':split,'image':path.name,'coordinates':len(objects),
                              'uncertain':checked['adapter_uncertain'],'missed_rows':len(checked['missed_annotation_ids'])}),flush=True)
        counts={name:dict(v) for name,v in totals.items()}
        for v in counts.values():
            v['precision']=v['tp']/(v['tp']+v['fp']) if v['tp']+v['fp'] else None
            v['recall']=v['tp']/(v['tp']+v['fn']) if v['tp']+v['fn'] else None
        result={'fixed_confidence_per_class':counts,'images':rows}
        (folder/'report.json').write_text(json.dumps(result,indent=2))
        summary['splits'][split]={'fixed_confidence_per_class':counts,'images':len(rows),
            'coordinate_targets':sum(r['adapter_boards'] for r in rows),
            'uncertain_proposals':sum(r['adapter_uncertain'] for r in rows)}
        (a.output_dir/'summary.json').write_text(json.dumps(summary,indent=2))


if __name__=='__main__':main()
