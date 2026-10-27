"""Evaluate a fixed YOLO checkpoint and export per-image annotation comparisons."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import time
import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
from prepare_yolo_pilot import NAMES
from yolo_pixels import class_names,translate_predictions
from b_live_geometry import measure,supported_quad,quad_center_and_axis


def mask_ious(reference,predicted):
    values=np.zeros((len(reference),len(predicted)),float)
    areas_a=[np.count_nonzero(m) for m in reference]
    areas_b=[np.count_nonzero(m) for m in predicted]
    boxes_a=[cv2.boundingRect(m.astype(np.uint8)) for m in reference]
    boxes_b=[cv2.boundingRect(m.astype(np.uint8)) for m in predicted]
    for i,(x,y,w,h) in enumerate(boxes_a):
        for j,(u,v,a,b) in enumerate(boxes_b):
            left,top=max(x,u),max(y,v);right,bottom=min(x+w,u+a),min(y+h,v+b)
            if right<=left or bottom<=top:continue
            inter=np.count_nonzero(reference[i][top:bottom,left:right]&predicted[j][top:bottom,left:right])
            union=areas_a[i]+areas_b[j]-inter
            if union:values[i,j]=inter/union
    return values


def assign(ious,allowed):
    if not ious.size:return []
    # One additional valid match must outweigh all possible IoU tie-break gains.
    reward=np.where(allowed,min(ious.shape)+1+ious,0.)
    rows,cols=linear_sum_assignment(-reward)
    return [(int(i),int(j)) for i,j in zip(rows,cols) if allowed[i,j]]


def assess(ious,gt_classes,pred_classes,threshold=.5):
    gt_classes=np.asarray(gt_classes);pred_classes=np.asarray(pred_classes)
    same=gt_classes[:,None]==pred_classes[None,:]
    matches=assign(ious,(ious>=threshold)&same)
    unmatched_gt=[i for i in range(len(gt_classes)) if i not in {i for i,j in matches}]
    unmatched_pred=[j for j in range(len(pred_classes)) if j not in {j for i,j in matches}]
    wrong=[]
    if unmatched_gt and unmatched_pred:
        sub=ious[np.ix_(unmatched_gt,unmatched_pred)]
        for i,j in assign(sub,sub>=threshold):
            a,b=unmatched_gt[i],unmatched_pred[j]
            if gt_classes[a]!=pred_classes[b]:wrong.append([a,b])
    counts={}
    for category,name in enumerate(NAMES):
        tp=sum(gt_classes[i]==category for i,j in matches)
        fp=sum(pred_classes[j]==category for j in unmatched_pred)
        fn=sum(gt_classes[i]==category for i in unmatched_gt)
        counts[name]={'tp':int(tp),'fp':int(fp),'fn':int(fn)}
    return {'matches':[[i,j,float(ious[i,j])] for i,j in matches],
            'missed_annotation_ids':unmatched_gt,'unmatched_prediction_ids':unmatched_pred,
            'wrong_class_pairs':wrong,'per_class':counts}


def ground_truth(path,shape):
    classes=[];masks=[]
    for line in path.read_text().splitlines():
        if not line.strip():continue
        tokens=line.split();classes.append(int(tokens[0]))
        points=np.asarray(list(map(float,tokens[1:]))).reshape(-1,2)*[shape[1],shape[0]]
        mask=np.zeros(shape[:2],np.uint8);cv2.fillPoly(mask,[np.rint(points).astype(np.int32)],1)
        masks.append(mask.astype(bool))
    return np.asarray(classes,int),masks


def annotation_overlay(image,masks,classes,prefix,marked):
    visual=image.copy();colours=[(255,255,0),(0,165,255),(0,0,255),(255,0,255)]
    for i,(mask,category) in enumerate(zip(masks,classes)):
        contours,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(visual,contours,-1,colours[category],3)
        ys,xs=np.where(mask)
        if not len(xs):continue
        label=f'{prefix}{i} {NAMES[category]}'+(' MISSED/UNMATCHED' if i in marked else '')
        cv2.putText(visual,label,(int(xs.min()),max(20,int(ys.min())-5)),cv2.FONT_HERSHEY_SIMPLEX,.55,colours[category],2)
    return visual


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--weights',type=Path,required=True);p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--imgsz',type=int,default=960)
    p.add_argument('--confidence',type=float,default=.25);p.add_argument('--scene-config',type=Path,default=Path('config/perception_full_table.toml'))
    a=p.parse_args()
    if a.output_dir.exists():p.error('Output exists; preserve previous evaluation')
    os.environ.setdefault('YOLO_CONFIG_DIR',str(Path('.local/ultralytics').resolve()))
    import torch,tomllib
    from ultralytics import YOLO
    if not torch.cuda.is_available():raise RuntimeError('CUDA required')
    model=YOLO(str(a.weights));names=class_names(model.names)
    config=tomllib.loads(a.scene_config.read_text());a.output_dir.mkdir(parents=True)
    summary={'checkpoint':str(a.weights.resolve()),'confidence':a.confidence,'mask_iou_threshold':.5,
             'imgsz':a.imgsz,'splits':{},'execution_allowed':False,
             'limits':['Scores compare predictions against exported annotation rows, which may contain errors or fragmented physical instances.',
                       'Pixel centres and angles are not robot coordinates. No physical grasp was tested.']}
    for split in ('val','test'):
        folder=a.output_dir/split;folder.mkdir()
        metrics=model.val(data=str((a.dataset/'data.yaml').resolve()),split=split,device=0,
            imgsz=a.imgsz,batch=8,plots=True,project=str(folder.resolve()),name='standard_metrics',verbose=False)
        standard={k:float(v) for k,v in metrics.results_dict.items()}
        totals={name:Counter() for name in NAMES};rows=[];label_warnings=[]
        for path in sorted((a.dataset/'images'/split).iterdir()):
            image=cv2.imread(str(path));gt_classes,gt_masks=ground_truth(a.dataset/'labels'/split/(path.stem+'.txt'),image.shape)
            torch.cuda.synchronize();started=time.perf_counter()
            result=model.predict(image,imgsz=a.imgsz,conf=a.confidence,iou=.7,retina_masks=True,device=0,verbose=False)[0]
            torch.cuda.synchronize();elapsed=(time.perf_counter()-started)*1000
            raw_masks=result.masks.data.cpu().numpy() if result.masks is not None else np.empty((0,*image.shape[:2]),bool)
            source_classes=result.boxes.cls.cpu().numpy().astype(int)
            pred_classes=np.array([NAMES.index(names[c]) for c in source_classes],int)
            masks=[m>.5 for m in raw_masks];scores=result.boxes.conf.cpu().numpy()
            checked=assess(mask_ious(gt_masks,masks),gt_classes,pred_classes)
            gt_iou=mask_ious(gt_masks,gt_masks)
            for i in range(len(gt_masks)):
                for j in range(i+1,len(gt_masks)):
                    if gt_iou[i,j]>=.9:label_warnings.append({'image':path.name,'annotation_ids':[i,j],'mask_iou':float(gt_iou[i,j]),'reason':'Near-duplicate ground-truth masks'})
            report,accepted,_,_=translate_predictions(image,raw_masks,source_classes,scores,names,config)
            objects=[measure(c,m) for c,m in accepted]
            pixel_checks=[]
            matches={j:i for i,j,_ in checked['matches']}
            for obj in objects:
                prediction=int(obj['candidate_id'].split('_')[-1])-1
                if prediction not in matches:continue
                reference=matches[prediction];quad=supported_quad(gt_masks[reference])
                if quad is None:continue
                centre,angle=quad_center_and_axis(quad)
                pixel_checks.append({'annotation_id':reference,'prediction_id':prediction,
                    'class_name':obj['class_name'],
                    'centre_error_px':float(np.linalg.norm(centre-obj['centre_px'])),
                    'image_angle_error_deg':float(abs((angle-obj['orientation_image_deg']+90)%180-90))})
            for name,count in checked['per_class'].items():totals[name].update(count)
            checked.update(image=path.name,predicted_classes=[NAMES[c] for c in pred_classes],
                ground_truth_classes=[NAMES[c] for c in gt_classes],prediction_ms=elapsed,
                adapter_boards=len(accepted),adapter_uncertain=len(report['uncertain_candidates']),
                adapter_objects=objects,supported_outline_pixel_checks=pixel_checks)
            rows.append(checked)
            left=annotation_overlay(image,gt_masks,gt_classes,'GT',set(checked['missed_annotation_ids']))
            right=annotation_overlay(image,masks,pred_classes,'P',set(checked['unmatched_prediction_ids']))
            cv2.putText(left,'EXPORTED LABELS',(20,35),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2)
            cv2.putText(right,'PREDICTIONS conf='+str(a.confidence),(20,35),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2)
            comparison=np.hstack([cv2.resize(left,(960,540)),cv2.resize(right,(960,540))])
            if not cv2.imwrite(str(folder/(path.stem+'.jpg')),comparison):raise OSError('Comparison save failed')
            print(json.dumps({'split':split,'image':path.name,'missed_rows':len(checked['missed_annotation_ids']),
                'unmatched_predictions':len(checked['unmatched_prediction_ids']),'adapter_boards':len(accepted)}),flush=True)
        counts={name:dict(c) for name,c in totals.items()}
        for value in counts.values():
            tp,fp,fn=value['tp'],value['fp'],value['fn']
            value.update(precision=tp/(tp+fp) if tp+fp else None,recall=tp/(tp+fn) if tp+fn else None)
        split_report={'standard_metrics':standard,'fixed_confidence_per_class':counts,'images':rows,
                      'annotation_warnings':label_warnings,
                      'prediction_ms_median':float(np.median([r['prediction_ms'] for r in rows])),
                      'prediction_ms_p95':float(np.percentile([r['prediction_ms'] for r in rows],95))}
        (folder/'report.json').write_text(json.dumps(split_report,indent=2,allow_nan=False))
        # Four comparison images per page, with all images available individually.
        for start in range(0,len(rows),4):
            tiles=[cv2.resize(cv2.imread(str(folder/(Path(r['image']).stem+'.jpg'))),(1280,360)) for r in rows[start:start+4]]
            cv2.imwrite(str(folder/f'contact_{start//4+1:02}.jpg'),np.vstack(tiles))
        summary['splits'][split]={k:v for k,v in split_report.items() if k!='images'}
        (a.output_dir/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
    print('Evaluation report:',a.output_dir/'summary.json',flush=True)


if __name__=='__main__':main()
