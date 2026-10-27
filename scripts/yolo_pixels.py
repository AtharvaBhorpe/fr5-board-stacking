"""YOLO11 instance masks adapted to the existing pixel-observation interface."""
import hashlib
import json
import math
import os
from pathlib import Path
import time
import cv2
import numpy as np
from fr5_perception.detector import region_mask
from inspect_b import describe_mask
from b_live_geometry import measure
from prepare_yolo_pilot import NAMES,canonical_names


def class_names(names):
    return canonical_names(names)


def translate_predictions(image,masks,classes,scores,names,config):
    names=class_names(names);masks=np.asarray(masks);classes=np.asarray(classes);scores=np.asarray(scores)
    if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3:
        raise ValueError('Expected original uint8 BGR image')
    if masks.ndim!=3 or masks.shape[1:]!=image.shape[:2]:
        raise ValueError('Masks must match original image resolution; enable retina_masks')
    if classes.shape!=(len(masks),) or scores.shape!=(len(masks),):raise ValueError('Mask/class/score counts differ')
    if not np.isfinite(masks).all() or not np.isfinite(classes).all() or not np.isfinite(scores).all():
        raise ValueError('Non-finite prediction')
    if np.any((scores<0)|(scores>1)) or np.any(classes!=np.floor(classes)) or np.any((classes<0)|(classes>=4)):
        raise ValueError('Invalid class ID or confidence')
    roi=region_mask(image.shape,config['classical']['pickup_polygon_normalized'])
    accepted=[];uncertain=[];hands=[];rejected=[]
    for number,(raw,category,score) in enumerate(zip(masks,classes,scores),1):
        mask=raw>.5;name=names[int(category)]
        metadata={'id':f'candidate_{number}','class_name':name,'class_id':NAMES.index(name),
                  'confidence':float(score),'detector_score':float(score),'text_label':name,
                  'mask_origin':'yolo11_seg','layer_order_verified':False}
        if not mask.any():
            rejected.append({'candidate_id':metadata['id'],'reason':'Empty predicted mask'});continue
        if name=='hand':
            ys,xs=np.where(mask)
            contours,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            hands.append({'candidate_id':f'hand_{number}','class_name':'hand','class_id':NAMES.index(name),
                'confidence':float(score),'centre_px':[float(xs.mean()),float(ys.mean())],
                'box_xyxy':[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)],
                'pixel_contours':[c.reshape(-1,2).tolist() for c in contours],
                'human_safety_validated':False,'execution_allowed':False})
            continue
        try:candidate={**describe_mask(image,mask,roi,config['classical']),**metadata}
        except ValueError as error:
            rejected.append({'candidate_id':metadata['id'],'class_name':name,'reason':str(error)});continue
        geometry=measure(candidate,mask)
        if geometry['supported_pixel_corners'] is None:
            uncertain.append({**candidate,'detection_status':'uncertain',
                'uncertainty_reason':'Instance detected; full outline not supported'})
        else:accepted.append((candidate,mask))
    report={'image_size':[image.shape[1],image.shape[0]],'model_backend':'yolo11_seg',
            'uncertain_candidates':uncertain,'hand_detections':hands,'rejected_proposals':rejected,
            'execution_allowed':False,'calibration_valid':False}
    return report,accepted,[m>.5 for m in masks],roi


def draw_hands(image,hands):
    for hand in hands:
        contours=[np.asarray(c,np.int32).reshape(-1,1,2) for c in hand['pixel_contours']]
        cv2.drawContours(image,contours,-1,(255,0,255),2)
        centre=tuple(np.rint(hand['centre_px']).astype(int))
        cv2.putText(image,f"HAND {hand['confidence']:.2f}",centre,cv2.FONT_HERSHEY_SIMPLEX,.6,(255,0,255),2)
    return image


class YoloModels:
    def __init__(self,settings):
        weights=Path(settings['weights'])
        if not weights.is_file():raise FileNotFoundError('Trained checkpoint missing: '+str(weights))
        for key in ('confidence','nms_iou'):
            if not math.isfinite(settings[key]) or not 0<settings[key]<=1:raise ValueError('Invalid '+key)
        for key in ('imgsz','max_detections'):
            if type(settings[key]) is not int or settings[key]<1:raise ValueError('Invalid '+key)
        folder=Path('.local/ultralytics').resolve();folder.mkdir(parents=True,exist_ok=True)
        os.environ.setdefault('YOLO_CONFIG_DIR',str(folder))
        import torch
        from ultralytics import YOLO
        if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable; CPU fallback disabled')
        self.torch=torch;self.settings=settings;self.model=YOLO(str(weights))
        if self.model.task!='segment':raise ValueError('Need an instance-segmentation checkpoint')
        self.names=class_names(self.model.names)
        self.checkpoint_sha256=hashlib.sha256(weights.read_bytes()).hexdigest()

    def detect(self,image,config):
        s=self.settings;torch=self.torch
        torch.cuda.synchronize();started=time.perf_counter()
        result=self.model.predict(source=image,device=0,imgsz=s['imgsz'],conf=s['confidence'],
            iou=s['nms_iou'],max_det=s['max_detections'],retina_masks=True,agnostic_nms=False,
            verbose=False,save=False)[0]
        if tuple(result.orig_shape)!=image.shape[:2]:raise ValueError('YOLO original image shape mismatch')
        count=len(result.boxes) if result.boxes is not None else 0
        if count and result.masks is None:raise ValueError('Detections returned without instance masks')
        masks=result.masks.data.detach().cpu().numpy() if count else np.empty((0,*image.shape[:2]),bool)
        classes=result.boxes.cls.detach().cpu().numpy() if count else np.empty(0)
        scores=result.boxes.conf.detach().cpu().numpy() if count else np.empty(0)
        report,accepted,raw,roi=translate_predictions(image,masks,classes,scores,self.names,config)
        torch.cuda.synchronize()
        report.update(detector_ms=(time.perf_counter()-started)*1000,
                      checkpoint_sha256=self.checkpoint_sha256)
        return report,accepted,raw,roi


def main():
    import argparse
    import tomllib
    from b_live_geometry import draw
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--settings',type=Path,default=Path('config/yolo_live.json'))
    p.add_argument('--image',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args()
    if a.output_dir.exists():p.error('Output exists; use a new run folder')
    image=cv2.imread(str(a.image))
    if image is None:p.error('Cannot read image')
    settings=json.loads(a.settings.read_text())
    config=tomllib.loads(Path(settings['scene_config']).read_text())
    report,accepted,masks,_=YoloModels(settings).detect(image,config)
    objects=[measure(item,mask) for item,mask in accepted]
    uncertain=[measure(item) for item in report['uncertain_candidates']]
    report.update(objects=objects,uncertain_objects=uncertain,source=str(a.image.resolve()),
                  coordinate_system='raw_image_pixels',source_header_available=False)
    a.output_dir.mkdir(parents=True,exist_ok=False)
    visual=draw_hands(draw(image.copy(),uncertain+objects),report['hand_detections'])
    if not cv2.imwrite(str(a.output_dir/'overlay.jpg'),visual):raise OSError('Overlay save failed')
    for i,mask in enumerate(masks,1):
        if not cv2.imwrite(str(a.output_dir/f'mask_{i:03}.png'),mask.astype(np.uint8)*255):raise OSError('Mask save failed')
    (a.output_dir/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({'boards':len(objects),'uncertain_boards':len(uncertain),
        'hands':len(report['hand_detections']),'detector_ms':report['detector_ms'],
        'output':str(a.output_dir),'execution_allowed':False},indent=2))


if __name__=='__main__':main()
