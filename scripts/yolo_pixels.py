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
    if masks.dtype.kind not in 'buif':raise ValueError('Masks must be numeric')
    if (masks.dtype.kind=='f' and not np.isfinite(masks).all()) or not np.isfinite(classes).all() or not np.isfinite(scores).all():
        raise ValueError('Non-finite prediction')
    if np.any((scores<0)|(scores>1)) or np.any(classes!=np.floor(classes)) or np.any((classes<0)|(classes>=4)):
        raise ValueError('Invalid class ID or confidence')
    roi=region_mask(image.shape,config['classical']['pickup_polygon_normalized'])
    boundary=cv2.subtract(roi,cv2.erode(roi,np.ones((5,5),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0))
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
        try:candidate={**describe_mask(image,mask,roi,config['classical'],roi_boundary=boundary,
                                      measure_appearance=False),**metadata}
        except ValueError as error:
            rejected.append({'candidate_id':metadata['id'],'class_name':name,'reason':str(error)});continue
        geometry=measure(candidate,mask)
        candidate['supported_mask_quad']=geometry['supported_pixel_corners']
        if geometry['supported_pixel_corners'] is None:
            uncertain.append({**candidate,'detection_status':'uncertain',
                'uncertainty_reason':'Instance detected; full outline not supported'})
        else:accepted.append((candidate,mask))
    report={'image_size':[image.shape[1],image.shape[0]],'model_backend':'yolo11_seg',
            'uncertain_candidates':uncertain,'hand_detections':hands,'rejected_proposals':rejected,
            'execution_allowed':False,'calibration_valid':False}
    return report,accepted,[m>.5 for m in masks],roi


def restore_geometry(report,accepted,original_size,working_size):
    """Map downsampled mask measurements to original pixel centres, not resized pixels."""
    sx=original_size[0]/working_size[0];sy=original_size[1]/working_size[1]
    def points(value):
        return ((np.asarray(value,float)+.5)*[sx,sy]-.5).tolist()
    candidates=[c for c,_ in accepted]+report['uncertain_candidates']
    for candidate in candidates:
        for key in ('pixel_corners','pixel_center'):
            candidate[key]=points(candidate[key])
        candidate['pixel_contours']=[points(c) for c in candidate['pixel_contours']]
        if candidate.get('supported_mask_quad') is not None:
            candidate['supported_mask_quad']=points(candidate['supported_mask_quad'])
        candidate['area_pixels']=float(candidate['area_pixels']*sx*sy)
        _,sides,_=cv2.minAreaRect(np.asarray(candidate['pixel_corners'],np.float32))
        candidate['size']['short_side_px'],candidate['size']['long_side_px']=sorted(map(float,sides))
    for hand in report['hand_detections']:
        hand['centre_px']=points(hand['centre_px'])
        hand['pixel_contours']=[points(c) for c in hand['pixel_contours']]
        x0,y0,x1,y1=hand['box_xyxy']
        hand['box_xyxy']=[math.floor(x0*sx),math.floor(y0*sy),math.ceil(x1*sx),math.ceil(y1*sy)]
    report['image_size']=list(original_size)
    report['geometry_image_size']=list(working_size)
    report['geometry_pixel_scale']=[sx,sy]
    return [(candidate,None) for candidate,_ in accepted]


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
        width=settings.get('geometry_max_width',0)
        if type(width) is not int or (width!=0 and width<320):raise ValueError('geometry_max_width must be zero or at least 320')
        if type(settings.get('fp16',False)) is not bool:raise ValueError('fp16 must be boolean')
        folder=Path('.local/ultralytics').resolve();folder.mkdir(parents=True,exist_ok=True)
        os.environ.setdefault('YOLO_CONFIG_DIR',str(folder))
        import torch
        from ultralytics import YOLO
        if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable; CPU fallback disabled')
        # GPU inference leaves only small CPU masks; large thread pools add latency here.
        cv2.setNumThreads(1);torch.set_num_threads(1)
        self.torch=torch;self.settings=settings;self.model=YOLO(str(weights))
        if self.model.task!='segment':raise ValueError('Need an instance-segmentation checkpoint')
        self.names=class_names(self.model.names)
        self.checkpoint_sha256=hashlib.sha256(weights.read_bytes()).hexdigest()

    def detect(self,image,config,include_masks=True):
        s=self.settings;torch=self.torch
        torch.cuda.synchronize();started=time.perf_counter()
        compact=bool(s.get('geometry_max_width',0))
        result=self.model.predict(source=image,device=0,imgsz=s['imgsz'],conf=s['confidence'],
            iou=s['nms_iou'],max_det=s['max_detections'],retina_masks=not compact,agnostic_nms=False,
            half=s.get('fp16',False),verbose=False,save=False)[0]
        if tuple(result.orig_shape)!=image.shape[:2]:raise ValueError('YOLO original image shape mismatch')
        count=len(result.boxes) if result.boxes is not None else 0
        if count and result.masks is None:raise ValueError('Detections returned without instance masks')
        original_size=(image.shape[1],image.shape[0])
        width=min(image.shape[1],s.get('geometry_max_width',0) or image.shape[1])
        height=max(1,round(image.shape[0]*width/image.shape[1]));working_size=(width,height)
        data=result.masks.data.detach() if count else None
        if data is not None and not compact and tuple(data.shape[1:])!=image.shape[:2]:
            raise ValueError('Retina masks must match the original image')
        if compact:
            from ultralytics.utils.ops import scale_masks
            working=cv2.resize(image,working_size,interpolation=cv2.INTER_LINEAR)
            # Remove network letterboxing on GPU before the much smaller CPU transfer.
            small=(scale_masks(data[None].float(),(height,width))[0]>.5).to(torch.uint8) if count else None
        else:working=image;small=data
        masks=small.cpu().numpy() if count else np.empty((0,height,width),bool)
        classes=result.boxes.cls.detach().cpu().numpy() if count else np.empty(0)
        scores=result.boxes.conf.detach().cpu().numpy() if count else np.empty(0)
        report,accepted,raw,roi=translate_predictions(working,masks,classes,scores,self.names,config)
        if compact:
            accepted=restore_geometry(report,accepted,original_size,working_size)
            roi=region_mask(image.shape,config['classical']['pickup_polygon_normalized'])
            raw=[m for m in (scale_masks(data[None].float(),image.shape[:2])[0]>.5).cpu().numpy()] if count and include_masks else []
        elif not include_masks:raw=[]
        torch.cuda.synchronize()
        report.update(detector_ms=(time.perf_counter()-started)*1000,
                      checkpoint_sha256=self.checkpoint_sha256,
                      prediction_class_names=[self.names[int(c)] for c in classes],
                      prediction_scores=scores.tolist())
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
