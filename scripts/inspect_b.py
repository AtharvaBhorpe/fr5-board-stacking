"""Offline GPU Grounding DINO + SAM 2 comparison. Never commands a robot."""
import argparse
import csv
from pathlib import Path
import hashlib
import json
import time
import tomllib

import cv2
import numpy as np
from fr5_perception.detector import region_mask
from fr5_perception.measurements import describe_region

FIELDS=['key','scenario','reviewed','tp_ids','fp_ids','missed_boards','failure_tags','notes']


def nms(boxes,scores,threshold):
    boxes=np.asarray(boxes,float).reshape(-1,4)
    scores=np.asarray(scores,float)
    if len(boxes)!=len(scores) or not np.isfinite(boxes).all() or not np.isfinite(scores).all():
        raise ValueError('Invalid detection boxes or scores')
    order=np.argsort(-scores);keep=[]
    while len(order):
        first=int(order[0]);keep.append(first);rest=order[1:]
        if not len(rest):break
        lo=np.maximum(boxes[first,:2],boxes[rest,:2])
        hi=np.minimum(boxes[first,2:],boxes[rest,2:])
        intersection=np.prod(np.maximum(hi-lo,0),axis=1)
        areas=np.prod(np.maximum(boxes[:,2:]-boxes[:,:2],0),axis=1)
        union=areas[first]+areas[rest]-intersection
        overlap=np.divide(intersection,union,out=np.zeros_like(intersection),where=union>0)
        order=rest[overlap<=threshold]
    return keep


def describe_mask(image,mask,roi,settings):
    mask=np.asarray(mask,dtype=bool)
    if mask.shape!=image.shape[:2]:raise ValueError('Mask and image shapes differ')
    area=int(mask.sum());fraction=area/mask.size
    if not settings['min_area_fraction']<=fraction<=settings['max_area_fraction']:
        raise ValueError('Mask area outside configured limits')
    if np.any(mask & (roi==0)):
        raise ValueError('Mask extends outside pickup region')
    boundary=cv2.subtract(roi,cv2.erode(roi,np.ones((5,5),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0))
    if np.any(mask & (boundary>0)):
        raise ValueError('Mask touches pickup/image boundary')
    binary=mask.astype(np.uint8)*255
    contours,_=cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    rect=cv2.minAreaRect(np.concatenate(contours))
    if min(rect[1])<=0:raise ValueError('Degenerate visible mask')
    components,labels,stats,_=cv2.connectedComponentsWithStats(binary)
    largest=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA]))
    component=(labels==largest).astype(np.uint8)*255
    measurement=describe_region(image,component)
    return {'pixel_corners':cv2.boxPoints(rect).tolist(),
            'pixel_center':np.mean(np.column_stack(np.where(mask))[:,::-1],axis=0).tolist(),
            'pixel_contours':[c.reshape(-1,2).tolist() for c in contours],
            'area_pixels':area,'rectangularity':float(area/(rect[1][0]*rect[1][1])),
            'visible_components':components-1,
            'largest_component_fraction':float(stats[largest,cv2.CC_STAT_AREA]/area),
            'appearance_measurement_scope':'largest visible component',
            'rectangle_scope':'minimum-area rectangle enclosing predicted visible mask; not an amodal outline',
            'metric_pose':None,'grasp_validated':False,'occlusion_verified':False,
            **measurement}


def self_test():
    assert nms([[0,0,10,10],[0,0,10,10],[20,20,30,30]],[.9,.8,.7],.7)==[0,2]
    assert nms([],[],.7)==[]
    image=np.zeros((300,400,3),np.uint8)
    image[60:220,80:260]=(0,0,200)
    mask=np.zeros((300,400),bool);mask[60:220,80:260]=True
    roi=np.full(mask.shape,255,np.uint8)
    limits={'min_area_fraction':.001,'max_area_fraction':.8}
    item=describe_mask(image,mask,roi,limits)
    assert item['visible_components']==1 and item['metric_pose'] is None
    assert item['appearance']['median_rgb']==[200.,0.,0.]
    bad=mask.copy();bad[0,0]=True
    try:describe_mask(image,bad,roi,limits)
    except ValueError:pass
    else:raise AssertionError('Boundary mask accepted')
    mask[120:130,80:260]=False
    item=describe_mask(image,mask,roi,limits)
    assert item['visible_components']==2
    assert item['grasp_validated'] is False
    print('PASS: duplicate boxes, mask coordinates, RGB measurement, boundary rejection and fragmented masks')


class Models:
    def __init__(self,settings):
        import torch
        from huggingface_hub import model_info
        from transformers import AutoProcessor,AutoModelForZeroShotObjectDetection,Sam2Processor,Sam2Model
        if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable; CPU fallback disabled')
        self.torch=torch
        self.settings=settings
        cache=settings['cache_dir']
        self.revisions={name:model_info(settings[name]).sha for name in ('detector_model','segmenter_model')}
        common={'cache_dir':cache,'trust_remote_code':False}
        print('Loading pretrained Grounding DINO...',flush=True)
        self.detector_processor=AutoProcessor.from_pretrained(settings['detector_model'],revision=self.revisions['detector_model'],**common)
        self.detector=AutoModelForZeroShotObjectDetection.from_pretrained(
            settings['detector_model'],revision=self.revisions['detector_model'],
            disable_custom_kernels=True,use_safetensors=True,**common).to('cuda').eval()
        print('Loading pretrained SAM 2...',flush=True)
        self.segmenter_processor=Sam2Processor.from_pretrained(settings['segmenter_model'],revision=self.revisions['segmenter_model'],**common)
        self.segmenter,loading=Sam2Model.from_pretrained(
            settings['segmenter_model'],revision=self.revisions['segmenter_model'],
            use_safetensors=True,output_loading_info=True,**common)
        if loading.get('missing_keys') or loading.get('mismatched_keys'):
            raise RuntimeError('SAM checkpoint has missing/mismatched image-model weights: '+str(loading))
        self.segmenter.to('cuda').eval()
        print('Both models loaded on CUDA.',flush=True)

    def detect(self,image,config):
        from PIL import Image
        torch=self.torch;s=self.settings
        roi=region_mask(image.shape,config['classical']['pickup_polygon_normalized'])
        ys,xs=np.where(roi>0)
        x0,y0,x1,y1=int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)
        crop=image[y0:y1,x0:x1]
        pil=Image.fromarray(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB))
        torch.cuda.synchronize();start=time.perf_counter()
        with torch.inference_mode():
            inputs=self.detector_processor(images=pil,text=s['prompt'],return_tensors='pt').to('cuda')
            outputs=self.detector(**inputs)
            result=self.detector_processor.post_process_grounded_object_detection(
                outputs,inputs.input_ids,threshold=s['box_threshold'],text_threshold=s['text_threshold'],
                target_sizes=[(crop.shape[0],crop.shape[1])])[0]
            boxes=result['boxes'].detach().cpu().numpy().reshape(-1,4)
            scores=result['scores'].detach().cpu().numpy()
            labels=result.get('text_labels',['board']*len(boxes))
            boxes[:,[0,2]]=np.clip(boxes[:,[0,2]],0,crop.shape[1])
            boxes[:,[1,3]]=np.clip(boxes[:,[1,3]],0,crop.shape[0])
            usable=[i for i,b in enumerate(boxes) if b[2]-b[0]>=4 and b[3]-b[1]>=4]
            keep=nms(boxes[usable],scores[usable],s['box_nms_iou'])[:s['max_detections']]
            indices=[usable[i] for i in keep]
            boxes=boxes[indices];scores=scores[indices];labels=[str(labels[i]) for i in indices]
            del outputs,inputs,result
            proposals=[];accepted=[];rejected=[];raw_masks=[]
            if len(boxes):
                sam_inputs=self.segmenter_processor(images=pil,input_boxes=[boxes.tolist()],return_tensors='pt').to('cuda')
                sam_outputs=self.segmenter(**sam_inputs,multimask_output=False)
                sizes=sam_inputs['original_sizes']
                if hasattr(sizes,'cpu'):sizes=sizes.cpu()
                masks=self.segmenter_processor.post_process_masks(sam_outputs.pred_masks.cpu(),sizes)[0]
                masks=masks.detach().cpu().numpy()
                qualities=sam_outputs.iou_scores.detach().float().cpu().numpy().reshape(len(boxes),-1)[:,0]
                if masks.shape!=(len(boxes),1,crop.shape[0],crop.shape[1]):
                    raise RuntimeError('Unexpected SAM mask shape: '+str(masks.shape))
                del sam_inputs,sam_outputs
                for i,(box,score,label,quality) in enumerate(zip(boxes,scores,labels,qualities)):
                    full=np.zeros(image.shape[:2],bool);full[y0:y1,x0:x1]=masks[i,0]>0
                    raw_masks.append(full)
                    proposal={'proposal_id':i+1,'box_xyxy':(box+[x0,y0,x0,y0]).tolist(),
                              'detector_score':float(score),'text_label':label,'sam_predicted_iou':float(quality)}
                    proposals.append(proposal)
                    try:
                        if not np.isfinite([score,quality]).all():raise ValueError('Non-finite model scores')
                        item=describe_mask(image,full,roi,config['classical'])
                        duplicate=False
                        for _,previous in accepted:
                            union=np.count_nonzero(full|previous)
                            overlap=np.count_nonzero(full&previous)/union if union else 0
                            if overlap>s['duplicate_mask_iou']:duplicate=True;break
                        if duplicate:raise ValueError('Near-duplicate predicted mask')
                        accepted.append(({**item,**proposal},full))
                    except ValueError as error:
                        rejected.append({**proposal,'reason':str(error)})
            torch.cuda.synchronize();elapsed=(time.perf_counter()-start)*1000
        accepted.sort(key=lambda pair:pair[0]['pixel_center'][0])
        for i,(item,_) in enumerate(accepted,1):item['id']=f'candidate_{i}'
        report={'mode':'pretrained_b_pixel_preview','image_size':[image.shape[1],image.shape[0]],
                'execution_allowed':False,'calibration_valid':False,
                'quad_candidates':[item for item,_ in accepted],
                'box_proposals':proposals,'rejected_proposals':rejected,'detector_ms':elapsed,
                'roi_crop_xyxy':[x0,y0,x1,y1],'model_revisions':self.revisions,
                'warnings':['Scores are model outputs, not calibrated correctness probabilities.',
                            'Predicted masks may merge instances or infer unsupported boundaries.',
                            'Metric coordinates, occlusion and suction suitability are unverified.']}
        return report,accepted,raw_masks,roi


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['self-test','inspect'])
    parser.add_argument('--settings',type=Path)
    inputs=parser.add_mutually_exclusive_group()
    inputs.add_argument('--image',type=Path)
    inputs.add_argument('--manifest',type=Path)
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args()
    if args.mode=='self-test':self_test();return
    if args.settings is None or args.output_dir is None or (args.image is None and args.manifest is None):
        parser.error('--settings, --output-dir and --image or --manifest are required')
    settings=json.loads(args.settings.read_text())
    config_path=Path(settings['scene_config'])
    config=tomllib.loads(config_path.read_text())
    config_hash=hashlib.sha256(config_path.read_bytes()).hexdigest()
    if args.manifest:
        source=json.loads(args.manifest.read_text())
        if source['config_sha256']!=config_hash:raise ValueError('Scene configuration differs from A manifest')
        images=[Path(row['source']) for row in source['images']]
    else:images=[args.image]
    if args.output_dir.exists():raise ValueError('Output folder exists; choose a new run name')
    for path in images:
        if not path.is_file():raise ValueError('Missing input: '+str(path))
    models=Models(settings)
    args.output_dir.mkdir(parents=True)
    (args.output_dir/'config_snapshot.toml').write_bytes(config_path.read_bytes())
    (args.output_dir/'b_settings.json').write_text(json.dumps(settings,indent=2))
    manifest={'config_sha256':config_hash,'execution_allowed':False,'model_revisions':models.revisions,'images':[],'errors':[]}
    tiles=[];reviews=[]
    for number,path in enumerate(images,1):
        image=cv2.imread(str(path))
        if image is None:raise ValueError('Unreadable input: '+str(path))
        report,accepted,raw_masks,roi=models.detect(image,config)
        folder=args.output_dir/path.stem;folder.mkdir()
        visual=image.copy();union=np.zeros(image.shape[:2],np.uint8)
        for i,mask in enumerate(raw_masks,1):
            if not cv2.imwrite(str(folder/f'proposal_{i}_mask.png'),mask.astype(np.uint8)*255):raise OSError('Mask save failed')
        for item,mask in accepted:
            colour=np.array([0,220,0],np.uint8)
            visual[mask]=(visual[mask].astype(float)*.7+colour*.3).astype(np.uint8)
            contours=[np.asarray(c,np.int32).reshape(-1,1,2) for c in item['pixel_contours']]
            cv2.drawContours(visual,contours,-1,(0,255,0),2)
            anchor=tuple(np.rint(item['pixel_center']).astype(int))
            cv2.putText(visual,item['id'],anchor,cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2)
            union[mask]=255
            item['mask_file']=f"proposal_{item['proposal_id']}_mask.png"
        points=config['classical']['pickup_polygon_normalized']
        if points:
            polygon=np.rint(np.array(points)*[image.shape[1]-1,image.shape[0]-1]).astype(np.int32)
            cv2.polylines(visual,[polygon],True,(0,220,255),2)
        cv2.putText(visual,'B MASK PREVIEW - NO ROBOT OUTPUT',(20,image.shape[0]-20),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,220,255),2)
        if not cv2.imwrite(str(folder/'overlay.jpg'),visual):raise OSError('Overlay save failed')
        if not cv2.imwrite(str(folder/'foreground.png'),union):raise OSError('Union mask save failed')
        report.update(source_image=str(path.resolve()),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                      foreground_semantics='Union of accepted B masks; not the A HSV foreground',source_header_available=False)
        (folder/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
        manifest['images'].append({'key':path.stem,'source':str(path.resolve()),'candidate_ids':[v['id'] for v,_ in accepted],'detector_ms':report['detector_ms']})
        reviews.append({'key':path.stem,**{name:'' for name in FIELDS[1:]}})
        tile=np.full((240,384,3),30,np.uint8);tile[:216]=cv2.resize(visual,(384,216))
        cv2.putText(tile,f'{path.stem}: {len(accepted)} B masks',(6,234),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1);tiles.append(tile)
        print(f'[{number}/{len(images)}] {path.name}: {len(accepted)} masks; {report["detector_ms"]:.1f} ms',flush=True)
    for start in range(0,len(tiles),12):
        sheet=np.full((720,1536,3),30,np.uint8)
        for i,tile in enumerate(tiles[start:start+12]):
            y,x=(i//4)*240,(i%4)*384;sheet[y:y+240,x:x+384]=tile
        if not cv2.imwrite(str(args.output_dir/f'contact_{start//12+1:02}.jpg'),sheet):raise OSError('Contact sheet save failed')
    (args.output_dir/'manifest.json').write_text(json.dumps(manifest,indent=2))
    with (args.output_dir/'review.csv').open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=FIELDS);writer.writeheader();writer.writerows(reviews)
    print('Saved:',args.output_dir)
    print('Review new candidate IDs, then use review_a.py score on this output folder.')


if __name__=='__main__':
    main()
