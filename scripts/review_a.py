"""Offline Stage A replay, visual ROI selection and manually reviewed scores."""
import argparse
import csv
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import tempfile
import time
import tomllib

import cv2
import numpy as np
from fr5_perception.detector import ClassicalDetector, region_mask
from fr5_perception.inspect import overlay

FIELDS=['key','scenario','reviewed','tp_ids','fp_ids','missed_boards','failure_tags','notes']


def selected(files, samples, indices=None):
    if not files:
        raise ValueError('No matching images in input folder')
    if indices is None:
        if samples<1:
            raise ValueError('--samples must be positive')
        indices=np.unique(np.linspace(0,len(files)-1,min(samples,len(files))).astype(int)).tolist()
    if not indices or min(indices)<0 or max(indices)>=len(files):
        raise ValueError('Image index outside available range')
    return [files[i] for i in sorted(set(indices))]


def write_image(path,image):
    if not cv2.imwrite(str(path),image):
        raise OSError('Could not write '+str(path))


def batch(input_dir, config_path, output, samples=24, indices=None):
    files=selected(sorted(input_dir.glob('*.png')),samples,indices)
    with config_path.open('rb') as source:
        config=tomllib.load(source)
    detector=ClassicalDetector(config)
    output.mkdir(parents=True,exist_ok=False)
    (output/'config_snapshot.toml').write_bytes(config_path.read_bytes())
    manifest={'config_sha256':hashlib.sha256(config_path.read_bytes()).hexdigest(),
              'input_dir':str(input_dir),'pickup_roi_assigned':bool(config['classical']['pickup_polygon_normalized']),
              'execution_allowed':False,'images':[],'errors':[]}
    rows=[]; tiles=[]
    for number,path in enumerate(files,1):
        image=cv2.imread(str(path))
        if image is None:
            manifest['errors'].append({'source':str(path),'error':'Unreadable image'});continue
        folder=output/path.stem
        folder.mkdir()
        try:
            start=time.perf_counter()
            report=detector.detect(image)
            elapsed=(time.perf_counter()-start)*1000
            assert report['execution_allowed'] is False
            assert report['calibration_valid'] is False
            report['source_image']=str(path.resolve())
            report['source_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
            report['detector_ms']=elapsed
            report['source_header_available']=False
            visual=overlay(image,report,config)
            write_image(folder/'overlay.jpg',visual)
            mask=cv2.bitwise_and(detector.foreground(image),region_mask(image.shape,config['classical']['pickup_polygon_normalized']))
            write_image(folder/'foreground.png',mask)
            (folder/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
            count=len(report['quad_candidates'])
            manifest['images'].append({'key':path.stem,'source':str(path.resolve()),
                                       'candidate_ids':[v['id'] for v in report['quad_candidates']],
                                       'detector_ms':elapsed})
            rows.append({'key':path.stem,**{field:'' for field in FIELDS[1:]}})
            tile=np.full((240,384,3),30,np.uint8)
            tile[:216]=cv2.resize(visual,(384,216))
            cv2.putText(tile,f'{path.stem} | {count} candidates',(6,234),cv2.FONT_HERSHEY_SIMPLEX,.43,(255,255,255),1)
            tiles.append(tile)
            print(f'[{number}/{len(files)}] {path.name}: {count} candidates, {elapsed:.1f} ms',flush=True)
        except (ValueError,cv2.error) as error:
            manifest['errors'].append({'source':str(path),'error':str(error)})
    if not rows:
        (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
        raise ValueError('No images completed; inspect manifest.json')
    for start in range(0,len(tiles),12):
        sheet=np.full((720,1536,3),30,np.uint8)
        for i,tile in enumerate(tiles[start:start+12]):
            y,x=(i//4)*240,(i%4)*384
            sheet[y:y+240,x:x+384]=tile
        write_image(output/f'contact_{start//12+1:02}.jpg',sheet)
    with (output/'review.csv').open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=FIELDS);writer.writeheader();writer.writerows(rows)
    times=[row['detector_ms'] for row in manifest['images']]
    manifest['timing']={'median_detector_ms':float(np.median(times)),
                        'p95_detector_ms':float(np.percentile(times,95)),
                        'scope':'Detector only; excludes image load, rendering, ROS transport and motion.'}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps({'output':str(output),'completed_images':len(rows),'errors':len(manifest['errors']),
                      'timing':manifest['timing'],'pickup_roi_assigned':manifest['pickup_roi_assigned']},indent=2))
    return manifest


def score(output,save=True):
    manifest=json.loads((output/'manifest.json').read_text())
    known={row['key']:set(row['candidate_ids']) for row in manifest['images']}
    totals=Counter(); tags=Counter(); issues=[]; reviewed=[]
    with (output/'review.csv').open(newline='') as stream:
        rows=list(csv.DictReader(stream))
    if len({row['key'] for row in rows})!=len(rows):
        raise ValueError('Duplicate review rows')
    if {row['key'] for row in rows}!=set(known):
        raise ValueError('Review keys differ from manifest')
    for row in rows:
        if row['reviewed'].strip().lower()!='yes':
            continue
        def ids(field):
            items=[v.strip() for v in row[field].split(';') if v.strip()]
            if len(items)!=len(set(items)):
                raise ValueError('Repeated IDs')
            return set(items)
        try:
            tp,fp=ids('tp_ids'),ids('fp_ids')
            fn=int(row['missed_boards'])
            if fn<0 or tp&fp or tp|fp!=known[row['key']]:
                raise ValueError('Classify every candidate exactly once; missed count must be nonnegative')
            if not row['scenario'].strip():
                raise ValueError('Add a scenario label')
        except ValueError as error:
            issues.append({'key':row['key'],'error':str(error)});continue
        totals.update(tp=len(tp),fp=len(fp),fn=fn)
        tags.update(v.strip() for v in row['failure_tags'].split(';') if v.strip())
        reviewed.append({'key':row['key'],'scenario':row['scenario'],'tp':len(tp),'fp':len(fp),
                         'fn':fn,'notes':row['notes'],'failure_tags':row['failure_tags']})
    precision=totals['tp']/(totals['tp']+totals['fp']) if totals['tp']+totals['fp'] else None
    recall=totals['tp']/(totals['tp']+totals['fn']) if totals['tp']+totals['fn'] else None
    summary={'reviewed_images':len(reviewed),'total_images':len(known),'tp':totals['tp'],'fp':totals['fp'],
             'fn':totals['fn'],'manual_precision':precision,'manual_recall':recall,
             'failure_tags':dict(tags),'review_errors':issues,'frames':reviewed,
             'execution_allowed':False,'limits':['Manual candidate assessment; no automatic polygon IoU scoring.',
             'Nearby frames are correlated; this is not a physical grasp success rate.',
             'Colour labels, metric accuracy and obstacle avoidance are not validated here.']}
    if save:
        (output/'review_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
    print(json.dumps(summary,indent=2))
    return summary


def roi(image_path,config_path,output):
    if output.exists():
        raise ValueError('Output configuration already exists; choose a new filename')
    image=cv2.imread(str(image_path))
    if image is None:raise ValueError('Unreadable ROI image')
    text=config_path.read_text();config=tomllib.loads(text)
    pattern=r'^pickup_polygon_normalized\s*=\s*\[\s*\]\s*$'
    if len(re.findall(pattern,text,re.MULTILINE))!=1:
        raise ValueError('Expected one empty pickup polygon; use a fresh base configuration')
    scale=min(1.,1280/image.shape[1],800/image.shape[0]);points=[]
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and len(points)<4:
            points.append([(x+.5)/scale-.5,(y+.5)/scale-.5])
    cv2.namedWindow('Review pickup ROI')
    cv2.setMouseCallback('Review pickup ROI',click)
    try:
        while True:
            view=cv2.resize(image,None,fx=scale,fy=scale)
            if points:
                polygon=np.rint(np.array(points)*scale).astype(np.int32)
                cv2.polylines(view,[polygon],len(points)==4,(0,255,255),2)
                for index,p in enumerate(polygon):cv2.putText(view,str(index+1),tuple(p),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
            cv2.putText(view,'Click 4 corners in perimeter order. U: undo; Enter: save; Esc: cancel',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,255,255),1)
            cv2.imshow('Review pickup ROI',view)
            key=cv2.waitKey(30)&255
            if key==27:return
            if key in (ord('u'),ord('U')) and points:points.pop()
            if key in (10,13) and len(points)==4:
                normalized=(np.array(points)/[image.shape[1]-1,image.shape[0]-1]).tolist()
                region_mask(image.shape,normalized)
                replacement='pickup_polygon_normalized = '+json.dumps(normalized)
                result=re.sub(pattern,replacement,text,count=1,flags=re.MULTILINE)
                tomllib.loads(result)
                with output.open('x') as stream:stream.write(result)
                print('Saved offline-review ROI configuration:',output)
                return
    finally:cv2.destroyAllWindows()


def self_test(config_path):
    import contextlib,io
    assert selected(list(range(10)),3)==[0,4,9]
    assert selected(list(range(10)),3,[8,2,2])==[2,8]
    with tempfile.TemporaryDirectory() as td,contextlib.redirect_stdout(io.StringIO()):
        root=Path(td);inputs=root/'input';inputs.mkdir()
        image=np.zeros((340,620,3),np.uint8)
        cv2.rectangle(image,(40,40),(200,270),(230,230,230),-1)
        cv2.rectangle(image,(340,50),(540,270),(0,0,220),-1)
        write_image(inputs/'a.png',image)
        write_image(inputs/'b.png',np.zeros_like(image))
        manifest=batch(inputs,config_path,root/'output',2)
        assert len(manifest['images'][0]['candidate_ids'])==2
        assert len(manifest['images'][1]['candidate_ids'])==0
        path=root/'output/review.csv'
        with path.open(newline='') as stream:rows=list(csv.DictReader(stream))
        for row,entry in zip(rows,manifest['images']):
            row.update(scenario='synthetic',reviewed='yes',tp_ids=';'.join(entry['candidate_ids']),missed_boards='0')
        with path.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=FIELDS);writer.writeheader();writer.writerows(rows)
        result=score(root/'output',False)
        assert result['tp']==2 and result['fp']==0 and result['fn']==0
        assert result['manual_precision']==1 and result['manual_recall']==1
        rows[0]['tp_ids']='invented_id'
        with path.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=FIELDS);writer.writeheader();writer.writerows(rows)
        result=score(root/'output',False)
        assert len(result['review_errors'])==1 and result['reviewed_images']==1
        assert (root/'output/contact_01.jpg').exists()
    print('PASS: sampling, real detector replay on fixtures, outputs, manual scoring and invalid-ID rejection')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['batch','score','roi','self-test'])
    parser.add_argument('--config',type=Path,default=Path('config/perception_multicolour.toml'))
    parser.add_argument('--input-dir',type=Path)
    parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--image',type=Path)
    parser.add_argument('--output-config',type=Path)
    parser.add_argument('--samples',type=int,default=24)
    parser.add_argument('--indices',type=int,nargs='+')
    args=parser.parse_args()
    try:
        if args.mode=='self-test':self_test(args.config)
        elif args.mode=='score':
            if args.output_dir is None:parser.error('--output-dir is required')
            score(args.output_dir)
        elif args.mode=='roi':
            if args.image is None or args.output_config is None:parser.error('--image and --output-config are required')
            roi(args.image,args.config,args.output_config)
        else:
            if args.input_dir is None or args.output_dir is None:parser.error('--input-dir and --output-dir are required')
            batch(args.input_dir,args.config,args.output_dir,args.samples,args.indices)
    except (OSError,ValueError,KeyError,cv2.error) as error:
        parser.exit(2,str(error)+'\n')
