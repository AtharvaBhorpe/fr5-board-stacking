"""Build a frozen YOLO dataset from explicitly reviewed CVAT-export images."""
import argparse
from collections import Counter
import hashlib
import io
import json
import math
from pathlib import Path, PurePosixPath
import zipfile

NAMES=['VARa','VARb','VARc','hand']
LEGACY_NAMES={'large_white':'VARa','small_white':'VARb','small_red':'VARc'}


def canonical_names(names):
    if isinstance(names,dict):
        try:names=[names[i] if i in names else names[str(i)] for i in range(len(names))]
        except KeyError:raise ValueError('Class IDs must be contiguous from zero') from None
    if not isinstance(names,(list,tuple)) or not all(isinstance(n,str) for n in names):
        raise ValueError('Class names must be a list or ID-to-name mapping')
    result=[LEGACY_NAMES.get(name,name) for name in names]
    if len(result)!=4 or set(result)!=set(NAMES):
        raise ValueError('Expected four distinct labels: '+', '.join(NAMES))
    return result


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_frozen_reviews(plan):
    rows={r['file_name']:r for r in plan['images']}
    if len(rows)!=len(plan['images']):raise ValueError('Duplicate review filename')
    fields=('reviewed','arrangement_group','split','verified_empty','exported_instances')
    for name,original in plan.get('frozen_review_rows',{}).items():
        if name not in rows or any(rows[name].get(k)!=original.get(k) for k in fields):
            raise ValueError('Previously reviewed image is locked; preserve its existing assignments: '+name)


def read_export(path):
    import yaml
    with zipfile.ZipFile(path) as z:
        files={}
        for name in z.namelist():
            p=PurePosixPath(name)
            if p.is_absolute() or '..' in p.parts:raise ValueError('Unsafe archive path')
            if name in files:raise ValueError('Duplicate archive member')
            if not name.endswith('/'):files[name]=z.read(name)
        metadata=[v for k,v in files.items() if PurePosixPath(k).name in ('data.yaml','dataset.yaml')]
        if len(metadata)!=1:
            if any(PurePosixPath(k).name=='annotations.xml' for k in files):
                raise ValueError('This is a CVAT XML export, not YOLO. Re-export as Ultralytics YOLO Segmentation 1.0 with Save images enabled; keep this XML ZIP as a backup.')
            raise ValueError('Need one data.yaml: export Ultralytics YOLO Segmentation with images')
        names=canonical_names(yaml.safe_load(metadata[0])['names'])
        images={};labels={}
        for name,data in files.items():
            p=PurePosixPath(name)
            if 'images' in p.parts and p.suffix.lower() in ('.png','.jpg','.jpeg','.bmp'):
                if p.name in images:raise ValueError('Duplicate image basename: '+p.name)
                images[p.name]=data
            if 'labels' in p.parts and p.suffix=='.txt':
                if p.stem in labels:raise ValueError('Duplicate label basename: '+p.stem)
                labels[p.stem]=data.decode('utf-8-sig')
        if not images:raise ValueError('No images in ZIP; enable Save images during CVAT export')
        stems=[Path(k).stem for k in images]
        if len(set(stems))!=len(stems):raise ValueError('Image stems must be unique')
        return images,labels,names


def validate_labels(text,names):
    names=canonical_names(names)
    rows=[];counts=Counter();seen=set()
    for line in text.splitlines():
        tokens=line.split()
        if not tokens:continue
        if len(tokens)<7 or len(tokens)%2!=1:raise ValueError('Expected segmentation polygons, not detection boxes')
        category=int(tokens[0]);xy=list(map(float,tokens[1:]))
        if not 0<=category<len(names):raise ValueError('Invalid class ID')
        if not all(math.isfinite(v) and 0<=v<=1 for v in xy):raise ValueError('Polygon coordinates must be finite and normalized')
        points=list(zip(xy[::2],xy[1::2]))
        area=abs(sum(x*y2-x2*y for (x,y),(x2,y2) in zip(points,points[1:]+points[:1])))/2
        if area<=1e-9:raise ValueError('Degenerate polygon')
        target=NAMES.index(names[category])
        row=str(target)+' '+' '.join(format(v,'.9g') for v in xy)
        if row in seen:raise ValueError('Duplicate annotation polygon')
        seen.add(row);rows.append(row);counts[names[category]]+=1
    return '\n'.join(rows)+('\n' if rows else ''),counts


def merge_exports(sources,output):
    if output.exists():raise ValueError('Merged ZIP exists; choose a new filename')
    images={};labels={};provenance=[]
    for source in sources:
        incoming,in_labels,names=read_export(source)
        provenance.append({'file':str(source),'sha256':digest(source)})
        for name,data in incoming.items():
            text,_=validate_labels(in_labels.get(Path(name).stem,''),names)
            if name in images:
                if images[name]!=data:raise ValueError('Different image bytes under the same name: '+name)
                previous=labels[name]
                if previous and text and set(previous.splitlines())!=set(text.splitlines()):
                    raise ValueError('Conflicting nonempty annotations; choose the reviewed version: '+name)
                if not previous:labels[name]=text
            else:images[name]=data;labels[name]=text
    output.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(output,'x',compression=zipfile.ZIP_STORED) as z:
        z.writestr('data.yaml',json.dumps({'names':NAMES}))
        z.writestr('merge_manifest.json',json.dumps({'sources':provenance},indent=2))
        for name,data in images.items():
            z.writestr('images/train/'+name,data)
            z.writestr('labels/train/'+Path(name).stem+'.txt',labels[name])
    return {'images':len(images),'annotated_images':sum(bool(t) for t in labels.values()),
            'output_zip':str(output),'limits':'Annotations retained, not automatically reviewed; no split assigned'}


def build(source,plan,output):
    validate_frozen_reviews(plan)
    if plan['source_zip_sha256']!=digest(source):raise ValueError('Export changed: create a new plan for the new snapshot')
    images,labels,names=read_export(source)
    selected=[];groups={};splits=Counter();counts={s:Counter() for s in ('train','val','test')};hashes={}
    for row in plan['images']:
        if row['reviewed'] is not True:continue
        name=row['file_name'];group=row['arrangement_group'].strip();split=row['split']
        if name not in images or Path(name).name!=name:raise ValueError('Unknown/unsafe image: '+name)
        if not group or group=='REVIEW_AND_ASSIGN' or split not in counts:
            raise ValueError('Reviewed images need an arrangement_group and train/val/test split: '+name)
        if group in groups and groups[group]!=split:raise ValueError('Arrangement appears in multiple splits: '+group)
        groups[group]=split
        text,instances=validate_labels(labels.get(Path(name).stem,''),names)
        if not instances and row.get('verified_empty') is not True:
            raise ValueError('No annotations: explicitly confirm verified_empty only after visual review: '+name)
        if instances and row.get('verified_empty') is True:raise ValueError('Nonempty image marked empty: '+name)
        h=hashlib.sha256(images[name]).hexdigest()
        if h in hashes:raise ValueError('Identical images selected twice: '+name)
        hashes[h]=split;counts[split].update(instances);splits[split]+=1
        from PIL import Image
        with Image.open(io.BytesIO(images[name])) as image:image.verify()
        selected.append((name,split,text))
    if len({r[0] for r in selected})!=len(selected):raise ValueError('Duplicate reviewed filename')
    if splits['train']<2 or splits['val']<1:raise ValueError('Need at least two training images and one independent validation image')
    target=plan.get('target_split')
    if target is not None and dict(splits)!=target:
        raise ValueError(f'Review plan targets {target}, but selected counts are {dict(splits)}. Complete pending labels and group/split assignments first.')
    warnings=[]
    for split in counts:
        if not splits[split]:continue
        missing=[n for n in NAMES if not counts[split][n]]
        if missing:warnings.append(f'{split}: absent classes: '+', '.join(missing))
    if not splits['test']:warnings.append('No independent test set selected; validation is used for model selection, not final testing')
    if len(selected)<20:warnings.append('Tiny smoke-test dataset; not an accuracy validation')
    output.mkdir(parents=True,exist_ok=False)
    for name,split,text in selected:
        im=output/'images'/split/name;label=output/'labels'/split/(Path(name).stem+'.txt')
        im.parent.mkdir(parents=True,exist_ok=True);label.parent.mkdir(parents=True,exist_ok=True)
        im.write_bytes(images[name]);label.write_text(text)
    # Use an explicit root so commands work from any working directory.
    import yaml
    dataset={'path':str(output.resolve()),'train':'images/train','val':'images/val',
             'names':dict(enumerate(NAMES))}
    if splits['test']:dataset['test']='images/test'
    (output/'data.yaml').write_text(yaml.safe_dump(dataset,sort_keys=False))
    report={'source_zip_sha256':digest(source),'images_per_split':dict(splits),
            'instances_per_split':{s:dict(c) for s,c in counts.items()},'warnings':warnings,
            'review_plan':plan,'robot_execution_allowed':False}
    (output/'manifest.json').write_text(json.dumps(report,indent=2))
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['inventory','build','merge'])
    p.add_argument('--zip',type=Path,required=True)
    p.add_argument('--plan',type=Path)
    p.add_argument('--extra-zip',type=Path,action='append',default=[])
    p.add_argument('--output-zip',type=Path)
    p.add_argument('--annotated-only',action='store_true',help='Draft plan only for images with exported instances')
    p.add_argument('--output-dir',type=Path)
    a=p.parse_args()
    if a.mode=='merge':
        if not a.extra_zip or a.output_zip is None:p.error('merge needs --extra-zip and --output-zip')
        print(json.dumps(merge_exports([a.zip,*a.extra_zip],a.output_zip),indent=2));return
    if a.plan is None:p.error('inventory/build needs --plan')
    if a.mode=='inventory':
        images,labels,names=read_export(a.zip)
        rows=[]
        for name in sorted(images):
            _,counts=validate_labels(labels.get(Path(name).stem,''),names)
            if a.annotated_only and not counts:continue
            rows.append({'file_name':name,'exported_instances':sum(counts.values()),
                         'reviewed':False,'arrangement_group':'','split':'','verified_empty':False})
        with a.plan.open('x') as f:json.dump({'source_zip_sha256':digest(a.zip),'images':rows},f,indent=2)
        print(f'Wrote {len(rows)} rows to {a.plan}; only set reviewed=true for completely reviewed images.')
    else:
        if a.output_dir is None:p.error('build needs --output-dir')
        report=build(a.zip,json.loads(a.plan.read_text()),a.output_dir)
        print(json.dumps({k:v for k,v in report.items() if k!='review_plan'},indent=2))


if __name__=='__main__':main()
