"""Append CVAT exports while freezing every previous image's review assignments."""
import argparse
import copy
from collections import Counter
import hashlib
import json
from pathlib import Path
import zipfile
from prepare_yolo_pilot import NAMES,digest,read_export,validate_labels,validate_frozen_reviews


def extend(previous_zip,previous_plan,new_zips,output_zip,output_plan):
    if output_zip.exists() or output_plan.exists():raise ValueError('Output already exists; use new filenames')
    plan=json.loads(previous_plan.read_text())
    if plan['source_zip_sha256']!=digest(previous_zip):raise ValueError('Previous ZIP does not match previous review plan')
    validate_frozen_reviews(plan)
    images,raw,names=read_export(previous_zip)
    labels={name:validate_labels(raw.get(Path(name).stem,''),names)[0] for name in images}
    for row in plan['images']:
        if row['file_name'] not in images:raise ValueError('Old review image missing from previous ZIP')
    by_hash={}
    for name,data in images.items():by_hash.setdefault(hashlib.sha256(data).hexdigest(),name)
    added=[];duplicates=[];sources=[]
    for source in new_zips:
        incoming,in_labels,in_names=read_export(source)
        sources.append({'file':str(source),'sha256':digest(source)})
        for name,data in incoming.items():
            text,counts=validate_labels(in_labels.get(Path(name).stem,''),in_names)
            fingerprint=hashlib.sha256(data).hexdigest()
            existing=name if name in images else by_hash.get(fingerprint)
            if existing is not None:
                if images[existing]!=data:raise ValueError('Filename collision with different image bytes: '+name)
                if text and set(labels[existing].splitlines())!=set(text.splitlines()):
                    raise ValueError('Existing image has changed annotations; review corrections separately, not as a new image: '+name)
                duplicates.append({'incoming':name,'existing':existing});continue
            images[name]=data;labels[name]=text;by_hash[fingerprint]=name
            added.append({'file_name':name,'exported_instances':sum(counts.values()),'reviewed':False,
                          'arrangement_group':'','split':'','verified_empty':False,'locked':False})
    if not added:raise ValueError('No genuinely new images; all incoming images already exist')
    if len({Path(name).stem for name in images})!=len(images):
        raise ValueError('Different image filenames share one stem; rename them before merging to avoid label collisions')
    updated=copy.deepcopy(plan)
    updated['frozen_review_rows']={row['file_name']:copy.deepcopy(row) for row in plan['images']}
    updated['images']=[{**copy.deepcopy(row),'locked':True} for row in plan['images']]+sorted(added,key=lambda r:r['file_name'])
    updated['extension_provenance']={'previous_zip_sha256':digest(previous_zip),'previous_plan_sha256':digest(previous_plan),
        'new_sources':sources,'deduplicated_images':duplicates,'new_images':len(added),
        'older_assignments_preserved':True}
    selected=Counter(r['split'] for r in updated['images'] if r['reviewed'])
    updated['target_split']=dict(selected)
    output_zip.parent.mkdir(parents=True,exist_ok=True);output_plan.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(output_zip,'x',compression=zipfile.ZIP_STORED) as archive:
        archive.writestr('data.yaml',json.dumps({'names':NAMES}))
        archive.writestr('extension_manifest.json',json.dumps(updated['extension_provenance'],indent=2))
        for name,data in images.items():
            archive.writestr('images/train/'+name,data)
            archive.writestr('labels/train/'+Path(name).stem+'.txt',labels[name])
    updated['source_zip_sha256']=digest(output_zip)
    validate_frozen_reviews(updated)
    with output_plan.open('x') as stream:json.dump(updated,stream,indent=2)
    return {'locked_previous_rows':len(plan['images']),'new_images_to_review':len(added),
        'exact_duplicate_images_skipped':len(duplicates),'previous_selected_splits':dict(selected),
        'output_zip':str(output_zip),'output_plan':str(output_plan)}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--previous-zip',type=Path,required=True)
    p.add_argument('--previous-plan',type=Path,required=True)
    p.add_argument('--new-zip',type=Path,nargs='+',required=True)
    p.add_argument('--output-zip',type=Path,required=True)
    p.add_argument('--output-plan',type=Path,required=True)
    a=p.parse_args()
    print(json.dumps(extend(a.previous_zip,a.previous_plan,a.new_zip,a.output_zip,a.output_plan),indent=2))


if __name__=='__main__':main()
