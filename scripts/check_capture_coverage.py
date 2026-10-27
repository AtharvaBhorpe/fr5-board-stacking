"""Check saved dataset images against a guide before handing hardware back."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import cv2


def layout_signature(scene):
    return json.dumps({key:scene.get(key) for key in ('boards','occluder','effect')},sort_keys=True)


def inspect_sample(folder,guide):
    metadata=json.loads((folder/'metadata.json').read_text())
    name=metadata['image_file']
    if Path(name).name!=name:raise ValueError('Invalid image filename')
    image_path=folder/name
    if hashlib.sha256(image_path.read_bytes()).hexdigest()!=metadata['image_sha256']:
        raise ValueError('Saved image checksum mismatch')
    image=cv2.imread(str(image_path))
    if image is None or list(image.shape[:2][::-1])!=metadata['image_size']:
        raise ValueError('Image unreadable or dimensions mismatch')
    scene=metadata.get('intended_guide_scene') or {};number=scene.get('id')
    matching=number in guide and layout_signature(scene)==layout_signature(guide[number])
    return {'image':str(image_path),'sha256':metadata['image_sha256'],'scene_id':number,
            'matches_current_intended_layout':matching,'metadata_warnings':metadata.get('warnings',[]),
            'camera_info_present':metadata.get('camera_info') is not None,
            'optional_streams_saved':list(metadata.get('attachments',{})),
            'image_quality':metadata.get('image_quality',{}),
            'source_timestamp_sec':metadata.get('source_timestamp_sec')}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--guide',type=Path,required=True)
    p.add_argument('--root',type=Path,default=Path('.local/datasets'))
    p.add_argument('--output',type=Path)
    a=p.parse_args();guide={s['id']:s for s in json.loads(a.guide.read_text())}
    records=[];errors=[]
    for path in sorted(a.root.glob('webcam_capture_*/samples/*/metadata.json')):
        try:records.append(inspect_sample(path.parent,guide))
        except (ValueError,KeyError,TypeError,OSError) as error:errors.append({'sample':str(path.parent),'error':str(error)})
    covered={r['scene_id'] for r in records if r['matches_current_intended_layout']}
    required={s['id'] for s in guide.values() if s.get('priority')}
    counts=Counter(r['sha256'] for r in records)
    result={'saved_images_checked':len(records),'priority_layouts_with_saved_metadata':len(covered&required),
            'priority_layouts_required':len(required),'missing_priority_scene_ids':sorted(required-covered),
            'missing_other_scene_ids':sorted(set(guide)-required-covered),
            'identical_image_groups':sum(n>1 for n in counts.values()),
            'camera_info_missing_images':sum(not r['camera_info_present'] for r in records),
            'integrity_errors':errors,'files_integrity_passed':bool(records) and not errors,
            'intended_priority_coverage_complete':required<=covered,
            'actual_layout_visually_verified':False,'annotation_quality_verified':False,
            'limits':['Guide IDs record operator intent, not proof the scene was physically arranged correctly.',
                      'Matching legacy guide layouts are counted; changed layouts need separate review.',
                      'Sharpness diagnostics have no validated pass/fail threshold; inspect full-resolution samples.'],
            'images':records}
    text=json.dumps(result,indent=2,allow_nan=False)
    if a.output:
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(text)
    print(json.dumps({k:v for k,v in result.items() if k!='images'},indent=2))


if __name__=='__main__':main()
