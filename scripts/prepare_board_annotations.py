"""Select candidate annotation images. No labels or split assignments are invented."""
import argparse
import hashlib
import itertools
import json
import os
import shutil
import tomllib
from pathlib import Path
import cv2
import numpy as np


def feature(image):
    small=cv2.resize(image,(96,54),interpolation=cv2.INTER_AREA)
    grey=cv2.cvtColor(cv2.resize(small,(9,8)),cv2.COLOR_BGR2GRAY)
    return grey[:,1:]>grey[:,:-1],small.mean(axis=(0,1))


def near(a,b):
    return np.count_nonzero(a[0]!=b[0])<=4 and np.max(np.abs(a[1]-b[1]))<8


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir',action='append',type=Path)
    p.add_argument('--source-label',action='append',
                   help='Optional filename prefix for each input folder, in matching order')
    p.add_argument('--existing-pack',type=Path,
                   help='Preserve its selected images, filenames and annotation metadata in a new pack')
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--limit',type=int,default=120)
    p.add_argument('--roi-config',type=Path,default=Path('config/perception_review_roi_v3.toml'))
    p.add_argument('--self-test',action='store_true')
    a=p.parse_args()
    if a.self_test:
        white=np.full((120,160,3),200,np.uint8);red=np.full_like(white,(0,0,180))
        assert near(feature(white),feature(white))
        assert not near(feature(white),feature(red))
        assert not near(feature(white),feature(np.full_like(white,160)))
        print('PASS: identical-frame suppression and colour-change preservation');return
    if not a.input_dir or a.output_dir is None or a.limit<1:p.error('Need input-dir, output-dir and positive limit')
    labels=a.source_label or [f's{i}' for i in range(len(a.input_dir))]
    if len(labels)!=len(a.input_dir) or any(not label or any(
            not (c.isalnum() or c in '_-') for c in label) for label in labels):
        p.error('Provide one safe source-label per input-dir (letters, digits, underscore or hyphen)')
    from fr5_perception.detector import region_mask
    config=tomllib.loads(a.roi_config.read_text())
    polygon=config['classical']['pickup_polygon_normalized']
    if not polygon:raise ValueError('Need pickup polygon for scene-change comparison')
    groups=[sorted(folder.glob('*.png')) for folder in a.input_dir]
    if any(not files for files in groups):raise ValueError('An input folder contains no PNGs')
    seed=[]
    if a.existing_pack:
        seed=json.loads((a.existing_pack/'manifest.json').read_text())['images']
        if len(seed)>a.limit:raise ValueError('limit must accommodate every preserved image')
    a.output_dir.mkdir(parents=True,exist_ok=False)
    (a.output_dir/'images').mkdir()
    chosen=[];features=[];tiles=[]
    names=set()
    def remember(image,path,metadata):
        name=metadata['file_name']
        if Path(name).name!=name or name in names:
            raise ValueError('Unsafe/duplicate filename; choose a different source-label: '+name)
        target=a.output_dir/'images'/name
        shutil.copy2(path,target)
        chosen.append(metadata);names.add(name)
        roi=region_mask(image.shape,polygon);ys,xs=np.where(roi>0)
        features.append(feature(image[ys.min():ys.max()+1,xs.min():xs.max()+1]))
        tile=np.full((240,384,3),25,np.uint8);tile[:216]=cv2.resize(image,(384,216))
        cv2.putText(tile,name,(5,234),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
        tiles.append(tile)
    for row in seed:
        path=a.existing_pack/'images'/row['file_name']
        if Path(row['file_name']).name!=row['file_name']:raise ValueError('Unsafe existing filename')
        image=cv2.imread(str(path))
        if image is None:raise ValueError('Unreadable preserved image: '+str(path))
        metadata=dict(row)
        metadata['preserved_from_pack']=str(a.existing_pack.resolve())
        remember(image,path,metadata)
    # Sample all parts of each recording before comparing candidates.
    pools=[]
    for source,files in enumerate(groups):
        slots=max(1,int(np.ceil((a.limit-len(seed))/len(groups))))
        coverage=np.unique(np.linspace(0,len(files)-1,min(len(files),slots)).astype(int))
        fallback=np.unique(np.linspace(0,len(files)-1,min(len(files),a.limit*3)).astype(int))
        preferred=set(coverage.tolist())
        indices=coverage.tolist()+[i for i in fallback if i not in preferred]
        pools.append([(source,files[i]) for i in indices])
    for bundle in itertools.zip_longest(*pools):
        for item in bundle:
            if item is None or len(chosen)>=a.limit:continue
            source,path=item
            image=cv2.imread(str(path))
            if image is None:raise ValueError('Unreadable image: '+str(path))
            roi=region_mask(image.shape,polygon)
            ys,xs=np.where(roi>0)
            f=feature(image[ys.min():ys.max()+1,xs.min():xs.max()+1])
            if any(near(f,previous) for previous in features):continue
            name=f'{labels[source]}_{path.stem}.png'
            remember(image,path,{'file_name':name,'source':str(path.resolve()),
                                'source_label':labels[source],
                                'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                                'arrangement_group':'REVIEW_AND_ASSIGN','split':'UNASSIGNED'})
    for start in range(0,len(tiles),12):
        sheet=np.full((720,1536,3),25,np.uint8)
        for i,tile in enumerate(tiles[start:start+12]):
            y,x=i//4*240,i%4*384;sheet[y:y+240,x:x+384]=tile
        if not cv2.imwrite(str(a.output_dir/f'contact_{start//12+1:02}.jpg'),sheet):raise OSError('Sheet save failed')
    if a.existing_pack:
        additions=a.output_dir/'added_images';additions.mkdir()
        for row in chosen[len(seed):]:
            original=a.output_dir/'images'/row['file_name']
            try:os.link(original,additions/row['file_name'])
            except OSError:shutil.copy2(original,additions/row['file_name'])
    (a.output_dir/'manifest.json').write_text(json.dumps({'images':chosen,
        'preserved_count':len(seed),'added_count':len(chosen)-len(seed),
        'limits':['Deduplication is a heuristic, not proof of distinct physical arrangements.',
                  'Review transitions/blur and group related scenes before splitting.',
                  'No annotation or class has been inferred.']},indent=2))
    print(json.dumps({'selected':len(chosen),'preserved':len(seed),'added':len(chosen)-len(seed),
                      'output':str(a.output_dir),
                      'next':'Review contact sheets; images/ is the full set, added_images/ only additions when extending'},indent=2))


if __name__=='__main__':main()
