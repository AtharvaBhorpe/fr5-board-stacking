"""Run baseline and refined B on identical images with one GPU model load. No motion."""
import argparse
import json
from pathlib import Path
import tomllib
import cv2
from inspect_b import Models
from b_live_geometry import measure,draw


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--images',type=Path,nargs='+',required=True)
    p.add_argument('--settings',type=Path,default=Path('config/pretrained_b_refined.json'))
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args()
    for image in a.images:
        if not image.is_file():p.error('Missing image: '+str(image))
    a.output_dir.mkdir(parents=True,exist_ok=False)
    settings=json.loads(a.settings.read_text())
    config=tomllib.loads(Path(settings['scene_config']).read_text())
    models=Models(settings);summary=[]
    for number,path in enumerate(a.images,1):
        image=cv2.imread(str(path))
        if image is None:raise ValueError('Unreadable image: '+str(path))
        for mode in ('baseline','refined'):
            models.settings['prompt_refinement']=mode=='refined'
            report,accepted,_,_=models.detect(image,config)
            folder=a.output_dir/f'{number:02}_{path.stem}'/mode
            folder.mkdir(parents=True)
            objects=[measure(item,mask) for item,mask in accepted]
            uncertain=[measure(item) for item in report.get('uncertain_candidates',[])]
            for item,mask in accepted:
                if not cv2.imwrite(str(folder/(item['id']+'_mask.png')),mask.astype('uint8')*255):
                    raise OSError('Mask save failed')
            if not cv2.imwrite(str(folder/'overlay.jpg'),draw(image.copy(),uncertain+objects)):
                raise OSError('Overlay save failed')
            report.update(source=str(path.resolve()),objects=objects,uncertain_objects=uncertain)
            (folder/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
            row={'image':str(path),'mode':mode,'accepted':len(objects),
                 'uncertain':len(uncertain),
                 'detector_ms':report['detector_ms'],'overlay':str(folder/'overlay.jpg')}
            summary.append(row);print(json.dumps(row),flush=True)
    (a.output_dir/'summary.json').write_text(json.dumps(summary,indent=2))
    print('Inspect masks manually; count alone does not establish correct instance separation.')


if __name__=='__main__':main()
