"""Short local YOLO11s segmentation pilot. No ROS or robot operation."""
import argparse
import json
from pathlib import Path
import os


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--epochs',type=int,default=10)
    p.add_argument('--batch',type=int,default=2)
    p.add_argument('--imgsz',type=int,default=640)
    a=p.parse_args()
    if not a.data.is_file():p.error('Build the reviewed pilot dataset first')
    if a.output_dir.exists():p.error('Output exists; use a new run name')
    if min(a.epochs,a.batch,a.imgsz)<1:p.error('epochs, batch and imgsz must be positive')
    settings_dir=Path('.local/ultralytics').resolve();settings_dir.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('YOLO_CONFIG_DIR',str(settings_dir))
    import torch
    from ultralytics import YOLO
    from prepare_yolo_pilot import NAMES
    if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable; CPU fallback disabled')
    import yaml
    config=yaml.safe_load(a.data.read_text())
    names=config['names']
    if isinstance(names,dict):names=[names[i] if i in names else names[str(i)] for i in range(len(names))]
    if names!=NAMES:
        raise ValueError('Dataset must use VARa, VARb, VARc, hand in that order. Rebuild it from the ZIP with prepare_yolo_pilot.py build; original exports and annotations can be reused.')
    report=a.data.parent/'manifest.json'
    if not report.is_file():raise ValueError('Need reviewed dataset manifest')
    print(json.dumps({'gpu':torch.cuda.get_device_name(0),'epochs':a.epochs,
        'class_names':NAMES,'data':str(a.data.resolve()),'purpose':'pipeline pilot, not deployment approval'}),flush=True)
    data_path=a.data.resolve();output_path=a.output_dir.resolve()
    weights=Path('.local/models/yolo11s-seg.pt').resolve();weights.parent.mkdir(parents=True,exist_ok=True)
    # Ultralytics' AMP self-check can download an auxiliary model into cwd.
    os.chdir(weights.parent.parent)
    model=YOLO(str(weights))
    model.train(data=str(data_path),epochs=a.epochs,imgsz=a.imgsz,batch=a.batch,
        device=0,workers=2,amp=True,cache=False,seed=42,deterministic=True,
        patience=20,warmup_epochs=1,mosaic=0.,hsv_h=0.,hsv_s=.1,hsv_v=.25,
        overlap_mask=False,plots=True,project=str(output_path.parent),
        name=output_path.name,exist_ok=False)
    print('Pilot checkpoint:',output_path/'weights/best.pt',flush=True)


if __name__=='__main__':main()
