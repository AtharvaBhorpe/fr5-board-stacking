# Portable Brev training

Create one x86_64 L40S instance (48 GB), named fr5-yolo-training. Use a supported
GPU/PyTorch base environment, at least 50 GB disk, preferably 100 GB, and a
provider that supports stop/start. Check the displayed total price before launch.

The local archive `.local/brev/fr5_yolo_training_001.tar` contains only the reviewed
augmented dataset, its manifest/checksums, two training scripts and a minimal Pixi
project. No camera recordings, Git history, credentials or existing checkpoints
are included. Source data and splits are unchanged. This starts a fresh pretrained
YOLO11s-seg run, not a resume of the laptop's running optimizer state.

On the laptop, install the official Brev CLI through Pixi if absent:

```bash
./.tools/pixi global install --channel conda-forge brev
export PATH="$HOME/.pixi/bin:$PATH"
brev login
brev refresh
brev ls
brev exec fr5-yolo-training 'mkdir -p ~/workspace'
brev copy .local/brev/fr5_yolo_training_001.tar fr5-yolo-training:workspace/
brev copy .local/brev/fr5_yolo_training_001.tar.sha256 fr5-yolo-training:workspace/
brev shell fr5-yolo-training
```

In the remote shell:

```bash
cd ~/workspace
sha256sum -c fr5_yolo_training_001.tar.sha256
test ! -e fr5_yolo_training_001 && tar -xf fr5_yolo_training_001.tar
cd fr5_yolo_training_001
sha256sum --quiet -c SHA256SUMS
nvidia-smi
```

Stop on any checksum failure. Install Pixi using its official installer if missing,
then `pixi install`. The core torch/torchvision/Ultralytics/numpy versions are pinned;
this lightweight training project avoids ROS and does not replicate the full
laptop environment's transitive dependency lock. Preserve the remote pixi.lock
with the results.

```bash
pixi run python - <<'PY'
from pathlib import Path
import torch, yaml
assert torch.cuda.is_available(), 'CUDA unavailable; do not train on CPU'
assert torch.isfinite(torch.ones((64,64),device='cuda') @ torch.ones((64,64),device='cuda')).all()
print(torch.__version__,torch.cuda.get_device_name(0))
p=Path('dataset/data.yaml')
c=yaml.safe_load(p.read_text());c['path']=str(p.parent.resolve())
p.write_text(yaml.safe_dump(c,sort_keys=False))
PY
nohup pixi run python scripts/train_yolo_pilot.py \
  --data dataset/data.yaml --output-dir outputs/yolo_cloud_001 \
  --epochs 100 --batch 32 --imgsz 960 > training.log 2>&1 &
tail -f training.log
```

Ctrl+C exits tail without stopping the detached training. A checksum difference
for dataset/data.yaml after rebasing its path is expected; verify before rebasing.
Download outputs and training.log before stopping/deleting the instance. Run the
existing local evaluator against the same dataset and full-table ROI. Never
replace the live checkpoint solely because training loss improved.
