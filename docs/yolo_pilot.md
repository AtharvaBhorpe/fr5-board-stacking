# YOLO11s segmentation pilot while CVAT annotation continues

This is an offline GPU training workflow. No robot or ROS process is started.
The existing perception-gpu Pixi environment includes pinned Ultralytics 8.3.228,
alongside the existing Torch 2.7.1+cu128. Dataset, model and run files stay in .local.

Canonical classes: 0=VARa (formerly large_white), 1=VARb (small_white),
2=VARc (small_red), 3=hand. The export reader accepts old and new names and remaps
class IDs by name. New merged ZIPs and built datasets use the canonical names.
Existing ZIPs, plans and checkpoints are preserved. Before training on an older
built dataset, rebuild from its original ZIP and reviewed plan into a new directory;
the training script rejects old data.yaml label names rather than silently changing it.

## Freeze an export

Finish and visually review at least 20–30 varied images first, preferably with all
four classes represented. Export the task from CVAT as **Ultralytics YOLO
Segmentation 1.0**, with images. Also retain a COCO master and CVAT backup. Save
the YOLO ZIP as `.local/datasets/cvat_exports/pilot_export_001.zip`.

You can export the whole partially annotated task: the review plan below excludes
unfinished images. Never assume no mask means no target. Inspect exported
polygons, especially disconnected masks: the YOLO conversion may connect fragments.

```bash
mkdir -p .local/datasets/cvat_exports
./.tools/pixi run --frozen python scripts/prepare_yolo_pilot.py inventory \
  --zip .local/datasets/cvat_exports/pilot_export_001.zip \
  --plan .local/datasets/cvat_exports/pilot_review_001.json
nano .local/datasets/cvat_exports/pilot_review_001.json
```

Change only fully reviewed rows, for example:

```json
{
  "file_name": "s0_scene_0180.png",
  "exported_instances": 8,
  "reviewed": true,
  "arrangement_group": "arrangement_01",
  "split": "train",
  "verified_empty": false
}
```

The filename and instance count above are examples; retain actual values from the
inventory. `reviewed=true` means every visible target board and hand is labelled,
classes and boundaries are checked, and the YOLO export has been visually checked.
Do not mark a partially annotated frame reviewed.

Use `split: "val"` for approximately 20% of the reviewed pilot images, from
different arrangements. Near-identical frames and lighting variants of the same
arrangement share one group and one split. Leave all unfinished rows unchanged.
Set `verified_empty: true` only for an explicitly reviewed image with no target
boards or hands. This pilot has no test split; reserve independent images for
the final test set and do not tune on them.

## Build and inspect

```bash
./.tools/pixi run --frozen python -m unittest discover -s tests -p test_yolo_pilot.py -v
./.tools/pixi run --frozen python scripts/prepare_yolo_pilot.py build \
  --zip .local/datasets/cvat_exports/pilot_export_001.zip \
  --plan .local/datasets/cvat_exports/pilot_review_001.json \
  --output-dir .local/datasets/yolo_pilot_001
```

Read the printed per-class train/validation counts and warnings. Missing classes
mean the run cannot assess those classes. The script validates format, bounds,
duplicate polygons, image decoding, explicit negatives and group isolation. It
cannot prove manual class/boundary correctness or catch all polygon topology issues.

## Train locally

Stop B inference or other local GPU training first. Browser-based CVAT annotation
can continue; the training reads only its immutable local snapshot.

```bash
./.tools/pixi run --frozen -e perception-gpu python scripts/train_yolo_pilot.py \
  --data .local/datasets/yolo_pilot_001/data.yaml \
  --output-dir .local/training/yolo_pilot_001 \
  --epochs 10 --batch 2 --imgsz 640
```

The initial run downloads pretrained weights if necessary. CUDA is required;
there is no CPU fallback. New output folders are required. If out of memory,
retry using a new output name and `--batch 1`. The pilot is a training/export
integration check, not a reliable deployment model.

## Inspect the checkpoint

```bash
xdg-open .local/training/yolo_pilot_001/results.png
xdg-open .local/training/yolo_pilot_001/val_batch0_labels.jpg
xdg-open .local/training/yolo_pilot_001/val_batch0_pred.jpg

./.tools/pixi run --frozen -e perception-gpu yolo segment predict \
  model=.local/training/yolo_pilot_001/weights/best.pt \
  source=.local/datasets/yolo_pilot_001/images/val \
  device=0 imgsz=640 retina_masks=True conf=0.25 \
  project=.local/offline_yolo name=pilot_001 save=True
```

Compare visible per-instance boundaries, class assignment, missed/merged boards
and hand detections. Low pilot confidence is not evidence that a workspace is safe.
No predicted data is sent to the motion controller by this workflow.

Continue CVAT annotation while training. Export a NEW ZIP later, create a NEW
review plan and dataset folder, then run the main training from the pretrained
YOLO11s checkpoint using reviewed data. Do not use resume=true to change the
dataset halfway through an existing run. Changing the ZIP invalidates its old plan.

No reviewed CVAT export has been supplied at implementation time, so real pilot
accuracy and actual training duration remain unmeasured.
