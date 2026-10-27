# Appearance augmentation without re-annotation

Review newly annotated originals with `extend_yolo_review.py` and
`review_yolo_plan.py`, preserving older arrangement/split assignments. Build the
combined originals with `prepare_yolo_pilot.py build` first.

```bash
./.tools/pixi run --frozen -e perception-gpu python scripts/augment_board_dataset.py \
  --dataset .local/datasets/reviewed_refresh_001 \
  --output-dir .local/datasets/augmented_refresh_001
```

The output is a separate YOLO dataset: original train images plus four deterministic
appearance variants each (dim, bright, soft shadow, sparse neutral board marks).
Validation and test images/labels are copied byte-for-byte without augmentation.
All polygons are unchanged. Source originals and annotations are untouched.
The manifest records source hashes, seeds and lineage. Existing output folders
and already augmented inputs are rejected. Inspect augmentation_preview.jpg and
representative full-resolution outputs before training.

Small marks are clipped to eroded visible board masks and excluded from annotated
hands. This depends on correct source masks. Appearance variants do not create new
overlap layouts, resolve hidden instances, correct class labels, or validate metric
coordinates. Never split a source and its variants into different evaluation sets.
Review the originals once; synthetic children inherit train and need no new
train/val/test classification. Visual appearance quality still requires inspection.

Train the new data.yaml with train_yolo_pilot.py, batch 8, imgsz 960, up to 100 epochs
with existing early stopping. Choose checkpoints on validation; evaluate the test
set only after final selection. Do not infer improvement from training loss alone.
