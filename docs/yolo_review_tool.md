# Batch-fill a YOLO review plan

Use the local browser editor instead of editing JSON rows individually. It reads
the exported masks and displays thumbnails with polygon outlines. It does not
upload images or infer that annotations are complete.

New labels are VARa, VARb, VARc, hand. Older export names are accepted automatically.

To append exports without changing an existing review, use
`scripts/extend_yolo_review.py --previous-zip OLD.zip --previous-plan OLD.json
--new-zip NEW1.zip NEW2.zip --output-zip COMBINED.zip --output-plan COMBINED.json`.
Then open COMBINED.zip and COMBINED.json with the browser command below, using a
fresh output-plan filename. The default **New images only** view hides older rows.
Older review fields are locked in both the browser and dataset builder. Original
ZIPs and plans are preserved; exact duplicate images are skipped, and conflicting
existing annotations cause an error rather than silently replacing old labels.
New frames of an old physical arrangement must use that arrangement's existing
group and split. New arrangements can receive new group names.

For a new export, first create an inventory using `prepare_yolo_pilot.py inventory`
with that ZIP and a new plan filename. Omit `--annotated-only` to include possible
negative images; blank images remain excluded until explicitly reviewed.

```bash
./.tools/pixi run --frozen -e perception-gpu python scripts/review_yolo_plan.py \
  --zip .local/datasets/cvat_exports/pilot_export_combined_001.zip \
  --plan .local/datasets/cvat_exports/pilot_review_combined_002.json \
  --output-plan .local/datasets/cvat_exports/pilot_review_visual_003.json
```

Open the localhost URL printed in the terminal. Keep the terminal running.

1. Select thumbnails from one physical arrangement; click an image to enlarge it.
2. Enter a group such as `oct27_layout_01`, then choose `train`, `val` or `test`.
3. Click **Confirm selected reviewed + assign group/split**. This is your explicit
   confirmation that every target instance in those images has been checked.
4. Repeat for the other arrangements. Keep each arrangement in one split. Changing
   lighting, hands or robot pose alone is insufficient to establish independence.
   To move an existing group to a different split, select every reviewed image
   in that group and apply the new split together.
5. Exclude images with incomplete annotations. For genuinely empty scenes, select
   the explicit empty-scene confirmation before assigning the group.
6. Click **Save new plan**. Wait for its saved-path message. The output is a new
   file; the source plan is untouched. Then stop the server with Ctrl+C.

The tool preserves source ZIP identity and row metadata, refreshes annotation
counts from the ZIP, rejects split leakage, and fills `target_split` from the
actual reviewed counts. A mask count is exported annotation rows, not proof that
each physical instance is correctly segmented. This tool edits review metadata;
use CVAT to fix masks or classes and make a new export/inventory afterward.

Use the saved plan with `prepare_yolo_pilot.py build` and a new dataset output
directory. At least two train images and one val image are required. The dataset
builder retains its additional image-integrity and duplicate checks.

Aim for roughly 70/15/15 train/val/test by independent arrangements, with all four
classes and important scenarios represented in each split when possible. Exact
image counts are less important than preventing leakage. Prefer previously unseen
arrangements for test; never use test results to choose checkpoints, thresholds or
training settings. Two-split pilot plans still work but the builder warns that no
independent test set exists. When test images are selected, data.yaml includes
`test: images/test`; training continues to use only train and val.

After choosing the final checkpoint using validation, evaluate test once:

```bash
./.tools/pixi run --frozen -e perception-gpu yolo segment val \
  model=.local/training/yolo_pilot_001/weights/best.pt \
  data=.local/datasets/yolo_pilot_001/data.yaml \
  split=test device=0 imgsz=640 \
  project=.local/evaluation name=final_test_001
```

Use the actual final checkpoint and newly built three-split dataset paths. The
older yolo_pilot_001 dataset has no test split; rebuild before using this command.
