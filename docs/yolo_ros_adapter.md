# YOLO worker and shared ROS interface

`scripts/yolo_pixels.py` converts original-resolution YOLO masks into the existing
board observation schema. It uses the same GPU-worker protocol, source stamps,
latest-frame handling, debug image and fresh-snapshot service as B. B's default
configuration remains available. Run one backend at a time on these topics.

YOLO requires a local segmentation checkpoint with the exact four trained class
names: VARa, VARb, VARc, hand. CUDA is required. The adapter
does not silently download a COCO checkpoint or fall back to CPU.

The stable ROS class IDs are 0=VARa (large white), 1=VARb (small white),
2=VARc (small red), 3=hand. Older checkpoint names large_white/small_white/small_red
are normalized to the new names without editing the checkpoint. Source class order
is remapped by name, so a differently ordered export/model cannot swap destinations.

`retina_masks=True` is used and returned mask dimensions must equal original image
dimensions. Board masks use the configured pickup polygon. Supported outlines
produce centres and image angles; other board instances remain in uncertain_objects.
Hand masks are reported separately across the entire image, never as board pick
targets. No hand-triggered stop is implemented and no human-safety claim is made.

## Check and run after training

`config/yolo_live.json` points to `.local/training/yolo_pilot_001/weights/best.pt`.
Update that path if the training output differs. Do not use synthetic smoke-test
weights for a demo. The active input ROI is `config/perception_full_table.toml`.
It covers the full original camera frame so neither table half is excluded. This
is broader than an accurately traced table perimeter: background and fixed
destination fixtures are not geometrically excluded. The older right-side ROI
file remains available for reproducible historical comparisons. Custom settings
files must also set scene_config to config/perception_full_table.toml when used
for the new full-table workflow. Restart the inference worker after changing ROI.
Pixel centre/orientation fields, image resolution and ROS interfaces are unchanged.
Model weights are not changed by selecting a different ROI. Fixed fixtures still
require correct annotation/model behaviour or motion-side target exclusion.

```bash
./.tools/pixi run --frozen -e perception python -m unittest discover -s tests -p test_yolo_pixels.py -v
./.tools/pixi run --frozen -e perception python -m unittest discover -s tests -p test_live_b_pixels.py -v

./.tools/pixi run --frozen -e perception-gpu python scripts/yolo_pixels.py \
  --image .local/datasets/refinement_cases/reported_misses_001.png \
  --output-dir .local/offline_yolo/real_checkpoint_001

./.tools/pixi run --frozen python scripts/check_yolo_ros_replay.py \
  --image .local/datasets/refinement_cases/reported_misses_001.png
```

The replay uses localhost ROS domain 173, publishes saved images, checks matching
debug stamps and fresh service results, then stops replay and verifies rejection
of stale results. It never uses a real camera or robot. A passing replay establishes
transport behavior, not model accuracy. Buffers are bounded during failures.

For live operation, stop B first and start the existing Logitech driver separately:

```bash
./.tools/pixi run --frozen -e camera-ros scene-camera
```

In another terminal:

```bash
./.tools/pixi run --frozen live-yolo-pixels
```

Then request fresh board coordinates, or view the common overlay:

```bash
./.tools/pixi run --frozen read-yolo-pixels
./.tools/pixi run --frozen -e camera-ros ros2 run image_view image_view --ros-args \
  -r image:=/perception/boards_pixels/debug_image
```

Standard interfaces remain `/perception/boards_pixels` (String JSON),
`/perception/get_boards_pixels` (Trigger), and the debug Image topic. New fields
include model_backend, checkpoint_sha256, class_name/class_id/confidence, and
separate hand_detections. IDs remain snapshot-local. All execution/safety validation
flags stay false. Pixel angle is not robot yaw; metric mapping remains external.

## Current combined review plan

The two supplied ZIPs were merged into
`.local/datasets/cvat_exports/pilot_export_combined_001.zip`. Identical image names
and bytes are retained once; nonempty labels are preserved over blank labels.
Conflicting nonempty labels cause an error instead of silently choosing a version.
Original ZIPs are unchanged.

The draft `.local/datasets/cvat_exports/pilot_review_combined_001.json` has 36 rows:
33 nonempty exported annotations, two confirmed negative pickup scenes with fixed
destination fixtures, and one pending hand annotation (`oct25_scene_0040.png`).
Targets are 27 train / 9 val; validation is not an independent final test set.
The two confirmed negatives share group `oct25_empty_pickup`. Keep that entire
arrangement in one split. Other group assignments and reviewed flags require
human review. Source ZIP checksums prevent accidentally training on another export.

Fix 0040 in CVAT and export the corrected annotations before including all 36.
Do not mark it verified_empty: a hand is visible. After an updated merged snapshot,
refresh the review plan's provenance through a new inventory, preserving reviewed
group decisions only after checking the updated labels. The current build refuses
an incomplete 27/9 selection. If arrangement groups prevent precisely 27/9,
adjust target_split to the valid grouped counts rather than splitting one group.

On 2026-10-27, injected-mask tests and an isolated GPU/ROS replay using a synthetic
four-class checkpoint passed. No real trained board model or physical pick was
validated. The synthetic checkpoint returned zero objects in that replay; positive
coordinate and hand-separation paths are covered by the injected-mask tests.
