# Offline perception speed and accuracy checks — 2026-10-28

These are saved-image tests on the local RTX 4070 Laptop GPU. They do not measure
Logitech capture, USB transport, physical accuracy, or robot execution.

## Launch

The trained checkpoint must exist locally at
`.local/brev_results/yolo_cloud_001/weights/best.pt` (weights are not committed).
SHA256: `67ee9c19255eea65462b4289ebb6b8f3cf0f88347ff5325e5dded77a3da64a58`.

```bash
./.tools/pixi run --frozen -e camera-ros scene-camera
```

In a separate terminal:

```bash
./.tools/pixi run --frozen live-yolo-fast
```

Read a fresh observation:

```bash
./.tools/pixi run --frozen read-yolo-pixels
```

The existing `live-yolo-pixels` task uses `config/yolo_live.json`, including its
existing pilot checkpoint path. It does not automatically select the cloud model.
Do not run both perception servers simultaneously.

## Speed checks

The initial full-HD mask/appearance path took about one second for a six-mask
scene. Cropped mask work and skipping unused colour statistics reduced this.
Native GPU masks, FP16, RAM frame transfer, reduced preview work, and separating
camera reception from blocking snapshot requests remove further overhead.

The first passing 30-FPS input replay produced 29.0 coordinate updates/second.
The final crowded held-out replay measured the entire 20-second streaming window:
29.7 updates/second, 18.8 ms median detector time, 35.1 ms median source-to-result
age and 48.2 ms p95 age. Repeated-source-timestamp and failure recovery checks
also passed. An empty 1080p synthetic scene produced 29.8 updates/second and
zero board targets.
The preview runs at one image/second in fast mode; it is not the coordinate rate.

The isolated ROS test checks positive board coordinates, hand separation,
fresh requests, unique snapshot IDs, source/debug timestamps, repeated source
timestamps, camera loss/reconnection, and GPU worker failure. It uses domain 173
with localhost-only discovery and sends no motion command.

```bash
./.tools/pixi run --frozen python scripts/check_yolo_ros_replay.py \
  --settings config/yolo_live_fast.json \
  --image .local/datasets/augmented_refresh_001/images/test/s0_scene_0143.png \
  --samples 15 --min-boards 1 --input-fps 30 --min-update-fps 24 \
  --stream-seconds 20
```

Results and logs are written under a new `.local/yolo_ros_replay_*` directory.
A cadence failure is recorded as a failure even when recovery checks pass.

## Accuracy tradeoff

All 48 validation and 46 test images were evaluated. Compact masks differ from
full-resolution retina masks, so speed is not free precision. At confidence .25,
with class-correct mask IoU >= .5:

| Test class | Full-resolution TP / FP / FN | Fast TP / FP / FN |
|---|---|---|
| VARa | 98 / 40 / 89 | 94 / 44 / 93 |
| VARb | 69 / 29 / 33 | 69 / 29 / 33 |
| VARc | 72 / 8 / 24 | 70 / 10 / 26 |
| hand | 78 / 18 / 18 | 78 / 18 / 18 |

For matched annotations with a supported quadrilateral, test centre error was
2.34 px median / 6.47 px p95 for full-resolution measurement (71 comparisons),
versus 5.09 px / 8.51 px for fast measurement (68 comparisons). Image-angle error
was .39 / 1.70 degrees versus .63 / 1.94 degrees. These are image-label errors,
not millimetre accuracy or physical grasp success. The subsets differ.

With the new outline checks, full-resolution coordinate targets increased from
54 to 95 across the test set. Fast mode emitted 85 targets. Full-resolution
recovery added 41 targets; 39 matched class-correct annotations at IoU >= .5.
Two recovered targets remained unmatched. Significant occlusions, merged masks,
and white size-class confusion remain limitations. A supported rectangle does
not prove topmost-layer identity or single-board isolation.

Reports: `.local/evaluation/yolo_cloud_fast_001` and
`.local/evaluation/yolo_native960_fast_001`. To reproduce fast operating-point
evaluation (not an mAP sweep):

```bash
./.tools/pixi run --frozen -e perception-gpu \
  python scripts/evaluate_yolo_fast.py \
  --settings config/yolo_live_fast.json \
  --dataset .local/datasets/augmented_refresh_001 \
  --output-dir .local/evaluation/yolo_fast_recheck_001
```

## Logitech conversion warning

The YUYV-to-BGR8 warning describes CPU colour conversion on each frame. A local
OpenCV proxy on synthetic 1920 x 1080 YUYV data measured .16 ms median and 6.0 ms
p95 over 200 samples. This is not a measurement of the actual camera driver.
Removing conversion from the camera driver merely moves it elsewhere if the
detector still needs BGR/RGB input.

1080p YUYV at 30 FPS requires 124.4 MB/s of image payload across USB, before
protocol overhead. Device-advertised format/rate support therefore matters.
The current ROS YAML does not explicitly set `time_per_frame`; the actual capture
rate must be measured. Earlier 5-FPS observations and TOML settings are not proof
that the current ROS driver is explicitly configured for 5 FPS.

When the camera returns, measure its source topic and inspect advertised device
formats. If 1080p/30 requires MJPEG, use a standard ROS driver that supports that
device format (for example `usb_cam`), preserving resolution, field of view,
focus, intrinsics, source frame ID and timestamps. Do not change to MJPEG blindly
in a driver that does not support it. ROS `image_transport` compression after
capture does not change the USB capture format.

```bash
./.tools/pixi run --frozen -e camera-ros \
  ros2 topic hz /scene/image_raw --window 60
```

Official references:
- https://index.ros.org/p/v4l2_camera/
- https://github.com/ros-drivers/usb_cam

No live-camera 24–30 FPS claim is made until capture and inference are tested
together. Metric calibration and robot-side freshness checks remain necessary.

## Additional teammate scenes

`sim_scenes.zip` supplied 13 unlabelled 1080p images. Both full-resolution and
fast modes completed with the same checkpoint and confidence. They emitted
115 raw board proposals each, with 55 versus 54 supported-coordinate proposals.
These counts are not true-positive counts. Visual review found brown tagged
fixtures mistaken for hands/boards, unresolved overlaps, and duplicate conflicting
VARa/VARb coordinates on one physical board in scene_01 (full-resolution mode).
Several separated rotated substrates also remained uncertain. Speed optimizations
do not resolve these model/instance-identity failures. The new samples were not
used for training or to tune thresholds.

Local results: `.local/evaluation/sim_scenes_best_001/REPORT.md`,
`visual_review.csv`, `summary.json`, and per-frame masks/coordinate overlays in
`full_resolution/` and `fast_live/`. No precision/recall is claimed without labels.
