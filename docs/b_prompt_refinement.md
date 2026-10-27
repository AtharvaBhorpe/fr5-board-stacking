# Experimental B prompt refinement

No weights, training, robot driver or motion are added. Baseline configuration
is unchanged. Opt in with `config/pretrained_b_refined.json`.

The first SAM box pass supplies cached image embeddings. Unsupported masks get
up to six point prompts total per frame, with three mask alternatives per prompt.
Long observed internal lines can suggest a positive point on one side and a
negative point on the other. Other independent proposals supply exclusion points
only from surfaces outside the current mask. These remain hypotheses, not known
instance identities. Image embeddings are reused for all decoder refinements.

Acceptance requires a single convex four-corner contour with raster IoU >= 0.93,
image edges near at least 50% of every edge and 70% on average, and no detected
long internal separator. Duplicate/nested masks and conflicting visible masks
are suppressed. The thresholds are provisional image checks, not measured robot
accuracy. Perspective quads need not have right angles in raw camera pixels.

For a slightly irregular contour, a fitted rotated rectangle may also be tested;
it must meet the same 0.93 mask IoU and image-edge checks. Baseline detections
which refinement cannot validate remain orange `UNCERTAIN` previews in
`uncertain_objects`, outside the accepted `objects` array. The reader reports
their count but does not hand their coordinates to the motion-side client.

Printed straight lines can cause false rejection. Occluded boards can be omitted.
No hidden boundary is completed. A convincing quad can still be an incorrect
instance. `instance_isolation_verified`, `layer_order_verified` and
`execution_allowed` remain false. No layer-order or top-board claim is made.

## Test the same inputs with both versions

```bash
./.tools/pixi run --frozen -e perception python -m unittest discover -s tests -p test_b_refinement.py -v
./.tools/pixi run --frozen -e perception python -m unittest discover -s tests -p test_live_b_pixels.py -v
./.tools/pixi run --frozen -e perception-gpu python scripts/compare_b_refinement.py \
  --images .local/datasets/camera_slot_002/board_examples_002/scene_0000.png \
           .local/datasets/camera_slot_002/board_examples_002/scene_0195.png \
  --output-dir .local/offline_b/refinement_manual_001
```

Use a new output folder on each run. Each input has baseline/refined subfolders,
an overlay, separate accepted mask PNGs and full report JSON with rejection reasons.
Do not score counts alone: compare mask boundaries to each physical board.

## Live observation only

Stop the existing B terminal with Ctrl+C; keep the camera running. Then:

```bash
./.tools/pixi run --frozen live-b-pixels settings:=config/pretrained_b_refined.json
```

The input ROI remains `config/perception_review_roi_v3.toml`; verify it covers
the current pickup area. Standard topic, service and pixel coordinates are
unchanged. Restart with `pixi run --frozen live-b-pixels` to restore baseline.

The live transport now uses lossless NumPy frames rather than PNG compression;
raw image resolution and pixel coordinates are unchanged. Timing fields include
`detector_ms`, `pipeline_ms` and `frame_transfer_write_ms`. Refinement adds decoder
and geometry work; it is not a speed optimization.

## Observed checks on 2026-10-26

The 70-test suite passed (the ROS calibration integration test was run separately
in camera-ros). On scene_0195, baseline merged the two white boards; refinement
isolated the right-hand board and omitted the robot-occluded board. On scene_0000,
baseline returned two candidates and refined mode accepted none. This does not
establish general overlap recall or layer ordering. Printed surfaces can be
rejected by the separator check.

An isolated ROS/GPU replay of scene_0195 passed continuous publication, matching
debug image, fresh-snapshot service and camera-loss rejection. One refined pass
took 441 ms with 711 ms source age at the client; this is not a sustained-rate
benchmark and no physical robot was used.

On a saved live frame reporting missed obvious boards, strict mode initially
accepted three outlines. The corrected fit restored the separated red board,
giving four accepted outlines and one orange uncertain P5-board preview. The
P5 mask had multiple visible components; it was not promoted to a complete board.
This was an offline replay, not a live acceptance test.
