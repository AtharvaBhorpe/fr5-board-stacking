# Camera and scene calibration

The observed grids contain ArUco 5x5 codes. The large grid's printed ID
layout matches OpenCV ChArUco **7 by 5**, which is the same physical grid the
user counted as 5 across by 7 down after rotating it. Measured checkerboard
cell side: **0.052 m**. Marker outer black-square side: **0.038 m**.
The smaller print was separately measured: cell side **0.032 m**, marker side
**0.022 m**. Its ID ordering matches **5 by 7**, independently checked on real
camera images. Both prints are mounted on rigid flat backing (user confirmed).

Separate board profiles and image directories are configured under
`calibration.boards`. The active profile is selected by `active_board` or
`--board large` / `--board small`. The intrinsic fitter combines views using
each print's own measured 3D point coordinates. Never relabel old large-grid
images as small-grid images by changing one global square length.

## Logitech intrinsics

Use one large grid at a time. Attach it flat to rigid matte backing and
keep fingers outside the printed pattern. Loose or bent paper is unsuitable
for accurate calibration. Keep the Logitech mount, resolution and lens
focus fixed. Autofocus was disabled; the current fixed focus setting is 35.

The original stream delivered 1920x1080 MJPEG at about 30 fps. One calibration
capture showed JPEG corruption and was rejected after visual inspection.
For stationary calibration, the camera was switched to uncompressed YUYV,
1920x1080 at 5 fps. These are capture settings, not the intended final
obstacle-monitoring update rate. Recheck image geometry and quality before
switching codec/profile for inference.

Collect at least 15 accepted, varied views; the user requested 25-30 for
redundancy and better spatial/angle coverage:

1. Include centre, corners and edges of the camera image where accessible.
2. Include several out-of-plane tilts in both directions. Rotating the sheet
   while it remains flat on the table is not sufficient.
3. Include moderate distance changes while retaining sharp focus. Do not
   change the focus control to make an individual view sharp.
4. Pause for each capture. Keep the complete pattern visible where possible.
5. Reject blur, glare, bends, clipping, corrupt frames and robot occlusion.

All images stay in ignored `.local/` folders. Use the camera checker with
`--capture-dir` to save a raw sample. Inspect its quality before acceptance:

```bash
pixi run -e perception fr5-logitech-calibration inspect --image PATH_TO_SAMPLE
pixi run -e perception fr5-logitech-calibration accept --image PATH_TO_SAMPLE
pixi run -e perception fr5-logitech-calibration fit
```

Replace `pixi` with `./.tools/pixi` when using this clone's local bootstrap.
The fitter reserves every fifth accepted view for validation. It reports
training/held-out pixel errors, image coverage and tilt diversity. The
thresholds in `config/perception.toml` are engineering checks; they do not
prove millimetre placement accuracy. Results are written to
`config/calibration/logitech_intrinsics.json`, including an explicit pass/fail
flag. Intrinsics alone never enable robot execution.

## Fixed scene/workcell frame

After lens calibration, fix reference markers on rigid, stationary surfaces.
Record their real corner coordinates and heights in a defined workcell frame.
Use a known visible grid or individually measured markers to estimate the
Logitech-to-workcell transform. Markers may also define pickup/destination
boundaries, but those boundaries need an explicit mapping to marker corners
or centres. Do not infer a boundary from an arbitrary collection of IDs.

Current grid sheets reuse IDs. Never treat repeated IDs as unique anchors.
Markers on a movable substrate are object markers, not table anchors. Keep
fixed references visible outside expected stacks and record their mounting
plane height, including backing/substrate thickness.

The motion owner also needs the workcell-to-robot-base transform, established
from independent measured points and the calibrated tool. A tabletop
homography applies only to its plane; source and destination stack surfaces
need height-aware geometry or wrist refinement.

## D405 and wrist mounting

Retain the D405 factory colour/depth intrinsics, depth scale and internal
stream extrinsics. The RealSense distortion model must be handled correctly;
do not blindly feed inverse-Brown coefficients into an OpenCV Brown model.
Local RGB/depth samples and factory stream metadata were recorded during
commissioning. Pixel depth validity is not a depth-accuracy guarantee.

For hand-eye calibration, fix one measured grid to the table. The motion
owner manually positions the wrist at 15-25 diverse, stopped poses that keep
the grid visible and inside the D405's usable range. Include rotation about
at least two axes. For every image record the measured base-to-flange pose
at capture time. Joint command targets cannot substitute for feedback.

The result is a camera-to-flange transform. Combine it with measured robot
pose and SDK deprojection to express local observations in the workcell.
Validate against separate known points and heights before accepting grasps.
These tools neither move the arm nor operate suction.
