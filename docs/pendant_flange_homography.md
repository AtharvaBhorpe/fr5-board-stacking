# Pendant flange poses to camera-guided board positions

The CSV contains **flange** XYZ in pendant-base millimetres and flange RX/RY/RZ
in degrees. The operator confirmed that the cup centre touched each mark.
Blank `tool_slot` fields are permitted in explicit `--input-frame flange` mode.
The required `--confirm-pendant-base` flag confirms the pose reference frame;
it must not be used for workpiece/user-frame-transformed coordinates.

The importer reads the current adapter and cup offsets from the two project
Xacro files. They currently sum to **112.9 mm along flange +Z**. It applies:

```text
p_pendant_cup = p_pendant_flange + Rz(rz) Ry(ry) Rx(rx) * offset_flange_cup
p_base_link_cup = diag(-1, -1, 1) * p_pendant_cup
```

Millimetres are converted to metres before the calculation. The angle convention
is FAIRINO's fixed-axis X/Y/Z convention, not an axis-angle rotation vector:
[FAIRINO data structures](https://fairino-doc-en.readthedocs.io/3.8.0/SDKManual/CPPDataStructure.html).

The original CSV, flange pose, orientation and source hashes are preserved.
The importer never changes the URDF, controller settings, robot pose or CSV.
Its tool-chain reader handles this project's literal, unrotated mount chain;
a different mount chain requires reviewing the reader.

## 1. Check the software and import the corrected points

Use one terminal for these commands so the variables remain defined:

```bash
cd /home/smane/Documents/ChatGPT/Roboinnovate/fr5-board-stacking

python3 scripts/import_pendant_points.py --self-test
./.tools/pixi run --frozen -e perception \
  python -m unittest discover -s tests -p test_pendant_flange_import.py -v

SURVEY="$PWD/.local/datasets/physical_validation/pendant_20261026_121540"
POINTS="$SURVEY/points_cup_from_flange_001.jsonl"
FITDIR="$SURVEY/fit_cup_from_flange_001"

python3 scripts/import_pendant_points.py \
  --csv "$SURVEY/pendant_points.csv" \
  --output "$POINTS" \
  --input-frame flange \
  --tip tcp \
  --confirm-pendant-base \
  --confirm-tip-contact
```

This outputs cup-tip coordinates in `base_link`, not flange coordinates.
All output files are exclusive-create: use a new numbered filename/folder when
repeating the import or solve. Do not overwrite earlier measurements or results.

Physical tool geometry remains provisional. `--confirm-tip-contact` records
contact only; it does not claim that the CAD offset was physically calibrated.
After the real relaxed-cup offset has been checked against the model, reimport
into a new file with the additional `--confirm-tool-geometry` flag. If the model
offset changes, regenerate both imported points and the homography.

## 2. Pair image marks with the converted cup positions

Use the original image in which those same fixed marks are visible. Camera
mount, focus, resolution and table geometry must match the inference setup.

```bash
./.tools/pixi run --frozen -e perception \
  python scripts/homography_no_rail.py mark \
  --points "$POINTS" \
  --image "$SURVEY/reference_lighting_v2.png" \
  --folder "$FITDIR" \
  --tip tcp \
  --exclude F5 V3
```

Click the exact points touched by the cup, in the prompted order:
**F1, F2, F3, F4, V1, V2, V4**. Press U to undo, Enter to save.
F5 and V3 are excluded because they are on the raised aluminium rail.

If a previous pixel annotation uses the identical image and labels, its clicks
remain geometrically usable, but do not reuse any old robot positions or H.

## 3. Fit and independently check the new mapping

```bash
./.tools/pixi run --frozen -e perception \
  python scripts/homography_no_rail.py solve \
  --points "$POINTS" \
  --image "$SURVEY/reference_lighting_v2.png" \
  --folder "$FITDIR" \
  --tip tcp \
  --exclude F5 V3 \
  --limit-mm 5
```

Only F1-F4 fit H and the surface plane. V1, V2 and V4 are independent checks.
`independent_check_passed` requires all independent XY/XYZ errors and all fit
plane residuals to be below 5 mm. Four-point homography fit error alone is not
evidence of accuracy.

With the CSV inspected on 2026-10-26, V1's converted cup point is approximately
5.85 mm from the F1-F4 fitted plane even before selecting image points. Thus
the present measurements cannot pass the 5 mm XYZ check. Check V1's saved
flange pose, contact/support height and the actual cup offset; do not force its
Z to match the plane, remove it merely to pass, or adjust the threshold silently.
If markers/camera have not moved, a corrected V1 measurement can use the same
image. Otherwise obtain a new matched reference image and survey as needed.

When the imported records include verified physical geometry and confirmed
contacts, the solve also needs `--reference-contact-verified` to record that
operator confirmation. That flag alone cannot certify provisional input records.
`execution_allowed` remains false; no script here sends motion.

## 4. Update the perception-to-motion handover

- Replace the old homography path with this run's `homography_validation.json`
  only after reviewing its independent errors and reference verification.
- Keep raw B inference in the original camera image. Undistort boundary points
  using the saved K/D, then apply H to get `base_link` X/Y in metres. Fit board
  centre and long-edge yaw in that metric plane, not from image-space angles.
- H already uses the **cup's measured contact positions**. Do not apply another
  112.9 mm translation to mapped board centres.
- H covers the surveyed plane. Board tops, destination supports and stacks at
  other heights require the appropriate height/parallax correction or depth.
  A single-layer known-height test remains the first integration scope.
- The motion interface must target the same `tcp` frame. If it targets flange
  or `wrist3_link`, use the robot model to convert the requested cup pose once.
  Do not treat board yaw as a complete tool orientation.
- Send each snapshot's instance ID, class, source timestamp, `base_link` board
  pose, destination board pose and calibration identifier. The motion side
  applies the grasp/tool relationship, approach clearance, reachability and
  collision checks; it requests a fresh observation before the next pick.
- First preview the mapped V points and a separated board with no execution.
  Physical clearance/contact and pick/place checks are separate from the fit.

Detector A/B models, camera intrinsics and pixel ROI polygons do not need
retraining/recalibration merely because the flange-to-cup import was corrected.
Any calibration derived from the old uncorrected robot points must be rebuilt.
