# Teammate scene evaluation — best YOLO11s-seg checkpoint

All 13 JPEG images completed in both full-resolution and fast-live mode. They contain no annotation files. Precision, recall and verified class/centre errors cannot be computed. The original ZIP and images were preserved and no training was performed.

Checkpoint SHA256: `67ee9c19255eea65462b4289ebb6b8f3cf0f88347ff5325e5dded77a3da64a58`. Confidence .25, network size 960, full camera-frame ROI.

Across the 13 images: **115 raw board proposals** in each mode; **55 full-resolution coordinate proposals / 60 uncertain**, versus **54 fast coordinate proposals / 61 uncertain**. These are proposal totals, not counts of correctly detected physical boards. No proposal was rejected by the ROI/area filters.

Visual inspection found **10 false hand masks across 9 images**, on brown cardboard fixtures. No hand is visibly present in the table workspace in these images. Hand output is not suitable for a demo stop trigger without addressing and validating this failure.

Important: scene_01 emits duplicate VARa/VARb coordinates for the same board in full-resolution mode. Most obvious separated white boards are detected, but some isolated rotated boards and red boards are withheld by the supported-outline check. Stacks and perpendicular overlaps remain unreliable. Brown destination fixtures are new confusers: some become board masks, and red masks can include brown backing.

| Image | Raw board proposals | Full coordinates | Fast coordinates | Fast uncertain | False hand masks |
|---|---:|---:|---:|---:|---:|
| scene_01.jpg | 7 | 3 | 2 | 5 | 1 |
| scene_02.jpg | 7 | 3 | 3 | 4 | 1 |
| scene_03.jpg | 9 | 3 | 3 | 6 | 1 |
| scene_04.jpg | 9 | 5 | 5 | 4 | 1 |
| scene_05.jpg | 8 | 5 | 5 | 3 | 1 |
| scene_06.jpg | 10 | 6 | 5 | 5 | 1 |
| scene_07.jpg | 8 | 6 | 5 | 3 | 1 |
| scene_08.jpg | 8 | 6 | 5 | 3 | 0 |
| scene_09.jpg | 9 | 6 | 6 | 3 | 0 |
| scene_10.jpg | 9 | 1 | 2 | 7 | 0 |
| scene_11.jpg | 12 | 3 | 2 | 10 | 0 |
| scene_12.jpg | 11 | 4 | 5 | 6 | 1 |
| scene_13.jpg | 8 | 4 | 6 | 2 | 2 |

## Frame findings

- **scene_01.jpg**: Same white board has overlapping VARa/VARb predictions; full-resolution mode emits two coordinates for it. Isolated rotated white and red outlines remain uncertain. Brown fixture predicted as hand.
- **scene_02.jpg**: Three separated left white boards yield coordinates. Red and right white remain uncertain. Brown fixture predicted as hand.
- **scene_03.jpg**: Offset white stack has merged/fragmented uncertain outlines. Separated lower red and white yield coordinates. Brown fixtures generate spurious raw proposals.
- **scene_04.jpg**: Separated left whites and lower red yield coordinates. Upper red stack and right white remain uncertain. Brown fixture predicted as hand.
- **scene_05.jpg**: Separated left whites and lower red yield coordinates. Rotating the brown fixture preserves a false hand prediction.
- **scene_06.jpg**: White placed on brown fixture remains uncertain. Middle right white passes full-resolution geometry but not fast geometry. Brown fixture also has board/hand proposals.
- **scene_07.jpg**: Red on the left brown fixture yields coordinates; right white on brown remains uncertain. A right brown fixture is classified as hand.
- **scene_08.jpg**: Red on right brown fixture remains uncertain. Isolated right middle white passes full-resolution but not fast geometry.
- **scene_09.jpg**: Both red proposals remain uncertain; one incorporates brown backing. Lower rotated left white remains uncertain. Several separated whites yield coordinates.
- **scene_10.jpg**: Overlapping left whites remain uncertain. Right brown fixture generates a spurious VARa mask. Red proposals include backing or irregular outlines. Right isolated upper white passes fast but not full-resolution geometry.
- **scene_11.jpg**: Overlap group remains uncertain. Brown fixtures produce spurious board proposals. Upper isolated right white passes full-resolution but not fast geometry.
- **scene_12.jpg**: Left brown fixture is predicted as hand; rail-crossing brown fixture has a spurious board proposal. Lower rotated left white remains uncertain.
- **scene_13.jpg**: Two brown fixtures are predicted as hands. Fast mode yields coordinates for lower red and isolated right white that remain uncertain in full-resolution mode.

## Recommended next actions

1. Resolve conflicting class predictions before treating proposals as distinct physical instances. Do not simply lower the confidence or geometry threshold.
2. Register tagged destination fixtures separately from markerless substrates, and use motion/task state to distinguish waiting boards from placed boards. Do not globally exclude all tagged areas if substrates can lie on them.
3. Add analogous brown-fixture and red-on-brown examples to the training set with correct visible instance masks and background/hand labels. Keep this evaluation set separate if it will be used for an independent score.
4. For the immediate demo, prefer separated substrates on black cloth; withhold ambiguous overlap picks or resolve them with the wrist/depth path. A rectangle alone does not identify the top layer.
5. Annotate these 13 images if measured precision/recall and pixel-centre error are required. Confirm physical VARa/VARb labels; appearance alone cannot establish their millimetre dimensions.

## Files

- `summary.json`: both settings, checkpoint/archive provenance, raw and accepted class counts.
- `visual_review.csv`: per-frame visual review.
- `full_resolution/contact_01.jpg`, `contact_02.jpg`: all coordinate overlays.
- `fast_live/contact_01.jpg`, `contact_02.jpg`: all fast coordinate overlays.
- Each mode also has a JSON report, raw mask contours, and coordinate overlay for every input image.

Green = supported image quadrilateral, orange = uncertain outline, magenta = predicted hand. Green does not certify a correct class, unique instance, metric pose or safe grasp.
