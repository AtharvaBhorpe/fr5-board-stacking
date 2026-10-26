# Vision 1: overhead camera board detector (branch `feat/vision1-overhead`)

Standalone pixi workspace (ROS 2 Humble, RoboStack). Run it next to the robot workspace on the same machine / ROS domain.

```bash
git clone -b feat/vision1-overhead https://github.com/AtharvaBhorpe/fr5-board-stacking.git vision1 && cd vision1
pixi install
pixi run vision                                    # overhead Logitech C920 (found by name), publishes /vision/boards
pixi run vision --ros-args -p device:=samples/01.jpg   # replay a photo, no camera needed
pixi run check                                     # self-check on recorded photos
```

## ROS API

| Name | Type | Content |
|---|---|---|
| `/vision/boards` | topic `std_msgs/String`, 5 Hz | JSON, latest state (below) |
| `/vision/detect` | service `std_srvs/Trigger` | fresh snapshot; same JSON in `.message` |

```python
msg = json.loads(s.data)   # {"stamp", "image_size", "calibrated", "boards": [...]}
```

Per board:

| Field | Meaning |
|---|---|
| `id` | index in this frame, left -> right in the image. **Not tracked** between frames |
| `variant` | `VARa` white 295x210, `VARb` white 150x210, `VARc` red 135x210 (mm) |
| `u`, `v`, `yaw_deg`, `corners` | image pixels (u right, v down); yaw of board +X, degrees in [-90, 90) |
| `x`, `y`, `z`, `yaw` | `base_link`, metres / radians; yaw of board +X about +Z, modulo pi. `z` = top of one sheet on the table (fixed) |
| `size_mm`, `size_ok` | measured short / long side; `size_ok` = within 10 mm of the variant size. **Pick only `size_ok` boards** (false = partly hidden, e.g. under the arm) |
| `zone` | `pick` (y < 0) or `drop` |
| `inside_calibration` | false = outside the calibration tags, position extrapolated (less accurate) |
| `corners_xy` | 4 corners in `base_link` |

Board +X is the long edge for VARa and the short edge for VARb / VARc (team geometry handoff).

## Limits

- Looks only at the pick area (right table in the image, `ROI` in `vision1/detect.py`).
- Boards must not touch or overlap: merged blobs are rejected (not reported).
- `config/calib.yaml` is copied from `feat/vision-transform`. Re-copy it after every recalibration; the camera must not move.
- Lab knobs (thresholds, ROI, area limits) are at the top of `vision1/detect.py` and `vision1/table.py`.
