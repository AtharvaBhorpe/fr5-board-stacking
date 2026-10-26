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
| `/vision/markers` | topic `visualization_msgs/MarkerArray` | RViz (Add -> By topic): green = pickable boards, red = destination poses. Display only, not in the MoveIt scene |

```python
msg = json.loads(s.data)   # {"stamp", "source", "calibrated", "boards": [...], "rejected": [...], "destinations": [...]}
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

`boards` holds only size-verified boards (pick these). Everything else (stacks, partly hidden boards, robot arm,
hands) is in `rejected` with a `reason`.

`destinations`: one per visible template tag (DICT_5X5_100, ids 1 / 2 / 16 = VARa / VARb / VARc), anywhere in the
image: `variant`, `tag_id`, `x`, `y`, `yaw` (target board pose, base_link, team offsets from
`feat/fr5-demo-integration` `config/perception_geometry.yaml`), `support_z` (template top, -0.055), `age` (s since
the tag was last seen; kept 10 s because small tags are missed in ~10 % of frames).

Board +X is the long edge for VARa and the short edge for VARb / VARc (team geometry handoff).

## Pick / place script (robot workspace)

From `fr5-board-stacking` (both repos side by side):

```bash
pixi run python3 ../vision1-overhead/robot/pick_one.py --hover-only   # real robot, first: check the position
pixi run python3 ../vision1-overhead/robot/pick_one.py                # pick, lift, put back (ENTER twice)
pixi run python3 ../vision1-overhead/robot/pick_one.py --place        # pick -> its template (--stack N, --press mm)
```

Simulation test (no robot, no camera): `pixi run sim` in `fr5-board-stacking`, then `robot/sim_regression.sh` here
(8 cases, ~3 min). **Never run `sim` and `robot` at the same time**: a leftover sim process makes the real robot load
mock hardware (it then reports SUCCESS but does not move).

## Limits

- Looks only at the pick area (right table in the image, `ROI` in `vision1/detect.py`).
- Boards must not touch or overlap: merged blobs are rejected (not reported).
- `config/calib.yaml` is copied from `feat/vision-transform`. Re-copy it after every recalibration; the camera must not move.
- Lab knobs (thresholds, ROI, area limits) are at the top of `vision1/detect.py` and `vision1/table.py`.
