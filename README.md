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
| `/vision/image` | topic `sensor_msgs/Image` (bgr8, 960x540) | camera frame with detections drawn; only published while someone subscribes |

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

**Learned model (opt-in, `-p model:=true`)**: boards may touch / overlap. `boards` then holds the top boards
(full rectangle visible); `variant` can be `UNKWN` (class confidence < 0.6, or a top board whose size matches no
variant). Extra fields `conf`, `model_class`, `top`. New list `occluded`: boards partly under others, `x` / `y` of
the visible part, `covered_by` = ids of the boards on top (fallback: touching boards). ~0.35 s per frame on CPU.

`destinations`: one per visible template tag (DICT_5X5_100, ids 1 / 2 / 16 = VARa / VARb / VARc), anywhere in the
image: `variant`, `tag_id`, `x`, `y`, `yaw` (target board pose, base_link, team offsets from
`feat/fr5-demo-integration` `config/perception_geometry.yaml`), `support_z` (template top, -0.055), `age` (s since
the tag was last seen; kept 10 s because small tags are missed in ~10 % of frames).

Board +X is the long edge for VARa and the short edge for VARb / VARc (team geometry handoff).

## Demo view: RViz with camera and pick panel

```bash
cd fr5-board-stacking && pixi run robot use_rviz:=false   # or: pixi run sim use_rviz:=false
cd vision1-overhead   && pixi run vision                  # add --ros-args -p model:=true for stacks
cd vision1-overhead   && robot/rviz.sh                    # builds the panel (robot env), opens RViz
```

RViz shows the robot and planning scene (attached board, obstacles), board markers (green = pickable, red = template
poses) and the **Pick** panel:

- camera image with detections (`/vision/image`, annotated, half size)
- one button per variant: free / covered count, template seen or not. Disabled (hover = reason) when there is no
  vision data, no calibration, no such board, no template, or a pick is running
- click a button **or a board in the camera image** = `pick_one.py --place --variant X` (+ `extra args`, saved with
  the RViz config, e.g. `--speed 0.2 --temp -0.25 0.10`). Its output goes to the log box; each ENTER prompt turns
  **Confirm** yellow. **Stop script** = Ctrl+C (a motion already sent still finishes: the e-stop is on the pendant)
- `/pick_panel/request` (`std_msgs/String`: `VARa` / `VARb` / `VARc` / `confirm` / `stop`) does the same from a
  script; `confirm` from the topic is refused unless the robot is MOCK. `/pick_panel/log`: every output line

`pick_one` refuses to start while the gripper reports vacuum on (a stopped run can leave a board on the cup):
take it off or `pixi run shell`, then `release`.

## Pick / place script (robot workspace)

From `fr5-board-stacking` (both repos side by side):

```bash
pixi run python3 ../vision1-overhead/robot/pick_one.py --hover-only   # real robot, first: check the position
pixi run python3 ../vision1-overhead/robot/pick_one.py                # pick, lift, put back (ENTER twice)
pixi run python3 ../vision1-overhead/robot/pick_one.py --place        # pick -> its template (--stack N, --press mm)
pixi run python3 ../vision1-overhead/robot/pick_one.py --place --variant VARa --temp X Y   # model: clear covering boards
```

`--press` (default 4 mm) goes below the computed board top. The real cup stopped ~10 mm high, but MoveIt's table
model refuses more than ~4 mm: measure where the cup touches the table and fix `table_top_z` / `tcp_z` in
`fr5-board-stacking`. If the requested `--variant` is in `occluded`, the board on top goes to the `--temp` spot
(stacked there, must be > 0.30 m from every template, default -0.25 0.10), vision looks again, repeat (max 4).

Simulation test (no robot, no camera): `pixi run sim` in `fr5-board-stacking`, then `robot/sim_regression.sh` here
(11 cases, ~10 min) and `robot/sim_clear_test.sh` (covered VARa: clear + place, model on). **Never run `sim` and `robot` at the same time**: a leftover sim process makes the real robot load
mock hardware (it then reports SUCCESS but does not move). `pick_one.py` prints `robot hardware: REAL FR5` or `MOCK` first, and stops if two robot stacks run.

## Limits

- Looks only at the pick area (right table in the image, `ROI` in `vision1/detect.py`).
- Classic (default): boards must not touch or overlap, merged blobs are rejected. Model: handles touching boards and
  stacks. Trained on auto-labels + synthetic pastes, then fine-tuned on 70 hand-reviewed real stack frames
  (`train/prelabel.py` SAM 3 pre-labels -> X-AnyLabeling review -> `train/add_real.py` -> `train/train.py`).
  10 held-out real stack frames (69 boards, `train/compare_real.py`): boards found classic 10 %, model 91 %
  (variant right 84 %, 14 false detections; at class confidence >= 0.6, the UNKWN cut, 39 / 40 named boards right).
  Held-out frames are all from one recording (dataset_2): check on the lab camera.
- `config/calib.yaml` is copied from `feat/vision-transform`. Re-copy it after every recalibration; the camera must not move.
- Lab knobs (thresholds, ROI, area limits) are at the top of `vision1/detect.py` and `vision1/table.py`.
