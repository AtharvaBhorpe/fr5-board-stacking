# FR5 board stacking: vision transform (branch `feat/vision-transform`)

Turns a **camera pixel** (fixed camera above the table, Logitech C920) into a **table point x / y in Fairino
`base_link`**, so a control loop (script, state machine or LLM / VLM agent) can send the robot to what the camera sees.

- **`vision_transform` node:** service `/vision/pixel_to_table`. Uses only the stored calibration
  (`config/calib.yaml`): no camera, no tags, no robot needed at runtime. Reloads the file when it changes.
- **Calibration app** (web page): tags on the table, hand-guide the robot's suction cup onto each tag, record, fit,
  verify, save. Read-only towards the robot (never moves it).

Model: one radial lens term + homography (table plane), fitted from 8 tags. Sim: ~0.5-1 mm mean error inside the tag
area (decision-log V-17 / V-18). Real accuracy = the verification result stored in `calib.yaml`.

## Quick start

```bash
git clone --branch feat/vision-transform https://github.com/AtharvaBhorpe/fr5-board-stacking.git
cd fr5-board-stacking
pixi install          # ROS 2 Humble (RoboStack); needs https://pixi.sh
pixi run build        # first build warns "Could not find activation scripts ... install/setup.sh": expected
pixi run vision       # the node: /vision/pixel_to_table (answers success=false until calibrated)
pixi run calib        # calibration app -> http://127.0.0.1:8080
pixi run probe 20     # read-only robot check (port 20004), 20 s
```

## ROS API

| Name | Type | Use |
|---|---|---|
| `/vision/pixel_to_table` | service `hk_fr5_vision_interfaces/srv/PixelToTable` | pixels -> table x / y |

```
float64[] u           # pixel column (from the left), sub-pixel ok, same image size as the calibration
float64[] v           # pixel row (from the top)
---
bool success          # false -> no valid calibration or bad request; message says why, arrays empty
string message
float64[] x           # base_link metres, same order as u / v
float64[] y
bool[] inside         # false = outside the calibrated tag area (still returned, less accurate)
```

```bash
pixi run ros2 service call /vision/pixel_to_table hk_fr5_vision_interfaces/srv/PixelToTable "{u: [960.0], v: [540.0]}"
```

```python
from hk_fr5_vision_interfaces.srv import PixelToTable
cli = node.create_client(PixelToTable, "/vision/pixel_to_table")
res = await cli.call_async(PixelToTable.Request(u=[812.4, 901.0], v=[440.2, 455.9]))   # e.g. board corners
if res.success: x, y = res.x, res.y      # metres in base_link; z = table top (-0.060) + board height
```

Rules for callers:
1. Pixels must come from **the same camera, pose and resolution** as the calibration (`image_size` in `calib.yaml`).
   If the camera moves, recalibrate.
2. The result lies on the **table plane**. A board's top surface is a few mm higher: for 6 mm boards the error is
   negligible (camera ~0.8 m high); for tall stacks pick the board's centre pixel, not its edge.
3. `inside = false`: the point is outside the tags' outline. Fine near it (< 10 cm: ~1-2 mm extra), poor far away
   (> 20 cm: up to a few cm). Calibrate with the tags covering the pick / place area.
4. `success = false`: not calibrated, broken file, or a bad request. Never guess: stop and report.

## Calibration procedure (lab, ~20-25 min)

The calibration maps camera pixels to table x / y. You hand-guide the robot's suction cup onto 8 tags; the app reads
the robot pose (read-only), fits, then you verify with a 9th tag and save. **The app never moves the robot.**

### A. Before the lab (once)

1. Print **9 ArUco tags**, dictionary `DICT_5X5_50`, any ids (e.g. 0-8), each about 60-80 mm, flat on stiff card.
   The exact size does not matter (only the tag centre is used). PNGs:
   `pixi run python3 -c "import cv2; d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50); [cv2.imwrite(f'tag{i}.png', cv2.copyMakeBorder(cv2.aruco.generateImageMarker(d,i,600),60,60,60,60,cv2.BORDER_CONSTANT,value=255)) for i in range(9)]"`
2. `git pull`, `pixi install`, `pixi run build`.

### B. Set up (5 min)

1. **Camera** fixed above the table, the whole pick / place area in view, **autofocus off**. From now on do not touch
   or move it (that would invalidate the calibration).
2. **Robot** powered and enabled, the gripper mounted as in operation (cup = tcp), pumps off.
3. **Our motion bringup must NOT run** (no `pixi run robot` / `sim`): after hand-guiding, the controller could jump
   back to its last commanded position.
4. Fairino web app: **manual mode**, drag (hand-guide) enabled. **A person at the e-stop.**
5. Laptop on the robot network (192.168.58.x).

### C. Robot link check (1 min, read-only)

`pixi run probe 20`, hand-guide the arm a little while it prints:

| Field | Expected | If not |
|---|---|---|
| `conn` / `fresh` | `True` / `True`, ~100 Hz | network / IP (`config/robot.yaml`), robot on? |
| `mode` | `manual` | switch the web app to manual mode |
| `flange check` | **< 5 mm** | **stop**: packet layout or frame wrong, report it (lab item VT1) |
| `cs_ok` | counting up | `cs_bad` counting instead = checksum convention differs; fine if the flange check is good |
| `tilt` | ~0 deg with the tool pointing straight down | |

### D. Start the app (1 min)

1. `pixi run calib`. The camera is found by name ("C920"); if not: `pixi run calib --list-cameras`, then
   `pixi run calib --camera N`.
2. Open **http://127.0.0.1:8080**. Header: camera pill green (1920×1080), robot pill green "manual".

### E. Place and identify the reference tags (3 min)

1. Lay **8 tags** flat over the **whole pick / place area**: 4 near its corners + 4 inside (2 in the middle,
   2 at edge middles). Fully visible, not overlapping, **taped down** (they must not move until you saved).
   No other tag in view.
2. Wait until the hint says "8 stable tag(s) in view".
3. Click **Identify reference tags** → 8 rows, step bar moves to *Record*.

### F. Record every tag 2-3× (8-10 min)

Record source: **Robot** (default). For each tag:

1. Hand-guide the tool **pointing straight down** (dock: tilt < 2°) and lower the cup until it **just touches the
   tag, centred on the tag's centre** (check by eye from above and from two sides).
2. The dock shows **cup on: tag N** (the reference tag the camera just lost) and tcp z ≈ -60 mm.
3. Click **Record tag N** (big button). If the arm hides other tags too ("several hidden"), use that tag's row
   **Record** button instead.
4. Lift ~3 cm, come back **from another direction**, re-centre, record again. A 3rd record is optional.
5. The row shows ●● and the **spread** (records apart). Spread > 2 mm (yellow): click the row, delete the worse
   record (✕), record again.

Warnings: *cup not on the table* = z far from -60 mm; *tilted* = re-align (FK still gives the true cup position);
*not in manual mode*; *tag moved* (row) = the tag was bumped: delete its records and record it again.

### G. Fit and review (1 min)

Click **Fit**.

| Shown | Good | Action otherwise |
|---|---|---|
| rms | ≤ 1 mm (≤ 2 acceptable) | check the spreads, re-record the worst tags |
| suspect tag | none | delete that tag's records, record it again, Fit again |
| tags | 8 | a skipped tag (warning list) was not seen / moved: fix and Fit again |
| lens effect / k | information only | large value = the lens term is doing real work |

Fit errors per tag are shown in the rows and as rings on the image (green < 1 mm, yellow < 2, red).

### H. Verify with a 9th tag (5 min)

1. Put the **9th tag** (any id that is not one of the 8) at a spot **inside** the area, not on a reference tag.
   It turns orange (*moving*) in the image.
2. Hand-guide the cup onto it as in F; the dock shows **cup on: spot · id**. Click **Record spot**.
3. Move the tag to the next spot (the app notices the move) and repeat: **5-6 spots** spread over the area
   (centre, near the corners and edges, the pick and place zones).
4. Result **PASSED** = mean ≤ 2 mm and max ≤ 4 mm (5+ spots). If not:
   - *errors point the same way*: systematic placement offset (always off-centre on one side) or a moved camera;
   - *largest errors outside the tag area*: move / add reference tags to cover that area, recalibrate;
   - one bad spot: delete it (✕) and record it again;
   - *is the cup on tag N?*: wrong tag used (e.g. a tag just removed); delete the spot.

### I. Save and commit (2 min)

1. Click **Save calibration** → `src/hk_fr5_vision/config/calib.yaml` (previous file kept as `calib.yaml.bak`).
   Header: "saved … · verified". A running `vision_transform` node reloads it within 1 s.
2. Optional check: click a few points in the image (*Test a point*), e.g. board corners.
3. Stop the app (Ctrl+C): the camera is then free for the board detection.
4. **Commit and push `calib.yaml`** (it contains the verification result) so everyone uses the same calibration.
5. The tags can be removed: the node needs only `calib.yaml`. **The camera must stay where it is.**

### J. Recalibrate / re-check

- **Recalibrate** when the camera was moved, bumped or refocused, the resolution changed, or the table moved
  relative to the robot.
- **Quick re-check** (5 min, no new fit): start the app, put the 9th tag at 3-5 spots, Verify uses the saved
  calibration; Save writes the new verification result into the file.
- **Start over** (bottom of the page) clears tags, records and spots; the saved file stays until you save again.

### Troubleshooting

| Symptom | Cause / fix |
|---|---|
| camera pill red "no camera named 'C920'" | not plugged / other name: `pixi run calib --list-cameras`, `--camera N` |
| robot pill red | network, robot off, or the motion bringup still running (stop it) |
| a tag never gets *stable* | glare, tag bent / too small in the image, motion: flatten it, change the light |
| unknown tag id appears (ghost) | add the printed ids to `tags.allowed_ids` in `config/calib_app.yaml` |
| "cup on" shows nothing while the cup is on a tag | the tag is still partly visible: lower / centre the cup, or use the row Record |
| record refused "not seen long enough" | the camera has not seen that tag for 10 frames yet: lift the cup, wait, retry |

Robot pose source: joints from the controller state port 20004 + our own forward kinematics (same chain as the
motion branch URDF, flange -> cup tip 112.9 mm). Alternatives in the page: *Pendant* (web-app x / y in mm; negated
automatically; flange only, so the tool must point straight down) or *Manual* (base_link metres).
First lab check of the robot link: `pixi run probe 20` while hand-guiding: `fresh True`, `mode manual`, **flange
check < 5 mm** (ours vs the controller's own flange pose; large = wrong packet layout / frame).

## Files

| Path | What |
|---|---|
| `src/hk_fr5_vision_interfaces/srv/PixelToTable.srv` | the service |
| `src/hk_fr5_vision/hk_fr5_vision/vision_transform.py` | the node |
| `src/hk_fr5_vision/hk_fr5_vision/calib.py` | fit / apply / save / load (no ROS) |
| `src/hk_fr5_vision/hk_fr5_vision/robot_pose.py` | port 20004 reader (read-only) + FK + `robot_pose_probe` |
| `src/hk_fr5_vision/hk_fr5_vision/calib_app.py`, `web/` | calibration app (aiohttp + one page) |
| `src/hk_fr5_vision/config/` | `calib.yaml` (THE calibration), `vision.yaml` (node), `calib_app.yaml` (app: camera, tags, limits), `robot.yaml` (robot IP, FK chain) |
| `src/hk_fr5_vision/launch/vision.launch.py` | node launch (`calib_file:=`, `reload_period:=`) |
| `tests/` | offline + browser tests, see `tests/README.md` |
| `notes/decision-log.md` | every decision with numbers (V-*), open lab items (VT*) |

## Limitations

- One camera, one pose: moving or refocusing the camera invalidates the calibration (recalibrate, ~15 min).
- Table plane only (no height): fine for flat boards, not for 3D objects.
- The port 20004 packet layout comes from the vendor SDK header (v3.9.0.1); confirm on the real controller with
  `pixi run probe` (VT1) before the first calibration.
