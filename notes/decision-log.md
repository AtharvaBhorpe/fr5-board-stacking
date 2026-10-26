# Decision log: branch `feat/vision-transform`

Pixel (camera above the table) -> x / y in Fairino `base_link`, for the control loop. Replaces the scratch tool
`pix2table` (not in git; lab test with 4 ArUco tags: 5-7 mm). Robot facts (frames, tool geometry, gripper) live in the
decision log of branch `feat/robot-motion-gripper`. Units: metres / radians in code, mm only in logs.

## 1. Decisions

| # | Topic | Decision | Why / options |
|---|---|---|---|
| V-1 | Branch | Orphan branch `feat/vision-transform`, worktree `~/hackathon/wt/feat-vision-transform/` | Same pattern as the other sub-projects; merged later with `--allow-unrelated-histories` |
| V-2 | Interface | **Service** `/vision/pixel_to_table` (several points per call) | One question, one answer, pure maths < 1 ms. Topic pair: caller must match replies to requests, no reply guarantee. Action: goal / feedback / cancel unused. User: "u decide" |
| V-3 | Names | Package `hk_fr5_vision` (node `vision_transform`, calibration app), interfaces in own package `hk_fr5_vision_interfaces` | Own interfaces keep the branch independent (vs `hk_fr5_interfaces` on the motion branch = cross-branch dependency until the merge) |
| V-4 | Environment | Own pixi env: `pixi.toml` / `pixi.lock` copied from the motion branch, same dependency list; `build` task = our 2 packages only; no Fairino driver path | Install from the cache 1.35 s, no extra disk (tested). Sharing the motion env needs a wrapper (activation sources the motion `install/`), new deps would land on the motion branch. Merge later: keep the union |
| V-5 | Explicit deps | `py-opencv 5.0.0`, `aiohttp 3.14.5`, `numpy 2.5.4`, `pyyaml 6.0.3` pinned (`pixi add`) | They were only transitive (via `ros-humble-desktop`); lock package list unchanged |
| V-6 | Method | Homography table plane <-> image from ArUco tag centres (`DICT_5X5_50`, fixed tag IDs only), least squares with 4-6 tags (2 in the middle for accuracy), C920 on USB | Proven in pix2table. **Focus is fixed** (user: autofocus off) -> no focus handling. Lens undistortion only if the accuracy is not enough |
| V-7 | Calibration storage | `hk_fr5_vision/config/calib.yaml`: homography, image size, tags (pixel + world), error per tag, RMS, camera name, date. Previous file kept as backup. **Committed after a good calibration** (user) | The node needs no camera and no tags after calibration; teammates get the same values |
| V-8 | Runtime node | `vision_transform` loads `calib.yaml` at start, reloads it when the file changes; no valid calibration -> `success = false`. Reply flags points outside the calibrated tag area (extrapolated) | Recalibrate without restarting the project |
| V-9 | Record current tcp | Calibration app: button per tag row fills x / y from the real robot: read-only client on the controller state port 20004 (robot in manual mode, hand-guided onto the tag). tcp = flange pose + R * (0, 0, 0.1129); pendant frame -> `base_link` = x, y negated. Warning if the tool tilts > ~2 deg. Fallbacks: pendant values typed in (x / y negated), direct entry | User workflow: open app, connect camera + robot, hand-guide tcp to a tag, record, all tags, fit, save. **Our bringup must not run while hand-guiding** (controller can jump back to the last commanded position) |
| V-10 | Camera ownership | Only the calibration app opens the camera; close it before the board detection runs | Two processes cannot open the C920 at once |
| V-11 | Calibration app | aiohttp + one HTML / JS / CSS page, no framework, no build step; polished UI (light / dark, overlay drawn in the browser), wizard (tag status, record, fit, per-tag error, save), test mode (pixel -> x / y, optional go-to plan-only, drift check), camera reconnect, runs without ROS | User: robust and beautiful; features reviewed as we go |
| V-12 | TCP for record | **Joints from port 20004 (`jt_cur_pos`, deg) + own forward kinematics (numpy)**: FR5 joint chain copied from the vendor URDF + our tool (flange -> cup tip 0.1129 m along flange z, no x / y offset), numbers in config with their source. `flange_cur_pos` from the same packet only as a cross-check (verifies the packet layout + pendant frame, VT1) | The Fairino web app / controller pose stops at the flange (no tool set). motion_shell gets the tcp via TF (robot_state_publisher FK on /joint_states with our URDF), which needs the bringup = not allowed while hand-guiding. Options: A flange pose + 112.9 mm (needs the pendant RPY convention for tilt, VT2), **B own FK (user)**: same kinematics as MoveIt (model errors cancel), result directly in base_link, testable offline vs the motion sim TF, C tool on the pendant (changes shared controller config) |

## 2. Open items

| # | Item | Status |
|---|---|---|
| VT1 | Port 20004 packet layout (v3.9.2) on the real controller: read-only test; `robot_mode`, flange pose offsets | open (lab) |
| VT2 | Pendant RPY convention for the tilt correction (same question as T12 on the motion branch) | open (lab) |
| VT3 | Calibration with 6 tags by hand-guiding, accuracy check with a moving tag, commit `calib.yaml` | open (lab) |

## 3. Step plan

| # | Step | Status |
|---|---|---|
| V1 | Branch + worktree, `.gitignore`, pixi env, this log | done `7ed958b` |
| V2 | `hk_fr5_vision_interfaces`: `PixelToTable.srv` (u[] / v[] -> success, message, x[] / y[], inside[]) | done |
| V3 | `calib.yaml` format + homography fit / apply + offline test | |
| V4 | `vision_transform` node: service, reload on change, test | |
| V5 | Port 20004 reader + FK (V-12) + offline tests: fake server, FK vs motion sim TF | |
| V6 | Calibration app backend: camera, tags, record tcp, fit, save | |
| V7 | Calibration app front end | |
| V8 | Launch file, README | |
