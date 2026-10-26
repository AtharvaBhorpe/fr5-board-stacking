# Tests (branch `feat/vision-transform`)

Run from the repo root after `pixi run build`. None needs the robot or the camera. ROS tests: use a free
`ROS_DOMAIN_ID` (e.g. `ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=78`) so they never talk to a running system.

| Script | Needs | Checks | Last result |
|---|---|---|---|
| `calib_check.py` | nothing (offline) | fit / apply / inside / lens term (distorted synthetic camera) / suspect tag rates / rejections / save + load through a symlink / lab homography | 46/46 |
| `vision_node_check.py` | ROS | `vision_transform` node: placeholder, live reload, values = `calib.apply`, rejections, broken / deleted file, bad parameter, latency, Ctrl+C | 17/17 |
| `robot_pose_check.py` | nothing | own FK vs our URDF TF (fixture), packet parser (framing, resync, checksum, plausibility), reader vs a fake controller (read-only, stale, reconnect), tilt / flange cross-check | 21/21 |
| `calib_app_check.py` | nothing | calibration app API: reference tags, records (robot / pendant / manual), limits, fit, moved tags, verification (any moving tag), save, session restore, stream | 38/38 |
| `web_ui_check.py` | firefox | the real page in headless Firefox clicks through the whole workflow (`web/selftest.js`), fails on any browser error; screenshots to `~/snap/firefox/common/hk_ui_shots/` | 21/21 |

Helpers:
- `synth_table.py`: synthetic camera image with tags at given base_link x / y (lab homography). `pixi run python3
  tests/synth_table.py /path/table.png` then `pixi run calib --image /path/table.png --no-robot` = the app without camera.
- `fake_controller.py`: fake port-20004 state stream. `pixi run python3 tests/fake_controller.py 20004` + `robot_ip:
  127.0.0.1` in a copy of `robot.yaml` = the app / probe without robot.
- `make_fk_reference.py`: one-off, run in the motion branch env, writes `fixtures/fk_reference.json` (TF of our URDF).
