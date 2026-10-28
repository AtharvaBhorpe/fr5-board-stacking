# MoveIt batch demo verification — 2026-10-27

The user had no hardware access; all runs used localhost ROS domain 62, explicitly fake FR5 hardware,
mock gripper, actual MoveIt planning/execution, and the RViz task dispatcher. No real robot or camera
services were launched. Gemma QAT/llama.cpp supplied actual per-pick image checks; the normal GPU vision
node detected the changing replay image. There were no injected VLM approvals or board detections.

## Results

- **counted**: passed; 2 controller dispatches; 45.41 s. Evidence: `counted.json`.
- **mixed**: passed; 3 controller dispatches; 70.46 s. Evidence: `mixed.json`.
- **cancel**: passed; 1 controller dispatches; 3.70 s. Evidence: `cancel.json`.
- **unchanged**: passed; 1 controller dispatches; 31.68 s. Evidence: `unchanged.json`.
- **missing**: passed; 0 controller dispatches; 0.00 s. Evidence: `missing.json`.

96 unit tests passed and the RViz panel build passed. A second demo launch was refused when a robot
publisher already existed on domain 62. Cancellation evidence includes `cancel: controller returned a result`.
An initial cancellation trial safely refused dispatch because no fresh overhead image was available;
that non-test is preserved in `cancel-precheck-no-image.json`. The runner now waits for a fresh image
before sending a test command. The bridge's freshness threshold was not relaxed.

## Interactive controls

The running demo starts with two separate B boards. Use `Pick two variant B boards` or `Reset A/B/C`
followed by `Sort all visible boards`. The panel displays batch progress, has an always-available
`Cancel batch` button, and accepts `batch status`, `cancel batch`, and existing single-pick commands.
Scene resets require idle simulation and no active batch. The demo is left in normal (not frozen) mode.

## What this proves and does not prove

It exercises the actual VLM/vision/ROS/MoveIt software path and batch termination behavior. The image
fixture removes a board after a successful mock controller cycle; it does not model contact, real vacuum,
destination stack growth, wrist depth, or measured placement accuracy. It therefore does not establish
physical multi-board success, robust depth calibration, or learned manipulation. The earlier real-world
pick/drop height uncertainty remains pending. No end-to-end human microphone test was performed here.

Source images are recorded board textures reprojected into the calibrated region, explicitly labeled
SIMULATED BOARD SCENE. Source removals occur in image pixels and are subsequently detected by vision.
The fixture only operates on domain 62 and refuses real hardware identity.
