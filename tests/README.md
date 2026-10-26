# Tests (branch `feat/task-main-controller`)

Run from the repo root after `pixi run build` (TM-6: also builds the motion / vision interfaces from the neighbouring
worktrees). Rule TM-9: every part is tested on its own before integration; TM-4: keep runs short.
ROS tests: use a free domain (`ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=82`).

| Script | Needs | Checks | Time | Last result |
|---|---|---|---|---|
| `world_core_check.py` | nothing (simulated time) | geometry, variants + size check, tracking under noise, zones + margin, processing-zone state machine (CL-4 / CL-8), stacks + FULL, obstacles, stale vision, fixed fault zone, events | < 1 s | 42/42 |
| `fake_world_check.py` | nothing | camera model vs the real calibration (`calib.apply` as reference), bbox round trip, scenario -> pixels -> world, noise, input stack, simulated operator (same / non-variant board back), hands | ~1 s | 17/17 |
| `world_ros_check.py` | ROS | real wiring: `fake_world` -> `world_model` -> real `vision_transform` -> `/task/world`; scripted pick -> processing -> DONE -> drop-off; stale vision; clean Ctrl+C; bad parameter | ~8 s | 13/13 |
