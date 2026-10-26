# Tests (branch `feat/task-main-controller`)

Run from the repo root after `pixi run build` (TM-6: also builds the motion / vision interfaces from the neighbouring
worktrees). Rule TM-9: every part is tested on its own before integration; TM-4: keep runs short.
ROS tests: use a free domain (`ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=82`).

| Script | Needs | Checks | Time | Last result |
|---|---|---|---|---|
| `world_core_check.py` | nothing (simulated time) | geometry, variants + size check, tracking under noise, zones + margin, processing-zone state machine (CL-4 / CL-19), stacks + FULL, obstacles, stale vision, fixed fault zone, events, arm shadow (CL-20) | < 1 s | 48/48 |
| `fake_world_check.py` | nothing | camera model vs the real calibration (`calib.apply` as reference), bbox round trip, scenario -> pixels -> world, noise, input stack, simulated operator (same / non-variant board back), hands | ~1 s | 17/17 |
| `world_ros_check.py` | ROS | real wiring: `fake_world` -> `world_model` -> real `vision_transform` -> `/task/world`; scripted pick -> processing -> DONE -> drop-off; stale vision; clean Ctrl+C; bad parameter | ~8 s | 13/13 |
| `skill_math_check.py` | nothing | tcp yaw choice, pick / place heights, zone acceptance, arm shadow capsule, board lookup | < 1 s | 12/12 |
| `skills_ros_check.py` | ROS + the motion worktree (bringup started from there, headless) | every skill + failure code in the full sim stack (motion + vision + fake world with contact sim + world model + skills) | ~3 min | 25/25 |

Helpers: `HK_SPEED` (skills test move speed, default 1.0), `HK_TEST_LOG_DIR` (node logs of the skills test).
Rule TM-13: start nodes with their output going to a log file, never an unread `stdout=PIPE` (a full 64 KB pipe
freezes a chatty node such as move_group). `skills_ros_check.py` writes `skills_check_<node>.log` and, if a
skill reports 'motion stack hung?', a gdb backtrace of move_group (needs `kernel.yama.ptrace_scope=0`).
