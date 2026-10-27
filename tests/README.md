# Tests (branch `feat/task-main-controller`)

Run from the repo root after `pixi run build` (TM-6: also builds the motion / vision interfaces from the neighbouring
worktrees). Rule TM-9: every part is tested on its own before integration; TM-4: keep runs short.
ROS tests: use a free domain (`ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=82`).

| Script | Needs | Checks | Time | Last result |
|---|---|---|---|---|
| `world_core_check.py` | nothing (simulated time) | geometry, variants + size check, tracking under noise, zones + margin, processing-zone state machine (CL-4 / CL-19), stacks + FULL, obstacles, stale vision, fixed fault zone, events, robot mask (CL-31: geometry, held board ignored, arm explains absence only, any part counts, zones frozen vs. running), reset_zone removes the zone's boards (CL-36) | < 1 s | 57/57 |
| `fake_world_check.py` | nothing | camera model vs the real calibration (`calib.apply` as reference), bbox round trip, scenario -> pixels -> world, noise, input stack, simulated operator (same / non-variant board back), hands | ~1 s | 17/17 |
| `world_ros_check.py` | ROS | real wiring: `fake_world` -> `world_model` -> real `vision_transform` -> `/task/world`; scripted pick -> processing -> DONE -> drop-off; stale vision; clean Ctrl+C; bad parameter | ~8 s | 13/13 |
| `skill_math_check.py` | nothing | tcp yaw choice, pick / place heights, zone acceptance, board lookup (arm mask moved to `world_core_check`) | < 1 s | 10/10 |
| `main_core_check.py` | nothing (simulated clock, `main_sim.py`) | main controller core: the user's scheduling example (exact order), priority change, nearest board, every failure policy, pause / stop / cancel while holding, per-job pause, CL-23 / CL-24, claims, counters (CL-27), validation; invariants after every step | ~30 s | 47/47 |
| `main_core_regress_check.py` | nothing (`main_sim.py`) | one scenario per review bug (TM-16: D3..R6, A1..A9, CL-28..CL-36), each built to fail on the core before its fix (`MAIN_CORE=<old main_core.py>`); invariants | ~40 s | 34/34 |
| `main_core_fuzz_check.py` | nothing (`main_sim.py`) | seeded fuzz: random layouts (CL-34), jobs, failure codes, commands, flicker, camera dropouts, world lag, disturbances while holding; an operator who reacts only to alerts / PAUSED / HALTED; every seed must keep the invariants and settle (`--seeds N`, `--seed S -v` for one trace) | ~2.5 min (20 seeds) | 40/40 seeds |
| `skills_ros_check.py` | ROS + the motion worktree (bringup started from there, headless) | every skill + failure code in the full sim stack (motion + vision + fake world with contact sim + world model + skills) | ~3 min | 26/26 |

`main_sim.py` (shared simulation, not a test): real `world_core` + `fake_core` on a simulated clock; skills refuse like
`skill_server`; robot mask + occlusion; held board seen by the camera; checker: invariants, loop + progress detector.

Helpers: `HK_SPEED` (skills test move speed, default 1.0), `HK_TEST_LOG_DIR` (node logs of the skills test).
Rule TM-13: start nodes with their output going to a log file, never an unread `stdout=PIPE` (a full 64 KB pipe
freezes a chatty node such as move_group). `skills_ros_check.py` writes `skills_check_<node>.log` and, if a
skill reports 'motion stack hung?', a gdb backtrace of move_group (needs `kernel.yama.ptrace_scope=0`).
