# Tests (branch `feat/task-main-controller`)

Run from the repo root after `pixi run build` (TM-6: also builds the motion / vision interfaces from the neighbouring
worktrees). Rule TM-9: every part is tested on its own before integration; TM-4: keep runs short.
ROS tests: use a free domain (`ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=82`).

| Script | Needs | Checks | Time | Last result |
|---|---|---|---|---|
| `world_core_check.py` | nothing (simulated time) | geometry, variants + size check, tracking under noise, zones + margin, processing-zone state machine (CL-4 / CL-19), stacks + FULL, obstacles, stale vision, fixed fault zone, events, robot mask (CL-31: geometry, held board ignored, arm explains absence only, any part counts, zones frozen vs. running), reset_zone removes the zone's boards (CL-36), fault zone = void + fallback position (CL-39) | < 1 s | 86/86 |
| `fake_world_check.py` | nothing | demo layout rules (calibrated area, reach, arm at home, profile, zone / input gaps), scenario -> Vision 1 JSON frame -> `vision1.parse` -> world, 1 mm noise, input stack, place on a sheet, simulated operator (same / UNKWN), hands as `obstacles[]`, contact-sim surfaces on sheets, a board lost in flight lands under the tcp / is gone (TM-38); real `world.yaml` | ~1 s | 29/29 |
| `inject_check.py` | ROS (no robot, no motion) | `tools/inject_vacuum_loss.py` against a stand-in gripper node + local XML-RPC server: refused unless HOLDING, real order vacuum OFF -> blow-off ON -> OFF (also after a controller error), default 1 s, sim path (TM-38) | ~10 s | 6/6 |
| `demo_ui_check.py` | nothing (no ROS) | `tools/vacuum_demo_ui.py` with a temp bridge dir: page + status, Now sends at once, Arm fires DELAY s into a carry (not before / not without a carry / not after a cancel), bridge not running -> message (TM-47) | ~5 s | 7/7 |
| `world_ros_check.py` | ROS | real wiring: `fake_world` (`/vision/boards` JSON) -> `world_model` -> `/task/world`; zone `base_z` from `top_z`; scripted pick -> processing -> DONE -> drop-off sheet; a zone without `size` -> loud ERROR; stale vision; clean Ctrl+C | ~8 s | 13/13 |
| `skill_math_check.py` | nothing | tcp yaw choice, pick / place heights, zone acceptance, board lookup, stack box from the world (CL-38), fault zone rules (CL-39) | < 1 s | 17/17 |
| `main_core_check.py` | nothing (simulated clock, `main_sim.py`) | main controller core: the user's scheduling example (exact order), priority change, nearest board, every failure policy, pause / stop / cancel while holding, per-job pause, CL-23 / CL-24, claims, counters (CL-27), validation; invariants after every step | ~30 s | 49/49 |
| `main_core_regress_check.py` | nothing (`main_sim.py`) | one scenario per review bug (TM-16: D3..R6, A1..A9, CL-28..CL-36), each built to fail on the core before its fix (`MAIN_CORE=<old main_core.py>`); invariants, TM-36 no home / hidden boards / deferred verify, TM-39 continuous jobs (count -1), TM-40 busy processing zone / park at home / fault no verify / re-pick the fallen board | ~40 s | 50/50 |
| `main_core_fuzz_check.py` | nothing (`main_sim.py`) | seeded fuzz: random layouts (CL-34), jobs, failure codes, commands, flicker, camera dropouts, world lag, disturbances while holding; an operator who reacts only to alerts / PAUSED / HALTED; every seed must keep the invariants and settle (`--seeds N`, `--seed S -v` for one trace) | ~2.5 min (20 seeds) | 21/21 |
| `vision1_check.py` | nothing | Vision 1 JSON adapter (V3): boards / zones -> detections, bad entries skipped, frames refused, real world.yaml effect (base_z, zone boards, UNKWN), UNKWN candidate rule | ~1 s | 20/20 |
| `main_node_check.py` | nothing (messages only) | controller node, pure parts: World.msg -> core dict (the core decides the same as on the snapshot), Job.msg <-> core job, SkillCode names, result rule (status + success + code agree), skill goals, config | ~2 s | 14/14 |
| `main_node_ros_check.py` | ROS (in-process fakes, no motion stack) | the controller node against scripted fake skill servers / get_state / world: timeouts, rejected / late-accepted goals, default-filled results, stop during a skill and before its goal is accepted, loop exception -> HALT, world outage, vanished skill server, tcp + status rate, status size, service exceptions, second controller, Ctrl+C and SIGTERM (separate process), Ctrl+C while a goal's accept is on its way | ~50 s | 22/22 |
| `main_ros_check.py` | ROS + the motion worktree (headless) | the controller in the FULL sim stack via its API only: the user's scheduling example (exact order, counters, stacks), no vacuum -> skip, hand -> wait_clear, pause / stop / cancel while holding, invalid job, list_jobs, Ctrl+C during a skill, vacuum lost mid-carry -> lost 1, re-picked, delivered 1 (TM-38) | ~4 min | 18/18 |
| `skills_ros_check.py` | ROS + the motion worktree (bringup started from there, headless) | every skill + failure code in the full sim stack (motion + vision + fake world with contact sim + world model + skills), incl. picking off a drop-off stack and placing back (scene box removed, S-11), vacuum loss via the inject tool -> the fallen board is a new input board after home (TM-38) | ~3 min | 30/30 |

`main_sim.py` (shared simulation, not a test): real `world_core` + `fake_core` on a simulated clock; skills refuse like
`skill_server`; robot mask + occlusion; held board seen by the camera; checker: invariants, loop + progress detector.

Helpers: `HK_SPEED` (skills test move speed, default 1.0), `HK_TEST_LOG_DIR` (node logs of the skills test).
Rule TM-13: start nodes with their output going to a log file, never an unread `stdout=PIPE` (a full 64 KB pipe
freezes a chatty node such as move_group). `skills_ros_check.py` writes `skills_check_<node>.log` and, if a
skill reports 'motion stack hung?', a gdb backtrace of move_group (needs `kernel.yama.ptrace_scope=0`).

Timers: waits in the tests follow `world.yaml` (gone / back / unknown_after) and the scenario's operator times (CL-40),
so tuning them at the lab does not break the tests.

Variants: the core logic tests (`world_core`, `main_core*`, `main_sim`) use the PLACEHOLDER variants of
`tests/test_variants.yaml` (white_small / white_large / red), merged over `world.yaml`, which holds the real Vision 1
variants VARa / VARb / VARc (TM-20). `fake_world_check`, `vision1_check` and the ROS / sim-stack tests run the real
variants and `scenario_demo.yaml` (V5, TM-24). Vision frames: ~5 Hz everywhere (fake_world `rate` 5, main_sim step 0.2 s).
