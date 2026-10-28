# FR5 board stacking: main controller (branch `feat/task-main-controller`)

The "brain" of the board-stacking cell: a **world model** built from the overhead camera (Vision 1), **skills**
(pick, place, home, ...) on top of the motion + gripper stack, and a **job scheduler** that turns jobs like
"process every VARa via processing_1, then stack it on drop_off_a" into robot actions, with failure handling.
Tested in simulation (MoveIt fake arm) and on the real FR5 (trial 1, 2026-10-27: 13 boards moved, none lost).

```
Vision 1 (overhead C920) --/vision/boards JSON--> world_model --/task/world--> task_main_controller --skills--> skill_server
                                                     ^  /task/events (picked / placed)                             |
                                                     +-------------------------------------------------------------+
skill_server --/motion/*, /gripper/*, /scene/*--> motion + gripper stack (branch feat/robot-motion-gripper)
```

## Layout and setup

All branches of the team repo `fr5-board-stacking` side by side as git worktrees (the build and the tools find their
neighbours by these folder names; override with env vars, see below):

```
<ws>/wt/feat-task-main-controller   this branch
<ws>/wt/feat-robot-motion-gripper   motion + gripper + scene (FROZEN, tested on the real robot)
<ws>/wt/feat-vision-transform       camera calibration (calib.yaml; build dependency of this branch)
<ws>/wt/feat-vision1-overhead       Vision 1 (overhead detector) + Vision 2 (wrist D405), the teammate's branch
<ws>/live_logs                      log sessions of real / sim runs (created by the tools)
```

```bash
git clone https://github.com/AtharvaBhorpe/fr5-board-stacking.git <ws>/repo && cd <ws>/repo
git worktree add ../wt/feat-task-main-controller feat/task-main-controller
git worktree add ../wt/feat-robot-motion-gripper feat/robot-motion-gripper
git worktree add ../wt/feat-vision-transform     feat/vision-transform
git worktree add ../wt/feat-vision1-overhead     feat/vision1-overhead
cd <ws>/wt/feat-robot-motion-gripper && pixi install && pixi run build     # see that branch's README
cd <ws>/wt/feat-vision1-overhead     && pixi install -e gpu                 # NVIDIA GPU (CPU: pixi install)
cd <ws>/wt/feat-task-main-controller && pixi install && pixi run build     # builds also the motion / vision interfaces
pixi global install tmux                                                    # for tools/trial_up.sh
```

Needs: Ubuntu 24.04, pixi, git; NVIDIA GPU + driver for Vision 1 model mode (CUDA >= 12.8 driver).

## Run

One command starts everything in a tmux session `trial`, in order, waiting for each part (nothing moves at start):

```bash
tools/trial_up.sh --sim            # simulation: fake arm + fake gripper + fake camera (scenario_demo.yaml)
tools/trial_up.sh                  # REAL robot (AUTO mode, a human at the e-stop) + Vision 1 on the GPU; debug logging
tools/trial_up.sh demo             # REAL robot, demo logging (normal logs, lite monitor)
tools/trial_down.sh                # stop everything cleanly, in the right order
tools/trial_attach.sh              # attach a terminal (Ctrl+b <n> = window n, Ctrl+b d = detach, keeps running)
```

Windows: 0 monitor, 1 motion, 2 vision (or fake), 3 world, 4 skills, 5 ctrl, 6 cli (`tc ...`), 7 bridge, 8 demo-ui.
**Vacuum-failure demo page:** http://localhost:8765, opened in the browser by `trial_up.sh` (not with `--no-gui`) and
written to `<session>/demo_ui_url.txt` (`tools/vacuum_demo_ui.py`, `--lan` for a phone on the same network): *Simulate vacuum failure* arms it (fires 1.5 s into the next carry), *Now* fires at once while holding; *Pause* / *Resume* / *Stop* = `tc pause` / `tc resume` / `tc stop` (whole controller; Stop is a smooth software stop,
not an e-stop).
Options: `--rviz`, `--no-gui`. Env: `ROS_DOMAIN_ID`, `HK_MOTION_WT`, `HK_VISION1_WT`, `HK_LIVE_ROOT`, `HK_TMUX_SESSION`.

Commands (window 6, or `pixi run python3 tools/task_cli.py ...`; `help` lists all):

| Command | What |
|---|---|
| `tc status` / `tc world` / `tc list` / `tc wait j3` | controller state, boards + zones, jobs, follow a job |
| `tc transfer VARa drop_off_a 2` | 2 input VARa boards -> drop_off_a (no count = 1, `0` = all, `-1` = continuous until cancelled, `any` = any variant; `fault` is a valid target) |
| `tc process VARa processing_1 drop_off_a 1` | input board -> processing zone; after the operator processed it -> drop_off_a |
| `tc board b7 drop_off_b` / `tc clear drop_off_a drop_off_b` | one board by id / move boards of a zone |
| `tc pause` `tc resume` `tc stop` `tc home` `tc cancel j3` `tc prio j3 10` | control (stop holds a board, resume drops it into the fault zone) |
| `tc reset_zone drop_off_a` | the operator emptied a zone by hand (world only) |
| `tc inject vacuum_loss [SECS]` | demo / test: while holding, vacuum off + blow-off SECS (default 1 s) -> the board drops, part-lost recovery runs (REAL robot: switches the gripper, only with a person at the e-stop) |

Fast interface for scripts / an agent (no process start per call): `<session>/bridge/state.txt` (compact state,
rewritten on every change), `bridge/events.txt` (results, job changes, alerts), commands into the pipe:
`echo "transfer VARa drop_off_a 1" > <session>/bridge/cmd` (`tools/agent_bridge.py`).

## ROS API

| Name | Type | Notes |
|---|---|---|
| `/task/add_job` | srv `hk_fr5_task_interfaces/AddJob` | validate + queue a `Job` (type transfer / process / clear_zone / home) |
| `/task/list_jobs` | srv `ListJobs` | queue + finished jobs with counters (delivered, to_fault, lost, skipped) |
| `/task/control` | srv `Control` | pause / resume / stop / home / cancel_job / set_priority / clear_queue |
| `/task/status` | `std_msgs/String` JSON, latched | controller state, current skill + phase, jobs, alerts, readiness |
| `/task/world` | `hk_fr5_task_interfaces/World`, latched | boards (id, variant, location input / zone / held, pose, top_z), zones (state, stack count, heights), obstacles |
| `/task/events` | `std_msgs/String` JSON | picked / placed / dropped / reset_zone (skills -> world model) |
| `/task/skills/{pick,place,home,wait_clear,release_safe}` | actions | used by the controller; debug: `tc skill ...` (only while paused) |
| input `/vision/boards` | `std_msgs/String` JSON | Vision 1 contract: `notes/vision1-changes.md` (boards `corners_xy`, zones with `size` + `top_z`) |

## Behaviour in one page

- **World:** boards are tracked by position (stable ids), located `input` (table, no zone), `zone` (inside a zone
  outline + 3 cm) or `held`. Jobs pick only `input` boards (never re-pick a placed one). Input boards may be added or
  taken away by hand at any time; one hidden by the arm keeps its id when seen again (TM-43). Zones come live from the Vision 1 tags (they can move). UNKWN detections inside zones
  are ignored (hands); a board the robot placed keeps its known variant. Drop-off stack height = sheet top + count x
  board thickness (the camera cannot count a stack).
- **Processing zone:** EMPTY -> LOADED (robot placed) -> PROCESSING (operator took it, 2 s) -> DONE (a board back, 2 s)
  -> picked -> EMPTY. A board put by hand into an empty processing zone = FAULT, clears itself when removed. A busy processing zone
  holds one board: every detection in it is that board (duplicate detections ignored, TM-42). **Operator rule:** take the board only after the arm has left the zone and keep it away >= 2 s (the camera
  must see the zone empty; while the arm covers it the zone state is frozen).
- **Pick:** hover, then a slow contact search until the vacuum seals (input stacks: height unknown) or a known height
  (zone boards). **Place:** 1 mm into the stack top, vacuum off + 2 s blow-off, the retract starts 0.5 s into it.
  If another board is queued (by priority) the next pick starts right there; otherwise home (park pose above the
  robot's aluminium profile, out of the camera view) and a camera check. Boards hidden by the arm are remembered by the
  world and count as pickable (priority is never changed by occlusion); a place not followed by home is confirmed by
  the camera later (warning only if not).
- **Failures:** grip failed -> the board is skipped (problem board), vision stale -> wait, obstacle -> wait_clear,
  board lost -> counted (a board that fell on the table is a new input board; its job picks the board nearest to the loss
  point next if it still needs one), nothing to do with the arm away from home -> parks at home, 3 motion failures in a row or a failed release -> HALTED (check, then `tc resume`). A board
  that cannot go anywhere is dropped into the **fault zone** (fixed spot behind the base, a void).

## Configuration (`src/hk_fr5_task/config/`)

| File | Main values |
|---|---|
| `world.yaml` | variants VARa 0.295 x 0.210, VARb 0.210 x 0.150, VARc 0.210 x 0.135 (5 mm), table_z -0.060, zone margin, timers (gone / back / unknown_after 2 s, stale_after 1.5 s, lost_after 2 s), **fault zone position**, zone_unknown_boards |
| `skills.yaml` | speeds (free moves 0.60 / carry with a board 0.45 / approach 0.30 / search 0.0225), hover_z 0.040 (100 mm above the table), input stacks <= 5 boards, press 4 mm, place_press 1 mm, blow_off_time 2 s, retract_delay 0.5 s, home_pose, fault_drop_z, timeouts |
| `main.yaml` | home_xy (= home_pose x / y), retry / halt policy, alert times, per-skill timeouts, deferred_verify |
| `scenario_demo.yaml` | the simulated table for `--sim` and the tests |

Zone geometry (tag id -> name, offset, size, sheet top) lives in Vision 1 (`vision1/table.py` `ZONES`).
Logging: `tools/live_env.sh` (sessions, `debug` / `demo`), `tools/live_monitor.py` (all streams, `--lite`).

## Tests

`tests/README.md`. Logic tests run in seconds without ROS (`pixi run python3 tests/world_core_check.py`, ...); ROS /
sim tests need a free domain (`ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=82 pixi run python3 tests/skills_ros_check.py`).
Last full run 2026-10-28: all pass (world_core 86, main_core 49, regress 50, fuzz 21, skill_math 17, fake_world 29, inject 6, skills_ros 30, main_ros 18, ...).

## Limitations / open

- The overhead camera cannot separate same-colour boards stacked on each other: the after-place check of a red on a
  red board can report NOT_VERIFIED (heights are still right). Planned: check with the wrist D405 (TM-O14).
- A real board in a zone misread as UNKWN is not seen (zone UNKWN ignored, TM-33).
- Vision 2 (wrist depth) is not used yet: input picks use the slower contact search (design agreed, TM-25).
- Rare planning failure at the start of a carry, recovered by the retry (TM-O15, under analysis).
- The robot arm mask is a simple band; camera position for the full mask still to measure (TM-O3).

## More

`notes/decision-log.md` (every decision TM-*, open items TM-O*), `notes/control-loop.md` (design CL-*),
`notes/trial-1.md` (first real-robot trial: setup, tests, results), `notes/vision1-changes.md` (Vision 1 contract),
`notes/scheduling-example.md` (how to give the controller parallel work: rules for an agent / VLM, the controller's
abilities and limits, the alerts that need a person).
Scheduling timeline recorder: `pixi run python3 tools/schedule_log.py <file>`.
