# Decision log: branch `feat/task-main-controller`

Main control loop: world model, skills, `task_main_controller` (state machine + job queue), dashboard, LLM agent.
Design, concepts, skills, failure modes and all control-loop decisions: **`control-loop.md`** (CL-*). This file:
branch / build / test decisions (TM-*) and the step log. Team-wide plan + time plan: `~/hackathon/plan/` (local).
Units: metres / radians in code, mm only in logs.

## 1. Decisions

| # | Topic | Decision | Why / options |
|---|---|---|---|
| TM-1 | Names | Branch `feat/task-main-controller`, worktree `~/hackathon/wt/feat-task-main-controller/`, packages `hk_fr5_task` + `hk_fr5_task_interfaces`, main node `task_main_controller` | user (was "task executive") |
| TM-2 | Environment | Own pixi env: copy of the vision branch env (= motion deps + opencv / aiohttp / numpy / pyyaml pinned) + **`ros-humble-vision-msgs` 4.1.1** (lock: +2 packages only); `build` task = every package under `src/` | Vision 1 contract = `vision_msgs/Detection2DArray` (project-plan P-1); same deps as the other branches keeps the later merge simple |
| TM-3 | Design doc | `control-loop.md` moved here from `~/hackathon/plan/` (that path is now a symlink); it stays the single place for CL-* decisions | user: one dedicated file for the biggest chunk |
| TM-4 | Test / sim time | Tests use synthetic data + the fake world, short timeouts; per step only the tests it touches; the full suite only at milestones / before robot sessions | user: 30 h is tight |
| TM-5 | T3 input | World model built and tested against the **fake world only** (Vision 1 is behind); **re-tested against the real Vision 1 branch and on the real setup once the teammate finishes** | user |
| TM-6 | Interfaces of the other branches (was TM-O1) | **A: `pixi run build` also builds them from the neighbouring worktrees** (read-only, not copied): `../feat-robot-motion-gripper/src/hk_fr5_interfaces`, `../feat-vision-transform/src/hk_fr5_vision_interfaces` + `hk_fr5_vision` (calib maths for the fake world). Sim until T6 = 3 terminals: `pixi run sim` (motion worktree), `pixi run vision` (vision worktree), our nodes. **Integration (T6): change the `build` task back to `--base-paths src` (all packages are then in one tree), one launch for the whole stack; nothing was copied, so no duplicate packages to remove** | user: A. Options: B copy the packages (drift -> merge conflicts), C develop in an integration branch now (breaks isolation). Downside: a fresh clone of this branch alone cannot build until T6 |
| TM-7 | Task interfaces (T2) | `hk_fr5_task_interfaces`: msgs `World` (header, vision_ok / age, boards, zones, obstacles, held board, gripper state), `Board` (world id, variant, location input / zone / held, x / y / yaw, measured length / width, per-variant thickness, top_z (NaN = unknown -> contact search), score, inside, size_ok, age), `Zone` (name, kind processing / drop_off / fault, state EMPTY / LOADED / PROCESSING / DONE / FAULT or OK / FULL, rotated rect, board_id, stack count / top, fixed = config fallback CL-15, ages, message), `Obstacle`, `Job` (types process / transfer / clear_zone / home, select variant / board / from_zone, zone, to, count 0 = all, source, state, progress), `SkillCode` (24 shared result codes = the failure table); actions `Pick` (board_id, contact_search, speed -> code, attach_yaw), `Place` (zone or explicit pose -> code, stack_count), `Home`, `WaitClear`, `ReleaseSafe` (feedback = phase); services `AddJob` / `ListJobs` / `Control` (pause / resume / stop / cancel_job / clear_queue / home) | One shared code list so the main controller and the LLM react alike; Job = the same small set for dashboard presets and the LLM (single board = transfer with select_board, CL-18); only std_msgs as dependency |

## 2. Open

| # | Item | Status |
|---|---|---|
| TM-O1 | Interfaces of the other branches | decided: TM-6 |

## 3. Step log

| # | Step | Status |
|---|---|---|
| T1 | Branch + worktree, `.gitignore`, pixi env (+ vision_msgs), notes, `control-loop.md` moved in | done `6058e7f` |
| T2 | `hk_fr5_task_interfaces` (TM-7) | done |
| T3 | World model + fake world | |
| T4 | Skills | |
| T5 | Main controller | |
| T6 | Integration branch | |
| T7 | Dashboard | |
| T8 | LLM agent | |
