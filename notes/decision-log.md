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

## 2. Open

| # | Item | Status |
|---|---|---|
| TM-O1 | How this branch gets the **interfaces of the other branches** (`hk_fr5_interfaces`: motion / gripper / scene; `hk_fr5_vision_interfaces`: PixelToTable) for building and running in sim, before the integration branch (T6) exists | decide before T3 |

## 3. Step log

| # | Step | Status |
|---|---|---|
| T1 | Branch + worktree, `.gitignore`, pixi env (+ vision_msgs), notes, `control-loop.md` moved in | done |
| T2 | `hk_fr5_task_interfaces` | |
| T3 | World model + fake world | |
| T4 | Skills | |
| T5 | Main controller | |
| T6 | Integration branch | |
| T7 | Dashboard | |
| T8 | LLM agent | |
