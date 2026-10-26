# Main control loop: design, decisions, rules (living document)

Lives in branch `feat/task-main-controller` `notes/` (also reachable as `~/hackathon/plan/control-loop.md`). Biggest chunk:
every decision, trade-off and rule about the control loop goes here (CL-*). Started 2026-10-26 evening.

## 1. Big picture

```
 Operator ── Dashboard (web) ──┬── job queue (buttons / forms) ─────────┐
                               └── chat ── LLM agent (option B) ────────┤ same commands
                                                                        v
                                         task_main_controller  (state machine, runs WITHOUT the LLM)
                                                 │ skill calls (actions)
                                                 v
                                         skill_server  (pick / place / home / wait_clear ...)
                                                 │
            ┌────────────────────┬───────────────┼──────────────────┬──────────────────┐
         /motion/*          /gripper/*        /scene/*      world_model  <── Vision 1 (Detection2DArray)
                                                                 │      <── /vision/pixel_to_table
                                                                 v
                                                  facts in base_link: boards, zones, stacks, obstacles
```

### 1.1 How the state machine and the LLM work together (game-dev analogy)

- The **executive** is like an NPC's state machine: it owns the robot's behaviour, every transition and every
  failure reaction. It runs a **job queue** and needs nothing else to work.
- The **LLM** is like a game director / quest giver: it never moves the NPC itself. It can only use the **same
  commands as the dashboard** (add / remove / reorder jobs, pause, resume, answer a question) plus read the world.
- The executive **asks** the LLM only at defined decision points ("grip failed twice on board 7: skip / retry /
  ask operator?"). If the LLM is slow, wrong or offline, the executive uses its **default policy** and goes on.
- So: dashboard buttons = rule-based control; LLM = natural-language control on top of the very same commands.
  Every command is validated by the executive before it is accepted (unknown zone, impossible job -> rejected).

## 2. Concepts (names used everywhere)

| Term | Meaning |
|---|---|
| **Board** | one physical board; seen by Vision 1, tracked by the world model; has a variant (e.g. A, B) |
| **Zone** | area on the table from Vision 1: `processing_1`, `processing_2`, `drop_off_1`, `drop_off_2` (+ a **fault zone**, CL-11). A zone **plus an extended margin around it** is reserved for processing / drop-off (CL-7) |
| **Input area** | everything outside the zones + margins. Input boards lie there, also as **input stacks**: 2+ stacks of the same variant possible (CL-7) |
| **Stack** | a pile of boards: input stacks (we take the top board) and drop-off stacks (we add on top); known / measured count + top height (+ scene obstacles `stack_n` for drop-off stacks) |
| **Variant** | 3 now (CL-9): **white small**, **white large** (clearly different sizes), **red**; physical size + thickness per variant in config. Later maybe a **processed** (printed, non-variant) board (CL-8) |
| **Skill** | one robot primitive with checks, executed by `skill_server` (e.g. `pick`, `place`) |
| **Job** | one operator / LLM request in the queue (e.g. "process all A boards in processing_1, then drop-off 2"); the executive breaks it into **steps** (skills + waits) |
| **Step** | one skill call or one wait inside a job |
| **World model** | the single source of facts (base_link), read by the executive, the dashboard and the LLM |

## 3. The processing simulation (user)

1. The robot places a board in a processing zone.
2. The operator **removes** it from the table: the camera sees the zone become empty -> "processing".
3. After a few seconds the operator **puts it back**: the camera sees a board in the zone -> "processed, ready".
4. A job can then move it on (e.g. to drop-off 2).

-> Each **processing zone has its own small state machine** (in the world model):

| Zone state | Meaning | Enter when |
|---|---|---|
| `EMPTY` | free, can receive a board | start; robot picked a processed board out |
| `LOADED` | robot placed board X, not yet taken | place skill succeeded into this zone |
| `PROCESSING` | board X taken away by the operator | zone empty for >= T_gone (debounce) while `LOADED` |
| `DONE` | a board came back: board X processed | a board seen in the zone for >= T_back while `PROCESSING` |
| `FAULT` | unexpected: board appeared while `EMPTY`, timeout, (CL-8) a non-processed board came back | see failure modes |

The world state must show a zone as **busy** for the whole time from the robot's place until the robot picks the
processed board out again (`LOADED`, `PROCESSING`, `DONE` = busy; only `EMPTY` accepts a board) (user).

**CL-8 (conditional, depends on Vision 1; refined by CL-19: 'processed' is a world-model state):** the teammate tries to detect a **printed / painted board** as a
"non-variant" (processed) board. If that works: a plain board goes into processing, the operator replaces it with a
printed board -> only a **non-variant board** coming back counts as `DONE` (processed) and goes to the drop-off;
if a **plain variant board** comes back -> "processing did not happen" -> the robot moves it to the **fault zone**.
Until Vision 1 delivers it: any board coming back = processed.

Identity: Vision 1 track ids will change when a board leaves and comes back; the world model keeps the
**board identity per zone** (the board that comes back into `processing_1` is the one that left it).

## 4. Skills (robot level, `skill_server`) - full definitions

Common to all skills: ROS 2 **action** (cancel = smooth stop through `/motion` cancel), feedback = current phase,
result = `success`, `code` (enum below), `message`, plus what changed in the world. All poses from the world
model in base_link. First real runs <= 20 % speed (safety rule).

### 4.1 `pick(board_id)`
- **Purpose**: take one board from where it lies (input area, **top of an input stack**, a processing zone in
  `DONE`).
- **Stack height** (input stacks): the overhead camera sees only the top board, not the count. Options (CL-open):
  (a) the operator / LLM tells the count; (b) **contact search**: slow LIN descent in small steps until the
  vacuum sensor reports a seal (the cup itself detects the top board); (c) D405 depth at the hover pose (D-1).
- **Preconditions**: nothing held (gripper IDLE, no attached board); board known, seen within T_fresh, inside the
  calibrated area (`inside`), not under an obstacle; robot ready.
- **Steps**: (1) target from world: x, y, yaw, length, width, variant; (2) `move_to` hover (z_hover above the board,
  yaw aligned, OMPL); (3) re-check the board is still there (fresh detection, moved < tol); (4) `move_delta` LIN down to
  board top + approach offset, slow; (5) `/gripper/grip`; (6) `/scene/attach_board(length, width, height, yaw rel. to
  tcp, dx, dy)`; (7) `move_delta` LIN up to z_lift; (8) check vacuum still on.
- **Postconditions**: board held + attached; world: board state `HELD`; its zone updated (e.g. processing zone ->
  `EMPTY`).
- **Failure codes**: `BOARD_NOT_FOUND` (gone / stale before start), `BOARD_MOVED` (moved during approach),
  `PLAN_FAILED`, `MOTION_FAILED`, `GRIP_FAILED` (no vacuum within timeout), `LOST_ON_LIFT` (vacuum lost after
  lift), `OBSTACLE` (hand in the work area -> paused / cancelled), `CANCELLED`, `GRIPPER_ERROR`, `SCENE_ERROR`.
- **Built-in retries**: `GRIP_FAILED` -> release, lift, re-detect, 1 retry with a slightly deeper descent;
  `PLAN_FAILED` -> 1 retry. Everything else: report to the executive.

### 4.2 `place(target)`
- **Target**: a zone (`processing_1/2`: single board, `drop_off_1/2`: on the stack) or explicit x, y, yaw.
- **Preconditions**: board held; target zone known and fresh; processing zone `EMPTY`; stack below max height.
- **Steps**: (1) place pose: zone centre (+ stack top + board thickness for drop-off; yaw = zone yaw or stack yaw);
  tcp yaw = target yaw - attach yaw; (2) hover `move_to`; (3) re-check target free / stack where expected;
  (4) LIN down to place height + small gap; (5) `/gripper/release` (vacuum off + blow-off); (6) `/scene/detach_board`
  (`keep_as_obstacle` = true on stacks, false in processing zones); (7) LIN up; (8) verify (camera: board now in the
  zone; later D405).
- **Postconditions**: nothing held; world: board in the target; processing zone -> `LOADED`; stack count + 1.
- **Failure codes**: `NOT_HOLDING`, `TARGET_UNKNOWN`, `TARGET_OCCUPIED`, `STACK_FULL`, `PLAN_FAILED`,
  `MOTION_FAILED`, `RELEASE_FAILED` (board still detected on the cup), `LOST_IN_TRANSIT` (vacuum lost before
  place: board dropped somewhere), `OBSTACLE`, `CANCELLED`, `NOT_VERIFIED` (camera does not see it in the zone).

### 4.3 `home()`
- Move to SRDF `ready` (OMPL). Precondition: none. Used at start, end, after errors. Codes: `PLAN_FAILED`,
  `MOTION_FAILED`, `CANCELLED`.

### 4.4 `wait_clear(timeout)`
- Wait until no obstacle (hand / unknown) is in the robot work area for T_clear. Codes: `TIMEOUT`.

### 4.5 `release_safe()` (recovery)
- If holding: move to a safe drop pose (e.g. back to the pick spot or a "reject" spot), release, detach.
  For "unknown state" recovery.

### 4.6 Later (D405, see `d405-ideas.md`)
- `inspect(target)`: measured board / stack height (D-1), pick / place verification (D-2), fine alignment (D-3).

## 5. Jobs (operator / LLM level, executed by `task_executive`)

Built from a few **job types** with simple parameters and conditions, so the dashboard can offer them as forms and
the LLM can produce them as structured commands:

| Job type | Parameters | Steps the executive generates |
|---|---|---|
| `transfer` | `select` (variant / board id / zone / any), `to` (zone), `count` (n / all) | per board: pick -> place |
| `process` | `select`, `zone` (processing_1/2), `then_to` (drop-off zone, optional), `count` | per board: pick -> place in zone -> **wait zone DONE** -> (pick -> place `then_to`) |
| `clear_zone` | `zone`, `to` | pick everything in a zone -> place `to` |
| `home` / `pause` / `resume` / `stop` | | |

Conditions (simple, dashboard-friendly): `count`, `until <zone> empty`, `only if <zone> EMPTY`, `timeout`.
Example (user): *process all variant A boards in processing_1, then put them on drop-off 2* =
`process(select=variant A, zone=processing_1, then_to=drop_off_2, count=all)`.

Executive behaviour (**CL-10, user: the more intelligent option**): **use waiting time**: while a processing zone is
busy, the executive works on other jobs / the other processing zone, and comes back when the zone is `DONE`.

**Operator side (CL-12, user):** the dashboard does NOT expose every skill and condition (complexity would explode).
It offers **a few basic preset tasks with simple conditions**, chosen to be easy to test and debug. **The LLM is the
main way to set up operations**: the operator says what to do, the LLM turns it into validated jobs.

## 6. Failure modes and policies (executive)

| Area | Failure | Default policy (no LLM) | LLM may decide |
|---|---|---|---|
| Perception | board not found / stale | re-observe 2 s, then skip the board (job continues) | |
| | board outside calibrated area | warn; allow (accuracy lower) or skip (config) | |
| | zone missing (not detected) | pause the job, alert | |
| Grasp | `GRIP_FAILED` after retry | skip the board, mark it `PROBLEM`, continue | retry / ask operator |
| | variant from Vision 1 does not match the measured size (world model) | treat as unknown, ask / skip | decide |
| | `LOST_ON_LIFT` / `LOST_IN_TRANSIT` | stop, home, alert (board somewhere unknown), re-observe | |
| Motion | `PLAN_FAILED` after retry | skip the step / board, home | |
| | `MOTION_FAILED` / e-stop / robot disconnected | stop everything, alert | |
| Safety | hand / obstacle in the work area | cancel motion (smooth stop), `wait_clear`, resume the step | |
| Place | target occupied / stack full | pause the job, alert | choose another zone |
| | processing zone busy (not `EMPTY`) | wait (other jobs meanwhile, CL-10) | |
| Process | (CL-8) plain variant board came back = not processed | pick it -> **fault zone** | |
| | `RELEASE_FAILED` | blow-off again, lift slowly, alert | |
| Process | processing timeout (no board back in T_proc) | alert, job waits / continues others | |
| | wrong variant came back | zone `FAULT`, alert | |
| | board appears in an `EMPTY` processing zone | zone `FAULT` (unknown board), alert | |
| System | camera / Vision 1 down (no detections > T) | pause, alert | |
| | LLM unavailable / timeout | default policy, dashboard shows "LLM offline" | |

## 7. LLM integration (CL-2: option B, user)

- Executive runs autonomously; the LLM **supervises**: natural-language commands -> validated jobs, decisions at
  defined decision points, explanations ("why did you skip board 7?").
- Tools the LLM gets (= dashboard commands): `get_world`, `list_jobs`, `add_job(job)`, `remove_job(id)`,
  `reorder`, `pause`, `resume`, `answer(decision_id, choice)`. Never motion / gripper / scene directly.
- Every LLM command is validated like a dashboard command; rejected commands go back to the LLM with the reason.

## 8. Decisions

| # | Decision | Why |
|---|---|---|
| CL-1 | State machine executive (node `task_main_controller`, branch `feat/task-main-controller`) + skills; works without the LLM | robust demo fallback; user's state-machine experience |
| CL-2 | LLM option B (supervisor: operator commands, decisions, explanations) | robust + intelligent (user) |
| CL-3 | Main **dashboard** web app with LLM chat; operator builds simple **job queues** from skills / conditions; the LLM gives deeper commands through the same command API (user) | |
| CL-4 | Processing is simulated: robot places, operator removes (= processing), operator returns (= done); zone state machine in the world model (user) | |
| CL-5 | Skills / jobs / failure modes defined verbosely before coding (user) | biggest chunk, many failure modes |
| CL-6 | This file is the single place for control-loop decisions (user) | |
| CL-7 | Zones + an extended margin around them are only for processing / drop-off; input boards lie elsewhere, also as **input stacks** (2+ of the same variant possible). Skill expansion (stacks) open for brainstorming | user |
| CL-8 | Processed-board detection (printed = non-variant board) **if Vision 1 can do it**: only a non-variant board back in a busy zone = processed; a plain board back = processing failed -> fault zone. Until then: any board back = processed | user idea, depends on the teammate |
| CL-9 | 3 variants: white small, white large (sizes clearly different), red. Physical size / thickness per variant in config; world model cross-checks the Vision 1 label against the measured size | user |
| CL-10 | Executive uses waiting time (works on other jobs while a zone is processing) | user: more intelligent option |
| CL-11 | A **fault zone** for boards with problems (failed processing, unknown, failed grip after retries) | user |
| CL-12 | Dashboard: only **a few preset tasks + simple conditions** (easy to test / debug); the **LLM is the main mode** for setting up operations | user: full skill / rule editor in the web app = too complex |
| CL-13 | LLM = **Claude** (assumed for now), internet always available; we write its instructions / tool definitions (agent.md etc.) | user |
| CL-14 | Timings, heights, speeds: start with defaults, change as needed | user |
| CL-15 | **Fault zone**: dynamic from Vision 1 (`zone/fault`) if the table has room; **fallback: a fixed base_link x / y / z in config, beside the drop-off zone but OFF the table** (a stool under it so boards never hit the floor). **TODO at implementation: check it is reachable** with the H-11 joint limits (j1 +-175, j2 -165..+10, j3 +10..+160, j5 -170..-10) and without swinging round the back (`tests/joint_config_check.py` style) | user; table may be too small for a 5th zone |
| CL-16 | Input stack height: **contact search only** for now (vacuum on, slow LIN descent, stop on the vacuum seal = smooth motion cancel). D405 only if Vision 2 ever lands | user |
| CL-17 | Board thickness is a **per-variant field** (all 3 variants 6 mm today); never one global thickness, so variants with other thicknesses can be added | user |
| CL-18 | LLM tools = job types **and** single validated steps (`pick board N`, `place in <zone>`). **The LLM part comes near the end**, once skills / actions are about final; parts of it can be written in parallel as pieces finalize, so it is not one large block at the end | user |
| CL-19 | **Processed is a world-model state, not a Vision 1 label** (user): Vision 1 only gives variant labels (white_small / white_large / red) and, if its expanded detection lands, a **non-variant** class (proposed `board/other`). The world model keeps each board's ORIGINAL variant through processing and sets `processed` when the board comes back into a busy processing zone. Only with `expanded_detection: true`: non-variant board back = processed (DONE); plain white / red board back = not processed (FAULT -> fault zone). `Board` gets `label` (raw Vision 1 class) + `processed` | user; replaces the 'processed' variant of CL-8 |
| CL-20 | **World update strategy: B (user, 2026-10-26)** - implemented with T4: held board / arm visible to the real overhead camera. A: snapshots only (world rebuilt from one Vision 1 snapshot at start + after each action, robot moved away) - user's proposal; misses the operator's processing (take + return between two snapshots) and hands during motion. **B (recommended): continuous + arm shadow mask** (ignore detections / pause empty + lost timers inside a band base -> tcp, ~12 cm wide, from TF) **+ settle check** (after a place the skill moves away, the world confirms from fresh frames); ~1 h, with T4. C: continuous, no mask (current T3; fine with the fake world, which hides the held board) | do not forget: needed before the real Vision 1 / setup test (TM-5) |
| CL-21 | **Planning: reactive** (user): on every tick each job says its next action from the world as it is NOW (pick board X / place in zone Y / wait / done), no pre-expanded step lists. A held board may only be placed by the job that picked it; boards chosen by a job are claimed (no other job takes them) | robust to moved / returned boards; waiting time falls out naturally |
| CL-22 | **Scheduling = explicit priority** (user): `Job.priority` (higher first, ties = queue order), changeable at runtime (`Control` `set_priority`, dashboard / LLM). The robot switches jobs only BETWEEN skills. Example (user): T1 (high) process variant A -> processing_1 -> drop_off_1, T2 transfer a stack -> drop_off_2: T1 pick + place into processing_1 -> while it is processed T2 pick + place -> the processed board is back -> T1 has priority: pick from processing_1 + place on drop_off_1 -> T1 next board -> repeat until the end conditions | user |
| CL-23 | After every skill result the controller decides only on a `/task/world` message NEWER than that result (max 1 s wait) | the skill's event reaches the world model just before the result: an older world would show e.g. processing_1 still EMPTY -> a 2nd board placed into it |
| CL-24 | count = 0 (all): a job is done when no candidate board was seen for 3 s of fresh world data and none of its boards is in flight | boards briefly hidden (arm shadow, the next board of a stack) must not end it early |
| CL-25 | `cancel_job` while holding: that board is first placed at its target, then the job is cancelled; `stop` (immediate): a held board goes to the fault zone on resume. `pause` = finish the current skill, then hold | user |
| CL-26 | Unowned processing-zone FAULT (unknown board): alert only, the operator clears it. Board choice: **nearest to the CURRENT tcp position first** (user: read via the motion controller's `/motion/get_state` right before choosing; fallback home position from `main.yaml` if it fails): short moves, deterministic. Failure policies + job validation as proposed (T5, revisit if needed) | user |

## 9. Open questions

1. Drop-off stacks: max count / height?
2. Which preset tasks does the dashboard offer (CL-12)? Proposal: process <variant> via processing <1/2> ->
   drop-off <1/2> (count n / all); transfer <variant> -> drop-off <1/2>; clear <zone> -> fault zone; home / pause /
   resume / stop.

Answered (round 3, user): fault zone (CL-15), input stack height (CL-16), thickness per variant (CL-17), LLM tools
+ timing (CL-18).
Answered (round 2, user): processing capacity 1 board, zone busy state (section 3); scheduling = use waiting time
(CL-10); variants (CL-9); LLM Claude + internet (CL-13); defaults for timings (CL-14); fault zone (CL-11).
