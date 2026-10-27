# Trial 1: first real-robot run of the whole pipeline (2026-10-27)

One page for the whole trial: setup values, bring-up, checks, tests T0-T5, stop / recovery, results.
(Background: `live-run.md`, `live-tests.md`, decision-log TM-19..TM-29.)

## 0. Rules (always)

- **Every robot motion only after the user's OK** in the session. User at the e-stop for every motion.
- Speeds as in skills.yaml (`speed` 0.2, `approach_speed` 0.1, `search_speed` 0.01): <= 20 %, do not raise in trial 1.
- **Hands are NOT detected** (Vision 1 sends no obstacles yet): nobody reaches into the work area while the arm moves.
  Board hand-overs (operator) only while the arm is at home / away from that zone.
- Stop at the first unexpected thing: `stop` (or the e-stop), read the logs, decide, then continue.
- Vision 2 (wrist depth) is NOT used: input picks use the full contact search (slower, ~5 s per pick).

## 1. Setup values (all set, with sources)

| What | Value | Where | Source |
|---|---|---|---|
| Zones (dynamic: positions come live from the tags) | tag 2 = `processing_1` (right table), tag 1 = `drop_off_a`, tag 16 = `drop_off_b` (left table) | Vision 1 `vision1/table.py` ZONES (**local edit, teammate commits it**) | user, 2026-10-27 |
| Zone geometry | board area 0.34 x 0.25 m, centre 0.205 m along the tag's +X, rotation 0 (tag +X = long side), sheet 5 mm -> top_z -0.055 | same | user's measurement; tag capture `live_logs/zone_tags/` |
| Fault zone (ours, a void) | (+0.4287, +0.7108), drop from tcp z 0.194; carry / retract there never lower | world.yaml, skills.yaml | hand-guided probe TM-29, joints inside the H-11 limits |
| Home | tcp (-0.233, 0.0, 0.243), yaw 1.543: above the aluminium profile | skills.yaml `home_pose`, main.yaml `home_xy` | user + teammate's park pose (TM-25) |
| Pick / place | press 4 mm, place 1 mm into the top, blow-off 2 s | skills.yaml | teammate's real-robot values (TM-25) |
| Board variants | VARa 0.295 x 0.210, VARb 0.210 x 0.150, VARc 0.210 x 0.135 (red), 5 mm | world.yaml | Vision 1 spec (TM-20) |
| Vision 1 | GPU, model mode (frozen), 5 Hz, latency ~0.13 s | `~/hackathon/wt/feat-vision1-overhead` | TM-27 / TM-28 |
| Cup length | flange -> cup tip 0.1074 | motion branch (frozen), vision-transform V-27 | teammate's contact tests H-12 |

Material: 3 red VARc, 4-5 white VARa / VARb. Keep every input board / stack >= 3 cm outside every sheet.

## 2. Bring-up (6 terminals, in this order)

Every terminal FIRST (terminal 1 with `new`, all others without):

    source ~/hackathon/wt/feat-task-main-controller/tools/live_env.sh new     # terminal 1 only: new debug session
    source ~/hackathon/wt/feat-task-main-controller/tools/live_env.sh         # terminals 2-6 (and any later one)

| # | Directory | Command | Ready when |
|---|---|---|---|
| 1 | `~/hackathon/wt/feat-task-main-controller` | `pixi run python3 tools/live_monitor.py` | "logging everything to ..." |
| 2 | `~/hackathon/wt/feat-robot-motion-gripper` | `pixi run robot use_rviz:=false` (robot in AUTO mode, not manual) | monitor quiet, no ERROR |
| 3 | `~/hackathon/wt/feat-vision1-overhead` | `pixi run -e gpu vision --ros-args -p model:=true` | "seg model on CUDAExecutionProvider"; monitor `VIS` 5 Hz |
| 4 | `~/hackathon/wt/feat-task-main-controller` | `pixi run ros2 run hk_fr5_task world_model --ros-args $HK_NODE_ARGS` | no "zone(s) SKIPPED" on the monitor |
| 5 | same | `pixi run ros2 run hk_fr5_task skill_server --ros-args $HK_NODE_ARGS` | (silent in debug) |
| 6 | same | `pixi run ros2 run hk_fr5_task task_main_controller --ros-args $HK_NODE_ARGS` | `tc status`: skills_ready + motion_ready True |

Command shortcut (in the task-branch directory): `alias tc='pixi run python3 tools/task_cli.py'` (`tc help` = all
commands). Logs of the whole trial: `~/hackathon/live_logs/current/` (`ros/by_node/<node>.log`, `ros/INDEX.txt`,
`monitor/*.jsonl`, `monitor/rosout.log`).

## 3. Checks before any motion

- [ ] Monitor `VIS`: ~5 Hz, gap max < 1.5 s, now-stamp ~0.1-0.2 s, no parse problems.
- [ ] `tc world`: vision_ok True; zones `processing_1` (EMPTY), `drop_off_a`, `drop_off_b` (OK) with base -0.055 and
      the fault zone (FIXED); positions on the sheets (compare `live_logs/zone_tags/zones_live.jpg`).
- [ ] `tc status`: state IDLE, skills_ready / motion_ready / world_ok True, no problem boards.
- [ ] First motion (OK + e-stop): `tc home` -> tcp at (-0.233, 0.0, 0.243) above the profile (`tc status` tcp).

## 4. Tests (in this order; each one assumes the previous one passed)

### T0 Perception only (no motion, ~5 min)

Layout: one board of each variant alone on the pick table; a 2-stack of one variant; a red VARc ON each sheet;
one white board 3-4 cm outside a sheet edge. Robot at home.
1. `tc world`: every board once, right variant, size_ok; boards on sheets = `zone` (that zone), others = `input`;
   the edge board = `input`.
2. By hand: slide a board 2 cm (same id); remove one (gone after ~2 s); cover a tag 5 s (the zone stays); put back.
- Exposes: red on brown, zone outline / margin vs the real sheets, tracking with real jitter, the home pose hiding
  boards, low-confidence detections.
- Pass: as described. Then clear the sheets.

### T1 First picks / places: one board, then a mixed stack of 2 (drop_off_a)

1. `tc transfer VARa drop_off_a 1` (one white VARa alone on the pick table). `tc wait j1`.
2. `tc transfer VARb drop_off_a 1` (a VARb onto the same stack).
- Watch: cup over the board centre at hover; contact-search seal z vs table + 5 mm (skill log "grip ... SEALED at tcp");
  release z = -0.055 + n x 5 mm - 1 mm; settle check passes; `tc world` count 1 then 2.
- Exposes: camera vs robot x / y offset, real board thickness, seal overtravel, place accuracy, boards sliding at
  blow-off, NOT_VERIFIED from jitter, stack height on a sheet.
- Pass: both DONE, drop_off_a count 2, boards on the sheet aligned with the zone.

### T2 Red input stack of 3 -> drop_off_b

Layout: 3 VARc stacked on the pick table. `tc transfer VARc drop_off_b 0` (all). `tc wait j3`.
- Watch: seal z ~15 / 10 / 5 mm above the table; after each pick the next board is seen at the same spot (new id,
  input); red on brown confirmed after each place; drop_off_b count 3, top -0.055 + 15 mm.
- Exposes: stacked boards offset / not flat, the stack lost / doubled after a pick, red-on-brown during the settle
  check, the arm hiding the stack at hover.
- Pass: DONE 3 delivered, nothing left on the pick table.

### T3 Processing with the operator + a parallel transfer

Layout: 1 VARa + 2 VARb on the pick table (outside the processing_1 sheet, which is on the same table).
Jobs: `tc process VARa processing_1 drop_off_a 1 10` and `tc transfer VARb drop_off_b 0 5`.
Operator: after the robot placed the VARa on processing_1 and moved away, take it off the sheet (>= 2 s), bring the
SAME board back after ~10 s (roughly the same spot). Hands only while the arm is away from processing_1.
- Expected order: VARa -> processing_1, VARb -> drop_off_b (meanwhile), VARa (DONE) -> drop_off_a, rest of VARb.
- Watch (monitor WRLD): processing_1 LOADED -> PROCESSING -> DONE; known-top pick from the sheet (-0.055 + 5 mm,
  press 4 mm); the board put back keeps its id.
- Exposes: the 2 s processing timers vs real vision flicker, the hand counted as a board, the arm over the zone,
  known-top pick errors (GRIP_FAILED -> retry 3 mm deeper), the scheduling order.
- Pass: both jobs DONE, order as expected, processing_1 EMPTY at the end.

### T4 Recovery commands (if time)

1. `tc transfer any drop_off_a 1`; while HOLDING: `tc pause` -> the board is placed first, then nothing moves;
   `tc resume`.
2. Next `tc transfer any drop_off_b 1`; while HOLDING: `tc stop` -> holds; `tc resume` -> release_safe: the board goes
   to the FAULT zone (behind the base, dropped from 0.194).
3. `tc clear drop_off_a drop_off_b 1` -> picks the top board of a drop-off stack (known top from our count).
- Exposes: the fault spot + drop height on the real robot, a pick from a real stack top, pause / stop semantics.
- Pass: each step as described; the dropped board leaves the world.

### T5 Edge cases (optional)

A board rotated ~45 deg; a board at the far reach corner; a non-variant object (UNKWN) on the pick table (`tc transfer
any ...` must not pick it); a board moved by hand while the robot hovers over it (re-decision).

## 5. Stop and recovery

| Situation | Do |
|---|---|
| Anything unexpected while moving | `tc stop` (holds a board, smooth stop) or the e-stop |
| Continue after `stop` | `tc resume` (a held board goes to the fault zone first) |
| A job misbehaves | `tc cancel jN` (a held board is placed first) |
| Controller HALTED (3 motion failures / release failed) | check the robot / gripper, then `tc resume` |
| A zone emptied by hand | `tc reset_zone <zone>` |
| A board on the cup, nothing running | `tc pause`, then `tc skill release_safe` (fault zone) |
| End | Ctrl+C in terminals 6 -> 4, then 3, 2, 1 (the motion terminal may print a harmless traceback) |

## 6. Results (fill in during the trial)

| Test | Result | Seal z / place z offsets | Notes, tuned values |
|---|---|---|---|
| Checks | | | |
| T0 | | | |
| T1 | | | |
| T2 | | | |
| T3 | | | |
| T4 | | | |
| T5 | | | |

After the trial: copy the results + every tuned value into decision-log.md; logs stay in the session folder.
