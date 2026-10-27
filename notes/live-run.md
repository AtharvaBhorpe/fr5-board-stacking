# First real-robot run of the main controller (2026-10-27)

Safety (CLAUDE.md section 3): a human at the e-stop for every motion; first run of new motion code at <= 20 % speed
(skills.yaml: `speed` 0.2, `approach_speed` 0.1, `search_speed` 0.01 - do not raise before the first runs passed);
no motion and no gripper switching without the user's OK in the session. Vision 2 (wrist depth) is NOT used yet:
input picks use the full contact search.

All terminals: the same `ROS_DOMAIN_ID` (unset = 0); `ROS_LOCALHOST_ONLY` unset if Vision 1 runs on another PC.

## 0. Before the slot

- [ ] Vision 1 runs on THIS PC (C9: `stamp` compared with our clock). Other PC -> clocks synced (chrony), else
      `world_model` logs "stamp ... off this clock" and uses the arrival time.
- [ ] Vision 1 in model mode (C13, FROZEN TM-27).
- [ ] Zone sheets placed (brown cardboard, tag beside the board area, inputs >= 3 cm outside every sheet).

## 1. Start (one terminal each, in this order)

Every terminal first joins the log session (TM-27); the FIRST one creates it:

    source ~/hackathon/wt/feat-task-main-controller/tools/live_env.sh new      # terminal 1 only (debug = trial run)
    source ~/hackathon/wt/feat-task-main-controller/tools/live_env.sh          # every other terminal
    #   demo run instead: ... live_env.sh new demo   (normal logging, node terminals not muted)

Everything of the run lands in `~/hackathon/live_logs/session_<date>_<time>/` (symlink `live_logs/current`): `ros/` =
one log file per node (motion, move_group, gripper, Vision 1, ours), `monitor/` = the monitor's files.

| # | Where | Command | Ready when |
|---|---|---|---|
| 1 | this branch | `pixi run python3 tools/live_monitor.py` (quiet console: WARN / ERROR of every node, jobs, vision stats; `--verbose` = everything) | "logging everything to ..." |
| 2 | `~/hackathon/wt/feat-robot-motion-gripper` | `pixi run robot use_rviz:=false` (gripper on by default; start = DO0 + DO1 off) | `/motion/get_state` answers |
| 3 | Vision 1 checkout | `pixi run -e gpu vision --ros-args -p model:=true` (model mode FROZEN, TM-27) | monitor: `VIS` stats at ~5 Hz |
| 4 | this branch | `pixi run ros2 run hk_fr5_task world_model --ros-args $HK_NODE_ARGS` | monitor: no "zone(s) SKIPPED" ERROR |
| 5 | this branch | `pixi run ros2 run hk_fr5_task skill_server --ros-args $HK_NODE_ARGS` | `ros/` log: "skills ready" |
| 6 | this branch | `pixi run ros2 run hk_fr5_task task_main_controller --ros-args $HK_NODE_ARGS` | `task_cli.py status`: skills_ready + motion_ready True |

Stopping: Ctrl+C per terminal (our nodes 6 -> 4 first, then Vision 1, motion, monitor last). The motion terminal may print
a shutdown traceback (frozen branch, harmless). In debug sessions `$HK_NODE_ARGS` = `--disable-stdout-logs`: our node terminals stay empty (no flood), every line is in
`ros/` + the monitor's `rosout.log`, and every WARN / ERROR shows on the monitor console. A crash traceback still prints.

Commands (Claude or a human): `pixi run python3 tools/task_cli.py help` - look: `status`, `world`, `list`, `wait JOB`;
jobs: `transfer VARIANT TO [COUNT]`, `board ID TO`, `process VARIANT ZONE [TO]`, `clear FROM TO`; control: `pause`,
`resume`, `stop`, `home`, `cancel JOB`, `prio JOB N`, `reset_zone ZONE`; debug: `skill pick board_id=...` (only while
paused / idle; moves the robot).
Stop everything: `task_cli.py control stop` (holds a board in place) or the e-stop; then Ctrl+C in 5 -> 1.

## 2. Checks before the first motion (no motion)

- [ ] Monitor: `VIS` ~5 Hz, gap max < 1.5 s (`stale_after`), now-stamp small (same clock); no parse problems.
- [ ] `WRLD`: vision_ok True; zones from Vision 1 with the measured `base_z` (touch-off, TM-O9) + the fault zone;
      boards on the table = `input`, boards on sheets = `zone`.
- [ ] Fault zone position + `fault_drop_z` set in world.yaml / skills.yaml (TM-O5), else the placeholder (-0.30, 0.70).

## 3. First motions (each one only after the user's OK)

1. `control home` -> the tcp goes to the park pose above the aluminium profile ([-0.233, 0.0, 0.243], skills.yaml
   `home_pose`). Check `/motion/get_state` (TM-O2: main.yaml `home_xy` = the measured x, y).
2. One board: `add type=transfer select_variant=VARa to=drop_off_a count=1` -> watch hover, contact search, lift, carry,
   place (1 mm press, 2 s blow-off), home, settle check. Hand on the e-stop.
3. Processing: `add type=process select_variant=VARa zone=processing_1 to=drop_off_a count=1`; the operator takes
   the board away after the arm left and puts the SAME variant back (`expanded_detection: false`).
4. Note every tuned value (press, place_press, blow_off_time, speeds, home_pose) in decision-log.md.

## 4. After the run

- `~/hackathon/live_logs/session_<date>_<time>/monitor/` (TM-26 / TM-27): `main.jsonl` (status, events, world changes, gripper, tcp 10 Hz,
  skill action status + feedback, vision stats, rosout), `vision_raw.jsonl` (every Vision 1 frame + gap / lag),
  `world_raw.jsonl` (every world message), `joints.jsonl`, `rosout.log` (all nodes, text). Frame gaps vs the timers
  (TM-O8): `vision_stats` lines / SLOW frame warnings.
- Open items: TM-O2 / O5 / O7 / O9 / O10 / O11.
