# FR5 manual test steps (CLI)

Worktree: `cd ~/hackathon/wt/feat-robot-motion-gripper` in every terminal.
Units: metres / radians. `frame: 1` = base frame, default (0) = TCP frame.
First real runs: `velocity_scaling: 0.1` (<= 20 % rule). Human at the e-stop watches the ARM.

## 0. Lab session 2026-10-27: open items (do in this order)
Only the items still open (full history: decision-log §7). **M** = robot moves, **G** = gripper switches:
both need the user's explicit OK in the session (safety rules 1-3), first runs `v 0.1`, e-stop person in place.
Log each result in decision-log (new entry per item) and close the T-row.

| # | Item | Check | Expected / pass | M/G |
|---|---|---|---|---|
| 1 | A + B | network, web UI auto mode + enabled, e-stop person (section A) | `ping` OK, no alarms | - |
| 2 | T15 | read-only, BEFORE the bringup: the C1 one-liner, then once more with `block` = 1: `GetToolDI(0, 1)` | reply format (`[err, value]`? bare int?) + whether `block` changes anything; if not `[0, x]`: fix `parse_reply` in `gripper_io.py` | - |
| 3 | T18 | `pixi run robot` (first real launch with the gripper node) | section B messages; gripper log: real I/O, start state read, **nothing switched**; G1: `/gripper/state` DO/DI = web UI | - |
| 4 | T12 | C0 `state` vs web UI TCP pose (no motion) | x, y negated (base rotated 180 deg, G6); roll = rx, pitch = ry, yaw = rz +- 180 deg (5m); else fix the RPY extraction / interface comment | - |
| 5 | T17 | terminal 3: `pixi run python3 notes/lag_probe.py 15`; during it `d 0 0 30 base`, `d 0 0 -30 base` (gripper node running = polling) | lag stays ~0.1 s (X-10); 2 s = queueing again (check 50 Hz, P-9) | M |
| 6 | T16 | board under the cup: `grip`, lift `d 0 0 30 base`, put down, `release`; also `grip` with no board | `SUCCESS ... in x s` (note x = time_to_vacuum); no board: NO_VACUUM after 3 s, pump off; after release: how long DI0 stays 1 (vacuum decay) in `gstate` / G1; no DI0 flicker (else raise `debounce_reads` 2) | M + G |
| 7 | T3 -> T10 | look at / measure the D405 mount yaw on the real wrist | note the angle; compare with the xacro; decide the home yaw (j6 of SRDF `ready`) when camera use is designed | - |
| 8 | T4 | jog the TCP (`jog`, `v 0.1`) gently to the table top + corners of table, profile, camera post; read `state` at each | table top z vs estimate -0.045 (E-2); update `obstacles.yaml` (formulas in its header); re-run `home` + `d` | M |
| 9 | T19 / T9 | vendor acceleration 6.283 rad/s^2 in `joint_limits.yaml` (now 2.0), re-test Pilz LIN at rising `v` | no `-4` / planning failures at high scaling; then revisit V-7 stop values, L-7 Pilz limits, M-11 tolerances | M |
| 10 | T26 | at the end: Ctrl+C in terminal 1 | terminal 1 shows the driver's `Stopping ...please wait...` and `System successfully stopped!` (on_deactivate = StopMotion / CloseRPC); gripper log: both pumps off | - |

Parked (optional, only if time): T13 blended motion (Pilz sequence), T8 IK benchmark (only if KDL fails noticeably).

## A. Real robot preparation
1. Ethernet laptop <-> controller. Find interface: `nmcli dev`
   `nmcli con add type ethernet ifname <iface> con-name fr5-robot ipv4.method manual ipv4.addresses 192.168.58.10/24`
   `nmcli con up fr5-robot` ; `ping -c3 192.168.58.2`
2. Web UI http://192.168.58.2 : **automatic mode** (circular arrows, not the hand), robot **enabled**, no alarms.
3. E-stop person in place, workspace clear.

## B. Start
- Real robot: `pixi run robot`      | Simulation: `pixi run sim`
- Real: expect `*** REAL ROBOT MODE ...`, driver `机械臂SDK连接成功` (SDK connected) and
  `机械臂硬件启动成功` (hardware started), controllers active, RViz arm = real pose.
- `机械臂SDK连接失败` (SDK connection failed) -> network / another client connected.

## C0. Faster: the command shell (terminal 2): `pixi run shell`  (mm / deg)
```
state                      # joints, tcp, rpy
v 0.1                      # speed (first real runs <= 0.2)
d 0 0 30 base              # 3 cm up (base frame)   | plan on / plan off = plan only
d 0 0 -30 base             # back down
home                       # move_to ready
to . . 300                 # keep x, y (and yaw), z = 300 mm
jog +z 10  | frame base    # jog along one axis (frame setting)
planner lin|ptp|ompl       # planner for `to`
Ctrl+C while moving        # cancel = smooth stop
grip [TIMEOUT_S]            # gripper: vacuum on, wait for vacuum (default 3 s)  | Ctrl+C = cancel (pump off)
release [BLOW_S]            # vacuum off + blow-off (default 1 s)
gstate                      # gripper state (DO0 / DO1 / DI0); says so if the gripper node is not running
attach 270 200 6            # board L W H mm under the cup [YAW DX DY MARGIN] (only after vacuum; lift with `d`, S-12)
detach | detach keep        # remove the board | keep it as obstacle stack_<n>
box add stack 300 220 50 -450 250 -20 [YAW]   # dynamic box NAME SX SY SZ X Y Z (mm, base_link) | box rm [NAME] (none = all)
history | help | quit
```

## C. Checks + first motions (terminal 2)
| # | Command | Expected |
|---|---|---|
| 1 | `pixi run ros2 service call /motion/get_state hk_fr5_interfaces/srv/GetState` | joints (rad) = web UI (deg) |
| 2 | `pixi run ros2 action send_goal --feedback /motion/move_delta hk_fr5_interfaces/action/MoveDelta "{dz: 0.03, frame: 1, velocity_scaling: 0.1, plan_only: true}"` | `SUCCESS: plan found`, NO motion |
| 3 | same without `plan_only`: `"{dz: 0.03, frame: 1, velocity_scaling: 0.1}"` | 3 cm straight UP; repeat 1: z +0.030 (web UI TCP z +30 mm) |
| 4 | `"{dz: -0.03, frame: 1, velocity_scaling: 0.1}"` | back down 3 cm |
| 5 | `pixi run ros2 action send_goal --feedback /motion/move_to hk_fr5_interfaces/action/MoveTo "{named_target: ready, velocity_scaling: 0.1, plan_only: true}"` | check planned path in RViz |
| 6 | same without `plan_only` | arm moves to home (tool down) |
| 7 | cancel test: `move_delta "{dz: 0.10, frame: 1, velocity_scaling: 0.05}"`, Ctrl+C after ~1 s | smooth stop, `Goal canceled`, terminal 1: `smooth stop: ...` |

More commands:
- Pose goal (tool down, yaw rad): `MoveTo "{x: -0.40, y: 0.10, z: 0.30, yaw: 1.57, velocity_scaling: 0.1}"`
- Straight line: add `planner: LIN` ; joint-optimal: `planner: PTP` (both do NOT avoid obstacles)
- TCP frame: `MoveDelta "{dz: 0.05}"` = 5 cm along the cup axis (DOWN when the tool points down)
- Rotate tool in place: `MoveDelta "{dyaw: 0.3}"`

## C1. Gripper (terminal 2)  (decision-log 5l; DO0 vacuum pump, DO1 blow-off, DI0 vacuum achieved)
Real robot: **ask before switching** (safety rule 3). First check = read only (step 20f):
`pixi run python3 -c "import xmlrpc.client as x; r=x.ServerProxy('http://192.168.58.2:20003/RPC2'); print(r.GetToolDI(0, 0), r.GetToolDO())"`
| # | Command | Expected |
|---|---|---|
| G1 | `pixi run ros2 topic echo /gripper/state` | state IDLE (or HOLDING if a board is held), DO/DI = web UI |
| G2 | `pixi run ros2 action send_goal --feedback /gripper/grip hk_fr5_interfaces/action/Grip "{}"` | board held: `SUCCESS: vacuum achieved in x s`; no board: `NO_VACUUM` after 3 s, pump off |
| G3 | `pixi run ros2 action send_goal /gripper/grip hk_fr5_interfaces/action/Grip "{timeout: 5.0}"` | own timeout |
| G4 | `pixi run ros2 action send_goal /gripper/release hk_fr5_interfaces/action/Release "{}"` | pump off, blow-off 1 s, `SUCCESS: released` |
- Sim: `pixi run ros2 param set /gripper/gripper_controller fake_vacuum false` -> grip fails (NO_VACUUM); while
  HOLDING -> state PART_LOST.
- Result codes (error_code.val): 1 SUCCESS | 2 NO_VACUUM | 3 CANCELLED | 4 PREEMPTED | 5 COMM_FAILURE |
  6 CONTROLLER_ERROR | 7 INVALID_GOAL. `use_gripper:=false` starts the bringup without the gripper node.
- Ctrl+C of the bringup switches **both pumps off** (a held board drops, W-10).

## Troubleshooting
- Default control rate is **50 Hz** (P-9). Check terminal 1: `controller_manager update_rate: 50 Hz`.
  At 125 Hz the FR5 lagged 2 s behind every command (and every stop). Arg is `update_rate` (underscore!).
  Unknown / misspelled launch args (e.g. `update-rate`) now stop the bringup with a list of valid names (T22).
- Lag check: `pixi run python3 notes/lag_probe.py 15`, then one small move: lag should be ~0.1 s.
- All plans fail `START_STATE_IN_COLLISION`: obstacle estimates (T4) do not match reality -> stop, report.
- Anything unexpected: **e-stop first**, then Ctrl+C in terminal 1.

## D. Finish
Ctrl+C in terminal 1 (driver StopMotion + disconnect). Web UI: **manual mode + disable**.

## Result codes (error_code.val)
1 SUCCESS | -1 PLANNING_FAILED | -2 INVALID_MOTION_PLAN (path in collision) | -4 CONTROL_FAILED |
-7 PREEMPTED (cancel / new goal) | -16 INVALID_GOAL_CONSTRAINTS (bad input) | -23 ROBOT_STATE_STALE |
-25 COMMUNICATION_FAILURE | -26 START_STATE_INVALID (OMPL: start in collision, e.g. board touching -> lift with move_delta) |
-27 GOAL_STATE_INVALID (unreachable / in collision)
