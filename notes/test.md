# FR5 manual test steps (CLI)

Worktree: `cd ~/hackathon/wt/feat-robot-motion-gripper` in every terminal.
Units: metres / radians. `frame: 1` = base frame, default (0) = TCP frame.
First real runs: `velocity_scaling: 0.1` (<= 20 % rule). Human at the e-stop watches the ARM.

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

## Troubleshooting
- Stutter / jerky motion: restart with `pixi run robot update_rate:=100`.
- All plans fail `START_STATE_IN_COLLISION`: obstacle estimates (T4) do not match reality -> stop, report.
- Anything unexpected: **e-stop first**, then Ctrl+C in terminal 1.

## D. Finish
Ctrl+C in terminal 1 (driver StopMotion + disconnect). Web UI: **manual mode + disable**.

## Result codes (error_code.val)
1 SUCCESS | -1 PLANNING_FAILED | -2 INVALID_MOTION_PLAN (path in collision) | -4 CONTROL_FAILED |
-7 PREEMPTED (cancel / new goal) | -16 INVALID_GOAL_CONSTRAINTS (bad input) | -23 ROBOT_STATE_STALE |
-25 COMMUNICATION_FAILURE | -27 GOAL_STATE_INVALID (unreachable / in collision)
