> [!CAUTION]
> # 🛑 FROZEN BRANCH: DO NOT CHANGE ANYTHING HERE 🛑
>
> **`feat/robot-motion-gripper` is completely tested on the real FR5 robot (2026-10-27) and is FROZEN.**
>
> **No code, config, URDF / geometry, MoveIt, gripper or scene value in this branch may be changed** - in particular
> not `src/hk_fr5_description/config/tool_geometry.yaml` (`tcp_z`, D405 mount yaw: measured on the real robot),
> `src/hk_fr5_motion/config/obstacles.yaml` (workcell, table top), the MoveIt config, the joint limits or
> `src/hk_fr5_gripper/config/gripper.yaml`.
>
> - **Humans and AI agents: read-only.** Do not commit, push, rebase, merge into, reformat or "fix" anything on this
>   branch, not even a typo, a test or a comment.
> - **Use it as a dependency only:** check it out, `pixi run build`, `pixi run robot` / `pixi run sim`, and call its ROS
>   API (`/motion/*`, `/gripper/*`, `/scene/*`) from YOUR branch.
> - **Need a different value or behaviour?** Do not change it here. Put the override in your own branch / launch
>   file, or ask the branch owner (GitHub: vortexcv) first.
> - Frozen state: commit `df9f226` (the teammate's real-robot measurements) + the commit that added this notice.

# FR5 board stacking: motion, gripper, scene (branch `feat/robot-motion-gripper`)

ROS 2 Humble + MoveIt 2 nodes for the Fairino FR5 with a Schmalz vacuum gripper. They give a control loop
(script, state machine or LLM / VLM agent) a small, fixed API:

- **motion:** move the suction-cup tip (`tcp`) to a pose or by a delta, read the state, cancel any time (smooth stop);
- **gripper:** grip (vacuum on, wait for the vacuum sensor), release (vacuum off + blow-off pulse), state topic;
- **scene:** tell MoveIt about the held board and the growing stack, so every plan avoids them.

**Only these nodes talk to MoveIt and the robot controller.** Use the ROS names below and nothing else.

Status: **tested on the real robot and frozen (2026-10-27)**: motion + cancel at 50 Hz, the gripper node (grip /
release with real boards), the scene API, with the measured tool geometry (`tcp_z` 0.0979, D405 mount yaw -90 deg,
decision-log H-12). Simulation tests: `tests/`. Remaining notes: [Limitations](#limitations).

## Quick start

```bash
git clone --branch feat/robot-motion-gripper --recurse-submodules https://github.com/AtharvaBhorpe/fr5-board-stacking.git
cd fr5-board-stacking
pixi install          # ROS 2 Humble + MoveIt 2 (RoboStack); needs https://pixi.sh
pixi run build        # first build warns "Could not find activation scripts ... install/setup.sh": expected
pixi run sim          # simulation: fake arm + fake gripper + RViz
pixi run shell        # 2nd terminal: command shell (mm / deg), type `help`
pixi run python3 my_loop.py   # your own node: pixi run sources the workspace
```

**Real robot: `pixi run robot`, but first read `notes/test.md`** (network, web UI in automatic mode, a person
at the e-stop, first runs at `velocity_scaling` <= 0.2). The gripper node switches **both pumps off at start**.

Bringup arguments (`ros2 launch hk_fr5_moveit_config bringup.launch.py name:=value`; unknown names are rejected):

| Argument | Default | Meaning |
|---|---|---|
| `use_fake_hardware` | `true` | `false` = real FR5 driver + real gripper I/O |
| `update_rate` | `50` | controller Hz; keep 50 on the real FR5 (125 lagged 2 s) |
| `use_rviz` / `use_obstacles` | `true` | RViz / static workcell boxes from `obstacles.yaml` |
| `use_motion_controller` / `use_gripper` | `true` | `/motion/*` / `/gripper/*` nodes |

## ROS API

The `.action` / `.srv` / `.msg` files in `src/hk_fr5_interfaces/` are the reference (every field commented).
A `0` in `velocity_scaling`, `timeout`, `blow_off_time` or `contact_margin` = configured default.

| Name | Type | Does |
|---|---|---|
| `/motion/move_to` | action `MoveTo` | absolute tcp pose (x, y, z, yaw [, roll, pitch]) or `named_target` `ready` |
| `/motion/move_delta` | action `MoveDelta` | relative move dx..dyaw, always a straight tcp line (Pilz LIN) |
| `/motion/get_state` | service `GetState` | joints + tcp pose (xyz, quaternion, roll / pitch / yaw) |
| `/joint_states` | topic `sensor_msgs/JointState` | 50 Hz, from ros2_control |
| `/gripper/grip` | action `Grip` | vacuum on, wait up to `timeout` (default 3 s) for vacuum achieved |
| `/gripper/release` | action `Release` | vacuum off, blow-off for `blow_off_time` (default 1 s) |
| `/gripper/state` | topic `GripperState` (latched, 10 Hz + on change) | `state`, `vacuum_on`, `blow_off_on`, `vacuum_achieved`, `message` |
| `/scene/attach_board` | service `AttachBoard` | held board (`length`, `width`, `height`, opt. `yaw`, `dx`, `dy`) as a box under the cup |
| `/scene/detach_board` | service `DetachBoard` | remove it, or `keep_as_obstacle: true` -> fixed box `stack_<n>` (in `name`) |
| `/scene/add_box` | service `AddBox` | dynamic box (`name`, `size`, `position`, `yaw`), e.g. boards waiting in the pick zone |
| `/scene/remove_box` | service `RemoveBox` | remove one dynamic box; `""` = all (incl. `stack_<n>`) |

All types are in package `hk_fr5_interfaces` (`action/`, `srv/`, `msg/`).

## Conventions

- **Frame `base_link`, metres, radians.** It is **rotated 180 deg about z** vs. the pendant / web UI:
  `x = -x_pendant`, `y = -y_pendant`, `z = z_pendant` (pendant in mm).
- **tcp** = tip of the relaxed suction cup (112.9 mm from the flange); tcp z-axis points out of the cup.
- **Orientation:** `MODE_DOWN_YAW` (default): tool straight down, only `yaw` used, R = Rz(yaw) Rx(pi).
  `MODE_FULL`: R = Rz(yaw) Ry(pitch) Rx(roll). `GetState.yaw` can be fed back as `MoveTo.yaw`.
  Home `ready`: tcp (-0.497, -0.102, 0.364), tool down, yaw +pi/2.
- **Deltas:** `FRAME_TCP` (default) = tool axes, so with the tool down **`dz > 0` moves DOWN**; `FRAME_BASE` = base
  axes, `dz > 0` = up.
- **Planners (`MoveTo.planner`):** `""` = OMPL, avoids obstacles; `"PTP"` / `"LIN"` = Pilz, no avoidance (a path through
  an obstacle is refused, not avoided). `move_delta` is always LIN.
- **Speed:** `velocity_scaling` in (0, 1], 0 -> 0.2, also scales acceleration. `plan_only: true` = plan + check, no motion.
- **One goal at a time per node:** a new goal preempts the running one (motion stops smoothly). Cancel = smooth stop.
- **Workcell:** table top z = -0.060 (measured); static boxes `table`, `profile`, `camera_post`, `floor_guard`.

## Result codes and what to do

Every result has `success` + `message` (readable reason). Motion: `error_code.val` = `moveit_msgs/MoveItErrorCodes`.

| Code | Name | Cause | Do |
|---|---|---|---|
| 1 | SUCCESS | | |
| -1 | PLANNING_FAILED | no path found | other target, or `planner: ""` (OMPL) |
| -2 | INVALID_MOTION_PLAN | path in collision | OMPL instead of PTP / LIN; lift first |
| -4 | CONTROL_FAILED | controller did not execute | stop; check robot (alarm, e-stop, auto mode) |
| -7 | PREEMPTED | cancelled / replaced by a new goal | expected if you did it |
| -10 | START_STATE_IN_COLLISION | robot already touches something | check `/scene` boxes vs. reality, lift with `move_delta` |
| -16 | INVALID_GOAL_CONSTRAINTS | bad field (NaN, scaling, mode, unknown name) | fix the goal |
| -23 / -25 | ROBOT_STATE_STALE / COMMUNICATION_FAILURE | no `/joint_states` / MoveIt down | stop; check the bringup |
| -26 | START_STATE_INVALID | OMPL from a contact pose (board touching) | lift with `move_delta` first (rule 3) |
| -12 / -27 / -31 | GOAL_IN_COLLISION / GOAL_STATE_INVALID / NO_IK_SOLUTION | target in collision or unreachable | other target |

Gripper: `error_code.val` = `GripperErrorCode`.

| Code | Name | Do |
|---|---|---|
| 1 | SUCCESS | grip: board held (state HOLDING) |
| 2 | NO_VACUUM | pump already off; adjust position, retry or report |
| 3 / 4 | CANCELLED / PREEMPTED | expected if you did it (cancelled grip: pump off) |
| 5 | COMM_FAILURE | controller unreachable, or the node's start reset not done yet; retry later |
| 6 | CONTROLLER_ERROR | controller returned an error code (in `message`); stop, check the web UI |
| 7 | INVALID_GOAL | negative / NaN time |

`GripperState.state`: IDLE, GRIPPING, HOLDING, RELEASING, **PART_LOST** (vacuum lost while holding, pump left on:
the board probably fell), COMM_ERROR (clears by itself), CONTROLLER_ERROR (a new goal clears it).
Scene services: `success` + `message`; after attach / add_box a contact is reported as
`WARNING: robot state now in collision: <a> <-> <b>` (success stays true).

## Pick and place

```
move_to above board (OMPL) -> move_delta down to tcp z = board top (LIN) -> grip
  SUCCESS:   [remove_box that board if you added it] -> attach_board(L, W, H) -> move_delta up (LIN)
             -> move_to above place spot (OMPL) -> move_delta down to tcp z = surface top + H (LIN)
             -> release -> detach_board(keep_as_obstacle=true) -> move_delta up (LIN)
  NO_VACUUM: pump is off; lift, adjust, retry or report
```

Rules (each from a measured problem; details `notes/decision-log.md` 5n, 5o):

1. **Attach only after grip SUCCESS** (vacuum achieved).
2. **A board added with `add_box` must be removed before `attach_board`**, else every move fails (-10 / -26).
3. **Lift right after attach with `move_delta` (LIN)**, not with an OMPL `move_to` from the contact pose.
4. Heights: board top = surface top + H (table top -0.060). The attached box is 2 mm short at its bottom
   (`contact_margin`), so a board resting on a surface is not a collision.
5. `keep_as_obstacle=true` turns the placed board into `stack_<n>` at its real height: the next board goes on top.
   Default `false` removes it. `remove_box("")` clears all dynamic boxes; static boxes cannot be removed.
6. Vacuum and blow-off pump are never on together (node interlock); `grip` / `release` handle the order.
7. `time_to_vacuum` varies a lot per pick: log it, don't decide on it.

### Minimal rclpy client

```python
import math
import rclpy
from rclpy.action import ActionClient
from hk_fr5_interfaces.action import Grip, MoveDelta, MoveTo, Release
from hk_fr5_interfaces.srv import AttachBoard, DetachBoard

rclpy.init()
node = rclpy.create_node("pick_demo")
def wait(f): rclpy.spin_until_future_complete(node, f); return f.result()
def act(c, goal): c.wait_for_server(); return wait(wait(c.send_goal_async(goal)).get_result_async()).result
def call(c, req): c.wait_for_service(); return wait(c.call_async(req))

move_to = ActionClient(node, MoveTo, "/motion/move_to")
move_delta = ActionClient(node, MoveDelta, "/motion/move_delta")
grip = ActionClient(node, Grip, "/gripper/grip")
release = ActionClient(node, Release, "/gripper/release")
attach = node.create_client(AttachBoard, "/scene/attach_board")
detach = node.create_client(DetachBoard, "/scene/detach_board")

up = lambda d: MoveDelta.Goal(dz=d, frame=MoveDelta.Goal.FRAME_BASE)   # base frame: dz > 0 = up
table_top, board_h = -0.060, 0.006

r = act(move_to, MoveTo.Goal(x=-0.45, y=-0.25, z=0.10, yaw=math.pi / 2))   # above the board
r = act(move_delta, up(table_top + board_h - 0.10))                         # down onto it (LIN)
g = act(grip, Grip.Goal())                                                  # default timeout 3 s
if g.success:
    call(attach, AttachBoard.Request(length=0.27, width=0.20, height=board_h))
    r = act(move_delta, up(0.10))                                           # lift with LIN (rule 3)
    r = act(move_to, MoveTo.Goal(x=-0.45, y=0.25, z=0.10, yaw=math.pi / 2))
    r = act(move_delta, up(table_top + board_h - 0.10))                     # place on the table
    act(release, Release.Goal())
    d = call(detach, DetachBoard.Request(keep_as_obstacle=True))
    print("placed, obstacle", d.name)
    act(move_delta, up(0.10))
print(r.success, r.message)                 # check success / message after EVERY step in real code
rclpy.shutdown()
```

## Packages and configuration

| Package | Contents (config in `config/*.yaml`: IPs, I/O, times, geometry; nothing hard-coded) |
|---|---|
| `hk_fr5_interfaces` | all actions / services / messages above |
| `hk_fr5_description` | URDF/xacro: FR5 + D405 mount + D405 + gripper + `tcp` (`tool_geometry.yaml`) |
| `hk_fr5_moveit_config` | SRDF (group `arm`, `base_link -> tcp`), limits, OMPL + Pilz, controllers, `bringup.launch.py` |
| `hk_fr5_motion` | `motion_controller` (`motion.yaml`), `scene_manager` (`scene.yaml`, `obstacles.yaml`), `motion_shell` |
| `hk_fr5_gripper` | `gripper_controller`: tool DO0 vacuum, DO1 blow-off, DI0 vacuum achieved via XML-RPC (`gripper.yaml`) |

Vendor: `external/frcobot_ros2` (submodule, never edited; `fairino_description`, `fairino_msgs`, driver `fairino_hardware_v3_9_0_1`).

## Limitations

- Tested on the real robot (2026-10-27, H-12): motion, cancel, gripper node (grip / release with boards), scene API.
  Not measured separately: motion lag while the gripper polls (T17), `get_state` roll / pitch / yaw vs. pendant
  (expected roll = rx, pitch = ry, yaw = rz +- 180 deg).
- `/gripper/state` pump values = last commanded: no DO read-back (`GetToolDO` crashed the controller).
- Table x/y and camera post box are estimates (OK for the demo); a wrong table shows as a collision warning on attach.
- Robot connection lost: the vendor driver keeps publishing the last joint values (no stale detection).
- Ctrl+C of the bringup: some nodes report "process has died (-2)"; the gripper still switches both pumps off.
- The vendor driver has the controller IP 192.168.58.2 hard-coded.

## More

`notes/decision-log.md` (decisions with options and reasons, open items in §7), `notes/implementation-ref.md`
(vendor facts, quirks, test results), `notes/test.md` (manual sim / real-robot steps, lab checklist),
`tests/README.md` (automated checks and how to run them).
