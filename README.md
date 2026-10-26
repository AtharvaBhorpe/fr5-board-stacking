# FR5 board stacking: motion, gripper and scene (branch `feat/robot-motion-gripper`)

ROS 2 Humble + MoveIt 2 control for the Fairino FR5 with the Schmalz vacuum gripper. This branch gives the
control loop a small, fixed API:

- **motion:** move the suction-cup tip (`tcp`) to a pose or by a delta, read the robot state, cancel at any time
  (smooth stop);
- **gripper:** grip with a timeout on the vacuum sensor, release with a blow-off pulse, state topic;
- **scene:** tell MoveIt about the picked board and the growing stack, so every plan avoids them.

**Only these nodes talk to MoveIt and the robot controller.** The control loop uses the ROS names below and
nothing else.

Status: everything works in simulation (tests in `tests/`). Real robot: motion and cancel tested (50 Hz);
the gripper and scene parts were tested in sim and against a mock of the controller, not yet on the robot
(see [Known limitations](#known-limitations)).

## Quick start

```bash
git clone --branch feat/robot-motion-gripper --recurse-submodules https://github.com/AtharvaBhorpe/fr5-board-stacking.git
cd fr5-board-stacking
pixi install          # ROS 2 Humble + MoveIt 2 from RoboStack (needs pixi, https://pixi.sh)
pixi run build        # colcon build incl. the Fairino driver (before the first build pixi warns
                      #   "Could not find activation scripts ... install/setup.sh": expected)
pixi run sim          # simulation: fake joints, fake gripper, RViz
pixi run shell        # 2nd terminal: command shell (mm / deg), type `help`
```

Real robot: `pixi run robot`. **First read `notes/test.md`** (network setup, web UI in automatic mode,
a person at the e-stop, first runs at <= 20 % speed).

Bringup arguments (`ros2 launch hk_fr5_moveit_config bringup.launch.py ...`, unknown names are rejected):

| Argument | Default | Meaning |
|---|---|---|
| `use_fake_hardware` | `true` | `false` = real FR5 (driver) and real gripper I/O |
| `update_rate` | `50` | controller rate in Hz (50 is required on the real FR5; 125 lagged 2 s) |
| `use_rviz` | `true` | `false` = headless |
| `use_obstacles` | `true` | static boxes from `obstacles.yaml` (table, profile, camera post, floor guard) |
| `use_motion_controller` | `true` | the `/motion/*` API |
| `use_gripper` | `true` | the `/gripper/*` API |

## ROS API

| Name | Type | Use |
|---|---|---|
| `/motion/move_to` | action `hk_fr5_interfaces/action/MoveTo` | absolute tcp pose or named pose (`ready`, `zero`) |
| `/motion/move_delta` | action `hk_fr5_interfaces/action/MoveDelta` | relative move, always a straight tcp line (Pilz LIN) |
| `/motion/get_state` | service `hk_fr5_interfaces/srv/GetState` | joints + tcp pose (xyz, quaternion, roll / pitch / yaw) |
| `/joint_states` | topic `sensor_msgs/JointState` | 50 Hz, from ros2_control |
| `/gripper/grip` | action `hk_fr5_interfaces/action/Grip` | vacuum on, wait for vacuum achieved (timeout) |
| `/gripper/release` | action `hk_fr5_interfaces/action/Release` | vacuum off + blow-off pulse |
| `/gripper/state` | topic `hk_fr5_interfaces/msg/GripperState` (latched) | pumps, vacuum sensor, state, 10 Hz + on change |
| `/scene/attach_board` | service `hk_fr5_interfaces/srv/AttachBoard` | picked board as a collision box under the cup |
| `/scene/detach_board` | service `hk_fr5_interfaces/srv/DetachBoard` | remove it, or keep it as stack obstacle `stack_<n>` |
| `/scene/add_box` | service `hk_fr5_interfaces/srv/AddBox` | dynamic obstacle (e.g. boards in the pick zone) |
| `/scene/remove_box` | service `hk_fr5_interfaces/srv/RemoveBox` | remove one / all dynamic obstacles |

The `.action` / `.srv` / `.msg` files in `src/hk_fr5_interfaces/` are the reference; every field is commented
there. Defaults: a `0` in `velocity_scaling`, `timeout`, `blow_off_time` or `contact_margin` means "use the
configured default".

### Conventions

- **Frame:** everything is in `base_link`, metres and radians. `base_link` is **rotated 180 deg about z** against the
  frame the teach pendant / web UI shows: `x = -x_pendant`, `y = -y_pendant`, `z = z_pendant` (pendant in mm).
- **tcp** = tip of the relaxed suction cup, 112.9 mm from the flange; its z-axis points out of the cup.
- **Orientation:** `MoveTo` default mode `MODE_DOWN_YAW`: tool pointing straight down, only `yaw` used
  (R = Rz(yaw) Rx(pi)). `MODE_FULL`: R = Rz(yaw) Ry(pitch) Rx(roll). `GetState.yaw` can be fed back as `MoveTo.yaw`.
  Home pose `ready`: tcp at (-0.497, -0.102, 0.364) pointing down, yaw +pi/2.
- **Deltas:** `frame = FRAME_TCP` (default) moves along the tool axes, so with the tool pointing down **`dz > 0`
  moves DOWN**; `FRAME_BASE` uses the base axes (`dz > 0` = up).
- **Planners:** `MoveTo.planner = ""` -> OMPL (avoids obstacles, any path shape); `"PTP"` / `"LIN"` -> Pilz
  (joint-space / straight line, no obstacle avoidance; a path through an obstacle is refused, not avoided).
- **Speed:** `velocity_scaling` in (0, 1], `0` -> 0.2; also used for the acceleration.
- **One goal at a time per node:** a new motion goal preempts the running one (`PREEMPTED`, smooth stop); the same
  for the gripper actions. Cancelling a motion goal stops the arm smoothly.
- `plan_only: true` plans and checks a move without moving.

### Result codes

Motion results carry `moveit_msgs/MoveItErrorCodes` (`error_code.val`) plus a readable `message`:

| Code | Name | Typical cause |
|---|---|---|
| 1 | SUCCESS | |
| -1 | PLANNING_FAILED | no path found (other target / planner) |
| -2 | INVALID_MOTION_PLAN | path in collision (an obstacle or the attached board in the way) |
| -4 | CONTROL_FAILED | controller did not execute |
| -7 | PREEMPTED | cancelled or replaced by a new goal |
| -16 | INVALID_GOAL_CONSTRAINTS | bad goal field (NaN, scaling, mode, unknown named target) |
| -23 | ROBOT_STATE_STALE | no recent `/joint_states` |
| -25 | COMMUNICATION_FAILURE | MoveIt not reachable |
| -26 | START_STATE_INVALID | OMPL from a start state in collision (e.g. board touching): lift with `move_delta` |
| -27 | GOAL_STATE_INVALID | target unreachable or in collision |

Gripper results carry `hk_fr5_interfaces/msg/GripperErrorCode`: 1 SUCCESS, 2 NO_VACUUM (timeout, pump switched
off), 3 CANCELLED, 4 PREEMPTED, 5 COMM_FAILURE, 6 CONTROLLER_ERROR, 7 INVALID_GOAL. `GripperState.state`: IDLE,
GRIPPING, HOLDING, RELEASING, PART_LOST (vacuum lost while holding, pump left on), COMM_ERROR (clears by itself
when the controller answers again), CONTROLLER_ERROR.

Scene services return `success` + `message`. After an attach / add the scene node checks the robot state and
appends `WARNING: robot state now in collision: <a> <-> <b>` to the message if something touches.

## Pick and place: the sequence and its rules

```
move_to above the board (OMPL)  ->  move_delta down onto the board (LIN)  ->  grip
  -> success: [remove_box of that board if you added it]  ->  attach_board(L, W, H)  ->  move_delta up (LIN)
     -> move_to above the place spot (OMPL, avoids obstacles with the board attached)
     -> move_delta down (LIN) to tcp z = stack top + board height  ->  release  ->  detach_board(keep_as_obstacle=true)
     -> move_delta up (LIN)
  -> NO_VACUUM: pump is already off; lift, retry or report
```

Rules (each from a measured problem, details in `notes/decision-log.md` 5n):

1. **Attach only after the grip reports success** (vacuum achieved).
2. **If the board was added with `add_box`, remove that box before `attach_board`.** Otherwise the attached board
   overlaps it and every move fails (START_STATE_IN_COLLISION / -26).
3. **Lift right after attach with `move_delta` (Pilz LIN), not with an OMPL `move_to`** from the contact pose.
4. Place height: tcp z = surface top + board height. The attached board is modelled 2 mm short at its bottom
   (`contact_margin`), so it can rest on a surface without counting as a collision.
5. `detach_board(keep_as_obstacle=true)` turns the placed board into `stack_<n>` (returned in `name`) at its real
   top height, so the next place goes on top of it. Default `false` removes the board. `remove_box("")` clears all
   dynamic boxes incl. the stack; the static workcell boxes cannot be removed.

### Minimal Python client (rclpy)

```python
import math
import rclpy
from rclpy.action import ActionClient
from hk_fr5_interfaces.action import Grip, MoveDelta, MoveTo, Release
from hk_fr5_interfaces.srv import AttachBoard, DetachBoard

rclpy.init()
node = rclpy.create_node("pick_demo")

def act(client, goal):                      # send an action goal, wait for its result
    client.wait_for_server()
    sent = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent)
    done = sent.result().get_result_async()
    rclpy.spin_until_future_complete(node, done)
    return done.result().result

def call(client, request):                  # call a service, wait for the response
    client.wait_for_service()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    return future.result()

move_to = ActionClient(node, MoveTo, "/motion/move_to")
move_delta = ActionClient(node, MoveDelta, "/motion/move_delta")
grip = ActionClient(node, Grip, "/gripper/grip")
release = ActionClient(node, Release, "/gripper/release")
attach = node.create_client(AttachBoard, "/scene/attach_board")
detach = node.create_client(DetachBoard, "/scene/detach_board")

up = lambda d: MoveDelta.Goal(dz=d, frame=MoveDelta.Goal.FRAME_BASE)   # base frame: dz > 0 = up
table_top, board_h = -0.045, 0.006                                     # estimate, see notes (T4)

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
print(r.success, r.message)
node.destroy_node()
rclpy.shutdown()
```

Every result has `success` and `message`; check them after each step (the example only prints the last one).

## Packages and configuration

| Package | Contents |
|---|---|
| `hk_fr5_interfaces` | MoveTo, MoveDelta, GetState, Grip, Release, GripperState, GripperErrorCode, AttachBoard, DetachBoard, AddBox, RemoveBox |
| `hk_fr5_description` | URDF/xacro: FR5 + D405 mount + D405 + gripper + `tcp`; `config/tool_geometry.yaml` |
| `hk_fr5_moveit_config` | own SRDF (group `arm`, chain `base_link -> tcp`), limits, OMPL + Pilz, controllers, `launch/bringup.launch.py` |
| `hk_fr5_motion` | `motion_controller`, `scene_manager`, `motion_shell`; `config/motion.yaml`, `scene.yaml`, `obstacles.yaml` |
| `hk_fr5_gripper` | `gripper_controller` (tool DO0 vacuum pump, DO1 blow-off, DI0 vacuum achieved, via the controller's XML-RPC); `config/gripper.yaml` |

Vendor code: `external/frcobot_ros2` (git submodule, never edited); only `fairino_description`, `fairino_msgs`
and the driver `fairino_hardware_v3_9_0_1` are built. All IPs, I/O numbers, times and geometry are in the yaml
files above, not in code. `obstacles.yaml` holds **estimated** workcell boxes (table top z = -0.045).

## Known limitations

- Real robot not yet tested for: the gripper (XML-RPC reply format of `GetToolDI` / `GetToolDO` is assumed,
  vacuum timing), the gripper polling next to the motion driver, the scene services with the real table height,
  the roll / pitch / yaw convention of `get_state` vs the pendant (expected: roll = rx, pitch = ry,
  yaw = rz +- 180 deg).
- The workcell boxes and the table height are estimates; a wrong table height shows up as a collision warning
  on attach (see `notes/decision-log.md` 5n-1, E-1 .. E-7).
- If the robot connection drops, the vendor driver keeps publishing the last joint values (no stale detection).
- Ctrl+C of the bringup: several nodes report "process has died (-2)" because ros2 launch sends a second SIGINT;
  the gripper still switches both pumps off. On the real robot check that the driver stops cleanly.
- The vendor driver has the controller IP 192.168.58.2 hard-coded.

## More

- `notes/decision-log.md`: every decision with options and reasons (motion 5a-5k, gripper 5l, review 5m, scene 5n),
  open items (section 7).
- `notes/implementation-ref.md`: vendor / library facts, quirks, test results.
- `notes/test.md`: manual test steps for sim and the real robot (CLI and `motion_shell`).
- `tests/README.md`: automated checks (sim / mock) and how to run them.
