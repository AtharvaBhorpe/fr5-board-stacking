# FR5 Hackathon: Implementation Reference (steps 10+)

Committed in the branch under `notes/` (R10). Companion to the local `CLAUDE.md` (rules, state) and
`decision-log.md` (history of decisions). This file holds **technical
findings** needed to build steps 10+: vendor/library facts, quirks, better
options found, and working rules. Each item says where it was verified.
Add to it as we go; mark items DONE/OBSOLETE instead of deleting.

Sections
1. Working rules learned
2. Fairino MoveIt config (`fairino5_v6_moveit2_config`): what it has, what we change
3. Fairino driver (`fairino_hardware_v3_9_0_1`): facts and quirks
4. MoveIt 2.5.10 / `moveit_configs_utils` behaviour
5. ros2_control 2.54 / JTC behaviour
6. Pilz and OMPL specifics
7. IK solver options
8. Kinematics facts (FK check, poses)
9. Teammate setup (reference only)
10. Step 10 proposal (DONE, decided in decision-log 5b) + per-step findings / results 10a-10n
11. Later-step reminders

---------------------------------------------------------------------------

## 1. Working rules learned

- R-1: Scan vendor/library source before designing; record quirks here first.
- R-2: Verify numbers independently (mesh bbox, FK, generated URDF) instead of
  trusting docs or teammate values.
- R-3: Prefer our own small code over patching/monkey-patching vendor code
  once a vendor component shows more than one incompatibility (D-k precedent).
- R-4: Claude's sandbox shell has flaky DDS discovery: GUI/topic tests there
  are not conclusive. Feed data directly in tests, or let the user verify on
  their terminal. Long offscreen tests: user prefers to check directly.
  **Correction (step 15):** the step 9 'no /joint_states' result was most likely MY test bug:
  `robot_state_publisher -p robot_description:="$(cat urdf)"` crashes ("Couldn't parse parameter
  override rule"); a whole URDF cannot go through `-p`. Use a params file
  (`/**: ros__parameters: robot_description: ...`) via `--params-file`. Always check the logs of
  every test process before blaming DDS.
  **Fix found 2026-10-25:** `export ROS_LOCALHOST_ONLY=1` (+ own `ROS_DOMAIN_ID`)
  makes cross-process discovery reliable in the sandbox (`ros2 param get`, `ros2 control list_controllers` worked).
- R-5: `pkill -f <pattern>` inside a Bash call can kill the calling shell
  (pattern is in its own command line). Kill by PID.
- R-5b: stopping a test `ros2 launch` (SIGINT/SIGKILL to the launch PID) can leave its
  child nodes running (orphans). After every runtime test: `ps -eo pid,args | grep -E "[m]ove_group|[r]os2_control_node|..."`
  and kill the PIDs. Always use an own `ROS_DOMAIN_ID` + `ROS_LOCALHOST_ONLY=1` so leftovers never reach the user's ROS graph.
  Reliable check: `ps -eo pid,args | grep -E "move_group|ros2_control_node|robot_state_publisher --ros|controller_manager/spawner|ros2 launch" | grep -v grep`
  (the `[x]yz` bracket trick missed processes once). Kill the `ros2 launch` PID with SIGINT first, wait ~4 s, then SIGKILL survivors.
- R-6: History rewrite only before a push; the pushed branch is never rewritten/force-pushed.
- R-7: Never edit `external/`. If a vendor build writes into `external/`,
  document it and restore with `git -C external/frcobot_ros2 checkout .` when needed.

---------------------------------------------------------------------------

## 2. Fairino MoveIt config `external/frcobot_ros2/fairino5_v6_moveit2_config` (read 2026-10-25)

Files: `config/{fairino5_v6_robot.srdf, .ros2_control.xacro, .urdf.xacro,
initial_positions.yaml, joint_limits.yaml, kinematics.yaml,
moveit_controllers.yaml, pilz_cartesian_limits.yaml, ros2_controllers.yaml,
moveit.rviz}`, `launch/*` (MSA-generated, `generate_*_launch` one-liners),
`.setup_assistant`. No `ompl_planning.yaml` (falls back to moveit_configs_utils defaults).

| Item | Vendor value | Issue for us | Our plan |
|---|---|---|---|
| SRDF group | `fairino5_v6_group`, joints j1-j6 + chain `base_link -> wrist3_link` | tip is not our TCP (F3) | own group `arm`, chain `base_link -> tcp` |
| SRDF group states | `pos1`, `pos2` (arbitrary joint sets) | not meaningful for us | own `zero`, `ready` (sec. 8) |
| SRDF disable_collisions | 11 pairs, arm links only (Adjacent / Never, MSA-sampled) | no tool links | reuse the 11 + tool pairs (sec. 10) |
| kinematics.yaml | KDL, search res 0.005, **timeout 0.005 s** | short timeout -> IK failures for pose goals | KDL, timeout 0.05 s |
| joint_limits.yaml | vel = URDF (3.15 / 3.2 rad/s), **acc 0.7 rad/s^2 all joints**, default scaling 0.1/0.1 | acc conservative (unknown real max) | keep acc 0.7 for now, default scaling 0.2 (F14) |
| moveit_controllers.yaml | controller `fairino5_controller`, FollowJointTrajectory | **duplicate YAML keys** (`action_ns`, `default` twice) | own clean file |
| ros2_controllers.yaml | update_rate 100 Hz, JTC position cmd + position state, JSB | 100 Hz vs driver ServoJ cmdT 8 ms (125 Hz), see sec. 3 | decide (sec. 10) |
| pilz_cartesian_limits.yaml | trans vel 1.0 m/s, acc 2.25, dec -5.0, rot vel 1.57 rad/s | fine | keep values |
| ros2_control.xacro | mock only (`mock_components/GenericSystem`), initial_value from yaml | no real plugin switch | own xacro with `use_fake_hardware` arg |
| package.xml | depends on warehouse_ros_mongo, setup_assistant, jsp_gui | heavy, we dropped those | own minimal deps |

---------------------------------------------------------------------------

## 3. Fairino driver `fairino_hardware_v3_9_0_1` (read 2026-10-25)

Plugin: `fairino_hardware/FairinoHardwareInterface` (SystemInterface), SDK `libfairino.so.2.3.0`.

Facts:
- **IP hard-coded**: `#define CONTROLLER_IP_ADDRESS "192.168.58.2"` in
  `fairino_hardware_interface.hpp`; no `<hardware><param>` read. Our config
  cannot change it. Matches our robot. If the IP ever changes -> vendor patch
  needed (would break "never edit external/", discuss then). Document the IP in our yaml anyway (for gripper XML-RPC).
- Interfaces: **exactly 1 command interface `position` and exactly 1 state
  interface `position` per joint**, else `on_init` FATAL. -> our ros2_control
  xacro must not add velocity/effort interfaces. (`<param initial_value>` is fine; ignored by driver.)
- `on_activate`: `RPC(ip)`, sleep 200 ms, blocking `GetActualJointPosDegree(0)`,
  copies actual joints into the command buffer (safe start: holds current pose).
  Fails activation if the read fails.
- `read()`: non-blocking `GetActualJointPosDegree(1)`, deg -> rad.
- `write()`: every cycle `ServoJ(cmd_deg, ext, acc=0, vel=0, cmdT=0.008, filterT=0, gain=0)`.
  SDK doc: cmdT = command period in s ("recommended [0.001~0.0016]", likely typo for 0.016).
  So the driver assumes an **8 ms (125 Hz)** command period.
- `on_deactivate`: `StopMotion()`, `CloseRPC()`.
- Units: SDK degrees, ROS radians (converted in driver).

Quirks / bugs (vendor, not fixed by us):
- Q3-1 `read()` error branch is a no-op statement (`hardware_interface::return_type::ERROR;`
  without `return`): read failures are silently ignored, state stays stale, returns OK.
- Q3-2 `write()` NaN check `std::any_of(&cmd[0], &cmd[5], ...)` checks only j1-j5 (end pointer off by one).
- Q3-3 `on_deactivate` uses `_ptr_robot.release()` (leaks the SDK object). Harmless.
- Q3-4 ServoJ is streamed every cycle even when no trajectory runs (JTC holds last command).
- Q3-5 Build side effect: CMake `file(CREATE_LINK ...)` turns the **tracked** regular files
  `libfairino/lib/libfairino.so` and `.so.2` into symlinks **inside the source tree**
  -> modifies the submodule working tree (hidden by `ignore = dirty`).
- Q3-6 Library path: `fairino_hardware` links with rpath `$ORIGIN` but lives in
  `install/lib/fairino_hardware_v3_9_0_1/` while `libfairino.so.2.3.0` is copied to
  `install/lib/`; the teammate added `LD_LIBRARY_PATH=.../libfairino/lib` in pixi activation.
- Q3-7 Build needs `CXXFLAGS="-include unistd.h"` (already in our pixi task, E6).
- Q3-8 `ros2_cmd_server` (same package) conflicts with the driver (single realtime client).
- Comments/log messages are Chinese.

Throwaway compile result (2026-10-25, M-10): `fairino_msgs` + driver built in
scratch build/install/log dirs (`colcon --log-base X build --build-base Y --install-base Z`),
**OK in 23 s**, our build/install untouched.
- 128 compiler warnings, 117 in `command_server.cpp` (cmd server, not used).
  In the plugin `fairino_hardware_interface.cpp`: unused params, multi-line
  comments, and line 173 `statement has no effect` (= Q3-1 confirmed).
  `command_server.cpp` has 2 `-Wreturn-type` (missing return): UB in the cmd server, another reason not to run it.
- fairino_msgs: only CMake dev warnings (CMP0148 FindPythonInterp), harmless.
- Q3-5 confirmed: submodule showed `T` (typechange) for `libfairino.so`, `libfairino.so.2`.
  Extra finding: the tracked files are **symlinks committed as plain text files**
  containing an absolute path from a Fairino developer machine
  (`/home/fairino/AAAmydata/.../libfairino.so.2`). CMake recreates them as real symlinks.
  Revert: `git -C external/frcobot_ros2 checkout -- fairino_hardware_v3_9_0_1/libfairino/lib` -> clean.
- Q3-6 confirmed: `ldd libfairino_hardware.so` -> `libfairino.so.2 => not found`.
  Install has only `install/fairino_hardware_v3_9_0_1/lib/libfairino.so.2.3.0` (no `.so.2`
  soname link). Works with `LD_LIBRARY_PATH=<ws>/external/frcobot_ros2/fairino_hardware_v3_9_0_1/libfairino/lib`
  (soname link exists there after the CMake step) -> teammate's activation fix is needed in step 19.

---------------------------------------------------------------------------

## 4. MoveIt 2.5.10 / `moveit_configs_utils` (read 2026-10-25, env `.pixi/.../moveit_configs_utils`)

- `MoveItConfigsBuilder(robot_name, package_name=...)`: without `.setup_assistant`
  it warns "Cannot infer URDF/SRDF" and assumes `config/<robot_name>.urdf` / `.srdf`.
  Passing explicit `robot_description(file_path=..., mappings=...)` and
  `robot_description_semantic(file_path=...)` avoids relying on that.
  Also looks for `config/<robot_name>.urdf.xacro` (MSA "modified urdf") automatically.
- Default file names: `kinematics.yaml`, `joint_limits.yaml`, `*_planning.yaml`,
  `pilz_cartesian_limits.yaml`, `*_controllers.yaml` (prefers `moveit_controllers.yaml`).
- `planning_pipelines()` default `load_all=True`: also loads **CHOMP** (and OMPL/Pilz)
  from `moveit_configs_utils/default_configs/`. Use
  `.planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])` to load only ours.
- Default pipeline = `ompl` if present.
- Pilz Cartesian limits are loaded **only if the pipeline is named exactly
  `pilz_industrial_motion_planner`** (key check in `to_moveit_configs`).
- OMPL without `planner_configs` -> `ompl_defaults.yaml` added (all planners).
  No per-group `default_planner_config` -> OMPL picks its own default
  (RRTConnect for this case); set it explicitly for group `arm`.
- Default OMPL request adapters: AddTimeOptimalParameterization, ResolveConstraintFrames,
  FixWorkspaceBounds, FixStartStateBounds, FixStartStateCollision, FixStartStatePathConstraints.
  Pilz default: no adapters (time-parameterises itself), `default_planner_config: PTP`.
- `/move_action` request: `pipeline_id` = `ompl` | `pilz_industrial_motion_planner`;
  `planner_id` = `RRTConnect` (OMPL config name) | `PTP` | `LIN` (Pilz).
- `generate_demo_launch()` includes `launch/*.launch.py` **of the config package**
  (rsp, move_group, moveit_rviz, warehouse_db, spawn_controllers, static_virtual_joint_tfs)
  -> needs those files and warehouse. Use the individual `generate_*_launch()` helpers
  (`generate_rsp_launch`, `generate_move_group_launch`, `generate_moveit_rviz_launch`,
  `generate_spawn_controllers_launch`) like the teammate did.
- `generate_moveit_rviz_launch` default rviz config = `config/moveit.rviz` of the package.
- `generate_spawn_controllers_launch` spawns `moveit_simple_controller_manager.controller_names` + `joint_state_broadcaster`.
- move_group gets `publish_robot_description_semantic: True`, monitored planning scene publishing on.
- RViz MotionPlanning: without SRDF `end_effector`, the interactive marker is
  placed at the chain tip (tcp) when the group has an IK solver. (Step 11: user RViz check passed.)

---------------------------------------------------------------------------

## 5. ros2_control 2.54 / JTC (verified from env binaries/headers 2026-10-25)

- controller_manager prints `[Deprecated] Passing the robot description parameter
  directly to the control_manager node is deprecated. Use '~/robot_description'
  topic from 'robot_state_publisher' instead.` -> vendor + teammate launch pass it
  as a parameter. Better: remap `~/robot_description` -> `/robot_description`.
- Deprecated CM params: `configure_components_on_start`, `activate_components_on_start`.
- JTC defaults (header): `action_monitor_rate 20.0`, `interpolation_method "splines"`,
  `allow_nonzero_velocity_at_trajectory_end true`,
  `set_last_command_interface_value_as_state_on_activation true`.
  Other params: `constraints.goal_time`, `constraints.stopped_velocity_tolerance`,
  `open_loop_control`, `state_publish_rate`, `cmd_timeout`, `allow_partial_joints_goal`.
- JTC with position-only state interface is allowed (driver exports only position).

---------------------------------------------------------------------------

## 6. Pilz / OMPL specifics (verified from env libraries 2026-10-25)

- Pilz needs **acceleration limits for every joint** (`robot_description_planning.joint_limits.<j>.max_acceleration`, `has_acceleration_limits: true`).
- Deceleration optional; if given **must be negative** ("joint_limit.max_deceleration MUST be negative!").
  If not set: logs "deceleration limit not set", uses -acceleration.
- Cartesian limits keys: `cartesian_limits: {max_trans_vel, max_trans_acc, max_trans_dec, max_rot_vel}`;
  rotational acc/dec are deprecated, derived from translational ratio.
- Pilz LIN/PTP Cartesian goals use IK of the group: tip must be the IK tip ->
  our chain to `tcp` makes `tcp` goals direct (F3 pays off).
- Pilz does not avoid obstacles (it only checks the computed path); OMPL does (F6).

---------------------------------------------------------------------------

## 7. IK solver options (checked 2026-10-25)

| Solver | Available | Notes |
|---|---|---|
| KDL `kdl_kinematics_plugin/KDLKinematicsPlugin` | yes (installed) | numeric, fine for 6-DOF; needs adequate timeout |
| TRAC-IK | **no** (not in RoboStack Humble) | - |
| pick_ik `ros-humble-pick-ik` 1.1.2 | yes (needs `pixi add`) | more robust numeric/global; extra dep |
| LMA, cached KDL | yes (installed) | no advantage here |

pick_ik package check (2026-10-25): `ros-humble-pick-ik 1.1.2` (build `_22`, same
RoboStack batch as MoveIt 2.5.10); deps all installed except `range-v3`
(header-only). Low ABI risk.

How IK is used by our system (matters for the choice):
- OMPL pose goal (`move_to`): goal sampler calls IK many times; needs a high
  success rate, any valid solution is OK (OMPL then plans the joint path).
- Pilz LIN (`move_delta`, optional `move_to`): IK at every waypoint, seeded with
  the previous one; needs **continuity** (no jump to another arm configuration),
  otherwise Pilz rejects the trajectory (joint velocity/jump checks).
- Pilz PTP with pose goal: one IK call, seed = current state.
- RViz interactive marker: IK on every drag.
- FR5 has no analytic IK plugin for MoveIt (IKFast would need OpenRAVE: out).
Decision (M-6): KDL. Benchmark vs pick_ik is OPTIONAL (decision-log T8), only if
KDL fails noticeably; not part of the step plan.

---------------------------------------------------------------------------

## 8. Kinematics facts (own FK from generated URDF, 2026-10-25)

- **HOME pose = SRDF `ready`** (decision M-14). Real robot starts at its actual joints (M-15).

- FK script reproduces RViz/tf2_echo: zero joints -> tcp `[-0.820, -0.315, 0.050]`, tool along -y.
- Candidate `ready` = `[0, -90, 90, -90, -90, 0]` deg -> tcp `[-0.497, -0.102, 0.364]`,
  **tcp z-axis = (0, 0, -1)** (pointing straight down), above the table side
  (table centre x ~ -0.42 per teammate workcell estimate). All joints within limits.
- Mirror `[0, -90, -90, -90, 90, 0]` -> tcp `[0.497, -0.102, 0.364]`, also down (other side, no table).

---------------------------------------------------------------------------

## 9. Teammate setup (reference only, `fr5-board-stacking/src/fr5_bringup`)

- Used Fairino's `fairino5_v6_moveit2_config` (SRDF to wrist3_link) + own URDF via
  `MoveItConfigsBuilder(...).robot_description(file_path=..., mappings={"use_fake_hardware": ...})`.
- Launch: rsp + move_group + rviz helpers, own `ros2_control_node` (description as **parameter**),
  spawners, gripper node; LogInfo banner "REAL ROBOT MODE ... e-stop".
- `use_fake_hardware` arg validated as "true"/"false" strings.
- `hardware.yaml`: robot_ip 192.168.58.2, gripper `tool_do: 0` (T1 done: DO0 = vacuum pump, user 2026-10-26).
- `workcell.yaml`: table 1.00 x 2.00 x 0.03 m, top_z -0.045 (TODO), centre (-0.42, 0.03);
  profile 1.00 x 0.16 at (-0.42, 0); camera post 0.10 x 0.07 x 1.70 at (-0.87, 0.116);
  board 0.27 x 0.20 x 0.006 m. Base for our obstacles.yaml (F11, step 13).

---------------------------------------------------------------------------

## 10. Step 10 proposal: `hk_fr5_moveit_config` (DONE: decided in decision-log 5b, kept for history)

Proposed files:

    src/hk_fr5_moveit_config/
      config/hk_fr5.urdf.xacro           # hk_fr5_description xacro + ros2_control macro; arg use_fake_hardware
      config/hk_fr5.ros2_control.xacro   # mock_components/GenericSystem | fairino_hardware/FairinoHardwareInterface
      config/initial_positions.yaml      # mock start pose
      config/hk_fr5.srdf                 # group arm: chain base_link -> tcp; states; disable_collisions
      config/kinematics.yaml
      config/joint_limits.yaml
      config/ompl_planning.yaml
      config/pilz_industrial_motion_planner_planning.yaml
      config/pilz_cartesian_limits.yaml
      config/moveit_controllers.yaml
      config/ros2_controllers.yaml
      CMakeLists.txt, package.xml
    (launch/ + moveit.rviz = step 11)

Tool collision pairs to disable (rigid w.r.t. each other or touching):
wrist3_link-d405_mount_link (Adjacent), d405_mount_link-gripper_link (Adjacent),
d405_mount_link-d405_link (Adjacent), wrist3_link-gripper_link (Never),
wrist3_link-d405_link (Never), d405_link-gripper_link (Never).
wrist1/wrist2 vs tool links stay **enabled** (j5/j6 move them relative to each other).

Open choices: decided, see decision-log 5b (M-*).

---------------------------------------------------------------------------

## 10b. Step 10 build + test results (2026-10-25)

- Build: 3 packages OK, no stderr for `hk_fr5_moveit_config`.
- `pixi task add` cannot overwrite an existing task (no --force): use
  `pixi task remove build` + `pixi task add build ...` (this moves `[tasks]` to
  the end of pixi.toml; moved back by hand for a clean diff).
- Test 1 xacro: fake -> `mock_components/GenericSystem`, real
  (`use_fake_hardware:=false`) -> `fairino_hardware/FairinoHardwareInterface`;
  every joint exactly `position` cmd + `position` state (driver-compatible); check_urdf OK.
- Test 2 SRDF vs URDF: chain base_link -> tcp valid; 17 disabled pairs, no unknown
  links, no duplicates; 28 pairs still checked (incl. tool vs base/shoulder/upperarm/
  forearm/wrist1/wrist2); `zero`, `ready` within limits; FK ready = tcp down at (-0.497, -0.102, 0.364).
- Test 3 MoveItConfigsBuilder: pipelines [ompl, pilz_industrial_motion_planner], default
  ompl; OMPL only RRTConnect (arm default); Pilz Cartesian limits loaded; KDL 0.05 s;
  scaling 0.2/0.2; `to_dict()` merges `robot_description_planning` = joint_limits +
  cartesian_limits (explicit merge in moveit_configs_utils, both kept).
  Cosmetic warning "Cannot infer SRDF" (fires in `__init__` before explicit path; no `.setup_assistant`).
- Test 4 runtime (scratch launch, 25 s, ROS_LOCALHOST_ONLY=1): rsp + ros2_control_node
  (description via `~/robot_description` topic remap, no deprecation) + spawners + move_group.
  `update rate is 125 Hz`, `ros2 param get` -> 125, hardware `FR5System` initialised,
  arm_controller + joint_state_broadcaster active, OMPL + Pilz pipelines loaded,
  "You can start planning now!".
  Log messages reviewed:
  - ERROR "No 3D sensor plugin(s) defined for octomap updates": standard, harmless (no sensors_3d).
  - WARN "Resolution not specified for Octomap": same cause.
  - WARN "Could not enable FIFO RT scheduling policy ... Operation not permitted":
    CM not realtime -> possible jitter at 125 Hz on the real robot. Step 19: consider
    realtime group/limits (control.ros.org userdoc) if motion stutters.
  - WARN JTC `[Deprecated]: "allow_nonzero_velocity_at_trajectory_end" is set to true.
    The default behavior will change to false.` -> option: set false explicitly.
  - kdl_parser root inertia (D-j, accepted).

## 10c. Step 11 test results (2026-10-25)

- Arg validation: `use_fake_hardware:=maybe`, `update_rate:=abc|0`, `use_rviz:=yes` -> clear
  RuntimeError, nothing started. `use_fake_hardware:=false` -> "needs the Fairino driver package
  'fairino_hardware_v3_9_0_1', which is not built". (Test gotcha: last `name:=value` on the
  command line wins.)
- Headless bringup (`use_rviz:=false`): both controllers active, update_rate 125,
  /joint_states at start = home (err 0), tcp at home [-0.497, -0.102, 0.364].
- **Home tcp quaternion = [0.707, 0.707, 0, 0]** (x,y,z,w) = Rz(+90 deg) * Rx(pi):
  home is tool-down with **yaw = +pi/2** in our F7 convention. `move_to yaw=0` rotates the tool 90 deg.
- `/move_action` tests (throwaway script):
  - OMPL RRTConnect to (-0.40, 0.10, 0.30): SUCCESS, final pos err 0.7-0.9 mm, orientation ~0.3-0.6 deg
    off: OMPL stops anywhere inside the goal tolerance (sphere radius 1 mm, 0.01 rad per axis).
    Step 16: choose tolerances deliberately. Pilz goals end exactly (0.00 mm).
  - Pilz PTP to home pose: SUCCESS, exact.
  - **Pilz LIN FAILED** with vendor Cartesian limits:
    `Joint acceleration limit of j3 violated ... Actual joint acceleration is 0.964822, while the limit is 0.7.`
    Pilz LIN uses Cartesian limits x scaling (2.25 m/s^2 x 0.2 = 0.45 m/s^2) and checks the
    resulting joint motion against the UNSCALED joint limits (0.7 rad/s^2). Vendor Cartesian
    acc 2.25 does not fit vendor joint acc 0.7.
  - Evidence run (scratch Cartesian limits acc/dec +-0.3 m/s^2, no repo change, via
    `.pilz_cartesian_limits(file_path=<abs path>)`): LIN dz -0.05 @0.2 SUCCESS (exact),
    LIN dz +0.05 @1.0 SUCCESS, LIN dx +0.15 @1.0 FAIL (j3 0.89 > 0.7). PTP fine.
    -> the real limiter is the conservative joint acc 0.7 (vendor default, real FR5 value unknown).
- After L-7 (Cartesian acc/dec 0.3): LIN @0.2 5 cm + 15 cm SUCCESS exact; LIN @1.0 FAIL
  (j4 0.83 / j1 0.80 > 0.7), depends on the arm configuration (OMPL may leave a different one).
  motion_controller must pass Pilz failures through with a clear message (step 16/17).
- move_group WARN "Execution of motions should always start at the robot's current state.
  Ignoring the state supplied as start state": set `request.start_state.is_diff = True`
  (and `planning_options.planning_scene_diff.is_diff = True`) in step 16.
- Durations 4-10 s per move include execution at 0.2 scaling (slow accel, M-4).

## 10d. Step 12 results (2026-10-25)

- `hk_fr5_interfaces` built (rosidl, Python 3.14): only CMake dev warning CMP0148 (FindPythonInterp), harmless.
- `ros2 interface package` lists MoveTo, MoveDelta, GetState; Python + C++ types generated.
- Defaults: MoveTo mode 0 (DOWN_YAW), velocity_scaling 0 (-> controller uses 0.2), planner "",
  named_target "", plan_only false; MoveDelta frame 0 (TCP); GetState.success **false** by default
  (safe: an unfilled response never looks valid). Result serialisation round-trip OK.
- `error_code` is `moveit_msgs/MoveItErrorCodes` -> callers compare `result.error_code.val` with
  `MoveItErrorCodes.SUCCESS` etc. Our own input validation should reuse existing codes
  (e.g. INVALID_GOAL_CONSTRAINTS, INVALID_GROUP_NAME?) -> decide mapping in step 14/16.
- pixi build task edited in place (python replace of the task line) instead of
  `pixi task remove/add` (keeps `[tasks]` position, one-line diff).

## 10e. Step 13 findings (2026-10-25)

Teammate workcell (`fr5_bringup/urdf/workcell_scene.xacro` + `config/workcell.yaml`, estimates):
- Robot base on a 16 cm aluminium profile; table top `top_z = -0.045` below base_link (TODO measure,
  T4); table 1.00 (x) x 2.00 (y) x 0.03 m slab, centre (-0.42, 0.03); profile 1.00 x 0.16 m at
  (-0.42, 0), height = gap between table and base; camera post 0.10 x 0.07 x 1.70 m at (-0.87, 0.116)
  standing on the table. Teammate modelled them as URDF links with **1 mm gaps** ("so MoveIt never
  sees permanent contacts"). Also zone frames + overhead camera frame (perception, not obstacles).
- Handoff safety rule: "No straight-down move onto the table until top_z is measured" (wrong table
  height + LIN descent = cup pushed into the table).
- Arm mesh bounding boxes (own STL parse): base_link x,y +-0.0745, **z 0.000 .. 0.079** -> base
  bottom exactly at base_link z = 0; profile top at z = -0.001 leaves the 1 mm gap. Base 149 mm
  wide on a 160 mm profile. Other links: shoulder z 0.074.., upperarm/forearm reach x -0.483/-0.435.
- Converted boxes (centre z from top_z, gap 0.001):
  table   size [1.00, 2.00, 0.03]  pos [-0.42, 0.03, top_z - 0.015] = [-0.42, 0.03, -0.060]
  profile size [1.00, 0.16, -top_z - 0.002 = 0.043]  pos [-0.42, 0.0, -0.001 - 0.0215 = -0.0225]
  post    size [0.10, 0.07, 1.70]  pos [-0.87, 0.116, top_z + 0.001 + 0.85 = 0.806]
- MoveIt services available in our move_group (step 10 log): ApplyPlanningSceneService,
  GetPlanningSceneService, StateValidationService (`/check_state_validity`).

## 10f. Step 13 results (2026-10-25)

- Build: 5 packages, `hk_fr5_motion` (ament_python, setuptools 79, pyyaml 6.0.3) no stderr.
- yaml validation (7 bad files: duplicate, negative size, no position, yaw 'abc', empty list,
  YAML syntax, missing file): each a precise error, exit code 1, nothing added. No move_group:
  "/apply_planning_scene not available (is move_group running?)", exit 1.
- Runtime (headless bringup, use_obstacles default true): loader logs 4 boxes, exits cleanly;
  `/get_planning_scene` returns table, profile, camera_post, floor_guard at the yaml values.
- `/check_state_validity`: home valid; zero valid; [0,30,0,0,0,0] (tcp z -0.35 over table) invalid
  (table, profile, floor_guard contacts); [180,30,0,0,0,0] (tcp z -0.35 at x +0.66, past near edge)
  invalid **only via floor_guard** -> O-1 proven necessary.
- Plan-only OMPL pose goals (tool down, yaw +pi/2): 5 cm above table SUCCESS; 1 cm into table
  **GOAL_STATE_INVALID (-27)** ("Unable to sample any valid states for goal tree") -> step 16 mapping.
- Reload with a moved camera_post: still 4 objects, post x -0.87 -> -0.77 (ADD with same id replaces).
- **Gotcha (Q-1): `geometry_msgs/Quaternion` defaults to w = 1.** Setting only x, y gave
  (0.707, 0.707, 0, 1); MoveIt warned "Orientation constraint ... probably incorrect ... Assuming
  identity" (= tool pointing UP) and the goal became invalid. motion_controller must always set all 4
  quaternion components (and normalise).
- CLI quirk: `ros2 interface show moveit_msgs/msg/CollisionObject` prints AttachedCollisionObject
  text and "maximum recursion depth exceeded" (Python 3.14 CLI). Use
  `python3 -c "from moveit_msgs.msg import X; print(X.get_fields_and_field_types())"`.
- `CollisionObject` (Humble) has an object `pose` + `primitive_poses` relative to it.

## 10g. Step 14 findings (2026-10-25)

- rclpy **3.3.22**. `ActionServer(node, type, name, execute_callback, callback_group=,
  goal_callback=, handle_accepted_callback=, cancel_callback=)`. All callbacks go through
  `await_or_execute`: **`async def` execute callbacks are supported**, so the controller can
  `await` the `/move_action` client futures inside its own execute callback (no blocking spin,
  no deadlock). Default goal_callback accepts everything; default handle_accepted executes at once
  -> several goals would run concurrently unless we enforce one active goal.
- GoalResponse ACCEPT/REJECT, CancelResponse ACCEPT/REJECT. ServerGoalHandle: abort, canceled,
  succeed, execute, is_active, is_cancel_requested, publish_feedback, request, status, goal_id.
  A REJECTed goal gives the client no result/message (only "rejected").
- MoveItErrorCodes values: SUCCESS 1, FAILURE 99999, PLANNING_FAILED -1, INVALID_MOTION_PLAN -2,
  CONTROL_FAILED -4, TIMED_OUT -6, PREEMPTED -7, START_STATE_IN_COLLISION -10, GOAL_IN_COLLISION -12,
  INVALID_GROUP_NAME -15, INVALID_GOAL_CONSTRAINTS -16, INVALID_ROBOT_STATE -17,
  FRAME_TRANSFORM_FAILURE -21, ROBOT_STATE_STALE -23, COMMUNICATION_FAILURE -25,
  GOAL_STATE_INVALID -27, ABORT -30, NO_IK_SOLUTION -31.

## 10h. Step 14 results (2026-10-25)

- rclpy `declare_parameter(name, Parameter.Type.X)` without a value + `get_parameter` raises
  `rclpy.exceptions.ParameterUninitializedException` (not None). Caught -> clear error:
  "parameter 'group' missing: start with --params-file ...", exit 1.
- Param file key `/**/motion_controller:` matches the node in namespace `/motion`.
- Bringup headless: `/motion/move_to`, `/motion/move_delta` (actions), `/motion/get_state`
  (service), node `/motion/motion_controller`; `ros2 param get` returns yaml values.
  SRDF from `/robot_description_semantic` (transient local) -> named targets ['ready', 'zero'].
- Validation (14 goals): valid move_to pose / named `ready` / valid move_delta -> FAILURE 99999
  "not implemented yet"; unknown named_target (lists known), NaN, inf, scaling 1.5 / -0.1 / 2.0,
  mode 2, frame 3, planner 'ptp' (case-sensitive) and 'CIRC' -> INVALID_GOAL_CONSTRAINTS -16 with
  precise messages; final_tcp_pose frame = base_link. get_state -> success false (step 15).
- Step 17 (C-9): all-zero move_delta -> succeed at once without moving (final_tcp_pose = current).

## 10i. Step 15 results (2026-10-25)

- BUG found by offline test: `ET.fromstring(urdf).iter("joint")` also returns the `<joint>` elements
  inside `<ros2_control>` (no `<child>`) -> AttributeError on the real /robot_description. Fix:
  `findall("joint")` (top-level URDF joints only). chain_joints -> ['j1'..'j6'], unknown tip -> KeyError.
- quat_to_rpy (R = Rz Ry Rx): 5000 random round trips same rotation (|dot|-1 <= 4.4e-16); home quat
  (0.7071, 0.7071, 0, 0) -> (180, 0, 90) deg; MODE_DOWN_YAW yaw round trip exact (0, 1.0, -2.5);
  gimbal lock pitch +-90 -> yaw 0, same rotation; unnormalised quat handled.
- Runtime (bringup): home -> success, joints j1..j6 = [0, -90, 90, -90, -90, 0] deg, tcp
  [-0.4970, -0.1021, 0.3641], rpy (180, 0, 90); after OMPL move to (-0.40, 0.10, 0.30) yaw 57.30 deg ->
  tcp [-0.3993, 0.1000, 0.3003], rpy (-179.85, 0.15, 57.57) (OMPL tolerance). Stamp = TF stamp.
- Failures: no URDF -> "no /robot_description yet"; URDF but no joint states -> "no /joint_states
  received yet"; stale -> "latest /joint_states is 2.5 s old (> state_timeout 1.0 s)".
- BUG (cosmetic) fixed: on SIGINT rclpy's signal handler shuts down first -> `spin()` raises
  `rclpy.executors.ExternalShutdownException` (not KeyboardInterrupt) -> traceback + exit 1.
  Catch both. After fix: 0 tracebacks on SIGINT.

## 10j. Step 16 findings (2026-10-25)

- **MoveIt Humble /move_action feedback = only 'PLANNING' (at start) and 'IDLE' (at end)**; no
  execution state ('MONITOR' never sent). Measured: OMPL [(0.0 s, PLANNING), (10.16 s, IDLE)],
  Pilz PTP [(0.0, PLANNING), (2.13, IDLE)]. -> T-4 mapping cannot produce EXECUTING (decision T-7).
- **rclpy logger: severity is fixed per call site** ("Logger severity cannot be changed between
  calls"): `log = info if ok else warn; log(...)` crashes the 2nd time with the other severity.
  Use one call per severity. (Crashed every failing goal: status ABORTED, code 0, empty message.)
- Results (headless, 14 goals): OMPL down-yaw pos err 0.56 mm, angles <= 0.41 deg; OMPL FULL
  (pi, 0.2, 1.0) pos 0.95 mm, angles <= 0.49 deg; Pilz PTP / LIN exact (0.00 mm, 0.00 deg);
  named `ready` via Pilz PTP exact joints; named `zero` via OMPL max joint err 0.564 deg (tol 0.57);
  plan_only: no motion, 0.0 s; scaling 0.2 vs 0.6: 10.0 s vs 5.2 s.
- **Relaxed joint tolerance 0.01 rad (T-2): OMPL `ready` ended 4-6 mm from home at the tcp**
  (tcp [-0.4971, -0.1061, 0.3706] / [-0.4957, -0.1074, 0.3707] vs [-0.4970, -0.1021, 0.3641]):
  6 joints x up to 0.01 rad, long lever arms. Pilz PTP to named targets is exact.
- Failures: into table / out of reach (x = -1.5) -> GOAL_STATE_INVALID (-27) after the full
  planning_time (5 s; OMPL goal sampler, not NO_IK); Pilz LIN into table -> INVALID_MOTION_PLAN (-2)
  in 0.0 s (Pilz ignores obstacles while planning, MoveIt's final check rejects; no retry).

- EXECUTING via arm_controller action status (T-7): reported 0.01-0.06 s after PLANNING (= planning
  time); plan_only never reports EXECUTING. Status topic `/arm_controller/follow_joint_trajectory/_action/status`
  (config `trajectory_status_topic`).
- Tolerance comparison -> decision-log T-10 (tight everywhere, no measured cost).
- Weak test: "scaling 0.2 vs 0.6" with OMPL compares different random paths (tight run 5.3 vs 5.8 s);
  use Pilz PTP (deterministic) to compare speeds in future tests.

- **Speed analysis (2026-10-25, user: "planning takes too long, stop-and-go")**: planning is NOT
  the bottleneck. User's RViz-check log: request -> execution start 69 ms (OMPL), 15 ms (LIN),
  21 ms (named). Fixed Pilz sequence (PTP home->P, LIN 10 cm down, LIN up, PTP home), planning
  12-19 ms everywhere, totals: A current (joint acc 0.7, Pilz acc 0.3, scaling 0.2) 13.8 s;
  B acc 2.0 / Pilz 1.0 / 0.2: 8.1 s; C current / 0.5: 8.8 s; D acc 2.0 / Pilz 1.0 / 0.5: 5.3 s.
  Time = execution: conservative limits (0.7 rad/s^2 x 0.2 = 0.14 rad/s^2), each move starts and
  ends at zero velocity (stop-and-go), and the `ros2 action send_goal` CLI adds process start +
  server discovery per command (a long-running task node will not). Failed OMPL goals wait the full
  planning_time (5 s) before GOAL_STATE_INVALID.
  Scratch method: `.joint_limits(file_path=...)` / `.pilz_cartesian_limits(file_path=...)` override in a
  copied launch file.
- **Latency to first motion (user observation confirmed)**: CLI `pixi run ros2 action send_goal`
  (new process each time): goal accepted / motion start after 0.5-2.2 s (`pixi run` activation alone
  0.24 s + CLI start-up + action server discovery). Persistent rclpy client: server found 0.07 s
  once, then goal accepted 1 ms, EXECUTING 12-14 ms after send. -> the felt delay was the test
  method, not planning. Task code must use one long-running node (never CLI calls per move).

## 10k. Step 17 results (2026-10-25)

- Offline maths (test_delta_math.py): TCP dz +0.10 with tool down -> z -0.10 (DOWN); BASE dz +0.10
  -> up; TCP dx along the tcp x-axis; TCP dyaw +0.3 -> base yaw -0.3 (tcp z = -base z), BASE dyaw
  +0.3 -> +0.3, position unchanged; 2000 random deltas x 2 frames vs independent Hamilton-product
  quaternion maths: worst 2.2e-16 m / 8.9e-16; matrix_to_quat round trip incl. all 4 Shepperd
  branches (180 deg cases).
- Runtime (/motion/move_delta, 11 cases from home): TCP dz +-0.05, BASE dx/dz +0.05: moved 50.0 mm,
  1.1 s each; pure rotations (TCP dyaw +0.3, BASE dyaw -0.3): 0.0 mm translation, 2.0 s; combined
  TCP / BASE d + rpy: 41-47 mm, 1.8 s. All end poses vs expectation <= 0.001 mm, 0.000 deg. Zero delta:
  SUCCESS in 0.00 s, no motion, no feedback. plan_only: no motion. BASE dz -0.60 (into table):
  INVALID_MOTION_PLAN (-2) in 0.01 s (Pilz plans the line, MoveIt's final check rejects).
- move_to regression after the shared `_prepare()` refactor: unchanged (success, -16 x2, -27, named ready).
  Into-table now fails after 1 s (planning_time 1.0, T-11).

## 10l. Step 18 results (2026-10-25)

- **MoveIt Humble /move_action ignores cancel during execution** (it accepts the cancel, the
  trajectory runs to the end, result SUCCESS). Cancelling the controller goal stops in 9 ms but
  commands 31 rad/s^2 / 2578 rad/s^3 (hard stop). A 1-point stop trajectory is no better (JTC restarts
  the spline from v = 0: 31 rad/s^2). **2-point stop trajectory (q, v) -> (q + vT/2, 0)** = constant
  deceleration: peak acc = stop_deceleration, jerk ~ normal motion. `open_loop_control: true` + 1 point
  also smooth (not needed). Measure smoothness from `/arm_controller/controller_state` `reference`
  (commanded, header stamps); /joint_states finite differences are too noisy (normal motion showed
  3 rad/s^2 instead of 0.6), and receipt-time stamps worse still.
- Implemented in motion_controller (decision-log 5j): smooth stop on cancel / preempt during execution;
  on stop during planning, the (MoveIt-executed anyway) trajectory is stopped as soon as the controller
  starts it; goal ends after the stop trajectory finished; stop owner tracked per MoveIt goal
  (`_moveit_owner`) so a preempt during planning stops the OLD plan, not the new goal.
- Test (19/19): cancel mid-execution -> CANCELED/PREEMPTED, stops mid-way, result pose = actual, still
  afterwards, peak acc 1.98, jerk 125; preempt (move_to / move_delta) -> old ABORTED/PREEMPTED, new from
  the stop pose (delta end = stop pose + 5 cm, 0.00 mm), peak acc 2.00, jerk 189; cancel during planning
  -> no motion; rapid fire 5 goals -> only the last executes; **full speed (1.18 rad/s) cancel -> stop in
  0.60 s, peak acc 2.01, jerk 125** (hard stop: 31 / 2578); preempt 5 ms after start -> old plan stopped
  at start; normal operation afterwards.
- Coast distance = v^2 / (2 a): at full speed 1.18 rad/s -> 0.35 rad on the fastest joint (not
  collision-checked, V-6); at default 0.2 scaling ~0.09 rad. Option (T14): validate the stop end state
  with /check_state_validity before sending, or a higher stop_deceleration at high speed.
- Test-script gotcha: scenarios must start from a known pose (from `zero` MoveIt said "Goal constraints
  are already satisfied" and PTP paths from `zero` hit the table).

## 10m. Real robot: command lag (2026-10-25, first real session)

- User: cancel looked like it did not stop; then: stop works but "sweeps almost the full path", and every
  command takes 1-2 s before the robot moves (instant in sim). Motion controller result was PREEMPTED
  (3.48 s) with a mid-way pose, i.e. the smooth stop was sent and accepted by the arm controller.
- `notes/lag_probe.py` (controller_state reference = commanded, feedback = read back by the driver):
  run 2 (one `d 0 0 20 base` at v 0.1): commanded 2.96 -> 3.63 s (0.67 s), actual 5.03 -> 5.75 s
  (0.72 s): **constant lag +2.07 / +2.12 s**, same motion shape. Max |commanded - actual| 0.052 rad.
  (Run 1 not usable: user command timing.)
- controller_state arrives at **79 Hz** (set 125 Hz): the control loop does not reach 125 Hz on the real
  robot (~12.6 ms per cycle; driver SDK calls each cycle: GetActualJointPosDegree + ServoJ over the network).
- Interpretation: the FR5 executes the streamed ServoJ points from a queue (FIFO) ~2.1 s deep (~166 points
  at 79 Hz) -> both start and stop reach the arm 2 s late; smooth stop + limits are not the cause.
  Driver: ServoJ every write() with cmdT 0.008 s hard-coded, no ServoMoveStart/ServoMoveEnd (Q3-4).
- **SOLVED at 50 Hz (P-9):** probe with update_rate 50: loop 50 Hz, lag +0.10 / +0.06 s, max error 0.0078 rad
  (0.44 deg). The FR5 queues streamed ServoJ points when they arrive faster than it consumes them.
  RULE: real robot runs at **update_rate 50**; check the log line `controller_manager update_rate: 50 Hz`.
  `ros2 launch` ignores misspelled args silently (`update-rate:=50` -> still 125 Hz).
- Re-measure after any driver / controller change: `pixi run python3 notes/lag_probe.py 15` + one small move.

## 10n. Step 20a/20b gripper findings + results (2026-10-25)

Fairino tool I/O (SDK header `fairino_hardware_v3_9_0_1/libfairino/include/robot.h`):
- `SetToolDO(id, status, smooth, block)`, `GetToolDI(id, block, *result)`, `GetToolDO(*do_state)`;
  tool I/O ids 0..1 only (`robot_types.h`: "only bit0-bit1 valid"). `GetDI` / `SetDO` = control box I/O.
  `block` doc: 0 = blocking, 1 = non-blocking. We call GetToolDI(id, 0).
- All three names exist as plain strings in `libfairino.so.2.3.0` (= XML-RPC method names, like ServoJ).
  **Unverified:** XML-RPC reply of GetToolDI / GetToolDO. Assumed `[error, value]` (Fairino Python SDK style);
  `gripper_io.parse_reply` also accepts a bare int, anything else -> CommError with the raw reply (20f).
- xmlrpc.client's default Transport has no timeout -> own Transport sets `conn.timeout` (rpc_timeout).

rclpy quirks found (step 20b):
- Q20-1 **rclpy logger fixes the severity per call site**: `log = warn if x else info; log(msg)` raises
  `ValueError: Logger severity cannot be changed between calls` the 2nd time with another severity
  (crashed the node on the first PART_LOST). Use separate `.warn(...)` / `.info(...)` calls.
- Q20-2 With rclpy's default signal handler, Ctrl+C shuts the context down before `finally:` runs ->
  shutdown logs print but "Failed to publish log message to rosout: publisher's context is invalid".
  Fix: `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` + catch KeyboardInterrupt.
  (Only SIGINT is handled; ros2 launch sends SIGINT first, SIGTERM only after its timeout.)
- Q20-4 With rclpy's handler off (Q20-2), a node started from a background / non-interactive shell inherits
  SIGINT as **ignored** -> Ctrl+C / launch SIGINT did nothing (found in the 20d headless launch test; /proc SigIgn
  bit 2 set). SIGTERM was not handled either (pumps would stay on). Fix: own `signal.signal(SIGINT / SIGTERM,
  raise KeyboardInterrupt)`; during the switch-off both are ignored (bounded by rpc_timeout per call).
  Verified: launch Ctrl+C -> both pumps off, all nodes "finished cleanly"; SIGTERM -> both off, exits.
- Q20-5 **rclpy callback groups (found in T20b):** by default all callbacks of a node are in ONE mutually exclusive
  group. An `async` **service or timer** callback holds that group for its whole duration, including while it
  awaits -> its own `_sleep` timer and the client response (ApplyPlanningScene) can never run = deadlock (the request
  still reached MoveIt, so the boxes appeared, but nothing after the await ran and later service calls timed out).
  Action execute callbacks are NOT affected (rclpy runs them as separate tasks outside the group) -> motion_controller
  / gripper_controller were fine. Fix in scene_manager: one `ReentrantCallbackGroup` for its services, timers
  (incl. the `_sleep` timers) and clients; mutual exclusion of scene changes via its own `_busy` flag.
- Q20-3 Test gotcha: SIGINT to the `ros2 run` wrapper PID is not forwarded to the node; a terminal Ctrl+C
  hits the whole process group -> tests start nodes with `start_new_session=True` and `os.killpg(pid, SIGINT)`.

Step 20b results (scratchpad tests, mock Fairino XML-RPC server = SimpleXMLRPCServer with SetToolDO /
GetToolDI / GetToolDO + test hooks):
- I/O offline 27/27: debounce (glitch reset, reads = 1), parse_reply (controller code, bad replies), FakeIo
  (delay, fake_vacuum false, pump off), XmlRpcIo vs mock (read/write, controller error 14, slow server ->
  CommError after 0.50 s, recovery, connection refused -> CommError at once).
- Debounce bug found by the W-13 test: after an outage the first good read is only 1 of 2 debounce reads, so
  recovery decided from the pre-outage vacuum value (brief false HOLDING). Fix: `Debounce.reset()` on comm loss.
- W-13 runtime (20/20 incl. the 17 below, updated): HOLDING -> server down -> back with vacuum -> HOLDING in
  1.01 s (= comm_retry_period); back without vacuum -> PART_LOST; CONTROLLER_ERROR kept across an outage, DOs
  re-read; unreachable at start -> robot appears -> start read -> HOLDING, nothing switched; 3 outages -> 3 logs.
- Node runtime 17/17 (before W-13): missing params / vacuum_do == blow_off_do -> exit 1 with message; fake IDLE, state
  10 Hz; real code path: start with DO0 on + DI0 = 1 -> HOLDING, no SetToolDO at start (W-9); DI0 drop ->
  PART_LOST in 0.10 s (debounce 2 x 50 ms), pump still on (W-4); error 14 -> CONTROLLER_ERROR; server down
  -> COMM_ERROR; server back -> DI read again, state stays COMM_ERROR (W-12); repeated errors logged once;
  Ctrl+C -> SetToolDO(0, 0) + SetToolDO(1, 0) (W-10), rc 0, no rosout warning; unreachable at Ctrl+C ->
  "NOT switched off" logged, exits in 0.17 s.

Step 20c results (grip / release actions, 2026-10-26, scratchpad `test_actions.py`, 27/27; 20b suites re-run 27/27 + 20/20):
- Fake: grip SUCCESS ttv 0.60 s (0.5 s fake delay + 2 debounce reads), 11 feedback msgs; grip while HOLDING ->
  SUCCESS in 2 ms, nothing switched; release 1.00 s (RELEASING with DO1 on seen, then IDLE all off);
  blow_off_time 0.3 -> 0.30 s; no vacuum: timeout 1.0 -> NO_VACUUM 1.01 s, default -> 3.03 s, pump off;
  timeout -1 / nan, blow_off_time -0.5 -> INVALID_GOAL; cancel grip -> CANCELED + pump off; release during grip
  -> grip PREEMPTED; grip during release -> release PREEMPTED, DO1 off, HOLDING; cancel release -> DO1 off at
  once; vacuum lost -> PART_LOST; grip from PART_LOST (pump still on) -> SUCCESS after 0.10 s (fresh reads);
  Ctrl+C during blow-off -> both off, rc 0.
- Real code path vs mock: grip = SetToolDO(1,0), (0,1); release = (0,0), (1,1), (1,0); no vacuum adds (0,0);
  controller error 14 -> CONTROLLER_ERROR (state + result), next grip clears it; robot unreachable -> COMM_FAILURE
  in 0.00 s (connection refused); robot back -> COMM_ERROR clears to IDLE; comm lost mid-grip -> COMM_FAILURE at
  the timeout.
- rclpy: several execute coroutines on the single-threaded executor; preemption = `self._active is not goal_handle`
  checked after each `_sleep` (timer-backed Future, same as motion_controller).

Step 20d results (2026-10-26): `use_gripper:=true|false` (default true) in bringup, gripper node gets
`use_fake_hardware` from the launch arg (later parameter wins, same pattern as update_rate); hk_fr5_gripper in
`pixi run build` + exec_depend of hk_fr5_moveit_config. Headless sim: /gripper/* and /motion/* together, grip
SUCCESS 0.60 s, state HOLDING, release SUCCESS, get_state OK, param use_fake_hardware True; use_gripper:=false ->
no node; use_gripper:=bad -> clear launch error. Real-mode launch NOT run (no robot; the override is the same
pattern as update_rate and the node's real path is tested against the mock).

Step 20e results (motion_shell gripper commands, 2026-10-26, headless sim, shell driven via stdin):
- `grip [TIMEOUT_S]`, `release [BLOW_S]`, `gstate`, gripper line in `status`; `run()` generic (describe callback,
  only `phase` feedback printed). Sim: grip 0.61 s, grip while holding 0.02 s, release 1.00 s / 0.3 s, `grip -1` /
  `grip 1 2` / `grip abc` -> usage errors, NO_VACUUM 1.01 s, Ctrl+C during grip -> CANCELLED (pump off); motion
  `d` commands unchanged.
- W-17 (use_gripper:=false): status "gripper not running", grip / release / gstate -> clear message after ~1 s, motion
  works; gripper started later -> works; stopped -> gstate says not running (no stale state); stopped mid-goal
  (SIGINT and SIGKILL) -> "action server disappeared" (no hang). `server_is_ready()` noticed the SIGKILLed node too.

T20b results (scene_manager, 2026-10-26, headless sim, scratchpad `test_scene_static.py`, `test_scene_smoke.py`):
- Static boxes vs obstacle_loader: bringup with obstacle_loader vs `use_obstacles:=false` + scene_manager, compared
  `/get_planning_scene` world objects (ids, dims, frame, pose, primitive pose): **IDENTICAL** (table, profile,
  camera_post, floor_guard). obstacle_loader + motion_controller unchanged in this step (removal = T20d).
- Services: attach 270 x 200 x 6 mm -> MoveIt attached 'board' on tcp, touch_links [gripper_link, tcp], dims
  0.27 x 0.20 x **0.004**, centre tcp z 0.002 (margin 2 mm); 2nd attach rejected; detach keep -> stack_0 at
  (-0.4970, -0.1021, 0.3621) = home tcp z 0.364 - 0.002 (tcp z points down), quat (0.707, 0.707, 0, 0) = tool down
  yaw 90 deg, no 'board' left in the world, nothing attached (one diff: attached REMOVE + world REMOVE board + ADD
  stack_0 works); add_box OK; static name rejected; box into the gripper -> success + "WARNING: robot state now in
  collision: gripper_link <-> probe"; remove static rejected; remove "" -> 3 boxes removed, only static left;
  detach with nothing attached rejected; clean exit 0.

T20c results (scene objects vs real planning, 2026-10-26, headless sim, scratchpad `test_scene_motion.py` +
`scene_lib.py`, 27/27). Setup: bringup `use_obstacles:=false use_gripper:=false` + scene_manager; table top -0.045
(estimate), board 270 x 200 x 6 mm, pick spot (-0.45, -0.25), stack spot (-0.45, +0.25), tool down yaw 90 deg, v 0.5.
- Pick: OMPL above, LIN down to tcp z -0.039 (board top, measured -0.0390); attach (2 mm) -> no warning; LIN lift 100 mm
  SUCCESS; LIN back down SUCCESS (board bottom 2 mm above the table).
- Default detach `{}` (keep_as_obstacle false) -> "removed", name '', no board / stack in the world.
- No margin (1e-6 m): state check warns `board <-> table`; OMPL plan_only SUCCESS **but its start-state adapter jumped
  1.022 then 0.261 rad** (move_group log, 5n-1); Pilz LIN plan + execute SUCCESS from the real state.
- Blocking: without a board tcp z -0.043 is a valid goal; with the board the same goal -> INVALID_MOTION_PLAN.
- Place: OMPL carry with the board attached, LIN down to -0.039 (measured -0.0390), detach keep -> stack_0 at
  (-0.4500, 0.2500, -0.0410), dims 0.27 x 0.20 x 0.004 (top -0.039 = real top); lift OK; next board at the old place
  height -> INVALID_MOTION_PLAN (stack_0 there); place on stack_0 at tcp z -0.033 (measured -0.0330) SUCCESS ->
  stack_1 at z -0.0350; lift OK.
- Dynamic wall 0.30 x 0.02 x 0.30 between the spots: no warning; straight LIN -> PLANNING_FAILED; OMPL around SUCCESS;
  remove "" -> 3 boxes (stack_0, stack_1, wall).
- Motion regression: move_to ready, move_delta dx 50 mm (tcp frame), cancel after 1 s -> CANCELED + PREEMPTED (-7),
  back to ready: all as before.
- T25 re-run (adapter FixStartStateCollision removed from ompl_planning.yaml, 28/28): OMPL from the contact state ->
  **-26 START_STATE_INVALID** (no jump); move_group adapters now: AddTimeOptimalParameterization, ResolveConstraintFrames,
  FixWorkspaceBounds, FixStartStateBounds (<= 0.1 rad joint-limit clamp, kept), FixStartStatePathConstraints; every
  other T20c check unchanged.
- Test bugs on the way (not code bugs): relative heights from the wrong start z; a wall placed through the robot
  (the attach/add warning flagged it: `wall <-> wrist1_link, gripper_link ...`).

T20d results (launch integration, 2026-10-26, scratchpad `test_t20d.py`, `test_scene_motion.py`, `test_stop.py`):
- Default bringup: node list has `/scene/scene_manager`, no obstacle_loader; `/get_planning_scene` world objects
  **identical** to the obstacle_loader reference (T20b); /scene/add_box works. `use_obstacles:=false`: world empty,
  /scene/add_box works. `ros2 pkg executables hk_fr5_motion`: motion_controller, motion_shell, scene_manager.
- T20c suite (28 checks) against the launched scene_manager: 28/28.
- Ctrl+C (SIGINT to the process group, like a terminal): several nodes "process has died, exit code -2"; same with
  the pre-T20d launch file (rsp, move_group, motion_controller) -> pre-existing (T26), not caused by T20d.

T20e results (motion_shell scene commands, 2026-10-26, scratchpad `test_shell_scene.py`): `attach L W H [YAW DX DY
MARGIN]`, `detach [keep]`, `box add NAME SX SY SZ X Y Z [YAW]`, `box rm [NAME]` (mm / deg), scene line in `status`.
No bringup: status "scene ○ not running", every scene command -> "scene_manager not running" (no hang). With bringup:
attach / 2nd attach rejected / detach keep -> stack_0 / box add / same name -> "replaced" (yaw 15 deg) / static name
rejected (add + rm) / rm wall / rm unknown lists dynamic boxes / rm all / detach nothing / margin 10 mm > 6 mm rejected /
attach with yaw 30 deg, offset (10, -5) mm, margin 1.5 mm -> modelled 4.5 mm / usage errors for bad input / history.

T22 results (unknown launch args, 2026-10-26): ros2 launch puts every CLI `name:=value` into
`context.launch_configurations`, declared or not (probe launch file: `a:=2 update-rate:=50 b:=x` -> a, b, update-rate).
bringup: `update-rate:=50` and `use_gripper_x:=false` -> "[ERROR] [launch]: ... unknown launch argument(s) ...; valid:
use_fake_hardware, update_rate, use_rviz, use_obstacles, use_motion_controller, use_gripper (note: underscores ...)",
exit 1, no process started; `--show-args` lists the 6 args; `update_rate:=50 use_rviz:=false` starts normally (log
"update_rate: 50 Hz", all nodes incl. /scene/scene_manager). Caveat: a check inside the launch file only sees args of
this launch; included vendor launch files are not affected.

---------------------------------------------------------------------------

## 11. Later-step reminders

- Step 16 (T11): retry policy for OMPL failures `INVALID_MOTION_PLAN` ("Computed path is not valid ... possibly due to postprocessing") and `PLANNING_FAILED`; error code for impossible goals is `GOAL_STATE_INVALID` (-27); always set all 4 quaternion fields (Q-1); `start_state.is_diff = True`.

- Step 11: remap `~/robot_description` for `ros2_control_node`; only ompl + pilz pipelines;
  `generate_*_launch` helpers, not `generate_demo_launch`; own `moveit.rviz`;
  banner + explicit `use_fake_hardware`; check interactive marker at tcp.
- Step 11: check start state not in collision at `ready` (wrist2 vs camera arm).
- Step 16/17: `/move_action` ids: `ompl`/`RRTConnect`, `pilz_industrial_motion_planner`/`PTP`|`LIN`.
- Step 19: build `fairino_msgs` + driver (Q3-5 submodule side effect, Q3-6 LD_LIBRARY_PATH);
  IP is hard-coded in driver; consider 125 Hz update_rate if motion stutters; accel limits tuning.
- ~~Step 20: T1 gripper questions (Schmalz model, DOs, DI).~~ DONE (decision-log 5l).
- Step 20f (lab): read-only GetToolDI / GetToolDO via XML-RPC first, check the reply format (10n).
