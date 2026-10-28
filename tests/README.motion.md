# Test scripts (sim / mock, no real robot)

Plain scripts, not colcon tests. Each prints `PASS` / `FAIL` lines (or, where marked *printed*, output to compare
with the referenced notes) and the number of passed checks. Results of every run so far:
`notes/implementation-ref.md` (sections 10n "Step 20..." and "T20..."), decisions: `notes/decision-log-motion.md` 5l / 5n.

**Run** from the worktree after `pixi run build`, one script at a time, in an isolated ROS graph:

    ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77 pixi run python3 tests/<script>.py
    ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77 pixi run bash tests/gripper_launch_check.sh

Logs go to `$HK_TEST_LOG_DIR` (default `/tmp/hk_fr5_tests`). Scripts that start a bringup run it headless
(`use_rviz:=false`); do not run them next to your own `pixi run sim`. After a run check for leftovers
(implementation-ref R-5b):
`ps -eo pid,args | grep -E "move_group|ros2_control_node|ros2 launch|_controller|scene_manager" | grep -v grep`

| Script | Needs | Checks | Last result |
|---|---|---|---|
| `gripper_io_check.py` | nothing (offline) | Debounce, reply parsing, FakeIo, XmlRpcIo vs `mock_fairino.py` (errors, 0.5 s timeout, refused), no `get_do` (H-2) | 26/26 |
| `gripper_node_check.py` | ROS | gripper_controller: bad config, fake state 10 Hz, real path vs mock: start reset both pumps off (H-7), interlock order (H-8), PART_LOST, CONTROLLER_ERROR, COMM_ERROR + auto recovery (W-13), goals refused + reset retried until it works, shutdown both pumps off | 27/27 |
| `gripper_actions_check.py` | ROS | grip / release: success, timeouts, invalid goals, cancel, preempt (W-14), PART_LOST, exact SetToolDO sequences vs mock (start reset, interlock), comm / controller errors | 28/28 |
| `gripper_launch_check.sh` | bringup | gripper in the bringup, CLI grip / release, use_gripper:=bad, SIGTERM -> pumps off (*printed*) | OK |
| `shell_gripper_check.py` | bringup | motion_shell grip / release / gstate, errors, Ctrl+C cancel (*printed*) | OK |
| `shell_no_gripper_check.py` | bringup | motion_shell with use_gripper:=false, gripper started / stopped / killed mid-goal (W-17) (*printed*) | OK |
| `scene_smoke_check.py` | bringup | /scene/* services: attach (MoveIt shape), detach keep -> stack_0, add / remove, every rejection, collision warning | 17/17 |
| `scene_motion_check.py` | bringup | scene vs planning: pick + lift with margin, default detach, blocking, place + stack_0 / stack_1, wall, T25 (OMPL from contact fails), motion regression | 28/28 |
| `scene_launch_check.py` | bringup | static boxes identical to the obstacle_loader reference, use_obstacles:=false, no obstacle_loader; Ctrl+C line is INFO (T26) | 6/6 + INFO |
| `shell_scene_check.py` | bringup | motion_shell attach / detach / box, without and with scene_manager (*printed*) | OK |
| `joint_config_check.py` | bringup | arm configuration (H-11): lab replay x3, table grid, left <-> right rows incl. pickup z -0.055; elbow up / wrist down / \|j1\| <= 175 / j2 -165..+10, j1 detour <= 30 deg, elbow above the shoulder-wrist line; unreachable points listed | 83/83 |
| `ctrl_c_probe.py` | bringup | informative (T26): nodes "finished cleanly" vs "died" after a terminal-style Ctrl+C | 3 clean / 5 died (known) |

Helpers: `testenv.py` (installed paths via the ament index, log dir), `scene_lib.py` (headless bringup, planning
scene reader), `mock_fairino.py` (XML-RPC server with SetToolDO / GetToolDI + test hooks; GetToolDI reply `[error, value]` as
on the real controller, decision-log H-1; no GetToolDO, H-2).

Not covered here (lost with earlier session scratchpads, documented instead, decision-log X-1): the motion tests of
steps 15-18 (implementation-ref 10i-10l). `scene_motion_check.py` re-checks the main motion paths (move_to,
move_delta, cancel).
