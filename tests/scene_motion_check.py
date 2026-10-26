"""T20c: scene objects vs real planning (sim). Table top -0.060 (measured, decision-log H-10), board 270 x 200 x 6 mm."""
import math, time, rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from moveit_msgs.srv import GetPlanningScene
from hk_fr5_interfaces.action import MoveTo, MoveDelta
from hk_fr5_interfaces.srv import AttachBoard, DetachBoard, AddBox, RemoveBox, GetState
from scene_lib import *  # noqa: F401,F403 (also sets up testenv)
TOP, H, YAW = -0.060, 0.006, math.pi / 2
PICK, STACK = (-0.45, -0.25), (-0.45, 0.25)
rclpy.init(); n = rclpy.create_node("scene_motion_test")
gps = n.create_client(GetPlanningScene, "/get_planning_scene"); gst = n.create_client(GetState, "/motion/get_state")
c = {k: n.create_client(t, f"/scene/{k}") for k, t in (("attach_board", AttachBoard), ("detach_board", DetachBoard), ("add_box", AddBox), ("remove_box", RemoveBox))}
mt, md = ActionClient(n, MoveTo, "/motion/move_to"), ActionClient(n, MoveDelta, "/motion/move_delta")
ok = 0
def check(name, cond, extra=""):
    global ok
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True); ok += bool(cond)
    if not cond: raise SystemExit(1)
def until(f, t=30.0):
    end = time.time() + t
    while not f.done() and time.time() < end: rclpy.spin_once(n, timeout_sec=0.01)
    return f.result()
def act(client, goal, cancel_after=None):
    h = until(client.send_goal_async(goal)); r = h.get_result_async()
    if cancel_after:
        end = time.time() + cancel_after
        while time.time() < end: rclpy.spin_once(n, timeout_sec=0.01)
        until(h.cancel_goal_async())
    res = until(r, 60.0); return res.status, res.result
def to(x, y, z, planner="", plan_only=False, v=0.5):
    return act(mt, MoveTo.Goal(x=x, y=y, z=z, yaw=YAW, planner=planner, plan_only=plan_only, velocity_scaling=v))[1]
def dz(d, plan_only=False):
    return act(md, MoveDelta.Goal(dz=d, frame=MoveDelta.Goal.FRAME_BASE, plan_only=plan_only, velocity_scaling=0.5))[1]
def tcp_z(): return call(n, gst, GetState.Request()).tcp_pose.pose.position.z
def down_to(z, plan_only=False): return dz(z - tcp_z(), plan_only)   # absolute tcp z (base frame)
def attach(margin=0.0): return call(n, c["attach_board"], AttachBoard.Request(length=0.27, width=0.20, height=H, contact_margin=margin))
def detach(**kw): return call(n, c["detach_board"], DetachBoard.Request(**kw))
def code(r): return f"{r.error_code.val}: {r.message}"

L = launch("use_gripper:=false", log="t20c_launch.log"); time.sleep(18); S = None   # T20d: scene_manager from the bringup
try:
    assert mt.wait_for_server(timeout_sec=20) and md.wait_for_server(timeout_sec=20)
    # --- 1. pick: board on the table at PICK, cup tip on its top -> attach -> lift (Pilz LIN)
    r = to(*PICK, 0.10); check("pick: move above pick spot (OMPL)", r.success, code(r))
    r = down_to(TOP + H); z = tcp_z()
    check(f"pick: LIN down to board top (tcp z = {TOP + H:.3f})", r.success and abs(z - (TOP + H)) < 2e-4, f"{code(r)}; z {z:.4f}")
    a = attach()
    check("attach on the table (2 mm margin): no collision warning", a.success and "WARNING" not in a.message, a.message)
    r = dz(0.10); check("lift 100 mm with the board (LIN): SUCCESS (margin works)", r.success, code(r))
    # --- 2. the same without margin (1e-6 m): contact with the table
    r = dz(-0.10); check("back down with the board (LIN, board bottom 2 mm above table)", r.success, code(r))
    d = detach(); check("detach default (keep_as_obstacle false): removed, name ''", d.success and d.name == "" and "removed" in d.message, d.message)
    w = world(scene(n, gps)); check("default detach: no board / stack in the world", "board" not in w and not any(k.startswith("stack_") for k in w), str(sorted(w)))
    a = attach(1e-6)
    print(f"INFO attach with margin 1e-6 m (board bottom = table top): {a.message}")
    r0 = to(*PICK, 0.10, plan_only=True)
    print(f"INFO OMPL move_to PLAN from contact (margin 1e-6 m): code {code(r0)}")
    check("T25: OMPL from a start state in collision now FAILS (no hidden start-state jump)", not r0.success, code(r0))
    r = dz(0.10, plan_only=True)
    print(f"INFO Pilz LIN lift PLAN with margin 1e-6 m: code {code(r)}")
    r2 = dz(0.10); z = tcp_z()
    print(f"INFO lift EXECUTE with margin 1e-6 m: code {code(r2)}; tcp z after {z:.4f}")
    check("without margin the board touches the table (state check warns)", "WARNING" in a.message and "board <-> table" in a.message, a.message[-80:])
    detach(); r = down_to(TOP + H) if z > TOP + H + 0.001 else None
    # --- 3. attached board blocks a goal that is free without it
    ZB = TOP + 0.002   # tcp 2 mm above the table: cup collision ends 5 mm above the tcp -> free without a board
    r = to(*PICK, 0.10); r2 = dz(ZB - 0.10, plan_only=True)
    check(f"no board: tcp to z {ZB:.3f} is a valid goal (plan_only)", r.success and r2.success, code(r2))
    attach(); r2 = dz(ZB - 0.10, plan_only=True)
    check("with board: same goal refused (board bottom would be 2 mm below the table top)", not r2.success, code(r2))
    # --- 4. place on the table at STACK, keep as stack_0
    r = to(*STACK, 0.10); check("carry board to the stack spot (OMPL with attached board)", r.success, code(r))
    r = down_to(TOP + H); z = tcp_z()
    check(f"place on the table: LIN down to tcp z {TOP + H:.3f}", r.success and abs(z - (TOP + H)) < 2e-4, f"{code(r)}; z {z:.4f}")
    d = detach(keep_as_obstacle=True); sc = world(scene(n, gps))
    st = sc.get("stack_0")
    check("detach keep -> stack_0, name returned", d.success and d.name == "stack_0" and st is not None, d.message)
    check(f"stack_0 geometry: 4 mm box, centre z {TOP + H - 0.002:.3f} (top {TOP + H:.3f} = real top, bottom 2 mm above table)",
          st[0] == (0.27, 0.2, 0.004) and abs(st[2][2] - (TOP + H - 0.002)) < 2e-4, str(st))
    r = dz(0.10); check("lift after place (cup 5 mm above stack_0 top)", r.success, code(r))
    # --- 5. stack_0 now blocks the old place height; placing on top works
    a = attach(); r = down_to(TOP + H, plan_only=True)
    check("next board: old place height refused (stack_0 is there)", a.success and not r.success, code(r))
    r = down_to(TOP + 2 * H); z = tcp_z()
    check(f"place on stack_0: tcp z = {TOP + 2 * H:.3f} (stack top + 6 mm): SUCCESS", r.success and abs(z - (TOP + 2 * H)) < 2e-4, f"{code(r)}; z {z:.4f}")
    d = detach(keep_as_obstacle=True); check("detach keep -> stack_1", d.success and d.name == "stack_1", d.message)
    r = dz(0.10); check("lift after 2nd place", r.success, code(r))
    # --- 6. OMPL avoids a dynamic box in the way
    b = call(n, c["add_box"], AddBox.Request(name="wall", size=[0.30, 0.02, 0.30], position=[-0.45, 0.0, 0.15], yaw=0.0))
    check("wall between the spots: no collision warning at the current state", b.success and "WARNING" not in b.message, b.message)
    r = to(*PICK, 0.10, planner="LIN", plan_only=True)
    check("dynamic wall between stack and pick: straight LIN refused", b.success and not r.success, code(r))
    r = to(*PICK, 0.10); check("OMPL plans around the wall and moves", r.success, code(r))
    rm = call(n, c["remove_box"], RemoveBox.Request(name=""))
    check("remove all dynamic: wall + stack_0 + stack_1", rm.success and "3 boxes" in rm.message, rm.message)
    # --- 7. motion regression
    r = act(mt, MoveTo.Goal(named_target="ready", velocity_scaling=0.5))[1]; check("regression: move_to ready", r.success, code(r))
    r = act(md, MoveDelta.Goal(dx=0.05, velocity_scaling=0.5))[1]; check("regression: move_delta tcp frame dx 50 mm", r.success, code(r))
    st_, r = act(md, MoveDelta.Goal(dz=-0.15, frame=MoveDelta.Goal.FRAME_BASE, velocity_scaling=0.1), cancel_after=1.0)
    check("regression: cancel during move -> CANCELED + PREEMPTED code", st_ == GoalStatus.STATUS_CANCELED and r.error_code.val == -7, code(r))
    r = act(mt, MoveTo.Goal(named_target="ready", velocity_scaling=0.5))[1]; check("regression: back to ready after cancel", r.success, code(r))
finally:
    stop(L)
    print(f"{ok} checks passed")
    print("--- scene_manager log ---"); print("\n".join(l for l in open("t20c_launch.log").read().splitlines() if "scene_manager" in l and "added '" not in l))
    n.destroy_node(); rclpy.shutdown()
