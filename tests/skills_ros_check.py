"""Skills in the full sim stack (T4, V5 TM-24: real variants, Vision 1 JSON, zone sheets): motion bringup (motion
worktree, fake arm + fake gripper + scene, headless), fake_world (+ contact simulation), world_model, skill_server. Every skill and every failure code
that can be provoked. ~2-3 min.

    ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=83 pixi run python3 tests/skills_ros_check.py
"""
import math
import re
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile

import json

from std_msgs.msg import String

from hk_fr5_interfaces.action import Release
from hk_fr5_interfaces.msg import GripperState
from hk_fr5_interfaces.srv import DetachBoard, GetState
from hk_fr5_task_interfaces.action import Home, Pick, Place, ReleaseSafe, WaitClear
from hk_fr5_task_interfaces.msg import SkillCode as C, World

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

HERE = os.path.dirname(os.path.abspath(__file__))
MOTION_WT = os.environ.get("HK_MOTION_WT") or os.path.realpath(os.path.join(HERE, "..", "..", "feat-robot-motion-gripper"))
PIXI = shutil.which("pixi") or os.path.expanduser("~/.pixi/bin/pixi")
TABLE = -0.060
def exe(pkg, name):
    return os.path.join(get_package_prefix(pkg), "lib", pkg, name)
LOG_DIR = os.environ.get("HK_TEST_LOG_DIR", tempfile.gettempdir())
os.makedirs(LOG_DIR, exist_ok=True)
def start(name, cmd, cwd=None, env=None):
    """Output straight into a log file. NEVER stdout=PIPE read only at the end: move_group logs > 64 KB, the full pipe
    blocks its next log write and freezes planning / execution (that was the 'move_group hang', TM-11 / TM-13)."""
    log = open(os.path.join(LOG_DIR, f"skills_check_{name}.log"), "w")
    p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, cwd=cwd, env=env)
    p.log_path = log.name
    log.close()
    return p
def stop(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
    killed = ""
    try:
        p.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL); p.wait(); killed = " [KILLED]"
    with open(p.log_path) as f:
        return f.read() + killed

tmp = tempfile.mkdtemp(prefix="skills_check_")
cfg = yaml.safe_load(open(os.path.join(get_package_share_directory("hk_fr5_task"), "config", "skills.yaml")))
cfg.update(speed=float(os.environ.get("HK_SPEED", "1.0")), approach_speed=0.25, search_speed=0.02, verify_timeout=3.0,
           carry_speed=0.6)      # TM-37: distinct from speed (checked in the skill_server step log)     # sim: fast (TM-4)
yaml.safe_dump(cfg, open(os.path.join(tmp, "skills.yaml"), "w"))
procs = {"motion": start("motion", [PIXI, "run", "--manifest-path", os.path.join(MOTION_WT, "pixi.toml"), "ros2", "launch",
                                    "hk_fr5_moveit_config", "bringup.launch.py", "use_rviz:=false"], cwd=MOTION_WT),
         "fake": start("fake", [exe("hk_fr5_task", "fake_world"), "--ros-args", "-p", "contact_sim:=true", "-p", "seed:=2"]),
         "world": start("world", [exe("hk_fr5_task", "world_model")]),
         "skills": start("skills", [exe("hk_fr5_task", "skill_server"), "--ros-args", "-p", f"config:={tmp}/skills.yaml"],
                         env=dict(os.environ, HK_VERBOSE="1"))}     # step log with the speed of every goal (TM-37)
rclpy.init(); n = rclpy.create_node("skills_test")
L = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
S = {"w": None, "g": None, "seal_z": None}
tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n)
REL = []                                          # TM-32: [t_start, t_end] of every RELEASING interval
ZS = []                                           # (t, tcp z) sampled while a skill runs
def on_gripper(m):
    prev = S["g"].state if S["g"] is not None else ""
    if m.state == GripperState.RELEASING and prev != GripperState.RELEASING:
        REL.append([time.time(), None])
    elif prev == GripperState.RELEASING and m.state != GripperState.RELEASING and REL:
        REL[-1][1] = time.time()
    if m.state == GripperState.HOLDING and (S["g"] is None or S["g"].state != GripperState.HOLDING):
        try:
            S["seal_z"] = tfb.lookup_transform("base_link", "tcp", rclpy.time.Time()).transform.translation.z
        except Exception:
            S["seal_z"] = None
    S["g"] = m
n.create_subscription(World, "/task/world", lambda m: S.update(w=m), L)
n.create_subscription(GripperState, "/gripper/state", on_gripper, L)
A = {k: ActionClient(n, t, f"/task/skills/{k}") for k, t in
     (("pick", Pick), ("place", Place), ("home", Home), ("wait_clear", WaitClear), ("release_safe", ReleaseSafe))}
fw_param = n.create_client(SetParameters, "/task/fake_world/set_parameters")
rel = ActionClient(n, Release, "/gripper/release")
det = n.create_client(DetachBoard, "/scene/detach_board")
gs = n.create_client(GetState, "/motion/get_state")
ev = n.create_publisher(String, "/task/events", 10)
def motion_state_ok():
    """motion_controller has joint states + TF (fix: run 7 started before /joint_states, ROBOT_STATE_STALE)."""
    if not gs.service_is_ready():
        return False
    f = gs.call_async(GetState.Request())
    end = time.time() + 2
    while not f.done() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
    return f.done() and f.result().success
def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
def until(fn, timeout):
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
        try:
            if fn():
                return True
        except (AttributeError, StopIteration, TypeError):
            pass
    return False
def set_fake(name, value):
    """Set a fake_world parameter. Call only from the main loop, never from a ROS callback (no nested spin)."""
    p = Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value))
    f = fw_param.call_async(SetParameters.Request(parameters=[p])); until(lambda: f.done(), 3)
HANG = {"dumped": False}
def dump_move_group(reason):
    """move_group stopped answering (TM-11): save all thread backtraces (needs ptrace permission, see TM-13)."""
    if HANG["dumped"]:
        return
    HANG["dumped"] = True
    pids = [l.split()[0] for l in subprocess.run(["ps", "-eo", "pid,args"], capture_output=True, text=True).stdout.splitlines()
            if "/move_group " in l and "--ros-args" in l]
    out = os.path.join(LOG_DIR, "move_group_backtrace.txt")
    if not pids:
        print(f"INFO hang ({reason}): no move_group process found", flush=True); return
    r = subprocess.run(["gdb", "-p", pids[0], "-batch", "-ex", "thread apply all bt"], capture_output=True, text=True, timeout=120)
    with open(out, "w") as f:
        f.write(f"reason: {reason}\npid {pids[0]}\n\n{r.stdout}\n{r.stderr}")
    denied = "ptrace" in r.stderr.lower() or "not permitted" in r.stderr.lower()
    print(f"INFO hang ({reason}): move_group backtrace -> {out}" + (" (ATTACH DENIED: ptrace_scope)" if denied else ""), flush=True)
def run(name, goal, timeout=60, hook=None):
    """Send a skill goal and wait for the result. hook(phase, goal_handle) is called from THIS loop (not from a ROS
    callback, so it may spin / call services) whenever a new feedback phase arrives. On timeout the goal is
    cancelled and its end awaited, so the next skill is not refused as 'busy'."""
    phases, gh_box = [], {}
    f = A[name].send_goal_async(goal, feedback_callback=lambda fb: phases.append(getattr(fb.feedback, "phase", "")))
    if not until(lambda: f.done(), 5) or not f.result().accepted:
        return None
    gh = gh_box["gh"] = f.result()
    r = gh.get_result_async()
    seen, end = 0, time.time() + timeout
    while not r.done() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
        try:
            ZS.append((time.time(), tfb.lookup_transform("base_link", "tcp", rclpy.time.Time()).transform.translation.z))
        except Exception:
            pass
        while hook is not None and seen < len(phases):
            seen += 1
            hook(phases[seen - 1], gh)
    if not r.done():
        print(f"INFO {name}: no result in {timeout} s -> cancelling", flush=True)
        gh.cancel_goal_async(); until(lambda: r.done(), 30)
    res = r.result().result if r.done() else None
    if res is not None and "motion stack hung" in res.message:
        dump_move_group(f"{name}: {res.message}")
    return res
def board(variant=None, bid=None):
    w = S["w"]
    return next((b for b in (w.boards if w else []) if (bid is None or b.id == bid)
                 and (variant is None or b.variant == variant) and b.location != "held"), None)
def board_near(x, y, tol=0.04):
    w = S["w"]
    return next((b for b in (w.boards if w else []) if math.hypot(b.x - x, b.y - y) < tol and b.location != "held"), None)
def bid(b):
    return b.id if b is not None else "missing"
def reset(label):
    """Known state before a test group: nothing on the cup (release + scene detach), robot home."""
    if S["g"] is not None and S["g"].state != GripperState.IDLE:
        rf = rel.send_goal_async(Release.Goal()); until(lambda: rf.done(), 5)
        if rf.done() and rf.result().accepted:
            rr = rf.result().get_result_async(); until(lambda: rr.done(), 5)
    if S["w"] is not None and S["w"].held_board_id:
        df = det.call_async(DetachBoard.Request(keep_as_obstacle=False)); until(lambda: df.done(), 5)
        ev.publish(String(data=json.dumps({"type": "dropped", "board": S["w"].held_board_id})))
        until(lambda: S["w"].held_board_id == "", 3)
    r = run("home", Home.Goal(), timeout=40)
    if r is None or not r.success:
        print(f"INFO reset before '{label}': home failed: {desc(r)}", flush=True)
STACK = (-0.21, -0.68)                           # input stack of 3 VARb (scenario_demo)
TH = 0.005                                       # board thickness (world.yaml VARa / VARb / VARc)
def zone(name):
    return next(z for z in S["w"].zones if z.name == name)
def desc(r):
    return "no result" if r is None else f"{r.code.value} {r.message}"

outs = {}
t0 = time.time()
try:
    from hk_fr5_interfaces.action import MoveTo
    mv = ActionClient(n, MoveTo, "/motion/move_to")
    from hk_fr5_interfaces.srv import AttachBoard
    at = n.create_client(AttachBoard, "/scene/attach_board")
    ready = until(lambda: all(a.server_is_ready() for a in A.values()) and mv.server_is_ready() and at.service_is_ready()
                  and S["w"] is not None and S["w"].vision_ok and len(S["w"].boards) == 3 and S["g"] is not None, 120)
    ready = ready and until(motion_state_ok, 60)    # valid robot state in the motion controller
    spin(3.0)                                    # move_group finishes loading the scene
    check("stack up: skills, world (3 boards), gripper, robot state", ready, f"{time.time() - t0:.0f} s")
    r = run("home", Home.Goal())
    check("home", r and r.success, desc(r))
    spin(1.0)

    red = board("VARa")
    r = run("pick", Pick.Goal(board_id=bid(red)))
    check("pick VARa from the input area by contact search: success, world holds it", r and r.success
          and "contact search" in r.message and until(lambda: S["w"].held_board_id == red.id, 2) and r.variant == "VARa", desc(r))
    sz = S["seal_z"]
    check("contact search sealed at the board top (table + 5 mm), not in the air / below", sz is not None
          and TABLE + TH - 0.015 <= sz <= TABLE + TH + 0.001, f"seal tcp z {sz}")
    n_rel = len(REL)
    r = run("place", Place.Goal(target="processing_1"))
    check("place VARa in processing_1: success, camera confirmed (settle check), zone LOADED", r and r.success
          and until(lambda: zone("processing_1").state in ("LOADED", "PROCESSING"), 3), desc(r))
    riv = REL[n_rel] if len(REL) > n_rel else None
    z0 = next((z for t, z in ZS if riv and t >= riv[0]), None)
    t_up = next((t for t, z in ZS if riv and z0 is not None and t >= riv[0] and z > z0 + 0.005), None)
    check("TM-32 retract starts 0.3-2 s after the release starts, while the blow-off still runs (blow_off_time "
          f"{cfg['blow_off_time']} s)", riv is not None and riv[1] is not None and t_up is not None
          and 0.3 <= t_up - riv[0] <= 2.0 and t_up < riv[1],
          f"release {riv and riv[1] and round(riv[1] - riv[0], 2)} s, retract after {t_up and riv and round(t_up - riv[0], 2)} s")

    ws = board_near(*STACK)
    r = run("pick", Pick.Goal(board_id=bid(ws)))
    sz = S["seal_z"]
    check("pick from an input stack of 3: contact search sealed on the stack top (table + 15 mm)", r and r.success
          and sz is not None and TABLE + 3 * TH - 0.015 <= sz <= TABLE + 3 * TH + 0.001, f"{desc(r)}; seal z {sz}")
    r = run("place", Place.Goal(target="drop_off_b", skip_home=True))
    try:
        tp = tfb.lookup_transform("base_link", "tcp", rclpy.time.Time()).transform.translation
    except Exception:
        tp = None
    zb = zone("drop_off_b")
    check("TM-36 place on drop_off_b with skip_home: success, stack count 1, the arm stays at hover over the zone "
          "(no home move, no blocking camera check)", r and r.success and r.stack_count == 1 and "no home" in r.message
          and tp is not None and abs(tp.x - zb.x) < 0.01 and abs(tp.y - zb.y) < 0.01 and abs(tp.z - cfg["hover_z"]) < 0.01,
          f"{desc(r)}; tcp {tp and (round(tp.x, 3), round(tp.y, 3), round(tp.z, 3))}")

    r = run("place", Place.Goal(target="drop_off_b"))
    check("place without a board: NOT_HOLDING", r and r.code.value == C.NOT_HOLDING, desc(r))
    r = run("pick", Pick.Goal(board_id="b99"))
    check("pick an unknown board: BOARD_NOT_FOUND", r and r.code.value == C.BOARD_NOT_FOUND, desc(r))

    check("processing_1: simulated operator -> DONE, VARa processed", until(lambda: zone("processing_1").state == "DONE"
          and board(bid=bid(red)).processed, 30))           # operator 5 s + gone 2 s + 5 s + back 2 s (CL-40)
    r = run("pick", Pick.Goal(board_id=bid(red)))
    check("pick the processed VARa board from processing_1 (known top, no search)", r and r.success
          and "contact search" not in r.message and until(lambda: zone("processing_1").state == "EMPTY", 2), desc(r))
    r = run("place", Place.Goal(target="drop_off_b"))
    check("place it on drop_off_b: stack count 2 (placed on top of the first board)", r and r.success and r.stack_count == 2, desc(r))
    top = S["w"].zones and next((b for b in S["w"].boards if b.location == "zone" and b.zone == "drop_off_b"
                                 and b.age < 1.0), None)
    r = run("pick", Pick.Goal(board_id=bid(top)))
    r2 = run("place", Place.Goal(target="drop_off_b"))
    check("pick the top board off the drop_off_b stack (stack box rebuilt without it, S-11 / CL-38), place it back on the "
          "same stack (no leftover box in the way)", r and r.success and r2 and r2.success and r2.stack_count == 2,
          f"{desc(r)} / {desc(r2)}")

    reset("busy processing zone")
    ws = board_near(*STACK)
    r = run("pick", Pick.Goal(board_id=bid(ws))); r2 = run("place", Place.Goal(target="processing_1"))
    check("VARb (stack 2 left) -> processing_1 LOADED", r and r.success and r2 and r2.success, f"{desc(r)} / {desc(r2)}")
    ws = board_near(*STACK)
    r = run("pick", Pick.Goal(board_id=bid(ws)))
    r2 = run("place", Place.Goal(target="processing_1"))
    check("place into the busy processing_1: TARGET_OCCUPIED, board still held", r and r.success and r2
          and r2.code.value == C.TARGET_OCCUPIED and until(lambda: S["g"].state == GripperState.HOLDING, 1), desc(r2))
    r = run("place", Place.Goal(target="drop_off_a"))
    check("so place it on drop_off_a instead", r and r.success, desc(r))

    reset("grip failed")
    wl = board("VARc")
    set_fake("force_vacuum", "off")
    r = run("pick", Pick.Goal(board_id=bid(wl)), timeout=90)
    set_fake("force_vacuum", "")
    check("no seal (vacuum forced off): GRIP_FAILED after the retry, nothing held, back at hover", r
          and r.code.value == C.GRIP_FAILED and until(lambda: S["g"].state != GripperState.HOLDING
                                                      and S["w"].held_board_id == "", 2), desc(r))

    reset("lost in transit")
    wl = board("VARc")
    r = run("pick", Pick.Goal(board_id=bid(wl)))
    lost = {"done": False}
    def lose(phase, gh):                             # runs in run()'s loop, not in a ROS callback
        if phase == "HOVER" and not lost["done"]:
            lost["done"] = True; set_fake("force_vacuum", "off")
    r2 = run("place", Place.Goal(target="drop_off_a"), hook=lose)
    set_fake("force_vacuum", "")
    check("vacuum lost while carrying: LOST_IN_TRANSIT, board dropped from the world, nothing held",
          r and r.success and r2 and r2.code.value == C.LOST_IN_TRANSIT
          and until(lambda: S["w"].held_board_id == "" and all(b.id != bid(wl) for b in S["w"].boards), 3), desc(r2))

    reset("obstacle")
    red2 = board("VARa")
    def hand(phase, gh):
        if phase == "HOVER":
            set_fake("hand_now", "-0.40 0.0 2.5")
    r = run("pick", Pick.Goal(board_id=bid(red2)), hook=hand)
    check("hand in the work area during a move: OBSTACLE (smooth stop)", r and r.code.value == C.OBSTACLE, desc(r))
    t1 = time.time(); r = run("wait_clear", WaitClear.Goal(timeout=10.0))
    check("wait_clear: succeeds after the hand is gone (+ clear time)", r and r.success and time.time() - t1 > 0.5, desc(r))

    reset("cancel")
    def cancel(phase, gh):
        if phase == "HOVER":
            gh.cancel_goal_async()
    r = run("pick", Pick.Goal(board_id=bid(board("VARa"))), hook=cancel)
    check("cancel during a move: CANCELLED", r and r.code.value == C.CANCELLED, desc(r))

    reset("release_safe")
    pb = board(bid=zone("processing_1").board_id) if zone("processing_1").board_id else None   # input stack is used up
    r = run("pick", Pick.Goal(board_id=bid(pb)))
    r2 = run("release_safe", ReleaseSafe.Goal())
    check("release_safe while holding: dropped into the fault zone (a void: the board leaves the world, CL-39), home",
          r and r.success and r2 and r2.success and "fault" in r2.message and until(lambda: board(bid=bid(pb)) is None
          and S["w"].held_board_id == "" and zone("fault").stack_count == 0, 3), f"{desc(r)} / {desc(r2)}")
    reset("release_safe with an empty cup")
    src = next((b for b in S["w"].boards if b.location == "zone" and b.zone in ("drop_off_a", "drop_off_b") and b.age < 1.0), None)
    r = run("pick", Pick.Goal(board_id=bid(src)))                         # a stack board (the fault zone is a void)
    set_fake("force_vacuum", "off")                                       # operator takes it off the cup (no skill running)
    lost = until(lambda: S["g"].state == GripperState.PART_LOST, 5)
    r2 = run("release_safe", ReleaseSafe.Goal())
    set_fake("force_vacuum", "")
    check("release_safe, world holds a board but the cup is empty: NOT_HOLDING (CL-32), no move, world unchanged",
          r and r.success and lost and r2 and r2.code.value == C.NOT_HOLDING and S["w"].held_board_id != "",
          f"{desc(r)} / lost {lost} / {desc(r2)}")
    reset("one skill at a time")
    f1 = A["home"].send_goal_async(Home.Goal()); until(lambda: f1.done(), 5)
    f2 = A["home"].send_goal_async(Home.Goal()); until(lambda: f2.done(), 5)
    check("a 2nd skill while one runs is refused (one skill at a time)", f1.result().accepted and not f2.result().accepted)
    until(lambda: f1.result().get_result_async().done(), 30)
finally:
    for k in [k for k in ("skills", "world", "fake") if k in procs] + [k for k in procs if k not in ("skills", "world", "fake")]:
        outs[k] = stop(procs[k])                     # every started process (motion last); logs in LOG_DIR/skills_check_<k>.log
    n.destroy_node(); rclpy.shutdown()
    shutil.rmtree(tmp, ignore_errors=True)
carry = re.findall(r"\] carry to [^:]*: move_to .* speed (\d+\.\d+)", outs["skills"])
hov = re.findall(r"\] hover over the board: move_to .* speed (\d+\.\d+)", outs["skills"])
check(f"TM-37: carries at carry_speed {cfg['carry_speed']}, hover moves at speed {cfg['speed']}",
      carry and hov and all(float(v) == round(cfg["carry_speed"], 2) for v in carry)
      and all(float(v) == round(cfg["speed"], 2) for v in hov), f"carry {sorted(set(carry))} hover {sorted(set(hov))}")
for k in ("skills", "world", "fake"):
    check(f"{k}: no traceback, clean exit", "Traceback" not in outs[k] and "[KILLED]" not in outs[k], outs[k][-300:])
print(f"total {time.time() - t0:.0f} s")
print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
