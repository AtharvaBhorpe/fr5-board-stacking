"""Main controller in the full sim stack (T5c-2): motion bringup (motion worktree, fake arm + fake gripper + scene,
headless), fake_world (Vision 1 JSON + contact simulation, own scenario; V5 TM-24), world_model, skill_server and
task_main_controller - every real node; the test only talks to the controller's API (/task/add_job, list_jobs,
control, status) and pokes the fake world (no vacuum, a hand). ~7-9 min. Node output -> log files (TM-13).

    pixi run python3 tests/main_ros_check.py          (HK_TEST_LOG_DIR = where the node logs go)

A  the user's scheduling example (CL-22): process 2 VARc via processing_1 -> drop_off_a (prio 10) + transfer the
   VARb stack -> drop_off_b (prio 5) -> the exact interleaved order, counters, stacks
B  vacuum lost mid-carry (fake vacuum off) -> lost 1, the fallen board is a new input board, picked again
   -> delivered 1 (TM-38); failures + commands on an extra board / the drop-off stacks (clear_zone picks from stacks): no vacuum -> skip, hand -> wait_clear,
   pause after a pick, stop while holding -> release_safe on resume, cancel_job while holding, invalid job, list_jobs,
   Ctrl+C on the controller during a skill
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
os.environ.setdefault("ROS_DOMAIN_ID", "85")

import rclpy                                                   # noqa: E402
import yaml                                                    # noqa: E402
from ament_index_python.packages import get_package_prefix, get_package_share_directory   # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue                    # noqa: E402
from rcl_interfaces.srv import SetParameters                   # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile             # noqa: E402
from std_msgs.msg import String                                # noqa: E402

from hk_fr5_interfaces.msg import GripperState                 # noqa: E402
from hk_fr5_interfaces.srv import GetState                     # noqa: E402
from hk_fr5_task_interfaces.msg import Job, World              # noqa: E402
from hk_fr5_task_interfaces.srv import AddJob, Control, ListJobs   # noqa: E402

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

HERE = os.path.dirname(os.path.abspath(__file__))
MOTION_WT = os.environ.get("HK_MOTION_WT") or os.path.realpath(os.path.join(HERE, "..", "..", "feat-robot-motion-gripper"))
PIXI = shutil.which("pixi") or os.path.expanduser("~/.pixi/bin/pixi")
SHARE = get_package_share_directory("hk_fr5_task")
LOG_DIR = os.environ.get("HK_TEST_LOG_DIR", tempfile.gettempdir())
os.makedirs(LOG_DIR, exist_ok=True)
def exe(pkg, name):
    return os.path.join(get_package_prefix(pkg), "lib", pkg, name)
def start(name, cmd, cwd=None):
    """Output straight into a log file - never an unread stdout=PIPE (TM-13)."""
    log = open(os.path.join(LOG_DIR, f"main_check_{name}.log"), "w")
    p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, cwd=cwd)
    p.log_path = log.name
    log.close()
    return p
def stop(p, sig=signal.SIGINT):
    if p.poll() is None:
        os.killpg(p.pid, sig)
    killed = ""
    try:
        p.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL); p.wait(); killed = " [KILLED]"
    with open(p.log_path) as f:
        return f.read() + killed

# ---------- configs for the test (fast sim, short end timers) ----------
tmp = tempfile.mkdtemp(prefix="main_check_")
scn = yaml.safe_load(open(os.path.join(SHARE, "config", "scenario_demo.yaml")))
scn["boards"] = scn["boards"] + [{"variant": "VARc", "x": 0.02, "y": -0.60, "yaw": 1.5708},    # demo + a 2nd VARc
                                 {"variant": "VARc", "x": -0.30, "y": 0.72, "yaw": 1.5708}]   # TM-38: a 3rd (left by A)
#   (layout rules: scenario_demo.yaml header; reach 0.30-0.80 m - a board at 0.26 m: NO_IK_SOLUTION, W9)
yaml.safe_dump(scn, open(os.path.join(tmp, "scenario.yaml"), "w"))
sk = yaml.safe_load(open(os.path.join(SHARE, "config", "skills.yaml")))
sk.update(speed=float(os.environ.get("HK_SPEED", "1.0")), approach_speed=0.25, search_speed=0.02, verify_timeout=3.0)
sk.update(carry_speed=sk["speed"])                          # TM-37: sim carries as fast as the other moves
yaml.safe_dump(sk, open(os.path.join(tmp, "skills.yaml"), "w"))
mc = yaml.safe_load(open(os.path.join(SHARE, "config", "main.yaml")))
mc.update(end_wait=3.0, partial_after=6.0)
yaml.safe_dump(mc, open(os.path.join(tmp, "main.yaml"), "w"))

procs = {"motion": start("motion", [PIXI, "run", "--manifest-path", os.path.join(MOTION_WT, "pixi.toml"), "ros2", "launch",
                                    "hk_fr5_moveit_config", "bringup.launch.py", "use_rviz:=false"], cwd=MOTION_WT),
         "fake": start("fake", [exe("hk_fr5_task", "fake_world"), "--ros-args", "-p", f"scenario:={tmp}/scenario.yaml",
                                "-p", "contact_sim:=true", "-p", "seed:=2"]),
         "world": start("world", [exe("hk_fr5_task", "world_model")]),
         "skills": start("skills", [exe("hk_fr5_task", "skill_server"), "--ros-args", "-p", f"config:={tmp}/skills.yaml"]),
         "ctrl": start("ctrl", [exe("hk_fr5_task", "task_main_controller"), "--ros-args", "-p", f"config:={tmp}/main.yaml"])}

rclpy.init(); n = rclpy.create_node("main_test")
L = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
S = {"w": None, "st": None, "g": None, "ev": {}}
def on_status(m):
    st = json.loads(m.data)
    S["st"] = st
    for e in st["events"]:
        S["ev"][(e["t"], e["text"])] = e          # every event once, in time order (status keeps only the last 50)
n.create_subscription(World, "/task/world", lambda m: S.update(w=m), L)
n.create_subscription(String, "/task/status", on_status, L)
n.create_subscription(GripperState, "/gripper/state", lambda m: S.update(g=m), L)
add_c, ctl_c, lst_c = n.create_client(AddJob, "/task/add_job"), n.create_client(Control, "/task/control"), n.create_client(ListJobs, "/task/list_jobs")
gs = n.create_client(GetState, "/motion/get_state")
fw_param = n.create_client(SetParameters, "/task/fake_world/set_parameters")

def until(fn, timeout):
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        try:
            if fn():
                return True
        except (AttributeError, StopIteration, TypeError, KeyError, IndexError):
            pass
    return False
def call(cli, req, timeout=10.0):
    if not cli.wait_for_service(timeout_sec=5.0):
        return None
    f = cli.call_async(req)
    return f.result() if until(f.done, timeout) else None
def control(cmd, job_id="", prio=0):
    return call(ctl_c, Control.Request(command=cmd, job_id=job_id, priority=prio))
def add(**kw):
    r = call(add_c, AddJob.Request(job=Job(source="test", **kw)))
    return r
def jobs():
    r = call(lst_c, ListJobs.Request())
    return {j.id: j for j in r.jobs} if r else {}
def finished(*ids):
    J = jobs()
    return all(i in J and J[i].state in ("DONE", "CANCELLED") for i in ids)
def zone(name):
    return next(z for z in S["w"].zones if z.name == name)
def held():
    return S["w"].held_board_id
def set_fake(name, value):
    p = Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value))
    f = fw_param.call_async(SetParameters.Request(parameters=[p])); until(f.done, 3)
def events():
    return [e["text"] for _, e in sorted(S["ev"].items())]
def seq(job_ids):
    """Successful picks / places of these jobs, in order, from the controller's event log."""
    out = []
    for t in events():
        jid = t.split(":", 1)[0]
        if jid in job_ids and ": picked " in t:
            out.append(f"pick:{jid}")
        elif jid in job_ids and ": placed " in t:
            out.append(f"place:{jid}->{t.rsplit(' in ', 1)[1]}")
    return out
def idle():
    st = S["st"]
    return st["current"] is None and not st["paused"] and not st["halted"] and all(
        j["state"] in ("DONE", "CANCELLED") for j in st["jobs"])
def motion_ok():
    f = gs.call_async(GetState.Request())
    return until(f.done, 2) and f.result().success

t0 = time.time()
outs = {}
try:
    ready = until(lambda: S["st"] is not None and S["st"]["skills_ready"] and S["st"]["motion_ready"] and S["w"] is not None
                  and S["w"].vision_ok and len(S["w"].boards) == 5 and S["g"] is not None, 150)
    ready = ready and until(motion_ok, 60)
    time.sleep(3.0)                                    # move_group finishes loading the scene
    check("stack up: controller (skills + motion ready), world (5 boards), gripper", ready, f"{time.time() - t0:.0f} s")
    r = control("home")
    homed = r is not None and r.success and until(lambda: S["st"]["current"] is not None, 10) and until(idle, 60)
    check("control home while idle: the robot moves home (operator's home command)", homed, str(r and r.message))

    # ---------- A: the user's scheduling example ----------
    ra = add(type="process", select_variant="VARc", zone="processing_1", to="drop_off_a", count=2, priority=10)
    rb = add(type="transfer", select_variant="VARb", to="drop_off_b", count=0, priority=5)
    a, b = (ra.id if ra and ra.success else ""), (rb.id if rb and rb.success else "")
    check("add_job: both jobs accepted", bool(a and b), f"{ra and ra.message} / {rb and rb.message}")
    t_a = time.time()
    done_a = until(lambda: finished(a, b), 600)
    want = [f"pick:{a}", f"place:{a}->processing_1", f"pick:{b}", f"place:{b}->drop_off_b", f"pick:{a}", f"place:{a}->drop_off_a",
            f"pick:{a}", f"place:{a}->processing_1", f"pick:{b}", f"place:{b}->drop_off_b", f"pick:{a}", f"place:{a}->drop_off_a",
            f"pick:{b}", f"place:{b}->drop_off_b"]
    got = seq({a, b})
    check("user example: exact order (T1 into processing, T2 meanwhile, processed board back -> T1 first, repeat)",
          done_a and got == want, " | ".join(got))
    J = jobs()
    check("user example: j1 DONE 2 delivered, j2 DONE 3 delivered, drop-offs 2 + 3, processing_1 EMPTY, nothing held",
          done_a and J[a].state == "DONE" and J[a].delivered == 2 and J[b].state == "DONE" and J[b].delivered == 3
          and zone("drop_off_a").stack_count == 2 and zone("drop_off_b").stack_count == 3
          and zone("processing_1").state == "EMPTY" and held() == "",
          f"{J.get(a) and (J[a].state, J[a].delivered)} {J.get(b) and (J[b].state, J[b].delivered)}, "
          f"stacks {zone('drop_off_a').stack_count}/{zone('drop_off_b').stack_count}, {time.time() - t_a:.0f} s")

    # ---------- B: failures + commands ----------
    r = add(type="transfer", select_variant="VARc", to="drop_off_b", count=1)        # the VARc left over by A
    e = r.id if r and r.success else ""
    until(lambda: S["st"]["current"] is not None and S["st"]["current"]["skill"] == "place"
          and S["st"]["phase"] == "HOVER", 60)                                     # the carry starts: still over the pick
    # directly via fake_world, right at the carry start: at sim carry speed 1.0 the arm crosses the rail within ~1.5 s
    # (the board would be gone, correct but not this case); the tool itself: skills_ros_check + inject_check
    set_fake("force_vacuum", "off")
    lost_seen = until(lambda: any("LOST" in x for x in events()), 20)
    set_fake("force_vacuum", "")
    until(lambda: finished(e), 180)
    Je = jobs().get(e)
    ev_e = [x for x in events() if "LOST" in x or "lost" in x or "dropped" in x]
    check("TM-38 vacuum lost mid-carry: lost 1, the fallen board seen again as a new input board, "
          "picked again -> job DONE, delivered 1",
          lost_seen and Je is not None and Je.state == "DONE" and Je.lost == 1 and Je.delivered == 1,
          f"{Je and (Je.state, Je.delivered, Je.lost, Je.attempts, Je.message)}, events {ev_e[-3:]}")

    r = add(type="transfer", select_variant="VARa", to="drop_off_a", count=1)
    c = r.id if r and r.success else ""
    until(lambda: S["st"]["current"] is not None and S["st"]["current"]["skill"] == "pick", 30)
    until(lambda: S["st"]["phase"] in ("DESCEND", "GRIP"), 30)
    set_fake("force_vacuum", "off")
    until(lambda: any("GRIP_FAILED" in t for t in events()), 90)
    set_fake("force_vacuum", "")
    until(lambda: finished(c), 120)
    Jc = jobs().get(c)
    check("no vacuum on a board: GRIP_FAILED -> skipped for good (problem board), no other one -> job DONE partial",
          Jc is not None and Jc.state == "DONE" and Jc.skipped == 1 and Jc.delivered == 0 and len(S["st"]["problem_boards"]) == 1
          and "partial" in Jc.message,
          f"{Jc and (Jc.state, Jc.delivered, Jc.skipped)}, problems {S['st']['problem_boards']}")

    r = add(type="clear_zone", from_zone="drop_off_b", to="drop_off_a", count=1)
    d = r.id if r and r.success else ""
    until(lambda: S["st"]["current"] is not None and S["st"]["current"]["skill"] == "pick" and S["st"]["phase"] == "HOVER", 30)
    set_fake("hand_now", "-0.30 0.0 2.0")
    saw_wait = until(lambda: S["st"]["current"] is not None and S["st"]["current"]["skill"] == "wait_clear", 30)
    until(lambda: finished(d), 120)
    Jd = jobs().get(d)
    check("a hand during a pick: OBSTACLE -> wait_clear -> re-decided -> delivered (clear_zone drop_off_b -> drop_off_a)",
          saw_wait and any("OBSTACLE" in t for t in events()) and Jd is not None and Jd.state == "DONE" and Jd.delivered == 1,
          f"wait_clear seen {saw_wait}, {Jd and (Jd.state, Jd.delivered)}")

    r = add(type="clear_zone", from_zone="drop_off_b", to="drop_off_a", count=1)
    e = r.id if r and r.success else ""
    was_held = until(lambda: held() != "", 60)
    control("pause")
    placed = was_held and until(lambda: held() == "" and S["st"]["current"] is None, 60)
    n_ev = len(events()); time.sleep(4.0)
    quiet = not any(": picked " in t for t in events()[n_ev:]) and S["st"]["current"] is None
    control("resume")
    until(lambda: finished(e), 60)
    check("pause while holding: the board is placed first, then nothing moves until resume; resume -> job DONE",
          placed and quiet and jobs()[e].state == "DONE" and jobs()[e].delivered == 1,
          f"held {was_held}, placed {placed}, quiet {quiet}, {jobs()[e].state} delivered {jobs()[e].delivered}")

    r = add(type="clear_zone", from_zone="drop_off_b", to="drop_off_a", count=1)
    f_ = r.id if r and r.success else ""
    until(lambda: held() != "", 60)
    control("stop")
    stopped = until(lambda: S["st"]["paused"] and S["st"]["current"] is None, 30) and held() != ""
    time.sleep(1.0)
    control("resume")
    rs = until(lambda: S["st"]["current"] is not None and S["st"]["current"]["skill"] == "release_safe", 30)
    rb = S["st"]["current"]["board"] if rs else ""
    until(lambda: finished(f_), 90)
    Jf = jobs().get(f_)
    gone = until(lambda: all(bb.id != rb for bb in S["w"].boards), 5)
    check("stop while holding: holds (paused); resume -> release_safe into the fault zone (a void: the board leaves the "
          "world, CL-39), counted to_fault (CL-25)", stopped and rs and gone and Jf is not None and Jf.to_fault == 1
          and zone("fault").stack_count == 0 and held() == "",
          f"stopped {stopped}, release_safe {rs} ({rb}), gone {gone}, {Jf and (Jf.state, Jf.to_fault)}")

    r = add(type="clear_zone", from_zone="drop_off_a", to="drop_off_b", count=3)
    g = r.id if r and r.success else ""
    was_held = until(lambda: held() != "", 60)
    control("cancel_job", g)
    until(lambda: finished(g), 90)
    Jg = jobs().get(g)
    check("cancel_job while holding: its board is placed at the target first, then CANCELLED (CL-25)",
          was_held and Jg is not None and Jg.state == "CANCELLED" and Jg.delivered == 1 and held() == "",
          f"held {was_held}, {Jg and (Jg.state, Jg.delivered)}")

    r = add(type="process", select_variant="VARc", zone="nowhere", to="drop_off_a", count=1)
    check("invalid job rejected with a reason (unknown zone), nothing moves", r is not None and not r.success
          and "processing zone" in r.message and S["st"]["current"] is None, str(r and r.message))
    J = jobs()
    st = S["st"]
    check("list_jobs = the jobs in /task/status (ids + states); status JSON has the controller fields",
          {j["id"]: j["state"] for j in st["jobs"]} == {k: v.state for k, v in J.items()}
          and all(k in st for k in ("state", "skills_ready", "motion_ready", "world_ok", "tcp", "phase")),
          f"{len(J)} jobs")

    # Ctrl+C on the controller during a skill: the skill is cancelled (smooth stop), clean exit
    r = add(type="clear_zone", from_zone="drop_off_a", to="drop_off_b", count=1)
    h = r.id if r and r.success else ""
    until(lambda: S["st"]["current"] is not None and S["st"]["current"]["skill"] == "pick"
          and S["st"]["current"]["job"] == h and S["st"]["phase"] == "HOVER", 90)     # this job's pick, not an older skill
    outs["ctrl"] = stop(procs["ctrl"])
    skill_log = open(procs["skills"].log_path).read()
    check("Ctrl+C on the controller during a skill: the skill is cancelled (smooth stop), the controller exits cleanly",
          procs["ctrl"].returncode == 0 and "Traceback" not in outs["ctrl"] and "cancelling the running skill" in outs["ctrl"]
          and until(lambda: "CANCELLED" in open(procs["skills"].log_path).read().split("shutting down")[-1]
                    or "cancelled" in open(procs["skills"].log_path).read()[-400:], 10),
          f"rc {procs['ctrl'].returncode}, {outs['ctrl'][-160:]!r}")
finally:
    order = [k for k in ("ctrl", "skills", "world", "fake") if k in procs] + [k for k in procs if k not in ("ctrl", "skills", "world", "fake")]
    for k in order:                                  # every started process (motion last), never a fixed name list
        if k not in outs:
            outs[k] = stop(procs[k])
    n.destroy_node(); rclpy.shutdown()
    shutil.rmtree(tmp, ignore_errors=True)
for k in ("ctrl", "skills", "world", "fake"):
    check(f"{k}: no traceback, clean exit", "Traceback" not in outs[k] and "[KILLED]" not in outs[k], outs[k][-200:])
print(f"total {time.time() - t0:.0f} s")
print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
