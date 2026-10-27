"""ROS checks of the task_main_controller node itself (T5c-1, CL-37 review N1-N7), ~1 min, no motion stack: the real node
in-process against FAKES - scriptable skill action servers (/task/skills/*), /motion/get_state, and a /task/world
publisher whose world follows the scripted skills. Covers what the full stack cannot produce on demand: node-side
timeouts, rejected goals, default-filled results (N2), stop during a skill and in the moment before its goal is accepted
(N4), an exception in the decision loop (N1), a world-model outage (N3), tcp call rate (N6), Ctrl+C / SIGTERM
cancelling the running skill (N5 / M7, controller as a separate process, output to a log file: TM-13), second review
M1-M9 (late accept, second controller, vanished server, status rate / size, service exceptions, late feedback).

    pixi run python3 tests/main_node_ros_check.py
"""
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
os.environ.setdefault("ROS_DOMAIN_ID", "84")

import rclpy                                                   # noqa: E402
import yaml                                                    # noqa: E402
from ament_index_python.packages import get_package_prefix     # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse   # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup      # noqa: E402
from rclpy.executors import MultiThreadedExecutor              # noqa: E402
from rclpy.node import Node                                    # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile             # noqa: E402
from std_msgs.msg import String                                # noqa: E402

from hk_fr5_interfaces.srv import GetState                     # noqa: E402
from hk_fr5_task_interfaces.action import Home, Pick, Place, ReleaseSafe, WaitClear   # noqa: E402
from hk_fr5_task_interfaces.msg import Board, Job, SkillCode, World, Zone              # noqa: E402
from hk_fr5_task_interfaces.srv import AddJob, Control, ListJobs                       # noqa: E402

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

HERE = os.path.dirname(os.path.abspath(__file__))
CD = os.path.join(HERE, "..", "src", "hk_fr5_task", "config")
TMP = tempfile.mkdtemp()
cfg = yaml.safe_load(open(os.path.join(CD, "main.yaml")))
cfg["skill_timeout"] = {k: 2.0 for k in ("pick", "place", "home", "wait_clear", "release_safe")}
cfg["skill_timeout"]["pick"] = 10.0                            # M3: a vanished server is found before the limit
cfg.update(world_timeout=1.0, end_wait=1.0, partial_after=3.0)
CFG_PATH = os.path.join(TMP, "main_test.yaml")
yaml.safe_dump(cfg, open(CFG_PATH, "w"))
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SKILLS = {"pick": Pick, "place": Place, "home": Home, "wait_clear": WaitClear, "release_safe": ReleaseSafe}
C = SkillCode


class Fakes(Node):
    """Skill servers + get_state + world, all scripted. script[skill] = list of behaviours (default ("ok", 0.2)):
    ("ok", s) success after s seconds (world follows) | ("code", NAME, s) failure | ("hang",) until cancelled |
    ("reject",) | ("slow_accept", s) / ("slow_reject", s) goal answer after s | ("abort_default",) aborted with a
    default-filled result | ("succeed_false",) SUCCEEDED but success false."""

    def __init__(self):
        super().__init__("fakes", namespace="task")
        cb = ReentrantCallbackGroup()
        self.script = {k: [] for k in SKILLS}
        self.cur, self.calls, self.cancels, self.tcp_calls, self.goals = {}, [], [], 0, []
        self.publish_world, self.n, self.cb = True, 0, cb
        self.reset()
        self.servers = {}
        for name in SKILLS:
            self.serve(name)
        self.create_service(GetState, "/motion/get_state", self._get_state, callback_group=cb)
        self.pub = self.create_publisher(World, "world", LATCHED)
        self.create_timer(0.1, self._world, callback_group=cb)
        self.status, self.events, self.n_status = None, [], 0
        self.create_subscription(String, "status", self._on_status, LATCHED, callback_group=cb)
        self.create_subscription(String, "events", lambda m: self.events.append(json.loads(m.data)), 50, callback_group=cb)

    def serve(self, name):
        self.servers[name] = ActionServer(self, SKILLS[name], f"skills/{name}", execute_callback=self._exec(name, SKILLS[name]),
                                          goal_callback=self._goal(name), cancel_callback=lambda _: CancelResponse.ACCEPT,
                                          callback_group=self.cb)

    def reset(self):
        """Two fresh VARa boards (new ids, so earlier problem boards do not matter), empty drop-off, nothing held."""
        self.n += 1
        self.boards = {f"r{self.n}a": (-0.45, 0.22), f"r{self.n}b": (-0.62, 0.20)}
        self.loc = {b: "input" for b in self.boards}
        self.held, self.stack = "", 0
        self.script = {k: [] for k in SKILLS}

    def _world(self):
        if not self.publish_world:
            return
        m = World(vision_ok=True, vision_age=0.05, held_board_id=self.held, gripper_state="IDLE")
        m.header.stamp = self.get_clock().now().to_msg()
        m.boards = [Board(id=b, variant="VARa", label="VARa", location=self.loc[b], zone="drop_off_1" if self.loc[b] == "zone" else "",
                          x=x, y=y, length=0.15, width=0.10, thickness=0.006, top_z=float("nan"), score=0.9, inside=True,
                          size_ok=True, age=0.0) for b, (x, y) in self.boards.items()]
        m.zones = [Zone(name="drop_off_1", kind="drop_off", state="OK", x=-0.15, y=0.55, length=0.24, width=0.20,
                        stack_count=self.stack, age=0.0),
                   Zone(name="fault", kind="fault", state="OK", x=-0.25, y=0.28, length=0.20, width=0.16, age=0.0)]
        self.pub.publish(m)

    def _on_status(self, m):
        self.status, self.n_status = json.loads(m.data), self.n_status + 1

    def _get_state(self, req, res):
        self.tcp_calls += 1
        res.success = True
        res.tcp_pose.pose.position.x, res.tcp_pose.pose.position.y = -0.497, -0.102
        return res

    def _goal(self, name):
        def cb(_g):
            beh = self.script[name].pop(0) if self.script[name] else ("ok", 0.2)
            self.cur[name] = beh
            self.goals.append(name)
            if beh[0] in ("slow_accept", "slow_reject"):
                time.sleep(beh[1])
            return GoalResponse.REJECT if beh[0] in ("reject", "slow_reject") else GoalResponse.ACCEPT
        return cb

    def _exec(self, name, typ):
        def ex(gh):
            beh = self.cur.get(name, ("ok", 0.2))
            g = gh.request
            self.calls.append((name, getattr(g, "board_id", "") or getattr(g, "target", "")))
            res = typ.Result()
            if beh[0] == "abort_default":
                gh.abort(); return typ.Result()
            if beh[0] == "succeed_false":
                gh.succeed(); return typ.Result()
            dur = {"ok": beh[1] if len(beh) > 1 else 0.2, "code": beh[2] if len(beh) > 2 else 0.2,
                   "hang": 30.0, "slow_accept": 0.3}.get(beh[0], 0.2)
            end = time.monotonic() + dur
            while time.monotonic() < end:
                if gh.is_cancel_requested:
                    self.cancels.append(name)
                    res.success, res.code, res.message = False, SkillCode(value=C.CANCELLED), "cancelled"
                    gh.canceled(); return res
                time.sleep(0.02)
            if beh[0] == "code":
                res.success, res.code, res.message = False, SkillCode(value=getattr(C, beh[1])), "scripted"
                gh.abort(); return res
            if name == "pick":
                self.held, self.loc[g.board_id] = g.board_id, "held"
            elif name in ("place", "release_safe") and self.held:
                self.loc[self.held], self.held = "zone", ""
                self.stack += 1
            res.success, res.code, res.message = True, SkillCode(value=C.SUCCESS), "ok"
            gh.succeed(); return res
        return ex


rclpy.init(args=["--ros-args", "-p", f"config:={CFG_PATH}"])
from hk_fr5_task.main_controller import MainController         # noqa: E402

fk = Fakes()
cl = Node("check_client", namespace="task")
add_c, ctl_c, lst_c = cl.create_client(AddJob, "add_job"), cl.create_client(Control, "control"), cl.create_client(ListJobs, "list_jobs")
ctrl = MainController()
ex = MultiThreadedExecutor(num_threads=10)
for n in (fk, cl, ctrl):
    ex.add_node(n)
threading.Thread(target=ex.spin, daemon=True).start()
ctrl.thread.start()


def until(fn, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            if fn():
                return True
        except (KeyError, TypeError, IndexError, AttributeError):
            pass
        time.sleep(0.05)
    return False

def call(client, req, timeout=5.0):
    if not client.wait_for_service(timeout_sec=5.0):
        return None
    f = client.call_async(req)
    return f.result() if until(f.done, timeout) else None

def control(cmd, job_id="", prio=0):
    return call(ctl_c, Control.Request(command=cmd, job_id=job_id, priority=prio))

def add(count=1):
    r = call(add_c, AddJob.Request(job=Job(type="transfer", select_variant="VARa", to="drop_off_1", count=count, source="test")))
    return r.id if r and r.success else None

def job(jid):
    r = call(lst_c, ListJobs.Request())
    return next((j for j in r.jobs if j.id == jid), None) if r else None

def done(jid):
    j = job(jid)
    return j is not None and j.state == "DONE"

def events(text):
    return [e["text"] for e in (fk.status or {}).get("events", []) if text in e["text"]]

def case(label):
    """Known state: no jobs running, controller resumed, fresh fake world."""
    control("clear_queue"); control("resume")
    until(lambda: fk.status["current"] is None, 10)
    fk.reset(); fk.calls.clear(); fk.cancels.clear()
    time.sleep(0.4)
    print(f"--- {label}", flush=True)


t0 = time.monotonic()
try:
    ready = until(lambda: fk.status is not None and add_c.service_is_ready(), 15)
    check("controller up: status latched, services ready", ready)

    case("happy path")
    a = add()
    check("add_job -> scripted pick -> place -> job DONE (delivered 1)", until(lambda: done(a), 15) and job(a).delivered == 1
          and [c[0] for c in fk.calls] == ["pick", "place"] and fk.calls[1][1] == "drop_off_1", str(fk.calls))
    st = fk.status
    check("/task/status: valid JSON with state, jobs, events, tcp, world_ok", all(k in st for k in
          ("state", "jobs", "events", "tcp", "world_ok", "phase")) and st["world_ok"] is True, str(list(st)))

    case("node-side timeout")
    fk.script["place"] = [("hang",)]
    a = add()
    got = until(lambda: "place" in fk.cancels, 6)
    check("skill without a result in its limit: goal cancelled, INTERNAL_ERROR, recovery home, then the job finishes",
          got and until(lambda: done(a), 20) and "home" in [c[0] for c in fk.calls] and events("place: no result in 2 s"),
          f"cancels {fk.cancels}, calls {fk.calls}")

    case("rejected goal")
    fk.script["pick"] = [("reject",)]
    a = add()
    check("rejected goal: INTERNAL_ERROR 'goal not accepted', then home + pick again -> DONE",
          until(lambda: done(a), 20) and events("goal not accepted") and "home" in [c[0] for c in fk.calls], str(fk.calls))

    case("N2 default-filled results")
    fk.script["pick"] = [("abort_default",), ("succeed_false",)]
    a = add()
    check("N2: aborted / SUCCEEDED results with success false and code 0 are NOT counted as SUCCESS",
          until(lambda: done(a), 25) and len(events("failed: INTERNAL_ERROR")) >= 2 and job(a).delivered == 1
          and [c[0] for c in fk.calls].count("pick") == 3, f"{fk.calls}, {events('INTERNAL_ERROR')[:3]}")

    case("stop during a skill")
    fk.script["pick"] = [("ok", 3.0)]
    a = add()
    until(lambda: "pick" in [c[0] for c in fk.calls], 5)
    time.sleep(0.3)
    control("stop")
    stopped = until(lambda: "pick" in fk.cancels, 2)
    paused = until(lambda: fk.status["paused"] and fk.status["current"] is None, 3)
    control("resume")
    check("stop during a skill: the goal is cancelled at once, controller paused; resume -> the job finishes",
          stopped and paused and until(lambda: done(a), 15), f"cancels {fk.cancels}, calls {fk.calls}")

    case("N4 stop before the goal is accepted")
    fk.script["pick"] = [("slow_accept", 1.5)]
    a = add()
    until(lambda: fk.status["current"] is not None, 5)
    control("stop")
    check("stop while the goal is not accepted yet: cancelled as soon as it is", until(lambda: "pick" in fk.cancels, 4),
          str(fk.cancels))
    control("resume")
    until(lambda: done(a), 15)
    case("N4 stop before a goal that is then rejected")
    fk.script["pick"] = [("slow_reject", 1.5)]
    a = add()
    until(lambda: fk.status["current"] is not None, 5)
    control("stop")
    until(lambda: fk.status["current"] is None, 5)
    control("resume")
    check("N4: the pending cancel is dropped when no goal started; the next skills are NOT cancelled -> DONE",
          until(lambda: done(a), 20) and not fk.cancels, f"cancels {fk.cancels}, calls {fk.calls}")

    case("N1 exception in the decision loop")
    real = ctrl.core.next_action
    boom = {"n": 0}
    def broken(*args, **kw):
        if boom["n"] == 0:
            boom["n"] = 1
            raise RuntimeError("test bug")
        return real(*args, **kw)
    ctrl.core.next_action = broken
    a = add()
    halted = until(lambda: "internal error in the controller" in fk.status["halted"], 3)
    alive = ctrl.thread.is_alive()
    ctrl.core.next_action = real
    control("resume")
    check("N1: an exception halts the core (visible in status), the loop stays alive; resume -> the job finishes",
          halted and alive and until(lambda: done(a), 15), f"halted '{fk.status['halted'][:60]}', alive {alive}")

    case("N3 world model outage")
    fk.publish_world = False
    down = until(lambda: fk.status["world_ok"] is False, 3)
    a = add()
    time.sleep(1.5)
    idle = not fk.calls
    fk.publish_world = True
    check("N3: no /task/world -> status world_ok false, no skill started; world back -> the job runs",
          down and idle and until(lambda: done(a), 15), f"down {down}, calls during outage {fk.calls}")

    case("N6 tcp call rate")
    n0 = fk.tcp_calls
    time.sleep(3.0)
    n = fk.tcp_calls - n0
    check("N6: idle with the world at 10 Hz: /motion/get_state at most every tcp_period (<= 3 calls in 3 s)", n <= 3, f"{n} calls")

    case("M1 goal accepted too late")
    fk.script["pick"] = [("slow_accept", 6.0)]
    a = add()
    late = until(lambda: events("goal not accepted within 5 s"), 8)
    cancelled = until(lambda: "pick" in fk.cancels, 4)
    check("M1: a goal accepted after 5 s is cancelled at once (no skill runs that the core does not know), job finishes",
          late and cancelled and until(lambda: done(a), 20), f"late {late}, cancels {fk.cancels}, calls {fk.calls}")

    case("M2 second controller")
    try:
        MainController(); second = "started"
    except RuntimeError as e:
        second = str(e)
    check("M2: a second controller in the same graph refuses to start (the first one answers)",
          "another task_main_controller is running" in second, second)

    case("M3 + M8 skill server gone mid-goal")
    fk.script["pick"] = [("hang",)]
    a = add()
    until(lambda: fk.calls and fk.calls[-1][0] == "pick", 5)
    t_gone = time.monotonic()
    fk.servers["pick"].destroy()
    gone = until(lambda: events("skill server gone"), 6)
    dt = time.monotonic() - t_gone
    not_ready = until(lambda: fk.status["skills_ready"] is False, 3)
    fk.serve("pick")
    ready_again = until(lambda: fk.status["skills_ready"] is True, 5)
    control("clear_queue")
    check("M3: the skill server vanishing mid-goal is found in ~2 s (not after the 10 s limit); M8 skills_ready "
          "false, true again when it is back", gone and dt < 5.0 and not_ready and ready_again and fk.status["motion_ready"],
          f"gone {gone} after {dt:.1f} s, not_ready {not_ready}, again {ready_again}")

    case("M4 status rate while idle")
    n0 = fk.n_status
    time.sleep(3.0)
    check("M4: idle with the world at 10 Hz: status only on change + 1 Hz (<= 5 messages in 3 s)", fk.n_status - n0 <= 5,
          f"{fk.n_status - n0} messages")

    case("M5 finished jobs in status")
    control("pause")
    ids = [add() for _ in range(25)]
    control("clear_queue"); control("resume")
    until(lambda: all(j["state"] == "CANCELLED" for j in fk.status["jobs"] if j["id"] in ids), 5)
    r = call(lst_c, ListJobs.Request())
    st = fk.status
    check("M5: status lists active + the last 20 finished jobs (jobs_total all); list_jobs returns all",
          len(st["jobs"]) <= 20 and st["jobs_total"] >= 25 and r is not None and len(r.jobs) >= 25,
          f"status {len(st['jobs'])} / total {st['jobs_total']}, list {len(r.jobs) if r else None}")

    case("M6 exception in a service")
    real_add = ctrl.core.add_job
    def broken_add(*args, **kw):
        raise RuntimeError("test bug in add_job")
    ctrl.core.add_job = broken_add
    r = call(add_c, AddJob.Request(job=Job(type="transfer", select_variant="VARa", to="drop_off_1", count=1)))
    ctrl.core.add_job = real_add
    check("M6: an exception in a service callback still answers: success false, 'internal error'",
          r is not None and not r.success and "internal error" in r.message, str(r and r.message))

    case("M9 late feedback")
    class FB:                                                       # what rclpy hands the feedback callback
        class feedback:
            phase = "DESCEND"
    ctrl._feedback(FB())
    time.sleep(0.2)
    check("M9: feedback arriving when no skill runs does not change the phase", ctrl.phase == "" and fk.status["phase"] == "",
          f"phase '{ctrl.phase}'")

    # N5: Ctrl+C on the controller process (the in-process controller leaves first: one controller only)
    ex.remove_node(ctrl); ctrl.quit = True; ctrl.thread.join(3); ctrl.destroy_node()
    fk.reset(); fk.calls.clear(); fk.cancels.clear()
    log = os.path.join(TMP, "controller.log")
    exe = os.path.join(get_package_prefix("hk_fr5_task"), "lib", "hk_fr5_task", "task_main_controller")
    p = subprocess.Popen([exe, "--ros-args", "-p", f"config:={CFG_PATH}"], stdout=open(log, "w"), stderr=subprocess.STDOUT)
    until(lambda: add_c.service_is_ready() and fk.status is not None, 10)
    time.sleep(1.0)
    fk.script["pick"] = [("hang",)]
    add()
    started = until(lambda: "pick" in [c[0] for c in fk.calls], 8)
    p.send_signal(signal.SIGINT)
    cancelled = until(lambda: "pick" in fk.cancels, 3)
    try:
        rc = p.wait(timeout=8)
    except subprocess.TimeoutExpired:
        p.kill(); rc = "killed"
    text = open(log).read()
    check("N5: Ctrl+C on the controller cancels the running skill (smooth stop) and exits cleanly",
          started and cancelled and rc == 0 and "Traceback" not in text, f"started {started}, cancel {cancelled}, rc {rc}, "
          f"log tail: {text[-200:]!r}")
    fk.reset(); fk.calls.clear(); fk.cancels.clear()
    log2 = os.path.join(TMP, "controller2.log")
    p = subprocess.Popen([exe, "--ros-args", "-p", f"config:={CFG_PATH}"], stdout=open(log2, "w"), stderr=subprocess.STDOUT)
    until(lambda: "main controller ready" in open(log2).read(), 10)
    time.sleep(1.0)
    fk.script["pick"] = [("hang",)]
    add()
    started = until(lambda: "pick" in [c[0] for c in fk.calls], 8)
    p.send_signal(signal.SIGTERM)
    cancelled = until(lambda: "pick" in fk.cancels, 3)
    try:
        rc = p.wait(timeout=8)
    except subprocess.TimeoutExpired:
        p.kill(); rc = "killed"
    text = open(log2).read()
    check("M7: SIGTERM takes the same path: the running skill is cancelled, clean exit", started and cancelled and rc == 0
          and "Traceback" not in text, f"started {started}, cancel {cancelled}, rc {rc}, log tail: {text[-200:]!r}")
    fk.reset(); fk.calls.clear(); fk.cancels.clear(); fk.goals.clear()
    log3 = os.path.join(TMP, "controller3.log")
    p = subprocess.Popen([exe, "--ros-args", "-p", f"config:={CFG_PATH}"], stdout=open(log3, "w"), stderr=subprocess.STDOUT)
    until(lambda: "main controller ready" in open(log3).read(), 10)
    time.sleep(1.0)
    fk.script["pick"] = [("slow_accept", 2.0)]
    add()
    sent = until(lambda: "pick" in fk.goals, 8)                # the accept answer is still on its way
    p.send_signal(signal.SIGINT)
    cancelled = until(lambda: "pick" in fk.cancels, 6)
    try:
        rc = p.wait(timeout=12)
    except subprocess.TimeoutExpired:
        p.kill(); rc = "killed"
    text = open(log3).read()
    check("shutdown race: Ctrl+C while a goal's accept is on its way -> the goal is cancelled once accepted, clean exit",
          sent and cancelled and rc == 0 and "Traceback" not in text, f"sent {sent}, cancel {cancelled}, rc {rc}, "
          f"log tail: {text[-200:]!r}")
finally:
    ex.shutdown()
    rclpy.shutdown()
print(f"total {time.monotonic() - t0:.0f} s")
print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
