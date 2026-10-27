"""task_main_controller node (T5c, control-loop.md CL-37): a thin ROS wrapper around main_core (all decisions there).

    /task/add_job    AddJob     validate + queue a job (dashboard / LLM)
    /task/list_jobs  ListJobs   queue + finished jobs
    /task/control    Control    pause / resume / stop / cancel_job / set_priority / clear_queue / home
    /task/status     String     JSON (latched, on change + 1 Hz): controller state, jobs, current skill + phase, tcp, events
    /task/events     String     world events from the core (e.g. 'dropped' after NOT_HOLDING, D7) for the world model

One decision thread: tick -> core.next_action -> skill goal (/task/skills/*) -> wait (per-skill limit) -> core.on_result.
Every core call holds one lock (never while waiting for a skill), so service replies are exact and 'stop' reaches the
core while the skill runs (its goal is cancelled at once). tcp from /motion/get_state before each decision (CL-26),
fallback home_xy. Starts running: with no jobs nothing moves.
"""
import json
import math
import os
import signal
import threading
import time
import traceback

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from hk_fr5_interfaces.srv import GetState
from hk_fr5_task.main_core import MainCore
from hk_fr5_task_interfaces.action import Home, Pick, Place, ReleaseSafe, WaitClear
from hk_fr5_task_interfaces.msg import Job, SkillCode, World
from hk_fr5_task_interfaces.srv import AddJob, Control, ListJobs

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SKILLS = {"pick": Pick, "place": Place, "home": Home, "wait_clear": WaitClear, "release_safe": ReleaseSafe}
CODE_NAMES = {getattr(SkillCode, n): n for n in dir(type(SkillCode))
              if n.isupper() and not n.startswith("_") and isinstance(getattr(SkillCode, n), int)}
FINAL_JOBS = ("DONE", "CANCELLED")
STATUS_FINISHED = 20                     # M5: finished jobs kept in /task/status (list_jobs returns all)
JOB_IN = ("type", "select_variant", "select_board", "from_zone", "zone", "to", "count", "max_attempts", "priority", "source")


# ---------- pure helpers (tests/main_node_check.py) ----------
def msg_fields(m):
    return {f: getattr(m, f) for f in m.get_fields_and_field_types()}


def world_to_dict(m):
    """World.msg -> the dict main_core expects (like World.snapshot() + 'stamp' in seconds)."""
    return {"stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, "vision_ok": m.vision_ok,
            "vision_age": m.vision_age, "held_board_id": m.held_board_id, "gripper_state": m.gripper_state,
            "boards": [msg_fields(b) for b in m.boards], "zones": [msg_fields(z) for z in m.zones],
            "obstacles": [msg_fields(o) for o in m.obstacles]}


def job_to_dict(j):
    """AddJob request Job -> the core's job dict (only the fields a caller sets)."""
    return {k: getattr(j, k) for k in JOB_IN}


def dict_to_job(d):
    """Core job (status entry) -> Job.msg."""
    j = Job()
    for k in j.get_fields_and_field_types():
        if k in d:
            setattr(j, k, type(getattr(j, k))(d[k]))
    return j


def code_name(value):
    return CODE_NAMES.get(value, "INTERNAL_ERROR")


def result_code(status, success, value):
    """N2: trust a skill result only when goal status, success flag and code agree (a crashed server's default-filled
    result has code 0 = SUCCESS)."""
    name = code_name(value)
    if status == GoalStatus.STATUS_SUCCEEDED:
        return name if success and name == "SUCCESS" else "INTERNAL_ERROR"
    if status == GoalStatus.STATUS_CANCELED:
        return name if name != "SUCCESS" else "CANCELLED"
    if status == GoalStatus.STATUS_ABORTED:
        return name if name != "SUCCESS" and not success else "INTERNAL_ERROR"
    return "INTERNAL_ERROR"


def goal_for(action):
    """Core action dict -> (skill name, goal msg). Speeds = the skills' defaults (velocity_scaling 0)."""
    skill = action["skill"]
    if skill == "pick":
        return skill, Pick.Goal(board_id=action["board"], contact_search=False, velocity_scaling=0.0)
    if skill == "place":
        return skill, Place.Goal(target=action["target"], velocity_scaling=0.0)
    if skill == "release_safe":
        return skill, ReleaseSafe.Goal(target=action.get("target", ""))
    if skill == "wait_clear":
        return skill, WaitClear.Goal(timeout=0.0)
    if skill == "home":
        return skill, Home.Goal(velocity_scaling=0.0)
    raise ValueError(f"unknown skill '{skill}'")


def load_config(main_path, world_path):
    with open(main_path) as f:
        cfg = yaml.safe_load(f)
    with open(world_path) as f:
        cfg["variants"] = list(yaml.safe_load(f)["variants"])
    for k in ("skill_timeout", "home_xy", "tick", "world_timeout", "tcp_period"):
        if k not in cfg:
            raise KeyError(f"{main_path}: missing '{k}'")
    return cfg


# ---------- node ----------
class MainController(Node):
    def __init__(self):
        super().__init__("main_controller", namespace="task")
        share = get_package_share_directory("hk_fr5_task")
        self.declare_parameter("config", os.path.join(share, "config", "main.yaml"))
        self.declare_parameter("world_config", os.path.join(share, "config", "world.yaml"))
        self.c = load_config(self.get_parameter("config").value, self.get_parameter("world_config").value)
        self.core = MainCore(self.c)
        self.lock = threading.Lock()                  # every core call (CL-37 A)
        self.wake = threading.Event()                 # new world / command -> decide now
        self.world, self.tcp, self.gh, self.phase = None, tuple(self.c["home_xy"]), None, ""
        self.quit, self.logged, self.last_status, self.cancel_next = False, None, "", False
        self.world_rx, self.world_down = None, False     # N3: monotonic time of the last /task/world
        self.tcp_t, self.tcp_dirty = 0.0, True           # N6: tcp read after each skill + every tcp_period
        self.running = False                             # M9: a skill goal is being sent / running
        self._refuse_second()                        # M2
        cb = ReentrantCallbackGroup()
        self.skill_clients = {n: ActionClient(self, t, f"skills/{n}", callback_group=cb) for n, t in SKILLS.items()}
        self.get_state = self.create_client(GetState, "/motion/get_state", callback_group=cb)
        self.create_subscription(World, "world", self._on_world, LATCHED, callback_group=cb)
        self.status_pub = self.create_publisher(String, "status", LATCHED)
        self.events_pub = self.create_publisher(String, "events", 50)
        self.create_service(AddJob, "add_job", self._safe(self._add_job), callback_group=cb)       # M6
        self.create_service(ListJobs, "list_jobs", self._safe(self._list_jobs), callback_group=cb)
        self.create_service(Control, "control", self._safe(self._control), callback_group=cb)
        self.create_timer(1.0, lambda: self._publish(force=True), callback_group=cb)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.get_logger().info("main controller ready: /task/add_job, list_jobs, control; status on /task/status")

    def _refuse_second(self):
        """M2: only one controller per ROS graph (two would fight over the skill server). A controller that ANSWERS
        /task/list_jobs is running; a stale discovery entry of a stopped one does not answer (restart stays possible)."""
        probe = rclpy.create_node("main_controller_probe", context=self.context)
        try:
            cli = probe.create_client(ListJobs, "/task/list_jobs")
            if cli.wait_for_service(timeout_sec=1.0):
                fut = cli.call_async(ListJobs.Request())
                rclpy.spin_until_future_complete(probe, fut, timeout_sec=1.0)
                if fut.done() and fut.result() is not None:
                    raise RuntimeError("another task_main_controller is running (it answers /task/list_jobs) - stop it first")
        finally:
            probe.destroy_node()

    def _safe(self, fn):
        """M6: an exception in a service callback still answers the caller (success false) and is logged."""
        def cb(req, res):
            try:
                return fn(req, res)
            except Exception as e:
                self.get_logger().error(f"internal error in {fn.__name__}: {e!r}\n{traceback.format_exc()}")
                if hasattr(res, "success"):
                    res.success, res.message = False, f"internal error: {e!r}"
                return res
        return cb

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- inputs ----------
    def _on_world(self, m):
        self.world = world_to_dict(m)
        self.world_rx = time.monotonic()
        self.wake.set()

    def _read_tcp(self):
        """tcp x / y from /motion/get_state (CL-26); fallback home_xy. The robot only moves in skills, so it is read
        after each skill and every tcp_period otherwise (N6), else the cached value."""
        if not self.tcp_dirty and time.monotonic() - self.tcp_t < self.c["tcp_period"]:
            return self.tcp
        self.tcp_t = time.monotonic()
        if self.get_state.service_is_ready():
            fut = self.get_state.call_async(GetState.Request())
            end = time.monotonic() + 1.0
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.005)
            if fut.done() and fut.result() is not None and fut.result().success:
                p = fut.result().tcp_pose.pose.position
                self.tcp, self.tcp_dirty = (p.x, p.y), False
                return self.tcp
        self.get_logger().warn("no tcp from /motion/get_state - using home_xy", throttle_duration_sec=10.0)
        return tuple(self.c["home_xy"])

    # ---------- services ----------
    def _add_job(self, req, res):
        with self.lock:
            if self.world is None:
                res.success, res.message = False, "no world model yet (/task/world)"
            else:
                res.success, res.message, res.id = self.core.add_job(job_to_dict(req.job), self.world, self.now())
        self.wake.set(); self._publish()
        return res

    def _list_jobs(self, req, res):
        with self.lock:
            res.jobs = [dict_to_job(j) for j in self.core.status(self.now())["jobs"]]
        return res

    def _control(self, req, res):
        with self.lock:
            res.success, res.message = self.core.control(req.command, req.job_id, req.priority, self.now())
            gh = self.gh
            if res.success and req.command == "stop" and gh is None and self.core.current is not None:
                self.cancel_next = True              # chosen, goal not accepted yet: cancel it as soon as it is
        if res.success and req.command == "stop" and gh is not None:
            gh.cancel_goal_async()                   # smooth stop of the running skill (CL-25)
        self.wake.set(); self._publish()
        return res

    # ---------- decision thread ----------
    def _loop(self):
        while rclpy.ok() and not self.quit:
            self.wake.wait(timeout=self.c["tick"])
            self.wake.clear()
            if not self._world_fresh():
                continue
            try:
                self._round()
            except Exception as e:                   # N1: never die silently - HALT, visible in status, keep looping
                self.get_logger().error(f"internal error in the decision loop: {e!r}\n{traceback.format_exc()}")
                gh = self.gh
                if gh is not None:
                    gh.cancel_goal_async()
                with self.lock:
                    self.core.current = None         # the round did not finish: no skill is running for the core
                    self.core._halt(self.now(), f"internal error in the controller ({type(e).__name__}: {e}) - "
                                                "see the log, then resume")
                self.tcp_dirty = True
                self._publish()

    def _world_fresh(self):
        """N3 watchdog: decide only while /task/world keeps coming."""
        if self.world is None:
            return False
        age = time.monotonic() - self.world_rx
        if age > self.c["world_timeout"]:
            if not self.world_down:
                self.world_down = True
                self.get_logger().error(f"no /task/world for {age:.1f} s - not deciding (world model running?)")
                self._publish()
            return False
        if self.world_down:
            self.world_down = False
            self.get_logger().info("/task/world back - deciding again")
        return True

    def _round(self):
        tcp = self._read_tcp()
        with self.lock:
            a = self.core.next_action(self.world, self.now(), tcp)
            if a is not None:
                self.core.started(a)
        self._publish()
        if a is None:
            return
        self.get_logger().info(f"ACTION start {a} (tcp {[round(v, 4) for v in tcp]})")          # TM-26: verbose trace
        t0 = time.monotonic()
        code, msg = self._run(a)
        if code == "SUCCESS":                        # one call site per severity (rclpy logger trap)
            self.get_logger().info(f"ACTION result {a.get('skill')} {code} in {time.monotonic() - t0:.1f} s: {msg}")
        else:
            self.get_logger().warn(f"ACTION result {a.get('skill')} {code} in {time.monotonic() - t0:.1f} s: {msg}")
        self.tcp_dirty = True                        # the robot may have moved
        with self.lock:
            self.cancel_next = False                 # N4: a pending 'stop' cancel belongs to this action only
            self.core.on_result(a, code, msg, self.now())
            evs = self.core.take_world_events()
        for ev in evs:
            self.get_logger().info(f"world event from the core: {ev}")
            self.events_pub.publish(String(data=json.dumps(ev)))
        self._publish()

    def _run(self, action):
        """Run one skill: (SkillCode name, message). Node-side failures -> INTERNAL_ERROR."""
        try:
            skill, goal = goal_for(action)
        except ValueError as e:
            return "INTERNAL_ERROR", str(e)
        client = self.skill_clients[skill]
        if not client.wait_for_server(timeout_sec=2.0):
            return "INTERNAL_ERROR", f"{skill}: skill server not available"
        self.phase, self.running = "", True
        fut = client.send_goal_async(goal, feedback_callback=self._feedback)
        if not self._wait(fut, 5.0, stop_on_quit=False):      # even at shutdown: learn whether it was accepted
            fut.add_done_callback(self._late_accept(skill))     # M1: never leave a late-accepted skill running
            self.running = False
            return "INTERNAL_ERROR", f"{skill}: goal not accepted within 5 s"
        if not fut.result().accepted:
            self.running = False
            return "INTERNAL_ERROR", f"{skill}: goal not accepted"
        gh = fut.result()
        with self.lock:
            self.gh = gh
            cancel, self.cancel_next = self.cancel_next, False
        if cancel or self.quit:                      # 'stop' before the accept, or shutdown started meanwhile
            gh.cancel_goal_async()
        try:
            rf = gh.get_result_async()
            limit = float(self.c["skill_timeout"].get(skill, 120.0))
            how = self._wait_result(rf, client, limit)
            if how == "quit":                        # Ctrl+C / SIGTERM: cancel it here (shutdown() keeps spinning)
                gh.cancel_goal_async()
                self._wait(rf, 3.0, stop_on_quit=False)
                return "CANCELLED", f"{skill}: controller shutting down"
            if how == "gone":                        # M3
                return "INTERNAL_ERROR", f"{skill}: skill server gone while running"
            if how == "timeout":
                gh.cancel_goal_async()
                self._wait(rf, 30.0)
                return "INTERNAL_ERROR", f"{skill}: no result in {limit:.0f} s (cancelled)"
            res = rf.result()
            return result_code(res.status, res.result.success, res.result.code.value), res.result.message
        finally:
            with self.lock:
                self.gh = None
            self.phase, self.running = "", False

    def _late_accept(self, skill):
        def cb(f):
            r = f.result()
            if r is not None and r.accepted:
                self.get_logger().warn(f"{skill}: goal accepted too late - cancelling it (M1)")
                r.cancel_goal_async()
        return cb

    def _wait_result(self, rf, client, limit):
        """'done' | 'timeout' | 'gone' (server missing for 2 s, M3) | 'quit'."""
        end, gone_since = time.monotonic() + limit, None
        while not rf.done():
            if self.quit:
                return "quit"
            now = time.monotonic()
            if now > end:
                return "timeout"
            if client.server_is_ready():
                gone_since = None
            elif gone_since is None:
                gone_since = now
            elif now - gone_since > 2.0:
                return "gone"
            time.sleep(0.02)
        return "done"

    def _feedback(self, fb):
        if not self.running:                         # M9: late feedback of a finished goal
            return
        self.phase = getattr(fb.feedback, "phase", "")
        self._publish()

    def _wait(self, fut, timeout, stop_on_quit=True):
        end = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > end or (self.quit and stop_on_quit):
                return False
            time.sleep(0.01)
        return True

    # ---------- outputs ----------
    def _publish(self, force=False):
        with self.lock:
            st = self.core.status(self.now())
            new = self._new_events()
        jobs = st["jobs"]
        finished = [j["id"] for j in jobs if j["state"] in FINAL_JOBS][-STATUS_FINISHED:]       # M5
        st["jobs"] = [j for j in jobs if j["state"] not in FINAL_JOBS or j["id"] in finished]
        st.update(jobs_total=len(jobs), phase=self.phase, tcp=[round(v, 4) for v in self.tcp],
                  world_ok=not self.world_down and self.world_rx is not None,
                  skills_ready=all(c.server_is_ready() for c in self.skill_clients.values()),       # M8
                  motion_ready=self.get_state.service_is_ready())
        cmp = json.dumps(st)                         # M4: change test without the always-changing world_age
        st["world_age"] = None if self.world_rx is None else round(time.monotonic() - self.world_rx, 2)
        if force or cmp != self.last_status:
            self.last_status = cmp
            self.status_pub.publish(String(data=json.dumps(st)))
        for t, level, msg in new:                    # one call site per severity (rclpy logger trap)
            if level == "error":
                self.get_logger().error(msg)
            elif level == "warn":
                self.get_logger().warn(msg)
            else:
                self.get_logger().info(msg)

    def _new_events(self):
        ev = self.core.events
        k = 0
        if self.logged is not None:
            k = next((i + 1 for i in range(len(ev) - 1, -1, -1) if ev[i] is self.logged), 0)
        out = ev[k:]
        if ev:
            self.logged = ev[-1]
        return out

    def shutdown(self, ex):
        """Ctrl+C / SIGTERM (N5 / M7): the ROS context is still valid (own signal handling). The decision thread
        cancels whatever it has in flight (also a goal whose accept is still on the way); keep the executor spinning
        until it is done so those answers arrive."""
        if self.running:
            self.get_logger().info("shutting down: cancelling the running skill (smooth stop)")
        self.quit = True
        self.wake.set()
        end = time.monotonic() + 10.0                # accept (5 s) + cancel answer (3 s) + margin
        while self.thread.is_alive() and time.monotonic() < end:
            ex.spin_once(timeout_sec=0.05)


def _sigterm(_sig, _frame):
    raise KeyboardInterrupt                          # M7: SIGTERM takes the Ctrl+C path (cancel the running skill)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)     # N5: Ctrl+C handled here, context stays valid
    signal.signal(signal.SIGTERM, _sigterm)
    try:
        node = MainController()
    except (OSError, KeyError, ValueError, RuntimeError) as e:
        print(f"[main_controller] ERROR: {e}")
        rclpy.shutdown()
        raise SystemExit(1)
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    node.thread.start()
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown(ex)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
