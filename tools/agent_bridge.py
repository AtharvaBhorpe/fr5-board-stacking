"""Fast agent bridge (demo mode, TM-31): ONE persistent node instead of a new process per command.

    pixi run python3 tools/agent_bridge.py [DIR]      (DIR default: $HK_LIVE_DIR/bridge or ~/hackathon/live_logs/bridge)

DIR/state.txt   compact current state, rewritten on every change (controller, current skill, jobs, zones, boards)
DIR/events.txt  one line per command result, job change, alert (append)
DIR/cmd         named pipe: echo "<command>" > DIR/cmd
Commands (as tools/task_cli.py): transfer VARIANT|any TO [COUNT] [PRIO] | board ID TO [PRIO] |
process VARIANT|any ZONE [TO] [COUNT] [PRIO] | clear FROM TO [COUNT] [PRIO] | pause [JOB] | resume [JOB] | stop | home |
cancel JOB | prio JOB N | clear_queue | reset_zone ZONE
"""
import json
import os
import queue
import sys
import threading
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from hk_fr5_task_interfaces.msg import Job, World
from hk_fr5_task_interfaces.srv import AddJob, Control

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
D = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.environ.get("HK_LIVE_DIR") or os.path.expanduser("~/hackathon/live_logs"), "bridge")
os.makedirs(D, exist_ok=True)
FIFO = os.path.join(D, "cmd")
if not os.path.exists(FIFO):
    os.mkfifo(FIFO)
EV = open(os.path.join(D, "events.txt"), "a", buffering=1)
Q = queue.Queue()
S = {"status": None, "world": None, "alerts": set(), "jobs": {}}


def ev(text):
    EV.write(f"{time.strftime('%H:%M:%S')} {text}\n")


def write_state():
    st, w = S["status"] or {}, S["world"]
    cur = st.get("current") or {}
    L = [f"{time.strftime('%H:%M:%S')} CTRL {st.get('state', '?')}{' PAUSED' if st.get('paused') else ''}"
         f"{' HALTED: ' + st['halted'] if st.get('halted') else ''} | skill {cur.get('skill', '-')} "
         f"{cur.get('job', '')} {cur.get('board', '')} {cur.get('target', '')} phase {st.get('phase') or '-'} | "
         f"ready skills {st.get('skills_ready')} motion {st.get('motion_ready')} world {st.get('world_ok')}"]
    if w is not None:
        L.append(f"WORLD vision_ok {w.vision_ok} held '{w.held_board_id}' gripper {w.gripper_state} "
                 f"obstacles {len(w.obstacles)} problems {st.get('problem_boards', [])}")
    for j in st.get("jobs", []):
        L.append(f"JOB {j.get('id')} {j.get('state')} {j.get('type')} {j.get('select_variant') or 'any'}"
                 f"{' from ' + j['from_zone'] if j.get('from_zone') else ''}{' via ' + j['zone'] if j.get('zone') else ''}"
                 f" -> {j.get('to') or '-'} | ok {j.get('delivered')} fault {j.get('to_fault')} lost {j.get('lost')} "
                 f"skip {j.get('skipped')} | {(j.get('message') or '')[:70]}")
    if w is not None:
        for z in w.zones:
            L.append(f"ZONE {z.name} {z.state} count {z.stack_count} top {z.stack_top_z:+.3f}"
                     f"{' board ' + z.board_id if z.board_id else ''}")
        for b in sorted(w.boards, key=lambda b: (b.location != "input", b.id)):
            if b.location == "zone" and b.age > 5.0:
                continue                                   # unseen zone boards (stack below / ghosts): skip
            L.append(f"BOARD {b.id} {b.variant}{'(' + b.label + ')' if b.label != b.variant else ''} {b.location}"
                     f"{' ' + b.zone if b.zone else ''} ({b.x:+.3f}, {b.y:+.3f}) age {b.age:.1f}")
    tmp = os.path.join(D, ".state.tmp")
    with open(tmp, "w") as f:
        f.write("\n".join(L) + "\n")
    os.replace(tmp, os.path.join(D, "state.txt"))


def on_status(m):
    st = json.loads(m.data)
    S["status"] = st
    for j in st.get("jobs", []):
        k = (j.get("state"), j.get("delivered"), j.get("to_fault"), j.get("lost"), j.get("skipped"))
        if S["jobs"].get(j["id"]) != k:
            S["jobs"][j["id"]] = k
            ev(f"JOB {j['id']} {k[0]} ok {k[1]} fault {k[2]} lost {k[3]} skip {k[4]} | {j.get('message', '')}")
    for e in st.get("events", []):
        key = json.dumps(e, sort_keys=True)
        if key not in S["alerts"]:
            S["alerts"].add(key)
            if e.get("level") in ("warn", "error"):
                ev(f"ALERT {e.get('level')} {e.get('text')}")
    write_state()


def on_world(m):
    S["world"] = m
    write_state()


def reader():
    while True:
        with open(FIFO) as f:                     # blocks until a writer opens the pipe
            for line in f:
                if line.strip():
                    Q.put(line.strip())


def main():
    rclpy.init()
    n = rclpy.create_node("agent_bridge", namespace="task")
    n.create_subscription(String, "/task/status", on_status, LATCHED)
    n.create_subscription(World, "/task/world", on_world, LATCHED)
    add = n.create_client(AddJob, "/task/add_job")
    ctl = n.create_client(Control, "/task/control")
    evp = n.create_publisher(String, "/task/events", 10)
    INT = ("count", "max_attempts", "priority")

    def send_job(cmd, **f):
        j = Job(source="agent")
        for k, v in f.items():
            setattr(j, k, int(v) if k in INT else str(v))
        fut = add.call_async(AddJob.Request(job=j))
        fut.add_done_callback(lambda x: ev(f"CMD {cmd} -> {'OK ' + x.result().id if x.result().success else 'REJECTED'} "
                                           f"{x.result().message}"))

    def send_ctl(cmd, c, job="", prio=0):
        fut = ctl.call_async(Control.Request(command=c, job_id=job, priority=int(prio)))
        fut.add_done_callback(lambda x: ev(f"CMD {cmd} -> {'OK' if x.result().success else 'REJECTED'} {x.result().message}"))

    def var(v):
        return "" if v in ("any", "") else v

    def handle(cmd):
        a = cmd.split()
        g = lambda i, d: a[i] if len(a) > i else d    # noqa: E731
        try:
            c = a[0]
            if c == "transfer":
                send_job(cmd, type="transfer", select_variant=var(a[1]), to=a[2], count=g(3, 0), priority=g(4, 0))
            elif c == "board":
                send_job(cmd, type="transfer", select_board=a[1], to=a[2], count=1, priority=g(3, 0))
            elif c == "process":
                send_job(cmd, type="process", select_variant=var(a[1]), zone=a[2], to=g(3, ""), count=g(4, 1),
                         priority=g(5, 0))
            elif c == "clear":
                send_job(cmd, type="clear_zone", from_zone=a[1], to=a[2], count=g(3, 0), priority=g(4, 0))
            elif c in ("pause", "resume"):
                send_ctl(cmd, c, g(1, ""))
            elif c in ("stop", "home", "clear_queue"):
                send_ctl(cmd, c)
            elif c == "cancel":
                send_ctl(cmd, "cancel_job", a[1])
            elif c == "prio":
                send_ctl(cmd, "set_priority", a[1], a[2])
            elif c == "reset_zone":
                evp.publish(String(data=json.dumps({"type": "reset_zone", "zone": a[1]})))
                ev(f"CMD {cmd} -> sent")
            else:
                ev(f"CMD {cmd} -> unknown command")
        except (IndexError, ValueError) as e:
            ev(f"CMD {cmd} -> bad arguments: {e}")

    pending = []                                  # (deadline, cmd): waiting for their service (discovery)

    def needs(cmd):
        c = cmd.split()[0] if cmd.split() else ""
        return add if c in ("transfer", "board", "process", "clear") else None if c == "reset_zone" else ctl

    def drain():
        while not Q.empty():
            pending.append((time.time() + 10.0, Q.get()))
        for item in list(pending):
            deadline, cmd = item
            cl = needs(cmd)
            if cl is None or cl.service_is_ready():
                pending.remove(item); handle(cmd)
            elif time.time() > deadline:
                pending.remove(item); ev(f"CMD {cmd} -> service not available (controller running?)")
    n.create_timer(0.05, drain)
    threading.Thread(target=reader, daemon=True).start()
    ev(f"bridge up: state {D}/state.txt, commands -> {FIFO}")
    print(f"agent bridge: {D}", flush=True)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
