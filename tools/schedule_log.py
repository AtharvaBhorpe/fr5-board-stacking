"""Scheduling recorder (TM-35): a timeline of how the main controller schedules jobs, for analysis / VLM training.

    pixi run python3 tools/schedule_log.py [FILE]     (default: $HK_LIVE_DIR/schedule_timeline.md)
    echo "AGENT <text>" >> FILE                         (an agent / human adds its own decisions to the same timeline)

One persistent node (no process start per read). Writes one line per event, time relative to the first job:
JOB (added / state / counters), ACTION (the skill the controller starts: skill, job, board, target), ZONE (processing /
drop-off state + stack count changes), EVENT (the controller's own event log). Read-only: sends nothing.
"""
import json
import os
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from hk_fr5_task_interfaces.msg import World

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.environ.get("HK_LIVE_DIR") or ".", "schedule_timeline.md")
F = open(OUT, "a", buffering=1)
S = {"t0": None, "jobs": {}, "cur": None, "zones": {}, "events": set()}


def line(kind, text):
    now = time.time()
    rel = f"{now - S['t0']:7.1f}" if S["t0"] else "      -"
    F.write(f"| {time.strftime('%H:%M:%S')} | {rel} | {kind} | {text} |\n")


def on_status(m):
    st = json.loads(m.data)
    for j in st.get("jobs", []):
        k = (j.get("state"), j.get("delivered"), j.get("to_fault"), j.get("lost"), j.get("skipped"))
        if S["jobs"].get(j["id"]) != k:
            if j["id"] not in S["jobs"]:
                S["t0"] = S["t0"] or time.time()
                line("JOB", f"{j['id']} added: {j.get('type')} {j.get('select_variant') or 'any'}"
                            f"{' via ' + j['zone'] if j.get('zone') else ''} -> {j.get('to') or '-'} count {j.get('count')} "
                            f"prio {j.get('priority')}")
            line("JOB", f"{j['id']} {k[0]} (delivered {k[1]}, fault {k[2]}, lost {k[3]}, skipped {k[4]})"
                        f"{' - ' + j['message'] if j.get('message') else ''}")
            S["jobs"][j["id"]] = k
    cur = st.get("current") or None
    key = json.dumps(cur, sort_keys=True) if cur else None
    if key != S["cur"]:
        if cur:
            line("ACTION", f"start {cur.get('skill')} job {cur.get('job') or '-'} board {cur.get('board') or '-'}"
                           f"{' -> ' + cur['target'] if cur.get('target') else ''}{' (' + cur['stage'] + ')' if cur.get('stage') else ''}")
        else:
            line("ACTION", "idle")
        S["cur"] = key
    for e in st.get("events", []):
        k = json.dumps(e, sort_keys=True)
        if k not in S["events"]:
            S["events"].add(k)
            line("EVENT", f"{e.get('level', '')}: {e.get('text', e)}")


def on_world(m):
    for z in m.zones:
        k = (z.state, z.stack_count, z.board_id)
        if S["zones"].get(z.name) != k:
            if z.name in S["zones"]:
                line("ZONE", f"{z.name} {z.state} count {z.stack_count}{' board ' + z.board_id if z.board_id else ''}")
            S["zones"][z.name] = k


def main():
    rclpy.init()
    n = rclpy.create_node("schedule_log", namespace="task")
    n.create_subscription(String, "/task/status", on_status, LATCHED)
    n.create_subscription(World, "/task/world", on_world, LATCHED)
    F.write(f"\n## Timeline {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n| clock | t (s) | kind | what |\n|---|---|---|---|\n")
    print(f"schedule log: {OUT}", flush=True)
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
