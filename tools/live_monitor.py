"""Live monitor for real runs (TM-O8): prints what changes and logs EVERYTHING as JSON lines.

    pixi run python3 tools/live_monitor.py [log_dir]       (default ~/hackathon/live_logs, outside the repo)

Watches /task/status (controller state, current skill + phase, jobs, alerts), /task/events (world events),
/task/world (vision_ok, boards, held, zones) and /vision/boards (frame timing: arrival gap + now - stamp, every 10 s;
parse problems via the real adapter). Read-only: sends nothing.
"""
import json
import os
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from hk_fr5_task import vision1
from hk_fr5_task_interfaces.msg import World

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
log_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/hackathon/live_logs")
os.makedirs(log_dir, exist_ok=True)
LOG = open(os.path.join(log_dir, time.strftime("live_%Y%m%d_%H%M%S.jsonl")), "a", buffering=1)


def log(kind, data):
    LOG.write(json.dumps({"t": round(time.time(), 3), "kind": kind, "data": data}) + "\n")


def say(text):
    print(f"{time.strftime('%H:%M:%S')} {text}", flush=True)
    log("print", text)


S = {"status": None, "seen_alerts": set(), "world": None, "gaps": [], "lags": [], "last_arrival": None, "frames": 0,
     "stat_t": time.time()}


def on_status(m):
    st = json.loads(m.data)
    log("status", st)
    old = S["status"] or {}
    cur = st.get("current") or {}
    key = (st.get("state"), st.get("paused"), cur.get("skill"), cur.get("job"), cur.get("board"), st.get("phase"))
    okey = (old.get("state"), old.get("paused"), (old.get("current") or {}).get("skill"),
            (old.get("current") or {}).get("job"), (old.get("current") or {}).get("board"), old.get("phase"))
    if key != okey:
        say(f"CTRL {st.get('state')}{' PAUSED' if st.get('paused') else ''} | skill {cur.get('skill') or '-'} "
            f"{cur.get('job') or ''} {cur.get('board') or ''} phase {st.get('phase') or '-'} | skills_ready "
            f"{st.get('skills_ready')} motion_ready {st.get('motion_ready')} world_ok {st.get('world_ok')}")
    jobs = {j["id"]: (j["state"], j.get("delivered"), j.get("to_fault"), j.get("skipped")) for j in st.get("jobs", [])}
    ojobs = {j["id"]: (j["state"], j.get("delivered"), j.get("to_fault"), j.get("skipped")) for j in old.get("jobs", [])}
    for jid, v in jobs.items():
        if ojobs.get(jid) != v:
            say(f"JOB  {jid} {v[0]} delivered {v[1]} to_fault {v[2]} skipped {v[3]}")
    for e in st.get("events", []):
        k = json.dumps(e, sort_keys=True)
        if k not in S["seen_alerts"]:
            S["seen_alerts"].add(k)
            say(f"EVT  {e.get('level', '')} {e.get('text', e)}")
    S["status"] = st


def on_events(m):
    log("event", m.data)
    say(f"WEV  {m.data}")


def on_world(m):
    zs = {z.name: (z.state, z.stack_count, round(z.base_z, 4)) for z in m.zones}
    bs = sorted((b.id, b.variant, b.location, b.zone) for b in m.boards)
    snap = {"vision_ok": m.vision_ok, "held": m.held_board_id, "zones": zs, "boards": bs, "gripper": m.gripper_state,
            "obstacles": len(m.obstacles)}
    log("world", snap)
    old = S["world"]
    if old is None or {k: v for k, v in snap.items()} != old:
        if old is None or old["vision_ok"] != snap["vision_ok"] or old["held"] != snap["held"] or old["zones"] != zs \
                or old["gripper"] != snap["gripper"] or old["obstacles"] != snap["obstacles"]:
            say(f"WRLD vision_ok {m.vision_ok} held '{m.held_board_id}' gripper {m.gripper_state} obstacles "
                f"{len(m.obstacles)} | zones {zs}")
        if old is None or old["boards"] != bs:
            say(f"WRLD boards {bs}")
    S["world"] = snap


def on_vision(m):
    now = time.time()
    try:
        data = json.loads(m.data)
        stamp, dets, problems = vision1.parse(data)
    except ValueError as e:
        say(f"VIS  frame refused: {e}")
        return
    if S["last_arrival"] is not None:
        S["gaps"].append(now - S["last_arrival"])
    S["last_arrival"] = now
    S["lags"].append(now - stamp)
    S["frames"] += 1
    if problems:
        say(f"VIS  parse problems: {problems}")
    log("vision", {"gap": S["gaps"][-1] if S["gaps"] else None, "lag": now - stamp, "boards": len(data.get("boards", [])),
                   "zones": [z.get("name") for z in data.get("zones", [])], "problems": problems})
    if now - S["stat_t"] >= 10.0 and S["gaps"]:
        g, l = S["gaps"], S["lags"]
        say(f"VIS  {S['frames']} frames / {now - S['stat_t']:.0f} s ({S['frames'] / (now - S['stat_t']):.1f} Hz) | gap mean "
            f"{sum(g) / len(g):.2f} max {max(g):.2f} s | now-stamp mean {sum(l) / len(l):.2f} max {max(l):.2f} s | "
            f"{len(data.get('boards', []))} boards, zones {[z.get('name') for z in data.get('zones', [])]}")
        log("vision_stats", {"frames": S["frames"], "gap_mean": sum(g) / len(g), "gap_max": max(g),
                             "lag_mean": sum(l) / len(l), "lag_max": max(l)})
        S.update(gaps=[], lags=[], frames=0, stat_t=now)


def main():
    rclpy.init()
    n = rclpy.create_node("live_monitor")
    n.create_subscription(String, "/task/status", on_status, LATCHED)
    n.create_subscription(String, "/task/events", on_events, 50)
    n.create_subscription(World, "/task/world", on_world, LATCHED)
    n.create_subscription(String, "/vision/boards", on_vision, 5)
    say(f"live monitor: logging to {LOG.name}")
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
