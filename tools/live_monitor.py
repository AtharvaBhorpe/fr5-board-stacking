"""Live monitor for real runs (TM-O8, TM-26): extreme logging of the whole pipeline + a readable live view.

    (started by tools/stack.py; session folder = $HK_LIVE_DIR)
    pixi run python3 tools/live_monitor.py [--verbose] [--dir DIR]

Read-only (sends nothing). Files always complete, in DIR (default: the session's monitor/ folder $HK_LIVE_DIR/monitor;
without a session: a new <repo>/live_logs/session_<date>_<time>/monitor):
    main.jsonl        everything with a kind: status (full), task events, world summary on change, gripper, tcp (10 Hz),
                      skill action status + feedback, motion action status, vision stats (10 s), rosout (all nodes)
    vision_raw.jsonl  every /vision/boards frame as received (+ arrival time, gap, now - stamp, parse problems)
    world_raw.jsonl   every /task/world message, all fields
    joints.jsonl      every /joint_states message
    rosout.log        all node log lines as text (time, level, node, message)
Console (default, quiet): every WARN / ERROR of any node, job changes, warn / error alerts, slow / refused frames,
vision stats every 10 s. --verbose: also controller / world / gripper changes, skill actions, our nodes' log lines.
"""
import argparse
import glob
import json
import math
import os
import re
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatusArray
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from hk_fr5_interfaces.msg import GripperState
from hk_fr5_task import vision1
from hk_fr5_task_interfaces.action import Home, Pick, Place, ReleaseSafe, WaitClear
from hk_fr5_task_interfaces.msg import World

LIVE_ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "live_logs")  # <repo>/live_logs
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SKILLS = {"pick": Pick, "place": Place, "home": Home, "wait_clear": WaitClear, "release_safe": ReleaseSafe}
LEVEL = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
BENIGN = (("scene.scene_manager", "remove_box: unknown box"),          # first stack clean-up after a start (W13)
          ("motion.motion_controller", "PREEMPTED: stopped"))         # contact search stopped by the seal
ROS_DIR = os.environ.get("ROS_LOG_DIR", "")
NODE_RE = re.compile(r"\] \[([^\]]+)\]: ")
STATUS = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="")
ap.add_argument("--verbose", action="store_true", help="full console view (the files are always complete)")
ap.add_argument("--lite", action="store_true", help="demo: no raw vision / world / joints / tcp streams (TM-31)")
args = ap.parse_args()
args.quiet = not args.verbose
RUN = args.dir or (os.path.join(os.environ["HK_LIVE_DIR"], "monitor") if os.environ.get("HK_LIVE_DIR") else
                   os.path.join(LIVE_ROOT, time.strftime("session_%Y%m%d_%H%M%S"), "monitor"))
os.makedirs(RUN, exist_ok=True)
F = {k: open(os.path.join(RUN, k), "a", buffering=1) for k in
     ("main.jsonl", "vision_raw.jsonl", "world_raw.jsonl", "joints.jsonl", "rosout.log")}


def clean(x):
    """NaN / inf -> None, so every line is strict JSON."""
    if isinstance(x, float):
        return None if math.isnan(x) or math.isinf(x) else x
    if isinstance(x, dict):
        return {k: clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [clean(v) for v in x]
    return x


def log(kind, data, f="main.jsonl"):
    F[f].write(json.dumps(clean({"t": round(time.time(), 4), "kind": kind, "data": data})) + "\n")


def say(text, important=False):
    if important or not args.quiet:
        print(f"{time.strftime('%H:%M:%S')} {text}", flush=True)
    log("print", text)


S = {"status": None, "seen_alerts": set(), "world": None, "gaps": [], "lags": [], "last_arrival": None, "frames": 0,
     "stat_t": time.time(), "gripper": None, "goal_status": {}}


def on_status(m):
    st = json.loads(m.data)
    log("status", st)
    old = S["status"] or {}
    cur, ocur = st.get("current") or {}, old.get("current") or {}
    key = (st.get("state"), st.get("paused"), cur.get("skill"), cur.get("job"), cur.get("board"), st.get("phase"),
           st.get("skills_ready"), st.get("motion_ready"), st.get("world_ok"))
    okey = (old.get("state"), old.get("paused"), ocur.get("skill"), ocur.get("job"), ocur.get("board"), old.get("phase"),
            old.get("skills_ready"), old.get("motion_ready"), old.get("world_ok"))
    if key != okey:
        say(f"CTRL {st.get('state')}{' PAUSED' if st.get('paused') else ''} | skill {cur.get('skill') or '-'} "
            f"{cur.get('job') or ''} {cur.get('board') or ''} {cur.get('target') or ''} phase {st.get('phase') or '-'} | "
            f"skills_ready {st.get('skills_ready')} motion_ready {st.get('motion_ready')} world_ok {st.get('world_ok')}")
    jobs = {j["id"]: (j["state"], j.get("delivered"), j.get("to_fault"), j.get("lost"), j.get("skipped"), j.get("message"))
            for j in st.get("jobs", [])}
    ojobs = {j["id"]: (j["state"], j.get("delivered"), j.get("to_fault"), j.get("lost"), j.get("skipped"), j.get("message"))
             for j in old.get("jobs", [])}
    for jid, v in jobs.items():
        if ojobs.get(jid) != v:
            say(f"JOB  {jid} {v[0]} delivered {v[1]} to_fault {v[2]} lost {v[3]} skipped {v[4]} | {v[5]}", important=True)
    if st.get("problem_boards") != old.get("problem_boards"):
        say(f"PROB problem boards {st.get('problem_boards')}")
    for e in st.get("events", []):
        k = json.dumps(e, sort_keys=True)
        if k not in S["seen_alerts"]:
            S["seen_alerts"].add(k)
            say(f"EVT  {e.get('level', '')} {e.get('text', e)}", important=e.get("level") in ("warn", "error"))
    S["status"] = st


def on_events(m):
    log("task_event", m.data)
    say(f"WEV  {m.data}")


def on_world(m):
    if not args.lite:
        log("world", message_to_ordereddict(m), "world_raw.jsonl")
    zs = {z.name: (z.state, z.stack_count, round(z.stack_top_z, 4), round(z.base_z, 4), z.board_id) for z in m.zones}
    bs = sorted((b.id, b.variant, b.label, b.location, b.zone, round(b.x, 3), round(b.y, 3)) for b in m.boards)
    snap = {"vision_ok": m.vision_ok, "held": m.held_board_id, "zones": zs, "gripper": m.gripper_state,
            "obstacles": [(o.label, round(o.x, 3), round(o.y, 3), o.in_work_area) for o in m.obstacles]}
    old = S["world"]
    if old is None or old["snap"] != snap:
        log("world_change", snap)
        say(f"WRLD vision_ok {m.vision_ok} age {m.vision_age:.2f} held '{m.held_board_id}' gripper {m.gripper_state} "
            f"obstacles {snap['obstacles']} | zones (state, count, top, base, board) {zs}")
    ids = [b[:5] for b in bs]                                   # board list without the jittering x / y
    if old is None or old["ids"] != ids:
        log("world_boards", bs)
        say(f"WRLD boards (id, variant, label, location, zone, x, y) {bs}")
    S["world"] = {"snap": snap, "ids": ids}


def on_vision(m):
    now = time.time()
    try:
        data = json.loads(m.data)
        stamp, dets, problems = vision1.parse(data)
    except ValueError as e:
        F["vision_raw.jsonl"].write(json.dumps({"t": now, "refused": str(e), "raw": m.data[:20000]}) + "\n")
        say(f"VIS  frame refused: {e}", important=True)
        return
    gap = None if S["last_arrival"] is None else now - S["last_arrival"]
    if gap is not None:
        S["gaps"].append(gap)
    S["last_arrival"] = now
    S["lags"].append(now - stamp)
    S["frames"] += 1
    if not args.lite:
        F["vision_raw.jsonl"].write(json.dumps(clean({"t": now, "gap": gap, "lag": now - stamp, "problems": problems,
                                                      "frame": data})) + "\n")
    if problems:
        say(f"VIS  parse problems: {problems}", important=True)
    if gap is not None and gap > 1.0:
        say(f"VIS  SLOW frame: {gap:.2f} s since the previous one (stale_after 1.5 s)", important=True)
    if now - S["stat_t"] >= 10.0 and S["gaps"]:
        g, l = S["gaps"], S["lags"]
        st = {"frames": S["frames"], "hz": S["frames"] / (now - S["stat_t"]), "gap_mean": sum(g) / len(g), "gap_max": max(g),
              "lag_mean": sum(l) / len(l), "lag_max": max(l), "boards": len(data.get("boards", [])),
              "zones": [z.get("name") for z in data.get("zones", [])]}
        log("vision_stats", st)
        say(f"VIS  {st['frames']} frames ({st['hz']:.1f} Hz) | gap mean {st['gap_mean']:.2f} max {st['gap_max']:.2f} s | "
            f"now-stamp mean {st['lag_mean']:.2f} max {st['lag_max']:.2f} s | {st['boards']} boards, zones {st['zones']}",
            important=True)
        S.update(gaps=[], lags=[], frames=0, stat_t=now)


def on_rosout(m):
    lvl = LEVEL.get(m.level, str(m.level))
    line = f"{m.stamp.sec + m.stamp.nanosec * 1e-9:.3f} {lvl:5} [{m.name}] {m.msg}"
    F["rosout.log"].write(line + "\n")
    log("rosout", {"level": lvl, "node": m.name, "msg": m.msg, "file": m.file, "line": m.line})
    if m.level >= 30:
        benign = any(m.name == node and text in m.msg for node, text in BENIGN)
        say(f"LOG  {lvl} [{m.name}] {m.msg}", important=not benign)
    elif m.name.startswith("task.") and m.name != "task.live_monitor":
        say(f"LOG  [{m.name}] {m.msg}")


def on_gripper(m):
    d = message_to_ordereddict(m)
    log("gripper", d)
    k = (m.state, m.vacuum_on, getattr(m, "blow_off_on", None), getattr(m, "vacuum_achieved", None))
    if k != S["gripper"]:
        say(f"GRIP {d}")
    S["gripper"] = k


def on_joints(m):
    F["joints.jsonl"].write(json.dumps({"t": round(time.time(), 4), "name": list(m.name), "pos": [round(p, 5) for p in m.position],
                                        "vel": [round(v, 5) for v in m.velocity]}) + "\n")


def status_cb(name):
    def cb(m):
        for s in m.status_list:
            gid = bytes(s.goal_info.goal_id.uuid).hex()[:8]
            key = (name, gid)
            if S["goal_status"].get(key) != s.status:
                S["goal_status"][key] = s.status
                log("action_status", {"action": name, "goal": gid, "status": STATUS.get(s.status, s.status)})
                if not name.startswith("/motion"):
                    say(f"ACT  {name} goal {gid} {STATUS.get(s.status, s.status)}")
    return cb


def feedback_cb(name):
    def cb(m):
        log("action_feedback", {"action": name, "feedback": message_to_ordereddict(m.feedback)})
    return cb


def index_ros(final=False):
    """ROS names each node's log file after the executable (python3.14_<pid>_...): ros/by_node/<node>.log links +
    ros/INDEX.txt (file -> nodes, lines, WARN / ERROR counts at the end) so every node's file is easy to find."""
    if not ROS_DIR or not os.path.isdir(ROS_DIR):
        return
    os.makedirs(os.path.join(ROS_DIR, "by_node"), exist_ok=True)
    rows = []
    for f in sorted(glob.glob(os.path.join(ROS_DIR, "**", "*.log"), recursive=True)):
        if "by_node" in f:
            continue
        nodes, n, warn, err = [], 0, 0, 0
        with open(f, errors="replace") as fh:
            for line in fh:
                n += 1
                if n <= 400 or final:
                    m = NODE_RE.search(line)
                    if m and m.group(1) not in nodes:
                        nodes.append(m.group(1))
                if final:
                    warn += line.startswith("[WARN]") or "] [WARN]" in line
                    err += line.startswith("[ERROR]") or "] [ERROR]" in line
                elif n > 400:
                    break
        for node in nodes[:3]:
            link = os.path.join(ROS_DIR, "by_node", f"{node}.log")
            if not os.path.lexists(link):
                os.symlink(os.path.relpath(f, os.path.dirname(link)), link)
        rows.append(f"{os.path.relpath(f, ROS_DIR)}  nodes {', '.join(nodes) or '?'}" +
                    (f"  lines {n}  WARN {warn}  ERROR {err}" if final else ""))
    with open(os.path.join(ROS_DIR, "INDEX.txt"), "w") as fh:
        fh.write("\n".join(rows) + "\n")


def main():
    rclpy.init()
    n = rclpy.create_node("live_monitor", namespace="task")
    n.create_subscription(String, "/task/status", on_status, LATCHED)
    n.create_subscription(String, "/task/events", on_events, 50)
    n.create_subscription(World, "/task/world", on_world, LATCHED)
    n.create_subscription(String, "/vision/boards", on_vision, 10)
    n.create_subscription(Log, "/rosout", on_rosout, 1000)
    n.create_subscription(GripperState, "/gripper/state", on_gripper, LATCHED)
    if not args.lite:
        n.create_subscription(JointState, "/joint_states", on_joints, qos_profile_sensor_data)
    for k, typ in SKILLS.items():
        n.create_subscription(GoalStatusArray, f"/task/skills/{k}/_action/status", status_cb(f"skill {k}"), 10)
        n.create_subscription(typ.Impl.FeedbackMessage, f"/task/skills/{k}/_action/feedback", feedback_cb(f"skill {k}"), 10)
    for k in ("move_to", "move_delta"):
        n.create_subscription(GoalStatusArray, f"/motion/{k}/_action/status", status_cb(f"/motion/{k}"), 10)
    tfb = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfb, n)

    def tcp():
        try:
            tr = tfb.lookup_transform("base_link", "tcp", rclpy.time.Time()).transform
            log("tcp", {"x": tr.translation.x, "y": tr.translation.y, "z": tr.translation.z,
                        "q": [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]})
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
    if not args.lite:
        n.create_timer(0.1, tcp)
    n.create_timer(30.0, index_ros)
    say(f"live monitor: logging everything to {RUN}", important=True)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for f in F.values():
            f.close()
        try:
            index_ros(final=True)
        except OSError as e:
            print(f"log index failed: {e}")


if __name__ == "__main__":
    main()
