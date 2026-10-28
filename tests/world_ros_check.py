"""ROS wiring check (T3, V5 TM-24): fake_world -> /vision/boards (Vision 1 JSON) -> world_model -> /task/world.
Scripted skill events: pick VARa -> processing_1 -> (simulated operator) -> DONE -> pick -> drop_off_b (on its sheet);
a Vision 1 zone without size -> loud ERROR; fake_world stopped -> vision stale. ~30 s.

    ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=82 pixi run python3 tests/world_ros_check.py
"""
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from hk_fr5_task_interfaces.msg import World

ok = bad = 0
def _cfg(name):
    import yaml
    return yaml.safe_load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "hk_fr5_task", "config", name)))

def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

def exe(pkg, name):
    return os.path.join(get_package_prefix(pkg), "lib", pkg, name)
import tempfile
LOGS = tempfile.mkdtemp(prefix="world_ros_check_")
def start(cmd):
    """Output into a temp file, never an unread PIPE (a full 64 KB pipe blocks the node's logging, TM-13)."""
    log = open(os.path.join(LOGS, os.path.basename(cmd[0]) + ".log"), "w")
    p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    p.log_path = log.name
    log.close()
    return p
def stop(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
    rc = None
    try:
        rc = p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL); p.wait(); rc = "killed"
    with open(p.log_path) as f:
        return f.read() + (" [KILLED]" if rc == "killed" else ""), rc

procs = {"fake": start([exe("hk_fr5_task", "fake_world"), "--ros-args", "-p", "seed:=1"]),
         "world": start([exe("hk_fr5_task", "world_model")])}
rclpy.init(); n = rclpy.create_node("world_ros_test")
W = {"m": None}
n.create_subscription(World, "/task/world", lambda m: W.update(m=m), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
ev = n.create_publisher(String, "/task/events", 10)
def until(fn, timeout):
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        try:
            if W["m"] is not None and fn(W["m"]):
                return True
        except (IndexError, KeyError, StopIteration):
            pass
    return False
def zone(m, name):
    return next(z for z in m.zones if z.name == name)
def send(d):
    ev.publish(String(data=json.dumps(d)))
outs = {}
try:
    got = until(lambda m: m.vision_ok and len(m.boards) == 3 and len(m.zones) == 4, 15)
    m = W["m"]
    check("world: vision ok, 3 boards, 3 Vision 1 zones + the fixed fault zone (fake_world JSON -> world_model)", got,
          m and f"{len(m.boards)} boards, {len(m.zones)} zones, vision_ok {m.vision_ok}")
    red = next(b for b in m.boards if b.variant == "VARa")
    check("VARa board: (-0.185, -0.37) within 3 mm (1 mm corner noise), inside, size ok with the real size, input",
          math.dist((red.x, red.y), (-0.185, -0.37)) < 0.003 and red.inside and red.size_ok and red.location == "input",
          f"({red.x:.4f}, {red.y:.4f})")
    check("input stack board: top_z unknown (NaN, CL-16)", math.isnan(next(b for b in m.boards if b.variant == "VARb").top_z))
    check("zone base_z = Vision 1 top_z (sheet top)", abs(zone(m, "drop_off_b").base_z - (-0.055)) < 1e-9
          and abs(zone(m, "drop_off_b").stack_top_z - (-0.055)) < 1e-9)

    send({"type": "picked", "board": red.id, "x": red.x, "y": red.y})
    check("event picked -> held_board_id, the VARa board location held", until(lambda m: m.held_board_id == red.id
          and next(b for b in m.boards if b.id == red.id).location == "held", 3))
    send({"type": "placed", "board": red.id, "zone": "processing_1", "x": -0.48, "y": -0.42, "yaw": 1.5708, "variant": "VARa"})
    check("event placed -> processing_1 LOADED", until(lambda m: zone(m, "processing_1").state == "LOADED", 3))
    op, wc = _cfg("scenario_demo.yaml")["operator"], _cfg("world.yaml")        # waits follow the configs (CL-40)
    check("simulated operator takes it -> PROCESSING (zone busy)", until(lambda m: zone(m, "processing_1").state == "PROCESSING",
                                                                         op["take_after"] + wc["gone_after"] + 3))
    check("board comes back -> DONE, same board id, processed flag (CL-19)", until(lambda m: zone(m, "processing_1").state == "DONE"
          and zone(m, "processing_1").board_id == red.id and next(b for b in m.boards if b.id == red.id).processed,
          op["return_after"] + wc["back_after"] + 4),
          W["m"] and zone(W["m"], "processing_1").state)
    b = next(b for b in W["m"].boards if b.id == red.id)
    send({"type": "picked", "board": red.id, "x": b.x, "y": b.y})
    check("pick the processed board -> processing_1 EMPTY", until(lambda m: zone(m, "processing_1").state == "EMPTY", 3))
    send({"type": "placed", "board": red.id, "zone": "drop_off_b", "x": -0.16, "y": 0.37, "yaw": 1.5708, "variant": "VARa"})
    th = wc["variants"]["VARa"]["thickness"]
    check("place on drop_off_b -> stack count 1 on the sheet (top = sheet + thickness), board seen there",
          until(lambda m: zone(m, "drop_off_b").stack_count == 1 and abs(zone(m, "drop_off_b").stack_top_z - (-0.055 + th)) < 1e-9
          and next(b for b in m.boards if b.id == red.id).zone == "drop_off_b"
          and next(b for b in m.boards if b.id == red.id).age < 0.5, 3))
    vb = n.create_publisher(String, "/vision/boards", 5)
    vb.publish(String(data=json.dumps({"stamp": time.time(), "calibrated": True, "boards": [],
                                       "zones": [{"name": "drop_off_c", "kind": "drop_off", "x": 0, "y": 0.6, "yaw": 0, "top_z": -0.055}]})))
    rclpy.spin_once(n, timeout_sec=0.3)
    send({"type": "teleport"})
    rclpy.spin_once(n, timeout_sec=0.3)

    outs["fake"] = stop(procs.pop("fake"))
    check(f"fake_world stopped -> vision_ok false after stale_after {wc['stale_after']} s (2 Hz republish)",
          until(lambda m: not m.vision_ok, wc["stale_after"] + 1.5))
finally:
    for k, p in procs.items():
        outs[k] = stop(p)
    n.destroy_node(); rclpy.shutdown()
wl = outs["world"][0]
check("world_model log: bad event reported, the zone without size as a loud ERROR (TM-22 A), no traceback",
      "unknown event type" in wl and "zone(s) SKIPPED: zone drop_off_c: KeyError 'size'" in wl and "Traceback" not in wl
      and wl.count("SKIPPED") == 1, wl[-400:])
check("all nodes stopped cleanly on Ctrl+C", all(o[1] == 0 for o in outs.values()), str({k: o[1] for k, o in outs.items()}))
print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
