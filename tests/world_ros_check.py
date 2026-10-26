"""ROS wiring check (T3): fake_world -> /vision1/detections -> world_model -> /vision/pixel_to_table (the REAL
vision_transform with the real calibration) -> /task/world. Scripted skill events: pick red -> processing_1 ->
(simulated operator) -> DONE -> pick -> drop_off_2; fake_world stopped -> vision stale; bad parameter. ~25 s.

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
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

def exe(pkg, name):
    return os.path.join(get_package_prefix(pkg), "lib", pkg, name)
def start(cmd):
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
def stop(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
    try:
        return p.communicate(timeout=5)[0], p.returncode
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL); return p.communicate()[0] + " [KILLED]", "killed"

procs = {"vision": start([exe("hk_fr5_vision", "vision_transform")]),
         "fake": start([exe("hk_fr5_task", "fake_world"), "--ros-args", "-p", "seed:=1"]),
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
    check("world: vision ok, 3 boards, 4 zones (fake_world -> pixel_to_table service -> world_model)", got,
          m and f"{len(m.boards)} boards, {len(m.zones)} zones, vision_ok {m.vision_ok}")
    red = next(b for b in m.boards if b.variant == "red")
    check("red board through the real service: (-0.35, 0.10) within 1 mm, inside the calibrated area, size ok",
          math.dist((red.x, red.y), (-0.35, 0.10)) < 0.001 and red.inside and red.size_ok and red.location == "input",
          f"({red.x:.4f}, {red.y:.4f})")
    check("input stack board: top_z unknown (NaN, CL-16)", math.isnan(next(b for b in m.boards if b.variant == "white_small").top_z))

    send({"type": "picked", "board": red.id, "x": red.x, "y": red.y})
    check("event picked -> held_board_id, red board location held", until(lambda m: m.held_board_id == red.id
          and next(b for b in m.boards if b.id == red.id).location == "held", 3))
    send({"type": "placed", "board": red.id, "zone": "processing_1", "x": -0.50, "y": 0.45, "yaw": 0.0, "variant": "red"})
    check("event placed -> processing_1 LOADED", until(lambda m: zone(m, "processing_1").state == "LOADED", 3))
    check("simulated operator takes it -> PROCESSING (zone busy)", until(lambda m: zone(m, "processing_1").state == "PROCESSING", 6))
    check("board comes back -> DONE, same board id", until(lambda m: zone(m, "processing_1").state == "DONE"
          and zone(m, "processing_1").board_id == red.id, 8), W["m"] and zone(W["m"], "processing_1").state)
    b = next(b for b in W["m"].boards if b.id == red.id)
    send({"type": "picked", "board": red.id, "x": b.x, "y": b.y})
    check("pick the processed board -> processing_1 EMPTY", until(lambda m: zone(m, "processing_1").state == "EMPTY", 3))
    send({"type": "placed", "board": red.id, "zone": "drop_off_2", "x": -0.15, "y": -0.55, "yaw": 0.0, "variant": "red"})
    check("place on drop_off_2 -> stack count 1, board seen there", until(lambda m: zone(m, "drop_off_2").stack_count == 1
          and next(b for b in m.boards if b.id == red.id).zone == "drop_off_2"
          and next(b for b in m.boards if b.id == red.id).age < 0.5, 3))
    send({"type": "teleport"})
    rclpy.spin_once(n, timeout_sec=0.3)

    outs["fake"] = stop(procs.pop("fake"))
    check("fake_world stopped -> vision_ok false within 1.5 s (2 Hz republish)", until(lambda m: not m.vision_ok, 2.5))
finally:
    for k, p in procs.items():
        outs[k] = stop(p)
    n.destroy_node(); rclpy.shutdown()
wl = outs["world"][0]
check("world_model log: bad event reported, no traceback, no dropped-frame warnings",
      "unknown event type" in wl and "Traceback" not in wl and "not available" not in wl, wl[-300:])
check("all nodes stopped cleanly on Ctrl+C", all(o[1] == 0 for o in outs.values()), str({k: o[1] for k, o in outs.items()}))
p = start([exe("hk_fr5_task", "world_model"), "--ros-args", "-p", "pixel_source:=camera"])
out = p.communicate(timeout=15)[0]; rc = p.returncode
check("pixel_source:=camera -> exit 1 with a clear error", rc == 1 and "pixel_source must be" in out, out[-150:])

print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
