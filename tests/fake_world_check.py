"""Offline checks of hk_fr5_task.fake_core (T3): camera model vs the real calibration (hk_fr5_vision calib.apply as
the independent reference), bbox round trip, scenario -> detections -> world (through world_core), events, input
stacks, simulated operator (same / processed), hands, noise. Simulated time, no ROS.

    pixi run python3 tests/fake_world_check.py
"""
import copy
import math
import os
import sys

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from hk_fr5_task.fake_core import Camera, FakeWorld, bbox_corners, bbox_from_pixels
from hk_fr5_task.world_core import World
from hk_fr5_vision import calib

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_DIR = os.path.join(HERE, "..", "src", "hk_fr5_task", "config")
CFG = yaml.safe_load(open(os.path.join(CFG_DIR, "world.yaml")))
SCN = yaml.safe_load(open(os.path.join(CFG_DIR, "scenario_demo.yaml")))
CAL = calib.load(os.path.join(get_package_share_directory("hk_fr5_vision"), "config", "calib.yaml"))
cam = Camera(CAL)

# 1 camera model
pts = [(x, y) for x in np.linspace(-0.65, 0.05, 8) for y in np.linspace(-0.8, 0.8, 9)]
err_rt = max(math.dist(cam.to_table(*cam.to_pixel(x, y)), (x, y)) for x, y in pts) * 1000
pix = [cam.to_pixel(x, y) for x, y in pts]
X, Y, _ = calib.apply(CAL, [p[0] for p in pix], [p[1] for p in pix])
err_ref = max(math.dist((a, b), p) for a, b, p in zip(X, Y, pts)) * 1000
check("world -> pixel -> world round trip < 0.01 mm over the table", err_rt < 0.01, f"{err_rt:.5f} mm")
check("to_pixel agrees with hk_fr5_vision calib.apply (independent code) < 0.01 mm", err_ref < 0.01, f"{err_ref:.5f} mm")
check("calibrated table maps into the 1920x1080 image", all(0 <= u <= 1920 and 0 <= v <= 1080 for u, v in pix))
u, v, th, sx, sy = bbox_from_pixels(bbox_corners(900.0, 500.0, 0.3, 120.0, 80.0))
check("bbox -> corners -> bbox round trip", abs(u - 900) < 1e-9 and abs(v - 500) < 1e-9 and abs(th - 0.3) < 1e-9
      and abs(sx - 120) < 1e-9 and abs(sy - 80) < 1e-9)

# 2 scenario -> detections -> world (same path as the world_model node, local pixel source)
def to_world_dets(dets):
    out = []
    for d in dets:
        corners = [cam.to_table(*p) for p in bbox_corners(d["u"], d["v"], d["theta"], d["sx"], d["sy"])]
        out.append({"cls": d["cls"], "id": d["id"], "score": d["score"], "corners": corners})
    return out
s = copy.deepcopy(SCN); s["noise_px"] = 0.0
fw = FakeWorld(s, cam, CFG["variants"])
w = World(CFG)
w.update(0.1, to_world_dets(fw.detections(0.1)))
snap = w.snapshot(0.1)
bs = {b["variant"]: b for b in snap["boards"]}
check("demo scenario: 5 zones (incl. fault), 3 boards (input), all variants size_ok", len(snap["zones"]) == 5 and len(bs) == 3
      and all(b["location"] == "input" and b["size_ok"] for b in bs.values()), str({k: (b["location"], b["size_ok"]) for k, b in bs.items()}))
check("board pose through pixels: red at (-0.45, 0.22), yaw 0.2, < 1 mm / 0.01 rad (perspective of a 15 cm board)",
      math.dist((bs["red"]["x"], bs["red"]["y"]), (-0.45, 0.22)) < 0.001 and abs(bs["red"]["yaw"] - 0.2) < 0.01,
      f"({bs['red']['x']:.4f}, {bs['red']['y']:.4f}) yaw {bs['red']['yaw']:.4f}")
check("white_large yaw 1.57 -> long side, yaw wrapped to (-pi/2, pi/2]", abs(abs(bs["white_large"]["yaw"]) - 1.57) < 0.01,
      f"{bs['white_large']['yaw']:.4f}")
zs = {z["name"]: z for z in snap["zones"]}
check("zone geometry through pixels (processing_1 centre, size)", math.dist((zs["processing_1"]["x"], zs["processing_1"]["y"]), (-0.50, 0.45)) < 0.001
      and abs(zs["processing_1"]["length"] - 0.20) < 0.002)

# 3 noise: board stays one id, jitter small
fw = FakeWorld(SCN, cam, CFG["variants"], seed=3)
w = World(CFG); ys = []
for k in range(50):
    t = 0.1 * (k + 1)
    w.update(t, to_world_dets(fw.detections(t)))
    ys.append([b for b in w.snapshot(t)["boards"] if b["variant"] == "red"][0]["y"])
check("0.3 px corner noise, 50 frames: 3 boards keep 3 ids, red board jitter < 1 mm", len(w.snapshot(t)["boards"]) == 3
      and (max(ys) - min(ys)) * 1000 < 1.0, f"jitter {(max(ys) - min(ys)) * 1000:.2f} mm")

# 4 events: input stack, place into processing, operator take / return
def sim(fw, w, t0, t1, step=0.1):
    t = t0
    while t < t1 - 1e-9:
        t = round(t + step, 6); w.update(t, to_world_dets(fw.detections(t)))
    return t
s = copy.deepcopy(SCN); s["noise_px"] = 0.0
fw, w = FakeWorld(s, cam, CFG["variants"]), World(CFG)
t = sim(fw, w, 0.0, 0.5)
stack = [b for b in w.snapshot(t)["boards"] if b["variant"] == "white_small"][0]
for k in range(3):
    fw.event(t, {"type": "picked", "x": stack["x"], "y": stack["y"]}); w.event(t, {"type": "picked", "board": stack["id"]})
    t = sim(fw, w, t, t + 0.3)
    left = [b for b in w.snapshot(t)["boards"] if b["variant"] == "white_small" and b["location"] == "input"]
    check(f"input stack of 3: pick {k + 1} -> {'board still seen there' if k < 2 else 'stack gone'}",
          (len(left) == 1) == (k < 2), f"{len(left)} input white_small boards")
    if k < 2:
        stack = left[0]
        fw.event(t, {"type": "placed", "x": -0.15, "y": 0.55 + 0.0, "yaw": 0.0, "variant": "white_small"})
        w.event(t, {"type": "placed", "board": [b for b in w.snapshot(t)["boards"] if b["location"] == "held"][0]["id"],
                    "zone": "drop_off_1", "x": -0.15, "y": 0.55, "yaw": 0.0})
        t = sim(fw, w, t, t + 0.3)
z = {z["name"]: z for z in w.snapshot(t)["zones"]}["drop_off_1"]
check("2 boards placed on drop_off_1: one detection (top), stack count 2", z["stack_count"] == 2
      and sum(1 for d in fw.detections(t) if d["cls"] == "board/white_small") == 1, f"count {z['stack_count']}")
try:
    fw.event(t, {"type": "picked", "x": 0.0, "y": 0.0}); check("pick where no board is: rejected", False)
except ValueError as e:
    check("pick where no board is: rejected", "no board within" in str(e), str(e))

for mode, want_label in (("same", "red"), ("nonvariant", "other")):
    s = copy.deepcopy(SCN); s["noise_px"] = 0.0; s["operator"]["return_variant"] = mode
    fw, w = FakeWorld(s, cam, CFG["variants"]), World(CFG)
    t = sim(fw, w, 0.0, 0.3)
    red = [b for b in w.snapshot(t)["boards"] if b["variant"] == "red"][0]
    fw.event(t, {"type": "picked", "x": red["x"], "y": red["y"]}); w.event(t, {"type": "picked", "board": red["id"]})
    fw.event(t, {"type": "placed", "x": -0.50, "y": 0.45, "yaw": 0.0, "variant": "red"})
    w.event(t, {"type": "placed", "board": red["id"], "zone": "processing_1", "x": -0.50, "y": 0.45, "yaw": 0.0})
    states, t_end = [], t + 7.0
    while t < t_end:
        t = sim(fw, w, t, t + 0.25); states.append({z["name"]: z for z in w.snapshot(t)["zones"]}["processing_1"]["state"])
    seq = [s0 for i, s0 in enumerate(states) if i == 0 or states[i - 1] != s0]
    back = [b for b in w.snapshot(t)["boards"] if b["zone"] == "processing_1"]
    check(f"operator ({mode}): LOADED -> PROCESSING -> DONE, back with label '{want_label}', same id, variant red, processed",
          seq == ["LOADED", "PROCESSING", "DONE"] and len(back) == 1 and back[0]["label"] == want_label
          and back[0]["variant"] == "red" and back[0]["processed"] and back[0]["id"] == red["id"],
          f"{seq} {[(b['id'], b['variant'], b['label'], b['processed']) for b in back]}")

s = copy.deepcopy(SCN); s["hands"] = [{"t": 1.0, "x": -0.3, "y": 0.0, "duration": 1.0}]
fw = FakeWorld(s, cam, CFG["variants"])
hands = [sum(d["cls"] == "obstacle/hand" for d in fw.detections(t)) for t in (0.5, 1.2, 1.9, 2.1)]
check("hand event: seen only from t 1.0 for 1.0 s", hands == [0, 1, 1, 0], str(hands))

print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
