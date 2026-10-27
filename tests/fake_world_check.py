"""Offline checks of hk_fr5_task.fake_core (T3, V5 TM-24): demo layout rules, scenario -> Vision 1 frame -> vision1.parse
-> world (the real adapter path), noise, events, input stacks, simulated operator (same / processed), hands as the
optional obstacles[], contact-sim surfaces on zone sheets. REAL world.yaml (VARa / VARb / VARc). Simulated time, no ROS.

    pixi run python3 tests/fake_world_check.py
"""
import copy
import json
import math
import os
import sys

import yaml

from hk_fr5_task import mask as M
from hk_fr5_task import vision1
from hk_fr5_task.fake_core import FakeWorld
from hk_fr5_task.world_core import World, in_rect, rect_corners

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_DIR = os.path.join(HERE, "..", "src", "hk_fr5_task", "config")
CFG = yaml.safe_load(open(os.path.join(CFG_DIR, "world.yaml")))
MCFG = yaml.safe_load(open(os.path.join(CFG_DIR, "main.yaml")))
SCN = yaml.safe_load(open(os.path.join(CFG_DIR, "scenario_demo.yaml")))
V, TZ, MARGIN = CFG["variants"], CFG["table_z"], CFG["zone_margin"]

def quiet(s):
    s = copy.deepcopy(s); s["noise_xy"] = 0.0
    return s

def frame_dets(fw, t):
    """Fake frame -> JSON text -> the real parser (what world_model does)."""
    _, dets, problems = vision1.parse(json.loads(json.dumps(fw.frame(t, 1000.0 + t))))
    assert not problems, problems
    return dets

# 1 demo layout rules (scenario header)
def rect(o, l=None, w=None):
    return {"x": o["x"], "y": o["y"], "yaw": o.get("yaw", 0.0), "length": l or o["length"], "width": w or o["width"]}
zones = [rect(z) for z in SCN["zones"]] + [rect(f) for f in CFG["fixed_zones"]]
EXTRA = [{"variant": "VARc", "x": 0.02, "y": -0.60, "yaw": 1.5708}]      # main_ros_check's 2nd VARc
boards = [rect(b, V[b["variant"]]["length"], V[b["variant"]]["width"]) for b in SCN["boards"] + EXTRA]
def corners(r):
    return rect_corners(r["x"], r["y"], r["yaw"], r["length"], r["width"])
def _edge_dist(px, py, r):
    """Distance from a point to a rotated rectangle (0 inside)."""
    c, s = math.cos(r["yaw"]), math.sin(r["yaw"])
    u, w = c * (px - r["x"]) + s * (py - r["y"]), -s * (px - r["x"]) + c * (py - r["y"])
    return math.hypot(max(abs(u) - r["length"] / 2, 0.0), max(abs(w) - r["width"] / 2, 0.0))
def sep(a, b):
    """Gap between two rectangles (-1 if they overlap): the closest pair of two disjoint rectangles has a corner."""
    if any(in_rect(px, py, b) for px, py in corners(a)) or any(in_rect(px, py, a) for px, py in corners(b)):
        return -1.0
    return min([_edge_dist(px, py, b) for px, py in corners(a)] + [_edge_dist(px, py, a) for px, py in corners(b)])
pts = [p for r in zones[:-1] + boards for p in corners(r)]
check("layout: every zone / board corner inside the calibrated area (x -0.66..0.09, y -0.86..0.84)",
      all(-0.66 <= x <= 0.09 and -0.86 <= y <= 0.84 for x, y in pts))
check("layout: every scenario zone / board centre 0.30-0.80 m from the base (the config fault zone is measured on the "
      "robot, TM-29)", all(0.30 <= math.hypot(r["x"], r["y"]) <= 0.80 for r in zones[:len(SCN["zones"])] + boards),
      str([round(math.hypot(r["x"], r["y"]), 2) for r in zones[:len(SCN["zones"])] + boards]))
arm = [M.capsule((0.0, 0.0), tuple(MCFG["home_xy"]), 0.12)]
check("layout: nothing under the arm at home (CL-34)", not any(M.hits(arm, corners(r)) for r in zones + boards))
check("layout: centres >= 0.20 m off the y = 0 mounting profile", all(abs(r["y"]) >= 0.20 for r in zones + boards))
zz = [sep(a, b) for i, a in enumerate(zones) for b in zones[i + 1:]]
check("layout: zones (incl. the fault zone) >= 2 x zone_margin apart", min(zz) >= 2 * MARGIN - 1e-6, f"min {min(zz):.3f}")
bz = [sep(b, z) for b in boards for z in zones]
check("layout: input stacks >= zone_margin + 3 cm outside every zone (R6)", min(bz) >= MARGIN + 0.03 - 1e-9, f"min {min(bz):.3f}")
bb = [sep(a, b) for i, a in enumerate(boards) for b in boards[i + 1:]]
check("layout: input stacks >= 3 cm apart (not touching)", min(bb) >= 0.03 - 1e-9, f"min {min(bb):.3f}")
check("layout: zone board area fits the largest variant (VARa) + 2 cm each way",
      all(z["length"] >= V["VARa"]["length"] + 0.04 - 1e-9 and z["width"] >= V["VARa"]["width"] + 0.04 - 1e-9 for z in SCN["zones"]))

# 2 scenario -> frame -> parse -> world
fw = FakeWorld(quiet(SCN), V)
fr = fw.frame(0.1, 1234.5)
check("frame = Vision 1 schema: stamp, calibrated, boards with corners_xy, zones with size + top_z, no fault zone",
      fr["stamp"] == 1234.5 and fr["calibrated"] and all(len(b["corners_xy"]) == 4 for b in fr["boards"])
      and all("size" in z and "top_z" in z for z in fr["zones"]) and {z["kind"] for z in fr["zones"]} == {"drop_off", "processing"})
w = World(CFG)
w.update(0.1, frame_dets(fw, 0.1))
snap = w.snapshot(0.1)
bs = {b["variant"]: b for b in snap["boards"]}
zs = {z["name"]: z for z in snap["zones"]}
check("demo: 3 Vision 1 zones + the fixed fault zone, 3 input boards, all size_ok with the real sizes",
      sorted(zs) == ["drop_off_a", "drop_off_b", "fault", "processing_1"] and len(bs) == 3
      and all(b["location"] == "input" and b["size_ok"] for b in bs.values()),
      str({k: (b["location"], b["size_ok"]) for k, b in bs.items()}))
check("board pose: VARa at (-0.185, -0.37), long side yaw pi/2 (wrapped)", math.dist((bs["VARa"]["x"], bs["VARa"]["y"]),
      (-0.185, -0.37)) < 1e-6 and abs(abs(bs["VARa"]["yaw"]) - 1.5708) < 1e-4, f"{bs['VARa']['yaw']:.4f}")
check("zone geometry + base_z = the scenario's top_z", math.dist((zs["processing_1"]["x"], zs["processing_1"]["y"]),
      (-0.48, -0.42)) < 1e-6 and abs(zs["processing_1"]["length"] - 0.34) < 1e-6 and zs["drop_off_a"]["base_z"] == -0.055)

# 3 noise: boards keep their ids, jitter small
fw = FakeWorld(SCN, V, seed=3)
w = World(CFG); ys = []
for k in range(50):
    t = 0.1 * (k + 1)
    w.update(t, frame_dets(fw, t))
    ys.append([b for b in w.snapshot(t)["boards"] if b["variant"] == "VARa"][0]["y"])
check(f"{SCN['noise_xy'] * 1000:.0f} mm corner noise, 50 frames: 3 boards keep 3 ids, centre jitter < 3 mm",
      len(w.snapshot(t)["boards"]) == 3 and (max(ys) - min(ys)) * 1000 < 3.0, f"jitter {(max(ys) - min(ys)) * 1000:.2f} mm")

# 4 events: input stack, place onto a drop-off
def sim(fw, w, t0, t1, step=0.1):
    t = t0
    while t < t1 - 1e-9:
        t = round(t + step, 6); w.update(t, frame_dets(fw, t))
    return t
DA = next(z for z in SCN["zones"] if z["name"] == "drop_off_a")
fw, w = FakeWorld(quiet(SCN), V), World(CFG)
t = sim(fw, w, 0.0, 0.5)
stack = [b for b in w.snapshot(t)["boards"] if b["variant"] == "VARb"][0]
for k in range(3):
    fw.event(t, {"type": "picked", "x": stack["x"], "y": stack["y"]}); w.event(t, {"type": "picked", "board": stack["id"]})
    t = sim(fw, w, t, t + 0.3)
    left = [b for b in w.snapshot(t)["boards"] if b["variant"] == "VARb" and b["location"] == "input"]
    check(f"input stack of 3: pick {k + 1} -> {'board still seen there' if k < 2 else 'stack gone'}",
          (len(left) == 1) == (k < 2), f"{len(left)} input VARb boards")
    if k < 2:
        stack = left[0]
        fw.event(t, {"type": "placed", "x": DA["x"], "y": DA["y"], "yaw": DA["yaw"], "variant": "VARb"})
        w.event(t, {"type": "placed", "board": [b for b in w.snapshot(t)["boards"] if b["location"] == "held"][0]["id"],
                    "zone": "drop_off_a", "x": DA["x"], "y": DA["y"], "yaw": DA["yaw"]})
        t = sim(fw, w, t, t + 0.3)
z = {z["name"]: z for z in w.snapshot(t)["zones"]}["drop_off_a"]
th = V["VARb"]["thickness"]
check("2 boards placed on drop_off_a: one board seen (top), count 2, stack top = sheet + 2 x thickness",
      z["stack_count"] == 2 and sum(1 for b in fw.frame(t, t)["boards"] if b["variant"] == "VARb") == 1
      and abs(z["stack_top_z"] - (-0.055 + 2 * th)) < 1e-9, f"count {z['stack_count']}, top {z['stack_top_z']}")
try:
    fw.event(t, {"type": "picked", "x": 0.0, "y": 0.0}); check("pick where no board is: rejected", False)
except ValueError as e:
    check("pick where no board is: rejected", "no board within" in str(e), str(e))

# 5 simulated operator (processing)
P1 = next(z for z in SCN["zones"] if z["name"] == "processing_1")
for mode, want_label in (("same", "VARa"), ("nonvariant", "UNKWN")):
    s = quiet(SCN); s["operator"]["return_variant"] = mode
    fw, w = FakeWorld(s, V), World(CFG)
    t = sim(fw, w, 0.0, 0.3)
    a = [b for b in w.snapshot(t)["boards"] if b["variant"] == "VARa"][0]
    fw.event(t, {"type": "picked", "x": a["x"], "y": a["y"]}); w.event(t, {"type": "picked", "board": a["id"]})
    fw.event(t, {"type": "placed", "x": P1["x"], "y": P1["y"], "yaw": P1["yaw"], "variant": "VARa"})
    w.event(t, {"type": "placed", "board": a["id"], "zone": "processing_1", "x": P1["x"], "y": P1["y"], "yaw": P1["yaw"]})
    op = SCN["operator"]                                  # waits follow the scenario + world timers (CL-40)
    states, t_end = [], t + op["take_after"] + CFG["gone_after"] + op["return_after"] + CFG["back_after"] + 1.5
    while t < t_end:
        t = sim(fw, w, t, t + 0.25); states.append({z["name"]: z for z in w.snapshot(t)["zones"]}["processing_1"]["state"])
    seq = [s0 for i, s0 in enumerate(states) if i == 0 or states[i - 1] != s0]
    back = [b for b in w.snapshot(t)["boards"] if b["zone"] == "processing_1"]
    check(f"operator ({mode}): LOADED -> PROCESSING -> DONE, back with label '{want_label}', same id, variant VARa, processed",
          seq == ["LOADED", "PROCESSING", "DONE"] and len(back) == 1 and back[0]["label"] == want_label
          and back[0]["variant"] == "VARa" and back[0]["processed"] and back[0]["id"] == a["id"],
          f"{seq} {[(b['id'], b['variant'], b['label'], b['processed']) for b in back]}")

# 6 hands -> optional obstacles[] (TM-24)
s = copy.deepcopy(SCN); s["hands"] = [{"t": 1.0, "x": -0.3, "y": 0.0, "duration": 1.0}]
fw = FakeWorld(s, V)
hands = [len(fw.frame(t, t)["obstacles"]) for t in (0.5, 1.2, 1.9, 2.1)]
w = World(CFG); w.update(1.2, frame_dets(fw, 1.2))
check("hand: in obstacles[] only from t 1.0 for 1.0 s; the world sees an obstacle in the work area",
      hands == [0, 1, 1, 0] and len(w.snapshot(1.2)["obstacles"]) == 1 and w.snapshot(1.2)["obstacles"][0]["in_work_area"],
      str(hands))

# 7 contact-sim surfaces: table, zone sheet, mixed stack on a sheet
V2 = copy.deepcopy(V); V2["VARc"]["thickness"] = 0.004
fw = FakeWorld({"zones": SCN["zones"], "boards": [{"variant": "VARa", "x": -0.185, "y": -0.37, "yaw": 1.5708}],
                "noise_xy": 0.0}, V2)
check("surface: input board on the table = table + its thickness; empty spot = table; empty zone = sheet top",
      abs(fw.surface_at(-0.185, -0.37, TZ) - (TZ + V["VARa"]["thickness"])) < 1e-9 and fw.surface_at(-0.40, -0.05, TZ) == TZ
      and fw.surface_at(DA["x"], DA["y"], TZ) == -0.055)
for v in ("VARb", "VARc"):                                        # VARb 5 mm, VARc 4 mm on top, on drop_off_a's sheet
    fw.event(0.0, {"type": "placed", "x": DA["x"], "y": DA["y"], "yaw": DA["yaw"], "variant": v})
top = fw.surface_at(DA["x"], DA["y"], TZ)
fw.event(0.0, {"type": "picked", "x": DA["x"], "y": DA["y"]})
after = fw.surface_at(DA["x"], DA["y"], TZ)
check("surface, mixed stack on a sheet: sheet + 5 + 4 mm, after picking the top: sheet + 5 mm (VARb below)",
      abs(top - (-0.055 + 0.009)) < 1e-9 and abs(after - (-0.055 + 0.005)) < 1e-9, f"{top} -> {after}")

print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
