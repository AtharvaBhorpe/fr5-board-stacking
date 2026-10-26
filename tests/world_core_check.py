"""Offline checks of hk_fr5_task.world_core (T3): geometry, variants, tracking, zones + margin, processing-zone state
machine (CL-4 / CL-8), stacks, obstacles, stale vision, fixed-zone fallback, events. Simulated time, no ROS.

    pixi run python3 tests/world_core_check.py
"""
import copy
import math
import os
import sys

import numpy as np
import yaml

from hk_fr5_task.world_core import World, in_rect, rect_corners, rect_from_corners, wrap_half

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = yaml.safe_load(open(os.path.join(HERE, "..", "src", "hk_fr5_task", "config", "world.yaml")))
V = CFG["variants"]
rng = np.random.default_rng(1)

def det(cls, x, y, yaw=0.0, l=None, w=None, id="", score=0.9, noise=0.0, inside=True):
    if cls.startswith("board/") and l is None:
        v = V[cls.split("/")[1]]; l, w = v["length"], v["width"]
    c = np.array(rect_corners(x, y, yaw, l, w)) + (rng.normal(0, noise, (4, 2)) if noise else 0)
    return {"cls": cls, "id": id, "score": score, "corners": c.tolist(), "inside": inside}

ZONES = [det("zone/processing", -0.45, 0.45, 0.0, 0.20, 0.16, "processing_1"),
         det("zone/processing", -0.45, -0.45, 0.0, 0.20, 0.16, "processing_2"),
         det("zone/drop_off", -0.20, 0.55, 0.0, 0.22, 0.18, "drop_off_1"),
         det("zone/drop_off", -0.20, -0.55, 0.0, 0.22, 0.18, "drop_off_2")]
def zone(w, n):
    return next(z for z in w.snapshot(w.last_frame_t or 0)["zones"] if z["name"] == n)
def boards(w, t):
    return {b["id"]: b for b in w.snapshot(t)["boards"]}

# 1 geometry
x, y, yaw, l, wd = rect_from_corners(rect_corners(-0.3, 0.2, 0.4, 0.2, 0.1))
check("rect_from_corners(rect_corners): x, y, yaw, length, width round trip", abs(x + 0.3) < 1e-9 and abs(y - 0.2) < 1e-9
      and abs(yaw - 0.4) < 1e-9 and abs(l - 0.2) < 1e-9 and abs(wd - 0.1) < 1e-9)
x, y, yaw, l, wd = rect_from_corners(rect_corners(0, 0, 0.4 + math.pi / 2, 0.1, 0.2))   # width > length given
check("long side found when the corners start on the short side; yaw in (-pi/2, pi/2]", abs(l - 0.2) < 1e-9
      and abs(wd - 0.1) < 1e-9 and abs(yaw - 0.4) < 1e-9, f"yaw {yaw:.3f}")
check("wrap_half: pi -> 0, -pi/2 -> pi/2, 3.0 -> 3.0 - pi", abs(wrap_half(math.pi)) < 1e-12
      and abs(wrap_half(-math.pi / 2) - math.pi / 2) < 1e-12 and abs(wrap_half(3.0) - (3.0 - math.pi)) < 1e-12)
r = {"x": 0, "y": 0, "yaw": math.pi / 4, "length": 0.2, "width": 0.1}
check("in_rect: rotated rect + margin", in_rect(0.07, 0.07, r) and not in_rect(0.07, -0.07, r) and in_rect(0.07, -0.07, r, 0.05) and not in_rect(0.07, -0.07, r, 0.04))

# 2 variants, tracking with noise
w = World(CFG); t = 0.0
for k in range(100):
    t += 0.1
    w.update(t, ZONES + [det("board/red", -0.60, 0.0, 0.3, noise=0.0008, id=f"tr{k % 7}"),
                         det("board/white_small", -0.60, 0.12, 0.0, noise=0.0008)])
bs = boards(w, t)
check("2 boards, 100 noisy frames (0.8 mm corner noise, track ids changing): exactly 2 stable ids",
      sorted(bs) == ["b1", "b2"], str(sorted(bs)))
check("variant + size_ok + location input + top_z unknown (input stack, CL-16) + thickness per variant",
      bs["b1"]["variant"] == "red" and bs["b1"]["size_ok"] and bs["b1"]["location"] == "input"
      and math.isnan(bs["b1"]["top_z"]) and bs["b1"]["thickness"] == V["red"]["thickness"], str(bs["b1"]))
check("yaw / size measured within noise", abs(bs["b1"]["yaw"] - 0.3) < 0.02 and abs(bs["b1"]["length"] - V["red"]["length"]) < 0.002)
w.update(t + 0.1, ZONES + [det("board/red", -0.60, 0.0, 0.3, l=0.30, w=0.10), det("board/white_small", -0.60, 0.12)])
check("wrong size for the label -> size_ok false (CL-9)", not boards(w, t + 0.1)["b1"]["size_ok"])
w.update(t + 0.2, ZONES + [det("board/blue", -0.60, 0.0, 0.3, l=0.15, w=0.10), det("board/white_small", -0.60, 0.12),
                           det("board/blue", -0.30, 0.30, l=0.15, w=0.10)])
bs = boards(w, t + 0.2)
check("unknown label on a known board: original variant + thickness kept, label 'blue', size_ok false (CL-19)",
      bs["b1"]["variant"] == "red" and bs["b1"]["label"] == "blue" and not bs["b1"]["size_ok"]
      and bs["b1"]["thickness"] == V["red"]["thickness"], str(bs["b1"]))
check("NEW board with an unknown label: variant 'unknown', thickness NaN", bs["b3"]["variant"] == "unknown"
      and math.isnan(bs["b3"]["thickness"]))
w = World(CFG)
w.update(0.0, [det("board/red", -0.6, 0.0), det("board/red", -0.6, 0.155)])
w.update(0.1, [det("board/red", -0.6, 0.002), det("board/red", -0.6, 0.157)])
check("two boards side by side (155 mm apart) stay two ids", sorted(boards(w, 0.1)) == ["b1", "b2"])
w.update(0.2, [det("board/red", -0.6, 0.100), det("board/red", -0.6, 0.157)])
check("a board jumping 98 mm -> new id (old one waits for lost_after)", sorted(boards(w, 0.2)) == ["b1", "b2", "b3"])
w.update(0.2 + CFG["lost_after"] + 0.05, [det("board/red", -0.6, 0.100), det("board/red", -0.6, 0.157)])
check("input board unseen > lost_after -> removed", sorted(boards(w, 1.5)) == ["b2", "b3"], str(sorted(boards(w, 1.5))))
w = World(CFG)
w.update(0.0, [det("board/red", -0.6, 0.0, score=0.1), det("board/red", -0.6, 0.3, score=0.5)])
check("score below min_score ignored", list(boards(w, 0.0)) == ["b1"] and boards(w, 0.0)["b1"]["y"] > 0.2)

# 3 zones + margin
w = World(CFG)
w.update(0.0, ZONES + [det("board/red", -0.45, 0.45 + 0.08 + 0.02), det("board/red", -0.45, 0.45 + 0.08 + 0.05)])
bs = boards(w, 0.0)
check("board 20 mm outside a zone edge = in the zone (margin 30 mm, CL-7); 50 mm outside = input",
      bs["b1"]["location"] == "zone" and bs["b1"]["zone"] == "processing_1" and bs["b2"]["location"] == "input")
check("4 zones from Vision 1, kinds, not fixed", [z["name"] for z in w.snapshot(0)["zones"]] ==
      ["drop_off_1", "drop_off_2", "processing_1", "processing_2"] and zone(w, "drop_off_2")["kind"] == "drop_off"
      and not zone(w, "processing_1")["fixed"])

# 4 processing zone state machine (CL-4): place -> operator takes it -> brings it back -> robot picks
c = copy.deepcopy(CFG); c["expanded_detection"] = False
w = World(c); t = 0.0
def run(dets, secs, step=0.1):
    global t
    end = t + secs
    while t < end - 1e-9:
        t = round(t + step, 6); w.update(t, ZONES + dets)
run([det("board/red", -0.60, 0.0)], 0.5)
w.event(t, {"type": "picked", "board": "b1"})
check("picked -> held, held_board_id", boards(w, t)["b1"]["location"] == "held" and w.snapshot(t)["held_board_id"] == "b1")
run([], 0.3)
check("held board not removed while unseen", "b1" in boards(w, t))
w.event(t, {"type": "placed", "board": "b1", "zone": "processing_1", "x": -0.45, "y": 0.45, "yaw": 0.0})
check("placed in processing_1 -> LOADED, board_id b1, nothing held", zone(w, "processing_1")["state"] == "LOADED"
      and zone(w, "processing_1")["board_id"] == "b1" and w.snapshot(t)["held_board_id"] == "")
run([det("board/red", -0.45, 0.45)], 1.0)
check("board seen in the zone -> stays LOADED, same id", zone(w, "processing_1")["state"] == "LOADED" and sorted(boards(w, t)) == ["b1"])
run([], CFG["gone_after"] - 0.2)
check("operator takes it, before gone_after -> still LOADED (debounce)", zone(w, "processing_1")["state"] == "LOADED")
run([], 0.4)
check("zone empty >= gone_after -> PROCESSING (busy), board kept", zone(w, "processing_1")["state"] == "PROCESSING"
      and "b1" in boards(w, t))
run([det("board/red", -0.44, 0.46, 0.1)], CFG["back_after"] - 0.2)
check("board back, before back_after -> still PROCESSING", zone(w, "processing_1")["state"] == "PROCESSING")
run([det("board/red", -0.44, 0.46, 0.1)], 0.4)
check("board back >= back_after -> DONE, same identity b1 (CL-4), processed flag set, variant still red (CL-19)",
      zone(w, "processing_1")["state"] == "DONE" and sorted(boards(w, t)) == ["b1"] and zone(w, "processing_1")["board_id"] == "b1"
      and boards(w, t)["b1"]["processed"] and boards(w, t)["b1"]["variant"] == "red")
w.event(t, {"type": "picked", "board": "b1"})
check("robot picks it -> EMPTY", zone(w, "processing_1")["state"] == "EMPTY" and zone(w, "processing_1")["board_id"] == "")
w.event(t, {"type": "placed", "board": "b1", "zone": "drop_off_2", "x": -0.2, "y": -0.55, "yaw": 0.0})
run([det("board/red", -0.2, -0.55)], 0.3)
check("drop-off: count 1, stack_top_z = table + 6 mm, board top_z = stack top, OK",
      zone(w, "drop_off_2")["stack_count"] == 1 and abs(zone(w, "drop_off_2")["stack_top_z"] - (CFG["table_z"] + 0.006)) < 1e-9
      and abs(boards(w, t)["b1"]["top_z"] - (CFG["table_z"] + 0.006)) < 1e-9 and zone(w, "drop_off_2")["state"] == "OK")

# CL-19: expanded detection on -> only a non-variant (printed) board back = processed
c = copy.deepcopy(CFG); c["expanded_detection"] = True
for back, want in (("board/other", "DONE"), ("board/red", "FAULT")):
    w = World(c); t = 0.0
    run([det("board/red", -0.6, 0.0)], 0.3); w.event(t, {"type": "picked", "board": "b1"})
    w.event(t, {"type": "placed", "board": "b1", "zone": "processing_2", "x": -0.45, "y": -0.45, "yaw": 0.0})
    run([det("board/red", -0.45, -0.45)], 0.3); run([], 1.2); run([det(back, -0.45, -0.45, l=0.15, w=0.10)], 1.2)
    z = zone(w, "processing_2")
    b1 = boards(w, t)["b1"]
    check(f"expanded detection on: '{back.split('/')[1]}' board back -> {want}, processed {want == 'DONE'}, variant still red",
          z["state"] == want and (want == "DONE" or "not processed" in z["message"]) and b1["processed"] == (want == "DONE")
          and b1["variant"] == "red", f"{z['state']} {z['message']} {b1['label']}")
w.event(t, {"type": "picked", "board": z["board_id"]})
check("FAULT board picked (to the fault zone) -> EMPTY", zone(w, "processing_2")["state"] == "EMPTY")

# unknown board in an EMPTY processing zone, fault clears when the zone is empty again
w = World(CFG); t = 0.0
run([det("board/white_small", -0.45, 0.45)], CFG["unknown_after"] + 0.3)
check("unknown board in an EMPTY processing zone -> FAULT 'unknown board'", zone(w, "processing_1")["state"] == "FAULT"
      and "unknown board" in zone(w, "processing_1")["message"])
run([], CFG["gone_after"] + 0.3)
check("zone empty again -> EMPTY", zone(w, "processing_1")["state"] == "EMPTY", zone(w, "processing_1")["message"])

# placed into a busy zone -> FAULT; processing timeout -> overdue message, logged once
c = copy.deepcopy(CFG); c["process_timeout"] = 3.0
w = World(c); t = 0.0
run([det("board/red", -0.6, 0.0), det("board/red", -0.6, 0.3)], 0.3)
w.event(t, {"type": "picked", "board": "b1"}); w.event(t, {"type": "placed", "board": "b1", "zone": "processing_1", "x": -0.45, "y": 0.45})
run([det("board/red", -0.45, 0.45), det("board/red", -0.6, 0.3)], 0.3)
run([det("board/red", -0.6, 0.3)], 6.0)
z = zone(w, "processing_1")
check("PROCESSING longer than process_timeout -> message 'overdue', state stays PROCESSING, logged once",
      z["state"] == "PROCESSING" and "overdue" in z["message"] and sum("overdue" in m for _, m in w.log) == 1, z["message"])
w.event(t, {"type": "picked", "board": "b2"})
w.event(t, {"type": "placed", "board": "b2", "zone": "processing_1", "x": -0.45, "y": 0.45})
check("placed into a busy zone -> FAULT", zone(w, "processing_1")["state"] == "FAULT" and "busy" in zone(w, "processing_1")["message"])

# 5 stacks: count, top, FULL, top board keeps its id
c = copy.deepcopy(CFG); c["max_stack"] = 3
w = World(c); t = 0.0
for k in range(3):
    run([det("board/white_small", -0.6, 0.0)], 0.2)
    i = [b for b in boards(w, t) if boards(w, t)[b]["location"] == "input"][0]
    w.event(t, {"type": "picked", "board": i}); run([], 0.1)
    w.event(t, {"type": "placed", "board": i, "zone": "drop_off_1", "x": -0.2, "y": 0.55, "yaw": 0.0})
    run([det("board/white_small", -0.2, 0.55)], 0.3)
    top = i
z = zone(w, "drop_off_1")
check("3 boards stacked: count 3, top 18 mm above the table, FULL at max_stack 3", z["stack_count"] == 3
      and abs(z["stack_top_z"] - (CFG["table_z"] + 0.018)) < 1e-9 and z["state"] == "FULL", f"{z['stack_count']} {z['state']}")
bs = boards(w, t)
check("top board of the stack = the last placed one (seen), lower ones kept (hidden)", bs[top]["age"] < 0.15
      and len([b for b in bs.values() if b["zone"] == "drop_off_1"]) == 3, str({k: round(v["age"], 2) for k, v in bs.items()}))

# 6 obstacles, stale vision, fixed zone fallback, events errors
w = World(CFG)
w.update(0.0, [det("obstacle/hand", -0.4, 0.2, l=0.12, w=0.08), det("obstacle/unknown", 0.5, 0.0, l=0.1, w=0.1)])
o = w.snapshot(0.0)["obstacles"]
check("obstacles: hand on the table in the work area, object at x 0.5 outside", [x["in_work_area"] for x in o] == [True, False]
      and o[0]["label"] == "hand")
check("vision ok right after a frame, stale after stale_after, NaN age before any frame",
      w.snapshot(0.1)["vision_ok"] and not w.snapshot(CFG["stale_after"] + 0.1)["vision_ok"]
      and math.isnan(World(CFG).snapshot(0)["vision_age"]))
c = copy.deepcopy(CFG); c["fixed_zones"] = [{"name": "fault", "kind": "fault", "x": -0.3, "y": 0.95, "yaw": 0.0, "length": 0.25, "width": 0.2}]
w = World(c); w.update(0.0, ZONES)
check("fixed fault zone (CL-15 fallback) present when Vision 1 has none", zone(w, "fault")["fixed"])
w.update(0.1, ZONES + [det("zone/fault", -0.3, 0.80, 0.0, 0.2, 0.15, "fault")])
check("Vision 1 detects the fault zone -> it replaces the fixed one", not zone(w, "fault")["fixed"] and abs(zone(w, "fault")["y"] - 0.80) < 1e-9)
for ev, text in [({"type": "picked", "board": "b99"}, "unknown board"), ({"type": "teleport"}, "unknown event"),
                 ({"type": "reset_zone", "zone": "nowhere"}, "unknown zone")]:
    try:
        w.event(1.0, ev); check(f"event {ev['type']} rejected", False)
    except ValueError as e:
        check(f"event {ev['type']} rejected: {text}", text in str(e), str(e))

# 7 arm shadow (CL-20 B): tcp at (-0.45, 0.45) above processing_1, band from the base, radius 0.12
SH = {"x": -0.45, "y": 0.45, "radius": 0.12}
w = World(CFG); t = 0.0
run([det("board/red", -0.6, -0.3)], 0.3)
w.update(t + 0.1, ZONES + [det("board/red", -0.6, -0.3), det("board/white_small", -0.40, 0.40)], SH); t += 0.1
check("shadow: a board seen under the arm (held board / arm) is ignored, the other is tracked",
      sorted(boards(w, t)) == ["b1"], str(sorted(boards(w, t))))
w.event(t, {"type": "picked", "board": "b1"}); w.event(t, {"type": "placed", "board": "b1", "zone": "processing_1", "x": -0.45, "y": 0.45})
for k in range(30):                                      # arm stays over the zone 3 s, zone looks empty
    t = round(t + 0.1, 6); w.update(t, ZONES, SH)
check("zone covered by the arm 3 s (looks empty): stays LOADED (timers frozen)", zone(w, "processing_1")["state"] == "LOADED")
for k in range(5):
    t = round(t + 0.1, 6); w.update(t, ZONES)
check("arm gone, zone empty < gone_after again (debounce restarted) -> still LOADED", zone(w, "processing_1")["state"] == "LOADED")
for k in range(10):
    t = round(t + 0.1, 6); w.update(t, ZONES)
check("arm gone, zone empty >= gone_after -> PROCESSING", zone(w, "processing_1")["state"] == "PROCESSING")
w = World(CFG); t = 0.0
run([det("board/red", -0.30, 0.10)], 0.3)
for k in range(30):                                      # arm over the input board 3 s
    t = round(t + 0.1, 6); w.update(t, ZONES, {"x": -0.30, "y": 0.10, "radius": 0.12})
check("input board hidden under the arm 3 s: not lost; its age shows it was not seen", "b1" in boards(w, t)
      and boards(w, t)["b1"]["age"] > 2.5)
t = round(t + 0.5, 6); w.update(t, ZONES)
check("arm gone, board not visible yet: grace period (lost_after from the shadow end)", "b1" in boards(w, t))

print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
