"""V3 (TM-22): Vision 1 JSON adapter (hk_fr5_task.vision1) + its effect in the world / core. No ROS.
Frames follow the agreed schema notes/vision1-changes.md; the REAL world.yaml (VARa / VARb / VARc, UNKWN).
Run: pixi run python3 tests/vision1_check.py"""
import copy
import json
import math
import os
import sys

import yaml

from hk_fr5_task import vision1
from hk_fr5_task.main_core import MainCore
from hk_fr5_task.world_core import World, rect_corners, rect_from_corners

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

CD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "hk_fr5_task", "config")
WCFG = dict(yaml.safe_load(open(os.path.join(CD, "world.yaml"))), fixed_zones=[])
MCFG = yaml.safe_load(open(os.path.join(CD, "main.yaml"))); MCFG["variants"] = list(WCFG["variants"])
TZ = WCFG["table_z"]

def board(variant, x, y, yaw=0.0, **kw):
    v = WCFG["variants"].get(variant, {"length": 0.25, "width": 0.18})
    b = {"id": 7, "variant": variant, "u": 900, "v": 500, "corners": [[1, 2]] * 4, "z": -0.055, "yaw": 0.3,
         "yaw_deg": 17.0, "zone": "pick", "conf": 0.91, "inside_calibration": True,
         "corners_xy": rect_corners(x, y, yaw, v["length"], v["width"])}
    b.update(kw)
    return b

FRAME = {"stamp": 1793104812.53, "source": "C920", "calibrated": True, "image_size": [1920, 1080],
         "boards": [board("VARa", -0.70, -0.25, 0.2), board("VARc", -0.45, 0.30, 1.5708, conf=None),
                    board("UNKWN", -0.60, 0.00, 0.0, inside_calibration=False)],
         "occluded": [{"variant": "VARb", "covered_by": 0}], "rejected": [{"reason": "has_tag"}],
         "destinations": [{"variant": "VARa", "x": -0.1, "y": 0.1, "yaw": 0.0, "age": 0.0}],
         "zones": [{"name": "drop_off_a", "kind": "drop_off", "x": -0.45, "y": 0.30, "yaw": 1.5708, "age": 0.0,
                    "size": [0.32, 0.24], "top_z": -0.055, "frames": 15, "tag_id": 1},
                   {"name": "processing_1", "kind": "processing", "x": -0.30, "y": -0.45, "yaw": 0.0, "age": 41.0,
                    "size": [0.32, 0.24], "top_z": -0.054}]}

# 1 parse: boards
stamp, dets, probs = vision1.parse(json.loads(json.dumps(FRAME)))
bd = [d for d in dets if d["cls"].startswith("board/")]
zd = {d["id"]: d for d in dets if d["cls"].startswith("zone/")}
check("stamp = Vision 1 capture time, no problems", stamp == FRAME["stamp"] and probs == [], str(probs))
check("3 boards -> board/<variant>, corners = corners_xy, no per-frame id", [d["cls"] for d in bd] ==
      ["board/VARa", "board/VARc", "board/UNKWN"] and bd[0]["corners"] == FRAME["boards"][0]["corners_xy"]
      and all(d["id"] == "" for d in bd))
check("conf -> score; conf missing / null (classic mode) -> 1.0; inside_calibration -> inside",
      bd[0]["score"] == 0.91 and bd[1]["score"] == 1.0 and bd[2]["inside"] is False and bd[0]["inside"] is True)
check("only boards[] + zones[] become detections (occluded / rejected / destinations ignored)", len(dets) == 5)

# 2 parse: zones
x, y, yaw, l, w = rect_from_corners(zd["drop_off_a"]["corners"])
check("zone -> zone/<kind> with its name; outline from centre + yaw + size round trip",
      zd["drop_off_a"]["cls"] == "zone/drop_off" and zd["processing_1"]["cls"] == "zone/processing"
      and abs(x + 0.45) < 1e-9 and abs(y - 0.30) < 1e-9 and abs(abs(yaw) - 1.5708) < 1e-4
      and abs(l - 0.32) < 1e-9 and abs(w - 0.24) < 1e-9, f"{x:.3f} {y:.3f} {yaw:.3f} {l:.3f} {w:.3f}")
check("zone top_z -> detection z (V2 base_z)", zd["drop_off_a"]["z"] == -0.055 and zd["processing_1"]["z"] == -0.054)

# 3 bad entries are skipped one by one, the rest of the frame is used
f = copy.deepcopy(FRAME)
f["zones"] += [{"name": "fault", "kind": "fault", "x": 0, "y": 0, "yaw": 0, "size": [0.2, 0.2], "top_z": TZ},
               {"name": "drop_off_b", "kind": "drop_off", "x": 0, "y": 0, "yaw": 0, "top_z": TZ},
               {"name": "drop_off_c", "kind": "drop_off", "x": 0, "y": 0, "yaw": 0, "size": [0.3, 0.2]},
               {"name": "", "kind": "drop_off", "x": 0, "y": 0, "yaw": 0, "size": [0.3, 0.2], "top_z": TZ}]
f["boards"] += [dict(board("VARb", -0.5, 0.0), corners_xy=[[0, 0], [1, 0], [1, 1]]), {"variant": "VARb"}]
_, dets2, probs2 = vision1.parse(f)
check("skipped with a reason each: fault kind, no size, no top_z, empty name, 3 corners, no corners_xy",
      len(probs2) == 6 and len(dets2) == 5 and any("fault" in p for p in probs2), "; ".join(probs2))
for name, data in (("calibrated false", dict(FRAME, calibrated=False)), ("calibrated missing",
                   {k: v for k, v in FRAME.items() if k != "calibrated"}),
                   ("no stamp", {k: v for k, v in FRAME.items() if k != "stamp"}), ("not an object", [1, 2])):
    try:
        vision1.parse(data); check(f"whole frame refused: {name}", False)
    except ValueError as e:
        check(f"whole frame refused: {name}", True, str(e))
_, d3, p3 = vision1.parse({"stamp": 1.0, "calibrated": True})
check("no boards / zones keys -> empty frame (not an error)", d3 == [] and p3 == [])

# 3b optional obstacles[] (TM-24, the LATER format)
_, d4, p4 = vision1.parse({"stamp": 1.0, "calibrated": True, "obstacles": [
    {"label": "hand", "corners_xy": rect_corners(-0.3, 0.1, 0.0, 0.12, 0.09)}, {"label": "hand"}]})
check("optional obstacles[] -> obstacle/<label>; a bad one skipped with a reason",
      [d["cls"] for d in d4] == ["obstacle/hand"] and len(p4) == 1 and p4[0].startswith("obstacle 1"), str(p4))

# 4 in the world model (real world.yaml)
wd = World(WCFG)
for k in range(5):
    wd.update(0.1 * (k + 1), dets)
s = wd.snapshot(0.5)
zs, bs = {z["name"]: z for z in s["zones"]}, {b["variant"]: b for b in s["boards"]}
check("zones from Vision 1: base_z = top_z, empty stack top = base_z",
      zs["drop_off_a"]["base_z"] == -0.055 and zs["drop_off_a"]["stack_top_z"] == -0.055
      and zs["processing_1"]["base_z"] == -0.054, str({n: z["base_z"] for n, z in zs.items()}))
check("VARa on the table -> input, size_ok with the real variant size", bs["VARa"]["location"] == "input"
      and bs["VARa"]["size_ok"], str((bs["VARa"]["length"], bs["VARa"]["width"])))
check("VARc lying on drop_off_a -> location zone (never a pick candidate of a normal job)",
      bs["VARc"]["location"] == "zone" and bs["VARc"]["zone"] == "drop_off_a")
check("UNKWN -> variant 'unknown', label UNKWN (nonvariant_labels)", "unknown" in bs and bs["unknown"]["label"] == "UNKWN")

# 5 core: UNKWN input board only when named
core = MainCore(MCFG)
job = {"id": "j1", "type": "transfer", "select_board": "", "select_variant": "", "from_zone": "", "_items": {}}
cand = {b["id"] for b in core._candidates(job, s)}
check("any-variant job: VARa is a candidate, UNKWN and the zone board are not",
      cand == {bs["VARa"]["id"]}, str(cand))
cand = {b["id"] for b in core._candidates(dict(job, select_board=bs["unknown"]["id"]), s)}
check("job naming the UNKWN board explicitly: it is the candidate", cand == {bs["unknown"]["id"]}, str(cand))
okj, msg, _ = core.add_job({"type": "transfer", "select_variant": "UNKWN", "to": "drop_off_a", "count": 1}, s, 0.5)
check("select_variant UNKWN is refused (not a variant)", not okj and "unknown variant" in msg, msg)

print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
