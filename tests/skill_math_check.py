"""Offline checks of hk_fr5_task.skill_math (T4): tcp yaw choice, pick / place heights, zone acceptance,
board lookup. No ROS.

    pixi run python3 tests/skill_math_check.py
"""
import math
import os
import sys

import yaml

from hk_fr5_task import skill_math as sm

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

HERE = os.path.dirname(os.path.abspath(__file__))
C = yaml.safe_load(open(os.path.join(HERE, "..", "src", "hk_fr5_task", "config", "skills.yaml")))
P2 = math.pi / 2

check("wrap_pi: 3pi/2 -> -pi/2, -pi -> pi, 0 -> 0", abs(sm.wrap_pi(3 * P2) + P2) < 1e-12 and abs(sm.wrap_pi(-math.pi) - math.pi) < 1e-12
      and sm.wrap_pi(0.0) == 0.0)
cases = [(0.3, P2, 0.3), (-0.3, P2, -0.3 + math.pi), (P2, P2, P2), (-P2 + 0.01, P2, P2 + 0.01), (1.2, -P2, 1.2 - math.pi)]
res = [sm.tcp_yaw_for(b, r) for b, r, _ in cases]
check("tcp yaw: board yaw or +pi, whichever is closer to the reference", all(abs(sm.wrap_pi(x - want)) < 1e-9
      for x, (_, _, want) in zip(res, cases)), str([round(x, 3) for x in res]))
check("tcp yaw always within pi/2 of the reference", all(abs(sm.wrap_pi(sm.tcp_yaw_for(a / 10, P2) - P2)) <= P2 + 1e-9
                                                         for a in range(-40, 41)))

hz, end, search = sm.pick_heights(C, {"top_z": float("nan")})
check("input board (top unknown): contact search from table + max input stack + clear", search
      and abs(end - (C["table_z"] + C["max_input_stack_height"] + C["search_clear"])) < 1e-12 and hz == C["hover_z"])
hz, end, search = sm.pick_heights(C, {"top_z": -0.054})
check("known top (zone board): descend to top - press, no search", not search and abs(end - (-0.054 - C["press"])) < 1e-12)
check("search end = table - press", abs(sm.search_end_z(C) - (C["table_z"] - C["press"])) < 1e-12)

pz = {"kind": "processing", "x": -0.5, "y": 0.45, "yaw": 0.1, "state": "EMPTY", "stack_count": 0, "stack_top_z": -0.06, "base_z": -0.06}
dz = {"kind": "drop_off", "x": -0.15, "y": -0.55, "yaw": 0.0, "state": "OK", "stack_count": 2, "stack_top_z": -0.048, "base_z": -0.06}
x, y, yaw, z = sm.place_target(C, pz, 0.006)
check("place in processing: on the table, tcp z = table + thickness + gap", (x, y, yaw) == (-0.5, 0.45, 0.1)
      and abs(z - (C["table_z"] + 0.006 + C["place_gap"])) < 1e-12)
x, y, yaw, z = sm.place_target(C, dz, 0.006)
check("place on a drop-off stack of 2: tcp z = stack top + thickness + gap", abs(z - (-0.048 + 0.006 + C["place_gap"])) < 1e-12)
check("zone_accepts: processing EMPTY ok, LOADED occupied; drop-off ok, FULL / count >= max full",
      sm.zone_accepts(C, pz) == "" and sm.zone_accepts(C, dict(pz, state="LOADED")) == "TARGET_OCCUPIED"
      and sm.zone_accepts(C, dz) == "" and sm.zone_accepts(C, dict(dz, state="FULL")) == "STACK_FULL"
      and sm.zone_accepts(C, dict(dz, stack_count=C["max_stack"])) == "STACK_FULL")

Z = {"name": "drop_off_1", "kind": "drop_off", "x": -0.15, "y": 0.55, "yaw": 0.3, "stack_top_z": C["table_z"] + 0.012,
     "base_z": C["table_z"]}
B = [{"id": "s1", "location": "zone", "zone": "drop_off_1", "length": 0.15, "width": 0.10, "thickness": 0.006},
     {"id": "s2", "location": "zone", "zone": "drop_off_1", "length": 0.10, "width": 0.07, "thickness": 0.006},
     {"id": "x9", "location": "zone", "zone": "drop_off_2", "length": 0.20, "width": 0.14, "thickness": 0.006}]
bx = sm.stack_box(C, Z, B)
check("stack_box: one box for a stack of 2 (height = stack top - margin, footprint = largest board on it + pad, zone yaw)",
      bx is not None and abs(bx["size"][2] - (0.012 - C["stack_box_margin"])) < 1e-9
      and abs(bx["size"][0] - (0.15 + 2 * C["stack_box_pad"])) < 1e-9 and abs(bx["size"][1] - (0.10 + 2 * C["stack_box_pad"])) < 1e-9
      and abs(bx["position"][2] - (C["table_z"] + bx["size"][2] / 2)) < 1e-9 and bx["yaw"] == 0.3, str(bx))
bs1 = sm.stack_box(C, Z, B, skip="s2")
check("stack_box: picking the top board -> the box without it (S-11), a board of another zone does not count",
      bs1 is not None and abs(bs1["size"][2] - (0.006 - C["stack_box_margin"])) < 1e-9, str(bs1))
check("stack_box: no box for an empty stack, for the last board being picked, or for a processing zone",
      sm.stack_box(C, dict(Z, stack_top_z=C["table_z"]), []) is None
      and sm.stack_box(C, dict(Z, stack_top_z=C["table_z"] + 0.006), B[:1], skip="s1") is None
      and sm.stack_box(C, dict(Z, kind="processing"), B) is None)
FZ = {"name": "fault", "kind": "fault", "x": -0.3, "y": 0.7, "yaw": 0.0, "state": "OK", "stack_count": 99, "stack_top_z": 0.5, "base_z": C["table_z"]}
check("fault zone (void, CL-39): always accepts, released from fault_drop_z, no stack box",
      sm.zone_accepts(C, FZ) == "" and sm.place_target(C, FZ, 0.006)[3] == C["fault_drop_z"] and sm.stack_box(C, FZ, B) is None)
# V4: zone sheet (base_z above the table)
SH = C["table_z"] + 0.005
x, y, yaw, z = sm.place_target(C, dict(pz, base_z=SH, stack_top_z=SH), 0.005)
check("V4 place in processing on a sheet: tcp z = base_z + thickness + gap", abs(z - (SH + 0.005 + C["place_gap"])) < 1e-12, str(z))
ZS = dict(Z, base_z=SH, stack_top_z=SH + 0.012)
bx = sm.stack_box(C, ZS, B)
check("V4 stack box on a sheet: stands on the table, top = stack top - margin (covers the sheet too)",
      bx is not None and abs(bx["size"][2] - (0.005 + 0.012 - C["stack_box_margin"])) < 1e-9
      and abs(bx["position"][2] - (C["table_z"] + bx["size"][2] / 2)) < 1e-9, str(bx))
check("V4 no stack box for an empty sheet, nor when its last board is being picked",
      sm.stack_box(C, dict(ZS, stack_top_z=SH), []) is None
      and sm.stack_box(C, dict(ZS, stack_top_z=SH + 0.006), B[:1], skip="s1") is None)
bs = [{"id": "b1", "x": -0.3, "y": 0.1, "variant": "red"}, {"id": "b2", "x": -0.31, "y": 0.11, "variant": "white_small"}]
check("board_at: nearest within tol, variant filter, None when far", sm.board_at(bs, -0.3, 0.1, 0.02)["id"] == "b1"
      and sm.board_at(bs, -0.3, 0.1, 0.02, "white_small")["id"] == "b2" and sm.board_at(bs, 0.0, 0.0, 0.02) is None)

print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
