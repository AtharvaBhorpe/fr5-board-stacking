"""Self-check on recorded overhead photos: fails if detection or variant labelling breaks."""
import os, sys
import cv2
sys.path.insert(0, os.path.dirname(__file__))
from detect import detect
import table

# Pickable boards, left -> right in the pick area. 06 also has the robot arm (must be rejected, size 80 x 250 mm).
EXPECTED = {"01": ["VARb", "VARc", "VARa"], "02": ["VARb", "VARc", "VARa"], "03": ["VARb", "VARc", "VARa"],
            "06": ["VARa", "VARb", "VARc"], "07": ["VARa", "VARb", "VARc"], "08": ["VARa", "VARc", "VARb"]}
CALIB = table.load()
root = os.path.join(os.path.dirname(__file__), "..", "samples")
for name, want in EXPECTED.items():
    rejected = []
    boards = table.pickable(CALIB, detect(cv2.imread(os.path.join(root, name + ".jpg")), rejected), rejected)
    boards = [b for b in boards if b["zone"] == "pick"]  # left table: white templates, see README Limits
    got = [b["variant"] for b in boards]
    assert got == want, f"{name}.jpg: got {got}, want {want}"
    assert all(-90 <= b["yaw_deg"] < 90 for b in boards)
    assert all(abs(b["z"] + 0.055) < 1e-9 and b["zone"] == "pick" for b in boards)
    assert all(abs(a - w) <= 10 for b in boards for a, w in zip(b["size_mm"], sorted(table.SIZES_MM[b["variant"]])))
# 06: the robot arm must be rejected, not published as a board
rej = []
table.pickable(CALIB, detect(cv2.imread(os.path.join(root, "06.jpg")), rej), rej)
assert any(r["u"] < 1300 for r in rej), rej  # arm rejected (any reason), never published
# zones (ZONES table, real lab values 2026-10-27): centre = tag centre + offset along the tag's +X / +Y, yaw = tag +X
# + rotation (then folded mod pi by the node), size [long, short], top_z and name / kind exactly as configured
import math
import numpy as np
tags = []
detect(cv2.imread(os.path.join(root, "composite_vara_dest.png")), None, tags)
zs = {z["tag_id"]: z for z in (table.zone(CALIB, t) for t in tags) if z}
assert sorted(zs) == sorted(table.ZONES), zs
for t in tags:
    if t["id"] not in zs:
        continue
    z, (name, kind, off, dyaw, size, top_z) = zs[t["id"]], table.ZONES[t["id"]]
    xy = table.to_base(CALIB, t["corners"])
    e = np.roll(xy, -1, axis=0) - xy
    xa = e[0] / np.linalg.norm(e[0]) - e[2] / np.linalg.norm(e[2])
    xa /= np.linalg.norm(xa)
    ya = np.array([-xa[1], xa[0]])
    cx, cy = xy.mean(0) + off[0] * xa + off[1] * ya
    assert abs(z["x"] - cx) < 1e-3 and abs(z["y"] - cy) < 1e-3, (z, cx, cy)
    d = (z["yaw"] - math.atan2(xa[1], xa[0]) - dyaw) % (2 * math.pi)
    assert min(d, 2 * math.pi - d) < 1e-3, (z, math.atan2(xa[1], xa[0]))
    assert (z["name"], z["kind"], z["size"], z["top_z"]) == (name, kind, list(size), top_z), z
    assert z["size"][0] >= z["size"][1]
assert len({z[0] for z in table.ZONES.values()}) == len(table.ZONES)    # unique zone names
print("ok")
