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
# zones: centre = template board pose (same offsets as DEST for the placeholders), yaw = long side, mod pi
import math
tags = []
detect(cv2.imread(os.path.join(root, "composite_vara_dest.png")), None, tags)
zs = {z["tag_id"]: z for z in (table.zone(CALIB, t) for t in tags) if z}
ds = {d["tag_id"]: d for d in (table.destination(CALIB, t) for t in tags) if d}
assert sorted(zs) == [1, 2, 16], zs
for i, z in zs.items():
    assert (z["x"], z["y"]) == (ds[i]["x"], ds[i]["y"]), (z, ds[i])
    long_is_x = table.SIZES_MM[ds[i]["variant"]][0] > table.SIZES_MM[ds[i]["variant"]][1]
    d = (z["yaw"] - ds[i]["yaw"] - (0 if long_is_x else math.pi / 2)) % math.pi
    assert min(d, math.pi - d) < 1e-3, (z, ds[i])
    assert z["size"][0] >= z["size"][1]
print("ok")
