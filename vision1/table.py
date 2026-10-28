"""Image boards -> base_link (metres) using the camera -> table calibration from feat/vision-transform.

Same math as hk_fr5_vision/calib.py `apply` (lens term + homography). config/calib.yaml is a copy of that branch's
verified file; re-copy it whenever the camera is recalibrated. Valid only while the camera does not move.
"""
import math
import os

import cv2
import numpy as np
import yaml

CALIB = os.path.join(os.path.dirname(__file__), "..", "config", "calib.yaml")
BOARD_TOP_Z = -0.055        # top of one sheet lying on the table, base_link (team geometry handoff)
INSIDE_TOL_PX = 2.0
SIZES_MM = {"VARa": (295, 210), "VARb": (150, 210), "VARc": (135, 210)}  # board X, board Y
SIZE_TOL_MM = 10            # ponytail: handoff says +-5 mm; 10 until checked on the real camera
# Zones by base_link y (m). ponytail: one split line; replace with polygons once processing area is placed.
ZONE_SPLIT_Y = 0.0          # y < split -> pick (right table in image), else drop (left table)


def load(path=CALIB):
    with open(path) as f:
        c = yaml.safe_load(f)
    if not c.get("calibrated"):
        raise ValueError(f"{path}: not calibrated")
    c["hull"] = cv2.convexHull(np.array([[t["u"], t["v"]] for t in c["tags"]], np.float32))
    return c


def to_base(c, px):
    """(n, 2) pixels -> (n, 2) base_link x / y in metres."""
    size = c["image_size"]
    centre = np.array(size, float) / 2.0
    d = np.asarray(px, float) - centre
    up = centre + d * (1.0 + c["lens_k"] * np.sum((d / float(max(size))) ** 2, axis=1, keepdims=True))
    q = np.asarray(c["homography"]) @ np.vstack([up.T, np.ones(len(up))])
    return (q[:2] / q[2]).T


def add_metric(c, board):
    """Add base_link fields to one detect() board. Variant is re-checked by measured size (white boards)."""
    xy = to_base(c, board["corners"])
    e = [np.linalg.norm(xy[(i + 1) % 4] - xy[i]) * 1000 for i in range(4)]
    sides = sorted([(e[0] + e[2]) / 2, (e[1] + e[3]) / 2])  # short, long (mm)
    if board["variant"] != "VARc":  # white: metric size beats pixel area
        board["variant"] = "VARa" if sides[0] > 180 else "VARb"
    want = sorted(SIZES_MM[board["variant"]])
    # board +X edge: long edge for VARa, short edge for VARb / VARc; yaw mod pi (rectangle is symmetric)
    i = int(np.argmax(e[:2])) if board["variant"] == "VARa" else int(np.argmin(e[:2]))
    dx, dy = xy[i + 1] - xy[i]
    yaw = (math.atan2(dy, dx) + math.pi / 2) % math.pi - math.pi / 2
    x, y = xy.mean(0)
    board.update({
        "x": round(float(x), 4), "y": round(float(y), 4), "z": BOARD_TOP_Z, "yaw": round(yaw, 4),
        "size_mm": [round(s, 1) for s in sides],
        "size_ok": all(abs(a - b) <= SIZE_TOL_MM for a, b in zip(sides, want)),
        "zone": "pick" if y < ZONE_SPLIT_Y else "drop",
        "inside_calibration": all(cv2.pointPolygonTest(c["hull"], tuple(map(float, p)), True) >= -INSIDE_TOL_PX
                                  for p in board["corners"]),
        "corners_xy": np.round(xy, 4).tolist(),
    })
    return board

# Destination templates (team geometry handoff, feat/fr5-demo-integration config/perception_geometry.yaml).
# Marker frame: origin = black-square centre, +X = corner 0 -> 1 (metric), +Y = Z x X. offset = marker centre -> board
# centre in that frame; yaw = board +X relative to marker +X.
DEST = {1: ("VARa", (-0.150, 0.0), math.pi / 2), 2: ("VARb", (-0.150, 0.0), math.pi / 2),
        16: ("VARc", (0.140, 0.0), -math.pi / 2)}
DEST_SUPPORT_Z = -0.055   # top of the template on the table
TAG_SIDE = 0.039
# ponytail: tag corners mapped on the table plane (-0.060), tags lie 5 mm higher -> ~2 mm parallax; fine for now.


# Zones for the main controller (vision1-changes.md C2 / C3 / C7 / C15): tag id -> name (unique), kind (drop_off /
# processing), offset (tag centre -> centre of the board area, marker frame, m), rotation (zone long side relative to
# marker +X), size [long, short] (m), top_z (sheet top, base_link, m).
# Lab values 2026-10-27 (measured, used in the first real-robot trial): brown cardboard sheets 5 mm on the table
# (-0.060) -> top_z -0.055; each tag's +X along its sheet's long side (rotation 0); board area 0.34 x 0.25 m centred
# 0.205 m along +X from the tag centre. Add zones the same way (new tag id -> unique name). Tags 1 / 2 / 16 are also in
# DEST above (pick_one.py templates): with the sheets on them, destinations[] is NOT valid for pick_one.py --place.
ZONES = {
    2: ("processing_1", "processing", (0.205, 0.0), 0.0, (0.34, 0.25), -0.055),
    1: ("drop_off_a", "drop_off", (0.205, 0.0), 0.0, (0.34, 0.25), -0.055),
    16: ("drop_off_b", "drop_off", (0.205, 0.0), 0.0, (0.34, 0.25), -0.055),
}


def tag_place(c, tag, offset, dyaw):
    """Tag + offset / rotation in its marker frame -> (x, y, yaw in [-pi, pi), tag side m), or None when the tag's
    metric size is wrong (bad detection)."""
    xy = to_base(c, tag["corners"])
    e = np.roll(xy, -1, axis=0) - xy
    side = np.linalg.norm(e, axis=1)
    if np.any(np.abs(side - TAG_SIDE) > 0.008):
        return None
    xa = e[0] / side[0] - e[2] / side[2]          # average the two edges pointing corner 0 -> 1
    xa /= np.linalg.norm(xa)
    ya = np.array([-xa[1], xa[0]])
    cx, cy = xy.mean(0) + offset[0] * xa + offset[1] * ya
    yaw = math.atan2(xa[1], xa[0]) + dyaw
    return round(float(cx), 4), round(float(cy), 4), round(math.atan2(math.sin(yaw), math.cos(yaw)), 4), side.mean()


def destination(c, tag):
    """One detected tag -> place pose for its variant, or None (unknown id, wrong metric size = bad detection)."""
    if tag["id"] not in DEST:
        return None
    variant, offset, dyaw = DEST[tag["id"]]
    p = tag_place(c, tag, offset, dyaw)
    return p and {"variant": variant, "tag_id": tag["id"], "x": p[0], "y": p[1], "yaw": p[2],
                  "support_z": DEST_SUPPORT_Z, "tag_side_mm": round(float(p[3]) * 1000, 1)}


def zone(c, tag):
    """One detected tag -> its zone (centre of the board area, yaw of the long side, not yet modulo pi), or None."""
    if tag["id"] not in ZONES:
        return None
    name, kind, offset, dyaw, size, top_z = ZONES[tag["id"]]
    p = tag_place(c, tag, offset, dyaw)
    return p and {"name": name, "kind": kind, "x": p[0], "y": p[1], "yaw": p[2], "size": list(size),
                  "top_z": top_z, "tag_id": tag["id"]}


PLAUSIBLE_MM = ((100, 230), (180, 320))  # short, long side of anything board-like (top board of unknown variant)


def pickable(c, boards, rejected, occluded=None):
    """Add metric fields; keep boards the robot can pick now, move the rest to `rejected` / `occluded`.
    Size-mismatched blobs are stacks, partly hidden boards or the robot arm: never pick them.
    Model boards: a top board whose size matches no variant but looks like a board -> variant 'UNKWN' (pickable,
    e.g. to clear it away). Occluded boards (model only) -> `occluded` with x / y of their visible part.
    Ids are kept from detect() so `covered_by` stays valid."""
    ok = []
    for b in boards:
        b.pop("area_px", None)
        if not b.get("top", True):
            x, y = to_base(c, [[b["u"], b["v"]]])[0]
            b.update({"x": round(float(x), 4), "y": round(float(y), 4), "zone": "pick" if y < ZONE_SPLIT_Y else "drop"})
            if occluded is not None:
                occluded.append(b)
            continue
        guess = b["variant"]
        if guess == "UNKWN":
            b["variant"] = b["model_class"]
        add_metric(c, b)
        (s0, s1), ((a0, a1), (c0, c1)) = b["size_mm"], PLAUSIBLE_MM
        if b["size_ok"]:
            ok.append(b)
        elif "model_class" in b and a0 <= s0 <= a1 and c0 <= s1 <= c1:
            b["variant"] = "UNKWN"
            ok.append(b)
        else:
            rejected.append({"reason": "size_mismatch", "variant_guess": guess, "u": b["u"], "v": b["v"],
                             "size_mm": b["size_mm"]})
    if not any("model_class" in b for b in boards):  # classic path: ids 0..n as before
        for i, b in enumerate(ok):
            b["id"] = i
    return ok


if __name__ == "__main__":
    import sys
    from detect import detect
    c = load()
    for path in sys.argv[1:]:
        for b in detect(cv2.imread(path)):
            add_metric(c, b)
            print(path, b["id"], b["variant"], b["x"], b["y"], round(math.degrees(b["yaw"]), 1), b["size_mm"],
                  b["size_ok"], b["zone"], b["inside_calibration"])
