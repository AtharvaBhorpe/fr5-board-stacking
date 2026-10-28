"""Overhead camera -> boards in image pixels. Pure OpenCV, no ROS.

    pixi run python vision1/detect.py samples/01.jpg      # prints JSON, writes samples/01.jpg.overlay.jpg
"""
import json
import math
import sys

import cv2
import numpy as np

import seg

# --- Lab knobs (pixel values are for the 1920x1080 overhead image) -------------------------------
# Where to look: pick area (right table), normalized [x, y] in image, perimeter order.
ROI = [[0.57, 0.15], [0.985, 0.15], [0.985, 0.97], [0.57, 0.97]]  # A/B on 755 frames: +34 boards, 0 lost
# Left table (drop-off / processing zones). Whole table = both (vision1-changes.md C1); the robot rail between them
# holds no boards. The model runs once per ROI crop: same scale as its training crop (the right one).
ROI_DROP = [[0.02, 0.15], [0.485, 0.15], [0.485, 0.97], [0.02, 0.97]]
ROIS = [ROI_DROP, ROI]
WHITE = ((0, 0, 100), (179, 80, 255))         # HSV lo, hi. Picked by A/B on 190 lab frames (eval_dataset.py)
RED = [((0, 110, 80), (12, 255, 255)), ((165, 110, 80), (179, 255, 255))]  # V<80 merges red with tape / shadows
MIN_AREA = 12000                               # px^2, smaller blobs are ignored
VARA_MIN_AREA = 38000                          # white board bigger than this -> VARa, else VARb
MAX_AREA = 90000                               # bigger -> merged boards, rejected
MIN_RECTANGULARITY = 0.88                      # blob area / fitted rectangle area
HULL_RECT = False  # True admits boards with a hand on them: keep False
RED_CLOSE = 9      # red paint is streaky with white margins; bigger close fills it
# ponytail: area thresholds assume the camera height stays fixed; re-tune (or use mm via calibration, MVP 3) if it moves.

# Board +X edge per variant (README "Board coordinates"): VARa = long edge, VARb / VARc = short edge.
X_IS_LONG_EDGE = {"VARa": True, "VARb": False, "VARc": False}
_P = cv2.aruco.DetectorParameters()
_P.adaptiveThreshWinSizeMax, _P.adaptiveThreshWinSizeStep = 53, 4      # small (~35 px) tags: A/B on 77 frames
_P.minMarkerPerimeterRate, _P.polygonalApproxAccuracyRate = 0.01, 0.05
ARUCO = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100), _P)


def _markers(gray):
    """{id: 4x2 corners}. Second pass on a sharpened copy: blurred ~35 px tags (tag 1 on the yellow template) decode
    there (A/B on 84 frames: tag 1 found 68 -> 76). errorCorrectionRate up would find more but decodes ghost ids."""
    out = {}
    sharp = cv2.addWeighted(gray, 1.8, cv2.GaussianBlur(gray, (0, 0), 2), -0.8, 0)
    for img in (gray, sharp):
        corners, ids, _ = ARUCO.detectMarkers(img)
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                out.setdefault(int(i), c.reshape(4, 2))
    return out


def _mask(hsv, ranges, roi, close=9):
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        m |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    m &= roi
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((close, close), np.uint8))


def _yaw_deg(box, x_is_long):
    """Angle of board +X in the image (u right, v down), degrees in [-90, 90): a rectangle is 180-deg symmetric."""
    e1 = (box[1] - box[0] + box[2] - box[3]) / 2  # average opposite edges (corners are a general quadrilateral)
    e2 = (box[2] - box[1] + box[3] - box[0]) / 2
    long_e, short_e = (e1, e2) if np.linalg.norm(e1) >= np.linalg.norm(e2) else (e2, e1)
    e = long_e if x_is_long else short_e
    return (math.degrees(math.atan2(e[1], e[0])) + 90) % 180 - 90


def _quad(contour):
    """4 real corners of the board outline (perimeter order), or None. Perspective makes it a general
    quadrilateral, so minAreaRect would overestimate the size by 10-20 mm."""
    hull = cv2.convexHull(contour)
    peri = cv2.arcLength(hull, True)
    for eps in (0.01, 0.02, 0.03, 0.04, 0.05):
        q = cv2.approxPolyDP(hull, eps * peri, True)
        if len(q) == 4:
            return q.reshape(4, 2).astype(np.float32)
    return None


def roi_px(w, h, roi=ROI):
    """ROI bounding box in pixels (x0, y0, x1, y1). Default: the crop the model was trained on."""
    return (round(roi[0][0] * (w - 1)), round(roi[0][1] * (h - 1)), round(roi[1][0] * (w - 1)), round(roi[2][1] * (h - 1)))


UNKWN_CONF = 0.6   # model class confidence below this -> variant "UNKWN" (any unclear board)
TOUCH_PX = 9       # px grown around a board mask before testing overlap
COVER_PX = 800     # px of board j inside occluded board i's hull -> j covers i


def _from_model(bgr, rois, tag_centres, reject):
    """Model masks -> boards. Full rectangle = on top (pickable). Partial = occluded: 'top': False, 'covered_by'."""
    out, masks = [], []
    for inst in (i for r in rois for i in seg.segment(bgr, r)):
        cs, _ = cv2.findContours(inst["mask"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cs:
            continue
        c = max(cs, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < MIN_AREA / 4:
            continue
        rect = cv2.minAreaRect(c)
        box = _quad(c)
        if box is not None and any(cv2.pointPolygonTest(box, tuple(map(float, t)), False) >= 0 for t in tag_centres):
            reject("has_tag", c, area)
            continue
        top = (box is not None and MIN_AREA <= area <= MAX_AREA
               and area / (rect[1][0] * rect[1][1]) >= MIN_RECTANGULARITY)
        variant = inst["cls"] if inst["conf"] >= UNKWN_CONF else "UNKWN"
        if box is None:
            box = cv2.boxPoints(rect).astype(np.float32)
        m = cv2.moments(c)
        b = {"variant": variant, "model_class": inst["cls"], "conf": round(inst["conf"], 3), "top": bool(top),
             "u": round(m["m10"] / m["m00"], 1), "v": round(m["m01"] / m["m00"], 1),
             "yaw_deg": round(_yaw_deg(box, X_IS_LONG_EDGE.get(inst["cls"], True)), 1),
             "corners": np.round(box, 1).tolist(), "area_px": int(area)}
        if top:  # centre of the 4 corners, not of the mask (same as the classic path)
            b["u"], b["v"] = round(float(box[:, 0].mean()), 1), round(float(box[:, 1].mean()), 1)
        hull = np.zeros_like(inst["mask"])
        cv2.fillPoly(hull, [cv2.convexHull(c)], 255)  # where the occluded board's missing corner / edge is
        out.append(b)
        masks.append((cv2.dilate(inst["mask"], np.ones((TOUCH_PX, TOUCH_PX), np.uint8)), hull))
    order = sorted(range(len(out)), key=lambda i: out[i]["u"])
    out, masks = [out[i] for i in order], [masks[i] for i in order]
    for i, b in enumerate(out):
        b["id"] = i
        if not b["top"]:  # j covers i: j lies inside i's visible outline (hull); thin slivers have no useful hull,
            others = [j for j in range(len(out)) if j != i]  # then any touching board (worst case: one extra move)
            b["covered_by"] = ([j for j in others if cv2.countNonZero(masks[j][0] & masks[i][1]) >= COVER_PX]
                               or [j for j in others if cv2.countNonZero(masks[j][0] & masks[i][0])])
    return out


def detect(bgr, rejected=None, tags=None, use_model=False):
    """Return [{'id','variant','u','v','yaw_deg','corners'}], sorted left -> right, id = index.
    rejected: optional list, gets {'reason','colour','u','v','area_px'} for every board-sized blob that was dropped.
    tags: optional list, gets {'id','corners'} for every DICT_5X5_100 marker in the whole image (destinations).
    With use_model (needs models/boards-seg.onnx): learned masks, boards also get 'top', 'conf', and occluded ones
    'covered_by' (ids of touching boards); variant may be 'UNKWN'. Without it: classic colour masks (all 'top')."""
    colour = "model"

    def reject(reason, c, area):
        if rejected is not None:
            (u, v), _, _ = cv2.minAreaRect(c)
            rejected.append({"reason": reason, "colour": colour, "u": round(u), "v": round(v), "area_px": int(area)})

    h, w = bgr.shape[:2]
    roi = np.zeros((h, w), np.uint8)
    cv2.fillPoly(roi, [np.int32(np.array(r) * [w - 1, h - 1]) for r in ROIS], 255)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    found = _markers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    tag_centres = [c.mean(0) for c in found.values()]
    if tags is not None:
        tags += [{"id": i, "corners": c.tolist()} for i, c in found.items()]

    if use_model and seg.available():
        return _from_model(bgr, [roi_px(w, h, r) for r in ROIS], tag_centres, reject)
    boards = []
    for colour, ranges, close in (("white", [WHITE], 9), ("red", RED, RED_CLOSE)):
        contours, _ = cv2.findContours(_mask(hsv, ranges, roi, close), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA:
                continue
            if area > MAX_AREA:
                reject("too_big", c, area)  # merged boards
                continue
            rect = cv2.minAreaRect(c)
            if (cv2.contourArea(cv2.convexHull(c)) if HULL_RECT else area) / (rect[1][0] * rect[1][1]) < MIN_RECTANGULARITY:
                reject("not_rectangular", c, area)  # overlapping / partly hidden boards
                continue
            box = _quad(c)
            if box is None:
                reject("no_4_corners", c, area)
                continue
            if any(cv2.pointPolygonTest(box, tuple(map(float, t)), False) >= 0 for t in tag_centres):
                reject("has_tag", c, area)  # destination template, not a board
                continue
            variant = "VARc" if colour == "red" else ("VARa" if area >= VARA_MIN_AREA else "VARb")
            boards.append({"variant": variant, "u": round(float(box[:, 0].mean()), 1), "v": round(float(box[:, 1].mean()), 1),
                           "yaw_deg": round(_yaw_deg(box, X_IS_LONG_EDGE[variant]), 1),
                           "corners": np.round(box, 1).tolist(), "area_px": int(area)})
    boards.sort(key=lambda b: b["u"])
    for i, b in enumerate(boards):
        b["id"] = i
    return boards


def draw(bgr, boards, font=1.0):
    out = bgr.copy()
    h, w = out.shape[:2]
    cv2.polylines(out, [np.int32(np.array(r) * [w - 1, h - 1]) for r in ROIS], True, (255, 255, 0), 2)
    for b in boards:
        col = (0, 255, 0) if b.get("top", True) else (0, 165, 255)  # orange = occluded
        cv2.polylines(out, [np.int32(b["corners"])], True, col, 3)
        cv2.putText(out, f'{b["id"]} {b["variant"]} {b["yaw_deg"]:.0f}deg' + ("" if b.get("top", True) else " under"), (int(b["u"]) - 80, int(b["v"])),
                    cv2.FONT_HERSHEY_SIMPLEX, font, (0, 200, 0), max(3, round(3 * font)))
    return out


if __name__ == "__main__":
    for path in sys.argv[1:]:
        img = cv2.imread(path)
        boards = detect(img)
        cv2.imwrite(path + ".overlay.jpg", draw(img, boards))
        print(path, json.dumps([{k: b[k] for k in ("id", "variant", "u", "v", "yaw_deg", "area_px")} for b in boards]))
