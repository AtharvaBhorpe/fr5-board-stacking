"""Overhead camera -> boards in image pixels. Pure OpenCV, no ROS.

    pixi run detect samples/01.jpg      # prints JSON, writes samples/01.jpg.overlay.jpg
"""
import json
import math
import sys

import cv2
import numpy as np

# --- Lab knobs (pixel values are for the 1920x1080 overhead image) -------------------------------
# Where to look: pick area (right table), normalized [x, y] in image, perimeter order.
ROI = [[0.585, 0.17], [0.96, 0.17], [0.96, 0.93], [0.585, 0.93]]
WHITE = ((0, 0, 110), (179, 70, 255))          # HSV lo, hi
RED = [((0, 90, 80), (12, 255, 255)), ((165, 90, 80), (179, 255, 255))]
MIN_AREA = 12000                               # px^2, smaller blobs are ignored
VARA_MIN_AREA = 38000                          # white board bigger than this -> VARa, else VARb
MAX_AREA = 90000                               # bigger -> merged boards, rejected
MIN_RECTANGULARITY = 0.88                      # blob area / fitted rectangle area
# ponytail: area thresholds assume the camera height stays fixed; re-tune (or use mm via calibration, MVP 3) if it moves.

# Board +X edge per variant (README "Board coordinates"): VARa = long edge, VARb / VARc = short edge.
X_IS_LONG_EDGE = {"VARa": True, "VARb": False, "VARc": False}
ARUCO = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100),
                                cv2.aruco.DetectorParameters())


def _mask(hsv, ranges, roi):
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        m |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    m &= roi
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))


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


def detect(bgr):
    """Return [{'id','variant','u','v','yaw_deg','corners'}], sorted left -> right, id = index."""
    h, w = bgr.shape[:2]
    roi = np.zeros((h, w), np.uint8)
    cv2.fillPoly(roi, [np.int32(np.array(ROI) * [w - 1, h - 1])], 255)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    tags = ARUCO.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))[0]
    tag_centres = [c.reshape(4, 2).mean(0) for c in tags]

    boards = []
    for colour, ranges in (("white", [WHITE]), ("red", RED)):
        contours, _ = cv2.findContours(_mask(hsv, ranges, roi), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if not MIN_AREA <= area <= MAX_AREA:
                continue
            rect = cv2.minAreaRect(c)
            if area / (rect[1][0] * rect[1][1]) < MIN_RECTANGULARITY:
                continue  # overlapping / partly hidden boards
            box = _quad(c)
            if box is None:
                continue
            if any(cv2.pointPolygonTest(box, tuple(map(float, t)), False) >= 0 for t in tag_centres):
                continue  # destination template, not a board
            variant = "VARc" if colour == "red" else ("VARa" if area >= VARA_MIN_AREA else "VARb")
            boards.append({"variant": variant, "u": round(float(box[:, 0].mean()), 1), "v": round(float(box[:, 1].mean()), 1),
                           "yaw_deg": round(_yaw_deg(box, X_IS_LONG_EDGE[variant]), 1),
                           "corners": np.round(box, 1).tolist(), "area_px": int(area)})
    boards.sort(key=lambda b: b["u"])
    for i, b in enumerate(boards):
        b["id"] = i
    return boards


def draw(bgr, boards):
    out = bgr.copy()
    h, w = out.shape[:2]
    cv2.polylines(out, [np.int32(np.array(ROI) * [w - 1, h - 1])], True, (255, 255, 0), 2)
    for b in boards:
        cv2.polylines(out, [np.int32(b["corners"])], True, (0, 255, 0), 3)
        cv2.putText(out, f'{b["id"]} {b["variant"]} {b["yaw_deg"]:.0f}deg', (int(b["u"]) - 80, int(b["v"])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 0), 3)
    return out


if __name__ == "__main__":
    for path in sys.argv[1:]:
        img = cv2.imread(path)
        boards = detect(img)
        cv2.imwrite(path + ".overlay.jpg", draw(img, boards))
        print(path, json.dumps([{k: b[k] for k in ("id", "variant", "u", "v", "yaw_deg", "area_px")} for b in boards]))
