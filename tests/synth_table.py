"""Synthetic camera image of the table for tests / app development: ArUco tags (DICT_5X5_50) drawn at given
base_link x / y through the lab homography (2026-10-26, ideal lens). Pixel truth = to_px(x, y).

    pixi run python3 tests/synth_table.py out.png            # 8 calibration tags + verification tag 6 in the middle
"""
import sys

import cv2
import numpy as np

H_MM = np.array([[-0.0030423858564885085, -1.5204218890076686, 448.12361309443975],
                 [-1.3693471816224512, 0.017101743089264865, 1368.8769844406138],
                 [-1.2643839849879774e-06, 0.0002496750503579246, 1.0]])
H = np.diag([0.001, 0.001, 1.0]) @ H_MM          # pixel -> base_link metres
HI = np.linalg.inv(H)
SIZE = (1920, 1080)
CALIB = {14: (0.0478, 0.8366), 13: (0.0893, -0.8626), 0: (-0.6635, -0.5954), 15: (-0.6224, 0.6732),
         1: (-0.30, 0.30), 2: (-0.30, -0.30), 3: (0.07, 0.0), 4: (-0.64, 0.0)}   # V-16 layout
VERIFY_ID = 6


def to_px(x, y):
    q = HI @ np.array([x, y, 1.0]); return float(q[0] / q[2]), float(q[1] / q[2])


def render(tags, path, side=90):
    """tags: {id: (x, y)} in metres -> PNG with each tag centred on to_px(x, y) (axis-aligned, white border)."""
    img = np.full((SIZE[1], SIZE[0], 3), 200, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    for i, (x, y) in tags.items():
        m = cv2.aruco.generateImageMarker(d, int(i), side)
        m = cv2.copyMakeBorder(m, 12, 12, 12, 12, cv2.BORDER_CONSTANT, value=255)
        u, v = to_px(x, y); n = m.shape[0]
        # place with sub-pixel accuracy: affine warp so the marker centre lands exactly on (u, v)
        A = np.float32([[1, 0, u - (n - 1) / 2], [0, 1, v - (n - 1) / 2]])
        w = cv2.warpAffine(cv2.cvtColor(m, cv2.COLOR_GRAY2BGR), A, SIZE, flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0))
        mask = cv2.warpAffine(np.full((n, n), 255, np.uint8), A, SIZE, flags=cv2.INTER_LINEAR)
        img = np.where(mask[..., None] > 127, w, img)
    tmp = path + ".tmp.png"
    cv2.imwrite(tmp, img)
    import os
    os.replace(tmp, path)   # atomic: the app never reads a half-written image


if __name__ == "__main__":
    render({**CALIB, VERIFY_ID: (-0.40, 0.10)}, sys.argv[1] if len(sys.argv) > 1 else "table.png")
