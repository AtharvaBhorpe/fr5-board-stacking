"""Camera -> table calibration: lens correction + homography from tag centres (pixels) to base_link x / y (metres).
No ROS. Used by the vision_transform node, the calibration app and the tests (decision-log V-6..V-8, V-17).

Model: p_u = c + (p_d - c) * (1 + k * |p_d - c|^2 / n^2)   (c = image centre, n = max(image_size); one radial lens
term k, fitted from the same tags with 6+ tags, else 0), then (x, y) = homography(p_u).

    c = fit([{"id": 14, "u": 352.5, "v": 260.5, "x": 0.0478, "y": 0.8366}, ...], (1920, 1080), "C920")
    save(path, c)                       # atomic, keeps <file>.bak, writes through the install symlink
    c = load(path)                      # NotCalibrated / ValueError with a reason
    x, y, inside = apply(c, [u], [v])   # metres; inside = within the tags' outline (else extrapolated)
"""
import math
import os
import shutil
import time

import cv2
import numpy as np
import yaml

VERSION = 1
MIN_TAGS = 4
SPREAD_MIN = 0.1   # smallest / largest spread of the tag positions (0 = all in a line), pixels and world
LENS_MIN_TAGS = 6     # fit the lens term k only with this many tags (4 tags fit any k exactly -> meaningless)
K_RANGE = 1.5         # k search range +-K_RANGE (C920-like lenses: |k| ~ 0.1-0.3 in this normalisation)
INSIDE_TOL_PX = 2.0   # a point this close outside the tag outline still counts as inside (corner tags lie ON it)
SUSPECT_RMS_MM = 2.0   # fit rms above this -> look for one badly recorded tag (needs 6+ tags)
SUSPECT_RATIO = 0.5    # suspect = leaving it out at least halves the rms of the other tags ...
SUSPECT_CLEAR = 3.0    # ... and leaving out any other tag helps 3x less (else ambiguous -> None)


class NotCalibrated(ValueError):
    """calib.yaml exists but holds no calibration yet (placeholder)."""


def _points(points):
    """Validate the tag list -> ids, pixels (n, 2), world (n, 2)."""
    if len(points) < MIN_TAGS:
        raise ValueError(f"need at least {MIN_TAGS} tags, got {len(points)}")
    ids, px, w = [], [], []
    for p in points:
        try:
            i = int(p["id"])
            row = [float(p[k]) for k in ("u", "v", "x", "y")]
        except (KeyError, TypeError, ValueError):
            raise ValueError(f"tag {p.get('id', '?') if isinstance(p, dict) else p}: needs id, u, v, x, y as numbers")
        if not all(math.isfinite(r) for r in row):
            raise ValueError(f"tag {i}: u, v, x, y must be finite numbers")
        if i in ids:
            raise ValueError(f"tag {i} appears twice")
        ids.append(i); px.append(row[:2]); w.append(row[2:])
    px, w = np.array(px), np.array(w)
    for name, a in (("pixel", px), ("world", w)):
        s = np.linalg.svd(a - a.mean(0), compute_uv=False)
        if s[0] == 0 or s[1] / s[0] < SPREAD_MIN:
            raise ValueError(f"tags lie (almost) in a line in {name} coordinates: spread them over the table")
    return ids, px, w


def _homography(px, w):
    H, _ = cv2.findHomography(px.astype(np.float64), w.astype(np.float64), 0)   # 0 = least squares, all points
    if H is None or not np.all(np.isfinite(H)) or abs(np.linalg.det(H)) < 1e-12:
        raise ValueError("fit failed (degenerate tag layout)")
    return H


def _undistort(px, k, size):
    """Lens correction (see module doc). px (n, 2) distorted pixels -> corrected pixels."""
    c = np.array(size, float) / 2.0
    d = np.asarray(px, float) - c
    return c + d * (1.0 + k * np.sum((d / float(max(size))) ** 2, axis=1, keepdims=True))


def _sq_err(H, upx, w):
    x, y, _ = _map(H, upx[:, 0], upx[:, 1])
    return float(np.sum((x - w[:, 0]) ** 2 + (y - w[:, 1]) ** 2))


def _fit_lens(px, w, size):
    """Best k (grid + local refine) and its homography. Cost = squared world residual of the tags."""
    def cost(k):
        try:
            u = _undistort(px, k, size)
            H = _homography(u, w)
            return _sq_err(H, u, w), H
        except ValueError:
            return float("inf"), None
    grid = np.linspace(-K_RANGE, K_RANGE, 61)
    k = float(grid[int(np.argmin([cost(g)[0] for g in grid]))])
    step = grid[1] - grid[0]
    for _ in range(40):
        cand = [k - step, k, k + step]
        k = float(cand[int(np.argmin([cost(c)[0] for c in cand]))])
        step *= 0.6
    H = cost(k)[1]
    if H is None:
        raise ValueError("fit failed (degenerate tag layout)")
    return k, H


def _map(H, u, v):
    q = np.asarray(H) @ np.vstack([np.asarray(u, float), np.asarray(v, float), np.ones(len(u))])
    return q[0] / q[2], q[1] / q[2], q[2]


def fit(points, image_size, camera=""):
    """points: [{"id", "u", "v", "x", "y"}] (pixels, metres). Returns the calibration dict (see save).
    err_mm = residual of the fit per tag. drop_rms_mm = rms of the OTHER tags when this tag is left out (6+ tags,
    else None): a badly recorded tag is the one whose removal makes the rest fit. suspect = that tag id, or None
    when unclear. A hint ("re-record this tag"), not proof (rates: decision-log V-14 / V-18). Each leave-out fit
    refits the lens term too (7+ tags). Residual alone / leave-one-out prediction blamed the wrong tag far more often."""
    ids, px, w = _points(points)
    size = [int(image_size[0]), int(image_size[1])]
    H0 = _homography(px, w)
    rms0 = math.sqrt(_sq_err(H0, px, w) / len(ids)) * 1000   # plan A (homography only), for information
    k, H = _fit_lens(px, w, size) if len(ids) >= LENS_MIN_TAGS else (0.0, H0)
    if abs(k) > 0.98 * K_RANGE:
        raise ValueError(f"lens term k {k:+.2f} at the search limit: tag ids or positions wrong?")
    px = _undistort(px, k, size)
    x, y, s = _map(H, px[:, 0], px[:, 1])
    if not (np.all(s > 0) or np.all(s < 0)):
        raise ValueError("fit folds the image (tag ids or positions swapped?)")
    err = np.hypot(x - w[:, 0], y - w[:, 1]) * 1000
    rms = float(np.sqrt(np.mean(err ** 2)))
    drop = [None] * len(ids)
    if len(ids) > MIN_TAGS + 1:   # the others must still be over-determined (5+)
        raw_px = _points(points)[1]
        for i in range(len(ids)):
            m = np.arange(len(ids)) != i
            try:   # refit lens k too (a fixed k would have bent towards the bad tag); same k if too few left
                km, Hm = _fit_lens(raw_px[m], w[m], size) if m.sum() >= LENS_MIN_TAGS else (k, _homography(px[m], w[m]))
                um = _undistort(raw_px[m], km, size)
                xk, yk, _ = _map(Hm, um[:, 0], um[:, 1])
                drop[i] = float(np.sqrt(np.mean((np.hypot(xk - w[m, 0], yk - w[m, 1]) * 1000) ** 2)))
            except ValueError:
                drop[i] = float("inf")
    suspect = None
    if drop[0] is not None and rms > SUSPECT_RMS_MM:
        i = int(np.argmin(drop))
        if drop[i] < SUSPECT_RATIO * rms and sorted(drop)[1] > SUSPECT_CLEAR * drop[i]:
            suspect = ids[i]
    raw = _points(points)[1]   # store the measured (distorted) pixels
    tags = [{"id": ids[i], "u": float(raw[i, 0]), "v": float(raw[i, 1]), "x": float(w[i, 0]), "y": float(w[i, 1]),
             "err_mm": round(float(err[i]), 2), "drop_rms_mm": None if drop[i] is None else round(drop[i], 2)}
            for i in range(len(ids))]
    return {"version": VERSION, "calibrated": True, "date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "camera": str(camera), "image_size": size, "lens_k": k, "homography": H.tolist(),
            "rms_mm": round(rms, 2), "rms_homography_only_mm": round(rms0, 2), "suspect": suspect, "tags": tags}


def apply(calib, u, v):
    """Pixels -> (x list, y list, inside list), metres. inside = within the convex outline of the tags (+ INSIDE_TOL_PX)."""
    if len(u) != len(v):
        raise ValueError(f"u and v lengths differ ({len(u)} vs {len(v)})")
    if not len(u):
        return [], [], []
    up = _undistort(np.column_stack([np.asarray(u, float), np.asarray(v, float)]), calib["lens_k"], calib["image_size"])
    x, y, _ = _map(calib["homography"], up[:, 0], up[:, 1])
    hull = cv2.convexHull(np.array([[t["u"], t["v"]] for t in calib["tags"]], np.float32))
    inside = [cv2.pointPolygonTest(hull, (float(a), float(b)), True) >= -INSIDE_TOL_PX for a, b in zip(u, v)]
    return x.tolist(), y.tolist(), inside


def lens_effect_mm(calib):
    """How much the lens term matters: largest difference (mm, on the table) between this calibration and a
    homography-only fit of the same tags (plan A), over the tags' outline corners + edge midpoints (where distortion
    is largest). ~0 = the lens is not distorted (k changes nothing); several mm = the lens term is doing real work."""
    raw = np.array([[t["u"], t["v"]] for t in calib["tags"]], float)
    w = np.array([[t["x"], t["y"]] for t in calib["tags"]], float)
    H0 = _homography(raw, w)
    hull = cv2.convexHull(raw.astype(np.float32)).reshape(-1, 2).astype(float)
    pts = np.vstack([hull, (hull + np.roll(hull, 1, axis=0)) / 2])
    a = _undistort(pts, calib["lens_k"], calib["image_size"])
    xa, ya, _ = _map(calib["homography"], a[:, 0], a[:, 1])
    xb, yb, _ = _map(H0, pts[:, 0], pts[:, 1])
    return float(np.max(np.hypot(xa - xb, ya - yb)) * 1000)


def load(path):
    """Read + validate calib.yaml. NotCalibrated for the placeholder, ValueError (with reason) for a bad file."""
    try:
        with open(path) as f:
            c = yaml.safe_load(f)
    except OSError as e:
        raise ValueError(f"cannot read {path}: {e.strerror}")
    except yaml.YAMLError as e:
        raise ValueError(f"{path} is not valid yaml: {e}")
    if not isinstance(c, dict):
        raise ValueError(f"{path}: not a calibration file")
    if c.get("version") != VERSION:
        raise ValueError(f"{path}: version {c.get('version')!r}, expected {VERSION}")
    if not c.get("calibrated"):
        raise NotCalibrated(f"{path}: not calibrated yet (run the calibration app)")
    try:
        H = np.array(c["homography"], float)
        size = [int(s) for s in c["image_size"]]
        k = float(c["lens_k"])
        ids, _, _ = _points(c["tags"])
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f"{path}: bad calibration ({e})")
    if H.shape != (3, 3) or not np.all(np.isfinite(H)) or len(size) != 2 or min(size) <= 0 or not math.isfinite(k):
        raise ValueError(f"{path}: bad homography, image_size or lens_k")
    return c


HEADER = """# Camera -> table calibration (decision-log V-7). Written by the calibration app; do not edit by hand.
# lens_k: pixel correction p_u = c + (p_d - c)(1 + k |p_d - c|^2 / n^2), c = image centre, n = max(image_size).
# homography: corrected pixel (u, v, 1) -> base_link (x, y, w), x / w and y / w in metres. Valid only for image_size,
# this camera pose and this focus (do not move the camera). rms_homography_only_mm = same fit without lens_k (info). tags: centre pixel, recorded tcp x / y (m), fit residual err_mm,
# drop_rms_mm = rms of the other tags without this one (6+ tags); suspect = tag that looks badly recorded.
"""


def save(path, calib):
    """Atomic write (temp file + rename), previous file kept as <file>.bak. Follows symlinks, so saving to the
    installed (symlink-install) path updates the source file that gets committed."""
    real = os.path.realpath(path)
    tmp = real + ".tmp"
    with open(tmp, "w") as f:
        f.write(HEADER)
        yaml.safe_dump(calib, f, sort_keys=False, default_flow_style=None, width=120)
    if os.path.exists(real):
        shutil.copy2(real, real + ".bak")
    os.replace(tmp, real)
    return real
