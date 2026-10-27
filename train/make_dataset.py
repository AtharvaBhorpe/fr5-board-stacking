"""Auto-labelled YOLO-seg dataset from the recorded overhead frames, no human labels.

    pixi run -e train python train/make_dataset.py ../logitech_dataset_1 ../logitech_dataset_2

Source: eval_out/results.json (pixi run eval ...). Per real frame (cropped to the detector ROI):
  - size-verified classic detections -> labels (full, unoccluded boards)
  - rejected blobs inside the table (merged / overlapping boards the classic detector could not split) are painted
    over with table colour; rejected blobs touching the ROI edge (robot arm, hands) stay as unlabelled negatives
  - 1..5 board "stickers" cut from other frames are pasted on top, overlapping each other and the real boards;
    labels = the visible part of every board (pasted boards hide what is under them)
ponytail: synthetic overlaps have no real sheet-edge shadows (a 3 px soft shadow is drawn); add real labelled
stacks if the model misses real ones.
"""
import json
import os
import random
import sys
from multiprocessing import Pool

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "vision1"))
import detect as D  # noqa: E402

OUT = os.path.join(HERE, "..", "train_data")
CLASSES = ["VARa", "VARb", "VARc"]
W, H = 1920, 1080
X0, Y0, X1, Y1 = D.roi_px(W, H)
COPIES = 2            # synthetic variants per real frame
MIN_VISIBLE = 0.25    # drop instances with less than this fraction of the board visible (also too hard for the robot)


def cls_px(b):
    return "VARc" if b["variant"] == "VARc" else ("VARa" if b["area_px"] >= D.VARA_MIN_AREA else "VARb")


def crop(im):
    return im[Y0:Y1, X0:X1]


def quad_mask(shape, corners, dx=X0, dy=Y0):
    m = np.zeros(shape[:2], np.uint8)
    cv2.fillPoly(m, [np.int32(np.array(corners) - [dx, dy])], 255)
    return m


def stickers(recs, n_per_class=150, seed=0):
    """Cut size-verified boards (eroded 2 px so no table pixels) -> [(class, bgr, mask)]."""
    rng = random.Random(seed)
    pool = {c: [] for c in CLASSES}
    for r in recs:
        for b in r["boards"]:
            if b["size_ok"]:  # stickers: dataset_1 boards with verified size only
                pool[b["variant"]].append((r["path"], b["corners"]))
    out = []
    for c, items in pool.items():
        for path, corners in rng.sample(items, min(n_per_class, len(items))):
            im = cv2.imread(path)
            c4 = np.array(corners, np.float32)
            x, y, w, h = cv2.boundingRect(c4)
            m = quad_mask(im.shape, c4, 0, 0)[y:y + h, x:x + w]
            m = cv2.erode(m, np.ones((5, 5), np.uint8))
            out.append((c, im[y:y + h, x:x + w].copy(), m))
    return out


def paste(img, masks, st, rng):
    """Paste one sticker at a random pose inside the crop. masks: list of [class, visible mask] (updated in place)."""
    c, bgr, m = st
    h, w = m.shape
    d = int(np.hypot(h, w)) + 4
    big, bm = np.zeros((d, d, 3), np.uint8), np.zeros((d, d), np.uint8)
    oy, ox = (d - h) // 2, (d - w) // 2
    big[oy:oy + h, ox:ox + w], bm[oy:oy + h, ox:ox + w] = bgr, m
    R = cv2.getRotationMatrix2D((d / 2, d / 2), rng.uniform(0, 360), 1.0)
    cx, cy = rng.uniform(0.1, 0.9) * img.shape[1], rng.uniform(0.1, 0.9) * img.shape[0]
    R[:, 2] += [cx - d / 2, cy - d / 2]
    size = (img.shape[1], img.shape[0])
    sb = cv2.warpAffine(big, R, size, flags=cv2.INTER_LINEAR)
    sm = cv2.warpAffine(bm, R, size, flags=cv2.INTER_NEAREST)
    if sm.sum() < 0.6 * m.sum():  # mostly outside the crop
        return
    sb = np.clip(sb.astype(np.float32) * rng.uniform(0.85, 1.15), 0, 255).astype(np.uint8)
    shadow = cv2.GaussianBlur(cv2.dilate(sm, np.ones((7, 7), np.uint8)), (9, 9), 0).astype(np.float32) / 255
    img[:] = (img * (1 - 0.35 * shadow[..., None] * (sm[..., None] == 0))).astype(np.uint8)
    img[sm > 0] = sb[sm > 0]
    for item in masks:
        item[1][sm > 0] = 0
    masks.append([c, sm, int(sm.sum() / 255)])


def polygons(masks, shape):
    lines = []
    for c, m, full in masks:
        vis = int(m.sum() / 255)
        if vis < MIN_VISIBLE * full or vis < 1500:
            continue
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        p = max(cs, key=cv2.contourArea)
        p = cv2.approxPolyDP(p, 1.5, True).reshape(-1, 2)
        if len(p) < 3:
            continue
        xy = (p / [shape[1], shape[0]]).flatten()
        lines.append(f"{CLASSES.index(c)} " + " ".join(f"{v:.5f}" for v in xy))
    return lines


def one(args):
    k, rec, split = args
    rng = random.Random(k)
    full = cv2.imread(rec["path"])
    base = crop(full).copy()
    table = np.median(base.reshape(-1, 3), 0)
    for r in rec["rejected"]:  # paint interior unknown blobs (unsplit stacks); edge blobs = arm / hands, keep
        blob = blobs_at(full, r)
        if blob is None:
            continue
        ys, xs = np.nonzero(blob)
        if xs.min() <= 2 or ys.min() <= 2 or xs.max() >= blob.shape[1] - 3 or ys.max() >= blob.shape[0] - 3:
            continue
        blob = cv2.dilate(blob, np.ones((15, 15), np.uint8))
        base[blob > 0] = np.clip(table + np.random.default_rng(k).normal(0, 6, (int((blob > 0).sum()), 3)), 0, 255)
    # every clean classic detection is a label, size_ok or not: dataset_2 was recorded with another camera pose,
    # the calibration does not fit it, so metric size fails there. White VARa / VARb by pixel area instead.
    real = [[cls_px(b), quad_mask(base.shape, b["corners"]), 0] for b in rec["boards"]]
    for item in real:
        item[2] = int(item[1].sum() / 255)
    stem = f"{k:05d}"
    for j in range(COPIES + 1):  # j = 0: real frame only
        img, masks = base.copy(), [[c, m.copy(), f] for c, m, f in real]
        if j:
            for _ in range(rng.randint(1, 5)):
                paste(img, masks, rng.choice(STICKERS), rng)
        name = f"{stem}_{j}"
        cv2.imwrite(os.path.join(OUT, "images", split, name + ".jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(os.path.join(OUT, "labels", split, name + ".txt"), "w") as f:
            f.write("\n".join(polygons(masks, img.shape)))


def blobs_at(full, r):
    """Mask (crop coords) of the white / red blob the classic detector rejected at (u, v)."""
    hsv = cv2.cvtColor(crop(full), cv2.COLOR_BGR2HSV)
    ranges = [D.WHITE] if r.get("colour") == "white" else D.RED
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        m |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab = cv2.connectedComponents(m)
    u, v = int(r["u"]) - X0, int(r["v"]) - Y0
    if not (0 <= v < lab.shape[0] and 0 <= u < lab.shape[1]):
        return None
    i = lab[v, u]
    if i == 0:  # centre of a non-convex blob may be outside it: nearest blob pixel
        ys, xs = np.nonzero(lab)
        if not len(xs):
            return None
        j = np.argmin((xs - u) ** 2 + (ys - v) ** 2)
        i = lab[ys[j], xs[j]]
    return np.uint8(lab == i) * 255


def init(st):
    global STICKERS
    STICKERS = st


if __name__ == "__main__":
    recs = json.load(open(os.path.join(HERE, "..", "eval_out", "results.json")))
    for s in ("train", "val"):
        os.makedirs(os.path.join(OUT, "images", s), exist_ok=True)
        os.makedirs(os.path.join(OUT, "labels", s), exist_ok=True)
    st = stickers(recs)
    # val = whole folders' tail (consecutive frames are near-duplicates: random split would leak)
    jobs = [(k, r, "val" if k % 10 == 0 else "train") for k, r in enumerate(recs)]
    jobs = [(k, r, "val" if (k // 25) % 8 == 0 else "train") for k, r, _ in jobs]
    with Pool(initializer=init, initargs=(st,)) as p:
        p.map(one, jobs, chunksize=4)
    with open(os.path.join(OUT, "data.yaml"), "w") as f:
        f.write(f"path: {os.path.abspath(OUT)}\ntrain: images/train\nval: images/val\nnames: {CLASSES}\n")
    print(f"{len(st)} stickers, {len(jobs) * (COPIES + 1)} images in {os.path.abspath(OUT)}")
