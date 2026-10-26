"""Evaluate the detector on folders of recorded overhead images (no labels needed).

    pixi run eval ../logitech_dataset_1 ../logitech_dataset_2 samples     # report + eval_out/*.jpg contact sheets

Checks: size vs variant spec, frame-to-frame jitter on consecutive frames, rejected blobs (possible misses) by reason,
robustness to lighting / noise / blur / JPEG (detections must not change), speed.
"""
import glob
import json
import math
import os
import sys
import time
from collections import Counter, defaultdict
from multiprocessing import Pool

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import detect as D  # noqa: E402
import table  # noqa: E402

OUT = os.path.join(os.path.dirname(__file__), "..", "eval_out")
CALIB = table.load()
PERTURB = {
    "dark_x0.7": lambda im: cv2.convertScaleAbs(im, alpha=0.7),
    "bright_x1.3": lambda im: cv2.convertScaleAbs(im, alpha=1.3),
    "noise_s8": lambda im: np.clip(im + np.random.default_rng(0).normal(0, 8, im.shape), 0, 255).astype(np.uint8),
    "blur_5": lambda im: cv2.GaussianBlur(im, (5, 5), 0),
    "jpeg_q40": lambda im: cv2.imdecode(cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 40])[1], 1),
    "warm_tint": lambda im: np.clip(im * np.array([0.85, 1.0, 1.15]), 0, 255).astype(np.uint8),
}


def run(im):
    rej = []
    t = time.perf_counter()
    boards = D.detect(im, rej)
    for b in boards:
        table.add_metric(CALIB, b)
    return boards, rej, (time.perf_counter() - t) * 1000


def one(path):
    im = cv2.imread(path)
    boards, rej, ms = run(im)
    rec = {"path": path, "ms": ms, "boards": boards, "rejected": rej}
    if hash(path) % 3 == 0:  # perturbation subset (~1/3 of images)
        base = sorted((b["variant"], b["x"], b["y"]) for b in boards)
        rec["perturb"] = {}
        for name, f in PERTURB.items():
            got = sorted((b["variant"], b["x"], b["y"]) for b in run(f(im))[0])
            same = len(got) == len(base) and all(
                a[0] == b[0] and math.hypot(a[1] - b[1], a[2] - b[2]) < 0.005 for a, b in zip(base, got))
            rec["perturb"][name] = same
    return rec


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def sheet(items, name, n=12):
    """items: (path, boards, rejected) -> 3-column contact sheet, rejected blobs marked in red."""
    tiles = []
    for path, boards, rej in items[:n]:
        im = D.draw(cv2.imread(path), boards)
        for r in rej:
            cv2.putText(im, r["reason"], (r["u"] - 90, r["v"]), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 3)
        for b in boards:
            if not b.get("size_ok", True):
                cv2.putText(im, f'BAD SIZE {b["size_mm"]}', (int(b["u"]) - 120, int(b["v"]) + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
        cv2.putText(im, os.path.relpath(path, ".."), (20, 1060), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
        tiles.append(cv2.resize(im, (640, 360)))
    if not tiles:
        return
    tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % 3)
    cv2.imwrite(os.path.join(OUT, name), np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)]))


def main():
    paths = sorted(p for d in sys.argv[1:] for p in glob.glob(os.path.join(d, "**", "*.*"), recursive=True)
                   if p.lower().endswith((".png", ".jpg")) and ".overlay." not in p)
    os.makedirs(OUT, exist_ok=True)
    seq = [run(cv2.imread(p))[2] for p in paths[::max(1, len(paths) // 30)]]
    print(f"speed (single process, detect + metric): median {pct(seq, 50):.0f} ms, max {max(seq):.0f} ms")
    with Pool() as pool:
        recs = pool.map(one, paths, chunksize=4)

    boards = [b for r in recs for b in r["boards"]]
    print(f"images: {len(recs)}   with >=1 board: {sum(bool(r['boards']) for r in recs)}   boards: {len(boards)}")
    print(f"speed under 16-process load: median {pct([r['ms'] for r in recs], 50):.0f} ms, p95 {pct([r['ms'] for r in recs], 95):.0f} ms "
          f"per frame (one process)")

    print("\nSIZE vs spec (mm)       n   size_ok   |err| median  p95   max")
    for v, (sx, sy) in table.SIZES_MM.items():
        bs = [b for b in boards if b["variant"] == v]
        want = sorted((sx, sy))
        err = [max(abs(a - w) for a, w in zip(b["size_mm"], want)) for b in bs]
        ok = sum(b["size_ok"] for b in bs)
        print(f"  {v}  {want}  {len(bs):5d}  {ok / max(1, len(bs)):6.1%}   {pct(err, 50):9.1f} {pct(err, 95):5.1f} "
              f"{max(err, default=float('nan')):5.0f}")
    for top in sorted({os.path.relpath(r["path"]).split(os.sep)[1] if r["path"].startswith("..") else
                       os.path.relpath(r["path"]).split(os.sep)[0] for r in recs}):
        bs = [b for r in recs if top in r["path"] for b in r["boards"]]
        print(f"  per folder {top:20s} boards {len(bs):4d}  size_ok {sum(b['size_ok'] for b in bs) / max(1, len(bs)):6.1%}")
    print(f"  inside_calibration: {sum(b['inside_calibration'] for b in boards) / max(1, len(boards)):.1%} of boards")

    rej = Counter(x["reason"] for r in recs for x in r["rejected"])
    print(f"\nREJECTED blobs (possible misses): {sum(rej.values())} in "
          f"{sum(bool(r['rejected']) for r in recs)} images: {dict(rej)}")

    # Jitter: consecutive frames in one folder with the same variants -> nearest-centre match
    by_dir = defaultdict(list)
    for r in recs:
        by_dir[os.path.dirname(r["path"])].append(r)
    dxy, dyaw, pairs, changed = [], [], 0, 0
    for rs in by_dir.values():
        for a, b in zip(rs, rs[1:]):
            va, vb = sorted(x["variant"] for x in a["boards"]), sorted(x["variant"] for x in b["boards"])
            if va != vb:
                changed += 1
                continue
            pairs += 1
            for x in a["boards"]:
                y = min((y for y in b["boards"] if y["variant"] == x["variant"]),
                        key=lambda y: math.hypot(x["x"] - y["x"], x["y"] - y["y"]))
                d = math.hypot(x["x"] - y["x"], x["y"] - y["y"]) * 1000
                if d < 20:  # same board, not moved by hand
                    dxy.append(d)
                    dyaw.append(abs((math.degrees(x["yaw"] - y["yaw"]) + 90) % 180 - 90))
    print(f"\nJITTER over {pairs} consecutive same-content frame pairs ({len(dxy)} board matches): "
          f"xy median {pct(dxy, 50):.2f} mm, p95 {pct(dxy, 95):.2f} mm, max {max(dxy, default=0):.1f} mm; "
          f"yaw median {pct(dyaw, 50):.2f} deg, p95 {pct(dyaw, 95):.2f} deg.  {changed} pairs changed content")

    pr = [r for r in recs if "perturb" in r]
    print(f"\nROBUSTNESS on {len(pr)} images (same variants and centres within 5 mm as the original):")
    for name in PERTURB:
        print(f"  {name:12s} {sum(r['perturb'][name] for r in pr) / max(1, len(pr)):6.1%}")

    bad = [r for r in recs if any(not b["size_ok"] for b in r["boards"])]
    print(f"\nimages with a size_ok=false board: {len(bad)}")
    item = lambda r: (r["path"], r["boards"], r["rejected"])  # noqa: E731
    sheet([item(r) for r in bad][::max(1, len(bad) // 12)], "bad_size.jpg")
    for reason in rej:
        rr = [r for r in recs if any(x["reason"] == reason for x in r["rejected"])]
        sheet([item(r) for r in rr][::max(1, len(rr) // 12)], f"rejected_{reason}.jpg")
    ok = [r for r in recs if r["boards"] and not r["rejected"]]
    sheet([item(r) for r in ok][::max(1, len(ok) // 12)], "accepted.jpg")
    empty = [r for r in recs if not r["boards"] and not r["rejected"]]
    sheet([item(r) for r in empty][::max(1, len(empty) // 12)], "empty.jpg")
    pub = [b for b in boards if b["size_ok"]]
    print(f"\nPUBLISHED (size-verified) boards: {len(pub)} of {len(boards)} detections, in "
          f"{sum(any(b['size_ok'] for b in r['boards']) for r in recs)} images  {dict(Counter(b['variant'] for b in pub))}")
    rnd = [recs[i] for i in np.random.default_rng(1).permutation(len(recs))]
    for k in range(4):  # random frames, only what the node publishes: eyeball false positives / misses
        sheet([(r["path"], [b for b in r["boards"] if b["size_ok"]], []) for r in rnd[k * 12:(k + 1) * 12]],
              f"published_random_{k}.jpg")
    with open(os.path.join(OUT, "results.json"), "w") as f:
        json.dump(recs, f, default=float)
    print(f"sheets + results.json in {os.path.abspath(OUT)}")


if __name__ == "__main__":
    main()
