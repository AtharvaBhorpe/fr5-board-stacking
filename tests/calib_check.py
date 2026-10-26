"""Offline checks of hk_fr5_vision.calib (V3, V3b): fit / apply / inside / lens term / suspect / rejections / save + load.

    pixi run python3 tests/calib_check.py

"True" camera = the lab homography of 2026-10-26 (4 tags, scratch pix2table calib.json, mm -> m)."""
import math
import os
import sys
import tempfile

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from hk_fr5_vision import calib

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

def raises(f, exc=ValueError, text=""):
    try:
        f()
    except exc as e:
        return text in str(e), str(e)
    return False, "no exception"

# lab homography, pixel -> world mm  ->  pixel -> world m
H_MM = np.array([[-0.0030423858564885085, -1.5204218890076686, 448.12361309443975],
                 [-1.3693471816224512, 0.017101743089264865, 1368.8769844406138],
                 [-1.2643839849879774e-06, 0.0002496750503579246, 1.0]])
H_TRUE = np.diag([0.001, 0.001, 1.0]) @ H_MM
H_INV = np.linalg.inv(H_TRUE)
SIZE = (1920, 1080)

def to_px(x, y):
    q = H_INV @ np.array([x, y, 1.0]); return q[0] / q[2], q[1] / q[2]
def to_w(u, v):
    q = H_TRUE @ np.array([u, v, 1.0]); return q[0] / q[2], q[1] / q[2]

WORLD = {14: (0.0478, 0.8366), 13: (0.0893, -0.8626), 0: (-0.6635, -0.5954), 15: (-0.6224, 0.6732),
         6: (-0.30, 0.30), 7: (-0.30, -0.30), 8: (0.07, 0.0), 9: (-0.64, 0.0)}   # V-16: 4 corners + 2 middle + 2 edge
SIX = [14, 13, 0, 15, 6, 7]
PP, F = np.array([980.0, 525.0]), 1400.0   # distorted camera: off-centre optical centre, other formula than calib
def to_px_lens(x, y, k1, k2=0.0):
    p = np.array(to_px(x, y)); r2 = np.sum(((p - PP) / F) ** 2)
    return tuple(PP + (p - PP) * (1 + k1 * r2 + k2 * r2 * r2))
def tags(ids, noise_px=0.0, rng=None, k1=0.0):
    out = []
    for i in ids:
        u, v = to_px_lens(*WORLD[i], k1) if k1 else to_px(*WORLD[i])
        if noise_px:
            u, v = u + rng.normal(0, noise_px), v + rng.normal(0, noise_px)
        out.append({"id": i, "u": u, "v": v, "x": WORLD[i][0], "y": WORLD[i][1]})
    return out

rng = np.random.default_rng(1)
probe_w = [(x, y) for x in (-0.6, -0.45, -0.3, -0.15, 0.0) for y in (-0.7, -0.35, 0.0, 0.35, 0.7)]
probe_px = [to_px(*p) for p in probe_w]
pu, pv = [p[0] for p in probe_px], [p[1] for p in probe_px]
def worst_mm(c):
    x, y, _ = calib.apply(c, pu, pv)
    return max(math.hypot(a - p[0], b - p[1]) for a, b, p in zip(x, y, probe_w)) * 1000

# 1 exact fits
c4 = calib.fit(tags([14, 13, 0, 15]), SIZE, "test")
check("4 tags exact: residual 0, no drop_rms / suspect, 25 probe points < 0.01 mm",
      c4["rms_mm"] < 0.01 and c4["suspect"] is None and all(t["drop_rms_mm"] is None for t in c4["tags"])
      and worst_mm(c4) < 0.01, f"worst {worst_mm(c4):.4f} mm")
c6 = calib.fit(tags(WORLD), SIZE, "test")
check("8 tags exact: lens_k ~0, residual + drop_rms ~0, no suspect, probes < 0.01 mm",
      abs(c6["lens_k"]) < 1e-6 and c6["rms_mm"] < 0.01 and max(t["drop_rms_mm"] for t in c6["tags"]) < 0.01
      and c6["suspect"] is None and worst_mm(c6) < 0.01, f"k {c6['lens_k']:.2e}")
check("ideal lens: lens_effect_mm ~0 (8 tags exact), 0 with 4 tags", calib.lens_effect_mm(c6) < 0.01
      and calib.lens_effect_mm(c4) == 0.0, f"{calib.lens_effect_mm(c6):.4f} mm")
check("4 and 5 tags: no lens term (k = 0)", c4["lens_k"] == 0.0 and calib.fit(tags([14, 13, 0, 15, 6]), SIZE)["lens_k"] == 0.0)

# 2 pixel noise (0.5 px ~ 0.7 mm here): 6 tags beat 4 tags on average
w4, w6, rms6, sus = [], [], [], []
for _ in range(50):
    w4.append(worst_mm(calib.fit(tags([14, 13, 0, 15], 0.5, rng), SIZE)))
    c = calib.fit(tags(WORLD, 0.5, rng), SIZE); w6.append(worst_mm(c)); rms6.append(c["rms_mm"]); sus.append(c["suspect"])
check("ideal lens, noise 0.5 px, 50 runs: 8 tags (+ lens term) worst probe error below 4 tags (mean)",
      np.mean(w6) < np.mean(w4), f"4 tags mean {np.mean(w4):.2f} mm, 8 tags mean {np.mean(w6):.2f} mm, rms mean {np.mean(rms6):.2f} mm")
check("noise 0.5 px: no tag flagged as suspect in 50 runs", all(s is None for s in sus), str([s for s in sus if s]))
check("noise 0.5 px: 8-tag probe error < 3 mm in every run", max(w6) < 3.0, f"max {max(w6):.2f} mm")

# 2b lens distortion (V-17): camera with radial distortion, different formula + off-centre optical centre
def worst_lens(c, k1):
    pp = [to_px_lens(*p, k1) for p in probe_w]
    x, y, _ = calib.apply(c, [q[0] for q in pp], [q[1] for q in pp])
    e = [math.hypot(a - p[0], b - p[1]) * 1000 for a, b, p in zip(x, y, probe_w)]
    return float(np.mean(e)), max(e)
for k1 in (0.10, -0.08):
    res, res4, ks, rms_a, rms_b = [], [], [], [], []
    for _ in range(30):
        c = calib.fit(tags(WORLD, 0.5, rng, k1), SIZE); res.append(worst_lens(c, k1)); ks.append(c["lens_k"])
        rms_a.append(c["rms_homography_only_mm"]); rms_b.append(c["rms_mm"])
        res4.append(worst_lens(calib.fit(tags([14, 13, 0, 15], 0.5, rng, k1), SIZE), k1))
    mean8, mean4 = np.mean([r[0] for r in res]), np.mean([r[0] for r in res4])
    check(f"lens k1 {k1:+.2f}: 8 tags mean error < 1.5 mm, worst < 4 mm, 3x better than 4 tags without lens term",
          mean8 < 1.5 and max(r[1] for r in res) < 4.0 and mean8 * 3 < mean4,
          f"8 tags mean {mean8:.2f} mm worst {max(r[1] for r in res):.2f}; 4 tags mean {mean4:.2f} mm; "
          f"fitted k {np.mean(ks):+.3f}")
    eff = calib.lens_effect_mm(c)
    check(f"lens k1 {k1:+.2f}: lens_effect_mm shows the distortion (> 2 mm)", eff > 2.0, f"{eff:.1f} mm")
    check(f"lens k1 {k1:+.2f}: fitted k has the right sign, rms with lens term < homography-only rms",
          all(np.sign(k) == -np.sign(k1) for k in ks) and np.mean(rms_b) < np.mean(rms_a),
          f"rms with k {np.mean(rms_b):.2f} mm, without {np.mean(rms_a):.2f} mm")

# 3 a wrongly recorded tag shows up in its leave-one-out error
for j, bad_id in enumerate(WORLD):   # one fixed case per tag, 30 mm off along x (8 tags, lens term fitted): found
    t = tags(WORLD); t[j] = dict(t[j], x=t[j]["x"] + 0.030)
    cb = calib.fit(t, SIZE)
    check(f"tag {bad_id} recorded 30 mm off (no noise): suspect = {bad_id}", cb["suspect"] == bad_id,
          f"suspect {cb['suspect']}, rms {cb['rms_mm']} mm, drop_rms {[g['drop_rms_mm'] for g in cb['tags']]}")
stats = {}
for mag in (10, 30):                 # random tag, random direction, with noise: rarely the wrong tag
    found = wrong = 0
    for _ in range(300):
        j = int(rng.integers(len(WORLD))); a = rng.uniform(0, 2 * math.pi); t = tags(WORLD, 0.5, rng)
        t[j] = dict(t[j], x=t[j]["x"] + mag / 1000 * math.cos(a), y=t[j]["y"] + mag / 1000 * math.sin(a))
        su = calib.fit(t, SIZE)["suspect"]
        found += su == list(WORLD)[j]; wrong += su is not None and su != list(WORLD)[j]
    stats[mag] = (found, wrong)
check("one tag 10 / 30 mm off, 300 random cases each: wrong suspect <= 2 % (a hint, not proof)",
      stats[10][1] <= 6 and stats[30][1] <= 6,
      f"found / wrong: 10 mm {stats[10]}, 30 mm {stats[30]}")
check("one tag 30 mm off: found in >= 75 % of the cases", stats[30][0] >= 225, f"{stats[30][0]} / 300")

# 4 inside flag
x, y, inside = calib.apply(c6, [to_px(-0.3, 0.0)[0], to_px(-0.9, 0.0)[0]], [to_px(-0.3, 0.0)[1], to_px(-0.9, 0.0)[1]])
check("inside: table centre True, x -0.90 (beyond the tags) False", inside == [True, False], str(inside))
check("apply: empty -> empty lists", calib.apply(c6, [], []) == ([], [], []))
r = raises(lambda: calib.apply(c6, [1.0, 2.0], [1.0]), ValueError, "lengths differ")
check("apply: u / v length mismatch rejected", r[0], r[1])

# 5 rejections
cases = [("3 tags", tags([14, 13, 0]), "at least 4"),
         ("duplicate id", tags([14, 13, 0, 15]) + [dict(tags([6])[0], id=14)], "twice"),
         ("NaN", [dict(tags([14])[0], x=float("nan"))] + tags([13, 0, 15]), "finite"),
         ("missing key", [{"id": 14, "u": 1, "v": 2, "x": 0.1}] + tags([13, 0, 15]), "needs id"),
         ("text value", [dict(tags([14])[0], u="abc")] + tags([13, 0, 15]), "needs id"),
         ("in a line", [{"id": i, "u": 100.0 * i + 50, "v": 500.0, "x": -0.1 * i, "y": 0.0} for i in range(5)], "line")]
for name, pts, text in cases:
    r = raises(lambda: calib.fit(pts, SIZE), ValueError, text)
    check(f"fit rejects {name}", r[0], r[1])

# 6 save / load through a symlink (= symlink-install), backup, atomic
with tempfile.TemporaryDirectory() as d:
    src = os.path.join(d, "src_calib.yaml"); link = os.path.join(d, "install_calib.yaml")
    open(src, "w").write("version: 1\ncalibrated: false\n"); os.symlink(src, link)
    r = raises(lambda: calib.load(link), calib.NotCalibrated, "not calibrated")
    check("load placeholder -> NotCalibrated", r[0], r[1])
    written = calib.save(link, c6)
    check("save via symlink: link kept, source file written, .bak = placeholder",
          os.path.islink(link) and written == os.path.realpath(src) and "calibrated: false" in open(src + ".bak").read()
          and not os.path.exists(src + ".tmp"))
    back = calib.load(link)
    check("load(save(c)) round trip: same homography + tags, probes < 0.01 mm",
          np.allclose(back["homography"], c6["homography"]) and back["tags"] == c6["tags"] and worst_mm(back) < 0.01)
    check("saved file starts with the header comment", open(src).read().startswith("# Camera -> table calibration"))
    for name, text, msg in [("missing file", None, "cannot read"), ("bad yaml", "a: [1, 2", "not valid yaml"),
                            ("not a dict", "- 1\n- 2\n", "not a calibration"), ("wrong version", "version: 9\n", "version 9"),
                            ("bad homography", yaml.safe_dump(dict(c6, homography=[[1, 2], [3, 4]])), "bad homography"),
                            ("no lens_k", yaml.safe_dump({k: v for k, v in c6.items() if k != "lens_k"}), "lens_k"),
                            ("lens_k nan", yaml.safe_dump(dict(c6, lens_k=float("nan"))), "lens_k"),
                            ("too few tags", yaml.safe_dump(dict(c6, tags=c6["tags"][:3])), "at least 4")]:
        p = os.path.join(d, name.replace(" ", "_") + ".yaml")
        if text is not None:
            open(p, "w").write(text)
        r = raises(lambda: calib.load(p), ValueError, msg)
        check(f"load rejects {name}", r[0], r[1])

# 7 the real lab calibration (4 tags) reproduces its homography and its tags
lab = [{"id": 0, "u": 1532.447998046875, "v": 816.139892578125, "x": -0.6635, "y": -0.59544},
       {"id": 13, "u": 1667.22314453125, "v": 229.43963623046875, "x": 0.08928, "y": -0.86261},
       {"id": 14, "u": 352.4923400878906, "v": 260.540283203125, "x": 0.04783, "y": 0.8366},
       {"id": 15, "u": 421.94671630859375, "v": 783.0250244140625, "x": -0.62235, "y": 0.67322}]
cl = calib.fit(lab, SIZE, "C920")
Hl = np.array(cl["homography"]); Hl = Hl / Hl[2, 2]
check("lab calib.json: same homography as pix2table (mm -> m)", np.allclose(Hl, H_TRUE, rtol=1e-6, atol=1e-9))

# 8 installed config/calib.yaml: placeholder (NotCalibrated) or a valid calibration, nothing else
inst = os.path.join(get_package_share_directory("hk_fr5_vision"), "config", "calib.yaml")
try:
    c = calib.load(inst); state = f"calibrated {c['date']}, {len(c['tags'])} tags, rms {c['rms_mm']} mm"
    good = True
except calib.NotCalibrated:
    state, good = "placeholder (not calibrated yet)", True
except ValueError as e:
    state, good = str(e), False
check("installed config/calib.yaml loads", good, state)

print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
