"""SAM 2 pre-labels for real stack frames -> label_real/*.png + X-AnyLabeling / labelme *.json (fix by hand).

    pixi run -e train python train/prelabel.py [N=80]      # SAM 3 (sam3.pt, gated on Hugging Face) if present, else SAM 2

Frames: classic detector rejected a too_big blob (merged / stacked boards). Image = detector ROI crop.
SAM 3 text prompt (PROMPTS) or SAM 2 'segment everything', kept if board-coloured, board-sized and roughly rectangular; class from colour + area
(same rule as the classic detector). Partly hidden white boards often get the wrong class: check every label.
"""
import json, os, sys
import cv2, numpy as np
from ultralytics import SAM
HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "vision1"))
import detect as D  # noqa: E402

OUT = os.path.join(HERE, "..", "label_real")
N = int(sys.argv[1]) if len(sys.argv) > 1 else 80
MIN_COLOUR = 0.6    # fraction of mask pixels that are board white / red
MIN_RECT = 0.55     # mask area / its min-area rectangle (notched lower boards still pass, hands / arm mostly not)
PROMPTS = ["white paper sheet", "red paper sheet"]
ONE_BOARD = 0.92    # rect_fill above this = a single whole board, not a stack
NESTED = 0.85       # child mask this much inside a bigger one counts as its part
WHITE = ((0, 0, 150), (179, 45, 255))  # stricter than the detector: skin (hands) must not pass


def label(m, hsv):
    """Board mask -> class name or None."""
    a = int(m.sum())
    if not (D.MIN_AREA / 3 <= a <= D.MAX_AREA * 1.2):
        return None
    cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cs, key=cv2.contourArea)
    (w, h) = cv2.minAreaRect(c)[1]
    if a / max(w * h, 1) < MIN_RECT:
        return None
    px = hsv[m]
    white = cv2.inRange(px[None], *WHITE).mean() / 255
    red = max(cv2.inRange(px[None], lo, hi).mean() / 255 for lo, hi in D.RED)
    if red >= 0.4:            # red paint is streaky, lower bar
        return "VARc"
    if white >= MIN_COLOUR:
        return "VARa" if w * h >= D.VARA_MIN_AREA else "VARb"
    return None


def rect_fill(m):
    cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    w, h = cv2.minAreaRect(max(cs, key=cv2.contourArea))[1]
    return m.sum() / max(w * h, 1)


def select(cand):
    """SAM returns both a stack and its boards. Mask with >= 2 parts inside: notched outline = stack -> keep the
    parts; clean rectangle = one board SAM also cut into pieces -> keep it. 1 part inside (red paint) -> keep it."""
    area = [m.sum() for _, m in cand]
    parts = [[j for j, (_, k) in enumerate(cand) if j != i and area[j] < area[i]
              and (m & k).sum() >= NESTED * area[j]] for i, (_, m) in enumerate(cand)]
    drop = set()
    for i, p in enumerate(parts):
        drop |= {i} if len(p) >= 2 and rect_fill(cand[i][1]) < ONE_BOARD else set(p)
    return [cand[i] for i in range(len(cand)) if i not in drop]


def polygon(m):
    cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cs, key=cv2.contourArea)
    c = cv2.approxPolyDP(c, 2.0, True)[:, 0]
    return c.astype(float).tolist()


def main():
    os.makedirs(OUT, exist_ok=True)
    recs = json.load(open(os.path.join(HERE, "..", "eval_out", "results.json")))
    hard = [r for r in recs if any(x["reason"] == "too_big" for x in r["rejected"])]
    pick = hard[::max(1, len(hard) // N)][:N]
    x0, y0, x1, y1 = D.roi_px(1920, 1080)
    if os.path.isfile("sam3.pt"):
        from ultralytics.models.sam import SAM3SemanticPredictor
        p3 = SAM3SemanticPredictor(overrides=dict(conf=0.25, task="segment", mode="predict", model="sam3.pt", imgsz=1008,
                                                  half=True, save=False, verbose=False))

        def masks_of(im):
            p3.set_image(im)
            return p3(text=PROMPTS)[0]
    else:
        sam = SAM("sam2.1_b.pt")

        def masks_of(im):
            return sam(im, verbose=False)[0]
    for i, r in enumerate(pick):
        name = "_".join(r["path"].split("/")[-3:]).replace(".png", "").replace(".jpg", "").lstrip("._")
        im = cv2.imread(r["path"])[y0:y1, x0:x1]
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        res = masks_of(im)
        masks = [] if res.masks is None else list(res.masks.data.cpu().numpy().astype(bool))
        cand = [(c, m) for m in masks if (c := label(m, hsv))]
        kept = select(cand)
        cv2.imwrite(os.path.join(OUT, name + ".png"), im)
        shapes = [{"label": c, "points": polygon(m), "group_id": None, "shape_type": "polygon", "flags": {}}
                  for c, m in kept]
        json.dump({"version": "2.4.0", "flags": {}, "shapes": shapes, "imagePath": name + ".png",
                   "imageData": None, "imageHeight": im.shape[0], "imageWidth": im.shape[1]},
                  open(os.path.join(OUT, name + ".json"), "w"))
        print(f"{i + 1}/{len(pick)} {name}: {[c for c, _ in kept]}", flush=True)


if __name__ == "__main__":
    main()
