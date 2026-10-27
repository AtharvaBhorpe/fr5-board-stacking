"""Classic detector vs ONNX models on the held-out real stack frames (train_data/*/val_real, reviewed labels).

    pixi run python train/compare_real.py models/boards-seg.onnx [other.onnx ...]

Per method: boards found (mask IoU >= 0.5 with a label), of those with the right variant, false detections.
Models run exactly as on the robot (vision1/seg.py, same crop). Classic = eval_out/results.json (its quads).
"""
import glob, json, os, sys
import cv2, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "vision1"))
import detect as D  # noqa: E402
import seg  # noqa: E402

VAL = os.path.join(HERE, "..", "train_data", "labels", "val_real")
CLASSES = ["VARa", "VARb", "VARc"]
IOU = 0.5


def name_of(path):
    return "_".join(path.split("/")[-3:]).rsplit(".", 1)[0].lstrip("._")


def gt(txt, shape):
    h, w = shape
    out = []
    for ln in open(txt).read().split("\n"):
        if ln.strip():
            v = ln.split()
            m = np.zeros((h, w), np.uint8)
            cv2.fillPoly(m, [np.int32(np.array(v[1:], float).reshape(-1, 2) * [w, h])], 1)
            out.append((CLASSES[int(v[0])], m.astype(bool)))
    return out


def score(preds, gts):
    """Greedy match by mask IoU. -> found, right class, false detections."""
    used, found, right = set(), 0, 0
    for pc, pm in preds:
        best, bj = 0, None
        for j, (gc, gm) in enumerate(gts):
            if j not in used:
                iou = (pm & gm).sum() / max((pm | gm).sum(), 1)
                if iou > best:
                    best, bj = iou, j
        if best >= IOU:
            used.add(bj); found += 1; right += pc == gts[bj][0]
    return found, right, len(preds) - found


def main():
    recs = {name_of(r["path"]): r for r in json.load(open(os.path.join(HERE, "..", "eval_out", "results.json")))}
    x0, y0, x1, y1 = D.roi_px(1920, 1080)
    shape = (y1 - y0, x1 - x0)
    methods = {"classic": None} | {os.path.basename(p): p for p in sys.argv[1:]}
    tot = {k: np.zeros(3, int) for k in methods}
    n_gt = 0
    for txt in sorted(glob.glob(os.path.join(VAL, "*.txt"))):
        name = os.path.basename(txt)[5:-4]  # real_<name>.txt
        r = recs[name]
        g = gt(txt, shape)
        n_gt += len(g)
        frame = cv2.imread(r["path"])
        row = [f"{name[-20:]:>20} gt {len(g):2d}"]
        for k, p in methods.items():
            if p is None:
                preds = []
                for b in r["boards"]:
                    m = np.zeros(shape, np.uint8)
                    cv2.fillPoly(m, [np.int32(np.array(b["corners"]) - [x0, y0])], 1)
                    preds.append((b["variant"], m.astype(bool)))
            else:
                if seg.MODEL != p:
                    seg.MODEL, seg._sess = p, None
                preds = [(i["cls"], i["mask"][y0:y1, x0:x1] > 0) for i in seg.segment(frame, (x0, y0, x1, y1))]
            s = score(preds, g)
            tot[k] += s
            row.append(f"{k[:12]:>12} {s[0]:2d}/{s[1]:2d}/{s[2]:2d}")
        print("  ".join(row))
    print(f"\n{n_gt} labelled boards in {len(glob.glob(os.path.join(VAL, '*.txt')))} frames. found / right variant / false")
    for k, (f, rt, fp) in tot.items():
        print(f"{k:>24}: found {f:3d} ({100 * f / n_gt:3.0f} %), right variant {rt:3d} ({100 * rt / n_gt:3.0f} %), "
              f"false {fp}")


if __name__ == "__main__":
    main()
