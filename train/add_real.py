"""Reviewed real labels (label_real/*.json, X-AnyLabeling / labelme) -> train_data (YOLO-seg). Run after make_dataset.py.

    pixi run -e train python train/add_real.py

Frames make_dataset.py put in val (same split rule, no leak) -> images/val_real + data_real.yaml (real-only score).
Train frames are written REPEAT times: 80 real frames vs ~2000 synthetic, they must count more.
"""
import glob, json, os, shutil

HERE = os.path.dirname(__file__)
SRC = os.path.join(HERE, "..", "label_real")
OUT = os.path.join(HERE, "..", "train_data")
CLASSES = ["VARa", "VARb", "VARc"]
REPEAT = 3


def yolo_lines(j):
    w, h = j["imageWidth"], j["imageHeight"]
    out = []
    for s in j["shapes"]:
        if s["label"] not in CLASSES or s.get("shape_type", "polygon") != "polygon" or len(s["points"]) < 3:
            continue
        xy = " ".join(f"{x / w:.5f} {y / h:.5f}" for x, y in s["points"])
        out.append(f"{CLASSES.index(s['label'])} {xy}")
    return out


def main():
    recs = json.load(open(os.path.join(HERE, "..", "eval_out", "results.json")))
    val_names = {"_".join(r["path"].split("/")[-3:]).rsplit(".", 1)[0].lstrip("._")
                 for k, r in enumerate(recs) if (k // 25) % 8 == 0}
    for d in ("images/val_real", "labels/val_real"):
        shutil.rmtree(os.path.join(OUT, d), ignore_errors=True)
        os.makedirs(os.path.join(OUT, d))
    n = {"train": 0, "val_real": 0, "bad": []}
    for f in sorted(glob.glob(os.path.join(SRC, "*.json"))):
        name = os.path.basename(f)[:-5]
        j = json.load(open(f))
        bad = [s["label"] for s in j["shapes"] if s["label"] not in CLASSES]
        if bad:
            n["bad"].append((name, bad))
        lines = "\n".join(yolo_lines(j))
        img = os.path.join(SRC, j["imagePath"])
        reps = [("val_real", "")] if name in val_names else [("train", f"_r{i}") for i in range(REPEAT)]
        for split, suf in reps:
            shutil.copy(img, os.path.join(OUT, "images", split, f"real_{name}{suf}.png"))
            with open(os.path.join(OUT, "labels", split, f"real_{name}{suf}.txt"), "w") as o:
                o.write(lines)
        n["val_real" if name in val_names else "train"] += 1
    with open(os.path.join(OUT, "data_real.yaml"), "w") as o:
        o.write(f"path: {os.path.abspath(OUT)}\ntrain: images/train\nval: images/val_real\nnames: {CLASSES}\n")
    print(f"real frames: {n['train']} train (x{REPEAT}), {n['val_real']} val_real; unknown labels: {n['bad']}")


if __name__ == "__main__":
    main()
