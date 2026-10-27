"""Model on real frames the classic detector could not handle (rejected too_big / not_rectangular blobs):
contact sheet eval_out/model_<name>.jpg. No labels exist for these, check by eye.

    pixi run -e train python train/eval_real.py runs/boards/weights/last.pt
"""
import json, os, sys
import cv2, numpy as np
from ultralytics import YOLO
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "vision1"))
import detect as D  # noqa: E402

m = YOLO(sys.argv[1])
recs = json.load(open("eval_out/results.json"))
hard = [r for r in recs if any(x["reason"] == "too_big" for x in r["rejected"])]
pick = hard[::max(1, len(hard) // 12)][:12]
x0, y0, x1, y1 = D.roi_px(1920, 1080)
tiles = []
for r in pick:
    im = cv2.imread(r["path"])[y0:y1, x0:x1]
    res = m.predict(im, imgsz=960, conf=float(os.environ.get("CONF", 0.4)), verbose=False)[0]
    t = res.plot(line_width=2, font_size=14)
    cv2.putText(t, os.path.basename(r["path"]), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    tiles.append(cv2.resize(t, (400, 444)))
rows = [np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]
out = f"eval_out/model_{os.path.basename(sys.argv[1]).split('.')[0]}.jpg"
cv2.imwrite(out, np.vstack(rows)); print(out)
