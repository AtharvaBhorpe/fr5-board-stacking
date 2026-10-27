"""Learned board segmenter (train/train.py -> models/boards-seg.onnx), onnxruntime only (no torch in the ROS env).

segment(bgr) -> [{'cls': 'VARa'|'VARb'|'VARc', 'conf', 'mask' (full-image uint8)}] for the detector ROI.
Decoding = Ultralytics YOLO-seg ONNX layout: output0 (1, 4 + nc + 32, N) = cx, cy, w, h, class scores, mask
coefficients; output1 (1, 32, S/4, S/4) mask prototypes.
"""
import os

import cv2
import numpy as np

MODEL = os.path.join(os.path.dirname(__file__), "..", "models", "boards-seg.onnx")
CLASSES = ["VARa", "VARb", "VARc"]
CONF = 0.4
IOU = 0.7          # per-class NMS; stacked boards overlap a lot, keep high
_sess = None


def available():
    return os.path.isfile(MODEL)


def _session():
    global _sess
    if _sess is None:
        import onnxruntime as ort
        _sess = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
    return _sess


def segment(bgr, roi_px):
    """roi_px = (x0, y0, x1, y1) crop (same crop as training)."""
    x0, y0, x1, y1 = roi_px
    img = bgr[y0:y1, x0:x1]
    s = _session()
    size = s.get_inputs()[0].shape[2]
    h, w = img.shape[:2]
    r = size / max(h, w)
    nh, nw = round(h * r), round(w * r)
    top, left = (size - nh) // 2, (size - nw) // 2
    lb = np.full((size, size, 3), 114, np.uint8)
    lb[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    x = lb[..., ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255
    out, protos = s.run(None, {s.get_inputs()[0].name: x})
    p = out[0].T                                   # (N, 4 + nc + 32)
    nc = len(CLASSES)
    scores = p[:, 4:4 + nc]
    cls, conf = scores.argmax(1), scores.max(1)
    keep = conf >= CONF
    p, cls, conf = p[keep], cls[keep], conf[keep]
    boxes = np.c_[p[:, 0] - p[:, 2] / 2, p[:, 1] - p[:, 3] / 2, p[:, 2], p[:, 3]]
    idx = []
    for c in np.unique(cls):
        m = np.nonzero(cls == c)[0]
        k = cv2.dnn.NMSBoxes(boxes[m].tolist(), conf[m].tolist(), CONF, IOU)
        idx += [m[i] for i in np.array(k).flatten()]
    res = []
    pr = protos[0]                                  # (32, S/4, S/4)
    ps = pr.shape[1]
    for i in idx:
        mk = 1 / (1 + np.exp(-(p[i, 4 + nc:] @ pr.reshape(32, -1)).reshape(ps, ps)))
        mk = cv2.resize(mk, (size, size), interpolation=cv2.INTER_LINEAR)
        bx, by, bw, bh = boxes[i]
        crop = np.zeros_like(mk)
        ys, ye = int(max(0, by)), int(min(size, by + bh))
        xs, xe = int(max(0, bx)), int(min(size, bx + bw))
        crop[ys:ye, xs:xe] = mk[ys:ye, xs:xe]       # mask only inside its box (as Ultralytics does)
        crop = crop[top:top + nh, left:left + nw]
        full = np.zeros(bgr.shape[:2], np.uint8)
        full[y0:y1, x0:x1] = (cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR) > 0.5).astype(np.uint8) * 255
        res.append({"cls": CLASSES[int(cls[i])], "conf": float(conf[i]), "mask": full})
    return res
