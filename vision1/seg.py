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
        so = ort.SessionOptions()
        so.intra_op_num_threads = 4  # CPU fallback: all cores (default) starved the robot stack, 12 cores for 2.4 Hz
        # GPU when available (onnxruntime CUDA build), else CPU
        prov = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in ort.get_available_providers()]
        _sess = ort.InferenceSession(MODEL, so, providers=prov)
        print(f"seg model on {_sess.get_providers()[0]}", flush=True)
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
    g = pr.shape[1] / size                          # proto px per letterbox px
    for i in idx:
        bx, by, bw, bh = boxes[i]                   # letterbox px
        # box in crop px (clipped); the mask is cut to its box, as Ultralytics does
        ox0, oy0 = max(0, int((bx - left) / r)), max(0, int((by - top) / r))
        ox1, oy1 = min(w, int(np.ceil((bx + bw - left) / r))), min(h, int(np.ceil((by + bh - top) / r)))
        if ox1 - ox0 < 2 or oy1 - oy0 < 2:
            continue
        # proto window covering the box (+1 cell), upsampled only to that window's size in crop px (speed: the
        # full-size upsample per instance took ~120 ms / frame, 6x the model on the GPU)
        ix0, iy0 = max(0, int((ox0 * r + left) * g) - 1), max(0, int((oy0 * r + top) * g) - 1)
        ix1, iy1 = min(pr.shape[2], int(np.ceil((ox1 * r + left) * g)) + 1), min(pr.shape[1], int(np.ceil((oy1 * r + top) * g)) + 1)
        logit = np.tensordot(p[i, 4 + nc:], pr[:, iy0:iy1, ix0:ix1], 1)   # sigmoid > 0.5 <=> logit > 0
        wx0, wy0 = (ix0 / g - left) / r, (iy0 / g - top) / r              # window extent in crop px
        wx1, wy1 = (ix1 / g - left) / r, (iy1 / g - top) / r
        up = cv2.resize(logit, (max(1, round(wx1 - wx0)), max(1, round(wy1 - wy0))), interpolation=cv2.INTER_LINEAR)
        sx, sy = ox0 - round(wx0), oy0 - round(wy0)
        sub = up[max(0, sy):max(0, sy) + oy1 - oy0, max(0, sx):max(0, sx) + ox1 - ox0] > 0
        full = np.zeros(bgr.shape[:2], np.uint8)
        full[y0 + oy0:y0 + oy0 + sub.shape[0], x0 + ox0:x0 + ox0 + sub.shape[1]] = sub.astype(np.uint8) * 255
        res.append({"cls": CLASSES[int(cls[i])], "conf": float(conf[i]), "mask": full})
    return res
