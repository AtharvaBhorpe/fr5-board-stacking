"""Image measurements shared by geometric and segmentation detectors."""

import cv2
import numpy as np


def describe_region(bgr, mask):
    if (bgr.dtype != np.uint8 or bgr.ndim != 3
            or bgr.shape[2] != 3):
        raise ValueError("Expected a uint8 BGR image.")
    if mask.shape != bgr.shape[:2] or mask.dtype != np.uint8:
        raise ValueError("Expected a uint8 mask matching the image.")

    binary = np.where(mask > 0, 255, 0).astype(np.uint8)
    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 1:
        raise ValueError("Expected one connected object mask.")

    _, sides, _ = cv2.minAreaRect(contours[0])
    short_side, long_side = sorted(float(v) for v in sides)
    if short_side <= 0:
        raise ValueError("Object mask has no measurable width.")

    interior = cv2.erode(
        binary, np.ones((5, 5), np.uint8),
        borderType=cv2.BORDER_CONSTANT, borderValue=0)
    pixels = bgr[interior > 0]
    if len(pixels) < 25:
        raise ValueError("Insufficient interior pixels for appearance.")

    # shortcut: median assumes most interior pixels show the board;
    # use a clean surface patch when printing or occlusion dominates.
    lab = cv2.cvtColor(
        pixels.astype(np.float32).reshape(-1, 1, 3) / 255.0,
        cv2.COLOR_BGR2LAB).reshape(-1, 3)

    return {
        "appearance": {
            "median_rgb": np.median(pixels[:, ::-1], axis=0).tolist(),
            "median_lab": np.median(lab, axis=0).tolist(),
            "colour_label": None,
            "sample_pixels": int(len(pixels)),
        },
        "size": {
            "long_side_px": long_side,
            "short_side_px": short_side,
            "long_side_m": None,
            "short_side_m": None,
        },
    }
