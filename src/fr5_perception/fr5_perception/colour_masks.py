"""Configurable HSV foreground masks; no semantic colour classification."""

import cv2
import numpy as np


def foreground_mask(bgr, ranges):
    if not isinstance(ranges, list) or not ranges:
        raise ValueError("At least one HSV range is required.")

    for bounds in ranges:
        if (not isinstance(bounds, list) or len(bounds) != 6
                or any(type(value) is not int for value in bounds)):
            raise ValueError("HSV ranges must contain six integers.")
        for low, high, maximum in zip(
                bounds[:3], bounds[3:], (179, 255, 255)):
            if not 0 <= low <= high <= maximum:
                raise ValueError(
                    "Invalid HSV bounds; split hue wraparound into two ranges.")

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(bgr.shape[:2], np.uint8)
    for bounds in ranges:
        selected = cv2.inRange(
            hsv, np.array(bounds[:3], np.uint8),
            np.array(bounds[3:], np.uint8))
        mask = cv2.bitwise_or(mask, selected)

    return cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
