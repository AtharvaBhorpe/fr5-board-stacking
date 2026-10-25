import argparse
from collections import Counter
import json
from pathlib import Path

import cv2
import numpy as np


DESTINATIONS = {1: "VARa", 2: "VARb", 16: "VARc"}


def detect_destinations(frame):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, params)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    contrast = cv2.createCLAHE(
        clipLimit=2.0, tileGridSize=(8, 8)
    ).apply(gray)
    enlarged = cv2.resize(
        contrast, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC
    )
    corners, ids, _ = detector.detectMarkers(enlarged)

    found = [] if ids is None else [int(i) for i in ids.flatten()]
    counts = Counter(found)
    observations = []

    for marker_id, corner in zip(found, corners):
        points = corner.reshape(4, 2) / 2.0
        observations.append({
            "tag_id": marker_id,
            "variant": DESTINATIONS.get(marker_id),
            "identity_valid": marker_id in DESTINATIONS
                              and counts[marker_id] == 1,
            "corners_px": points.tolist(),
            "placement_pose_robot": None,
        })

    return observations, {
        "missing_destination_ids": sorted(set(DESTINATIONS) - set(found)),
        "unexpected_tag_ids": sorted(set(found) - set(DESTINATIONS)),
        "duplicate_tag_ids": sorted(i for i, n in counts.items() if n > 1),
    }


def detect_source_candidates(frame, roi):
    x0, y0, x1, y1 = roi
    crop = frame[y0:y1, x0:x1]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    masks = {
        "white": cv2.inRange(hsv, (0, 0, 95), (179, 75, 255)),
        "red": cv2.bitwise_or(
            cv2.inRange(hsv, (0, 85, 65), (12, 255, 255)),
            cv2.inRange(hsv, (168, 85, 65), (179, 255, 255)),
        ),
    }

    candidates, rejected = [], []
    kernel = np.ones((3, 3), dtype=np.uint8)

    for colour, raw in masks.items():
        mask = cv2.morphologyEx(raw, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 2500:
                continue

            polygon = cv2.approxPolyDP(
                contour, 0.02 * cv2.arcLength(contour, True), True
            )
            width, height = cv2.minAreaRect(contour)[1]
            rectangularity = area / (width * height) if width * height else 0
            x, y, w, h = cv2.boundingRect(contour)
            reasons = []

            if (
                x <= 1 or y <= 1
                or x + w >= crop.shape[1] - 1
                or y + h >= crop.shape[0] - 1
            ):
                reasons.append("touches_search_boundary")
            if len(polygon) != 4 or not cv2.isContourConvex(polygon):
                reasons.append("not_a_convex_quadrilateral")
            if rectangularity < 0.88:
                reasons.append("irregular_outline")

            points = polygon.reshape(-1, 2) + np.array([x0, y0])
            item = {
                "colour": colour,
                "outline_px": points.tolist(),
                "area_px2": area,
                "rectangularity": float(rectangularity),
                "reasons": reasons,
            }
            if reasons:
                rejected.append(item)
            else:
                item.update({
                    "candidate_id": f"candidate-{len(candidates) + 1}",
                    "variant": None,
                    "single_sheet_verified": False,
                    "grasp_pose_robot": None,
                })
                candidates.append(item)

    return candidates, rejected


def perceive(frame, roi):
    tags, diagnostics = detect_destinations(frame)
    candidates, rejected = detect_source_candidates(frame, roi)
    return {
        "schema_version": 1,
        "mode": "offline_image",
        "image_size": [frame.shape[1], frame.shape[0]],
        "coordinate_space": "original_distorted_image_pixels",
        "source_roi_xyxy": list(roi),
        "destination_tags": tags,
        "source_candidates": candidates,
        "rejected_source_regions": rejected,
        "diagnostics": diagnostics,
        "metric_calibrated": False,
        "execution_allowed": False,
        "robot_targets": [],
    }


def annotate(frame, scene):
    overlay = frame.copy()
    x0, y0, x1, y1 = scene["source_roi_xyxy"]
    cv2.rectangle(overlay, (x0, y0), (x1 - 1, y1 - 1), (255, 180, 0), 2)

    items = []
    for tag in scene["destination_tags"]:
        label = f"ID {tag['tag_id']} {tag['variant'] or 'unknown'}"
        colour = (0, 255, 0) if tag["identity_valid"] else (0, 0, 255)
        items.append((tag["corners_px"], label, colour))
    for board in scene["source_candidates"]:
        items.append((
            board["outline_px"],
            f"{board['candidate_id']} {board['colour']}",
            (0, 255, 255),
        ))
    for rejected in scene["rejected_source_regions"]:
        items.append((rejected["outline_px"], "", (0, 0, 255)))

    for outline, label, colour in items:
        points = np.rint(outline).astype(np.int32)
        cv2.polylines(overlay, [points], True, colour, 2)
        if label:
            anchor = tuple(points[0])
            cv2.putText(
                overlay, label, anchor,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2,
            )

    return overlay


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("images", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("artifacts/scenes"))
    parser.add_argument(
        "--roi", nargs=4, type=int, default=(1120, 330, 1840, 980),
        metavar=("LEFT", "TOP", "RIGHT", "BOTTOM"),
        help="Source search region for the historical 1920x1080 photos.",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    for index, source in enumerate(args.images, start=1):
        frame = cv2.imread(str(source))
        if frame is None:
            raise RuntimeError(f"Cannot read {source}")
        if frame.shape[:2] != (1080, 1920):
            raise RuntimeError(f"Expected 1920x1080 image: {source}")
        x0, y0, x1, y1 = args.roi
        if not (0 <= x0 < x1 <= 1920 and 0 <= y0 < y1 <= 1080):
            raise RuntimeError("ROI must be inside the image.")

        scene = perceive(frame, args.roi)
        scene["image"] = str(source.resolve())
        stem = f"{index:02d}-{source.stem}"

        image_path = args.output / f"{stem}.png"
        if not cv2.imwrite(str(image_path), annotate(frame, scene)):
            raise RuntimeError(f"Cannot write {image_path}")
        (args.output / f"{stem}.json").write_text(
            json.dumps(scene, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )

        colours = Counter(c["colour"] for c in scene["source_candidates"])
        ids = sorted(t["tag_id"] for t in scene["destination_tags"])
        print(
            f"{source.name}: tags={ids}, "
            f"white={colours['white']}, red={colours['red']}, "
            f"rejected={len(scene['rejected_source_regions'])}"
        )

    print(f"\nResults: {args.output.resolve()}")


if __name__ == "__main__":
    main()
