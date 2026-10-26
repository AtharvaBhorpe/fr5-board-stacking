"""Add diagnostic metric classifications to an offline scene JSON."""

import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from classify_board import classify_board
from load_perception_geometry import (
    IncompleteCalibration,
    load_geometry,
    surface_points,
)


def classify_scene(scene, config, K, D):
    if scene.get("coordinate_space") != "original_distorted_image_pixels":
        raise ValueError("Expected original distorted image pixels")
    if scene.get("image_size") != config["camera"]["image_size_px"]:
        raise ValueError("Scene resolution differs from calibration")

    variants = config["variants"]
    if not variants:
        raise ValueError("No configured variants")

    thicknesses = []
    for spec in variants.values():
        size = np.asarray(spec["size_m"], dtype=float)
        if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError("Invalid configured board dimensions")
        thicknesses.append(size[2])

    if not np.allclose(thicknesses, thicknesses[0], atol=1e-9, rtol=0):
        raise ValueError(
            "Different thicknesses require separate plane hypotheses"
        )

    # Used only to select the shared thickness, never to infer identity.
    plane_variant = next(iter(variants))
    result = deepcopy(scene)
    result["execution_allowed"] = False
    result["robot_targets"] = []

    for candidate in result["source_candidates"]:
        candidate["variant"] = None
        candidate["grasp_pose_robot"] = None
        candidate["single_sheet_verified"] = False
        candidate["metric_outline_m"] = None

        try:
            points = surface_points(
                config, K, D,
                candidate["outline_px"],
                result["image_size"],
                "source",
                plane_variant,
            )
        except IncompleteCalibration as error:
            classification = {
                "status": "uncalibrated",
                "variant": None,
                "reason": str(error),
                "execution_allowed": False,
            }
        except ValueError as error:
            classification = {
                "status": "rejected",
                "variant": None,
                "reason": str(error),
                "execution_allowed": False,
            }
        else:
            candidate["metric_outline_m"] = points.tolist()
            classification = classify_board(points, candidate["colour"])

        candidate["metric_classification"] = classification

    matches = {}
    for candidate in result["source_candidates"]:
        classification = candidate["metric_classification"]
        if classification["status"] == "matched":
            matches.setdefault(classification["variant"], []).append(
                candidate["candidate_id"]
            )

    result["metric_classification_summary"] = {
        "frame_id": config["workcell"]["frame_id"],
        "common_board_thickness_m": float(thicknesses[0]),
        "matched_candidate_ids": matches,
        "multiple_candidates_for_variant": sorted(
            variant for variant, ids in matches.items() if len(ids) > 1
        ),
        "purpose": "diagnostic_only",
        "physical_accuracy_validated": False,
        "execution_allowed": False,
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scene", type=Path)
    parser.add_argument(
        "--config", type=Path, default=Path("config/perception_geometry.yaml")
    )
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    config, K, D = load_geometry(args.config)
    scene = json.loads(args.scene.read_text(encoding="utf-8"))
    result = classify_scene(scene, config, K, D)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
        handle.write("\n")

    for candidate in result["source_candidates"]:
        classification = candidate["metric_classification"]
        print(
            f"{candidate['candidate_id']}: "
            f"{classification['status']}; "
            f"variant={classification['variant']}; "
            f"reason={classification['reason']}"
        )
    print(f"Diagnostic report: {args.output.resolve()}")


if __name__ == "__main__":
    main()
