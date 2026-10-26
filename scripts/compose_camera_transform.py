"""Compose a diagnostic camera-to-base transform. No ROS or robot commands."""

import argparse
import json
from pathlib import Path

import numpy as np

from plane_geometry import validate_transform


def compose(T_base_link_board, T_camera_board):
    base_board = validate_transform(T_base_link_board)
    camera_board = validate_transform(T_camera_board)
    return validate_transform(base_board @ np.linalg.inv(camera_board))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-report", required=True, type=Path)
    parser.add_argument(
        "--image-index", required=True, type=int,
        help="One-based image entry in the pose report.",
    )
    parser.add_argument(
        "--solution-index", required=True, type=int,
        help="One-based pose candidate; selection is not validation.",
    )
    parser.add_argument("--board-placement", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    reports = json.loads(args.pose_report.read_text(encoding="utf-8"))
    placement = json.loads(
        args.board_placement.read_text(encoding="utf-8")
    )

    if not isinstance(reports, list):
        raise ValueError("Pose report must contain a list of image entries")
    if not 1 <= args.image_index <= len(reports):
        raise ValueError("Image index outside pose report")

    record = reports[args.image_index - 1]
    solutions = record["solutions"]
    if not 1 <= args.solution_index <= len(solutions):
        raise ValueError("Solution index outside selected image's candidates")

    if placement["schema_version"] != 1:
        raise ValueError("Unsupported board-placement schema")
    if placement["frame_id"] != "base_link":
        raise ValueError("Board placement must be expressed in base_link")
    if placement["units"] != {"length": "metres", "angle": "radians"}:
        raise ValueError("Expected metres and radians")
    if placement["pose_source"] != record["source"]:
        raise ValueError("Board placement does not identify this pose image")

    for field in ("measurement_method", "board_axes_description"):
        value = placement.get(field)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"Provide a nonempty {field}")

    selected = solutions[args.solution_index - 1]
    transform = compose(
        placement["T_base_link_board"],
        selected["T_camera_board"],
    )

    result = {
        "schema_version": 1,
        "purpose": "candidate_camera_extrinsic",
        "frame_id": "base_link",
        "transform_convention": "T_A_B maps coordinates from B into A",
        "T_base_link_camera": transform.tolist(),
        "pose_source": record["source"],
        "pose_report": str(args.pose_report.resolve()),
        "image_index": args.image_index,
        "solution_index": args.solution_index,
        "candidate_count": len(solutions),
        "selected_corner_rms_px": selected["corner_rms_px"],
        "board_placement": placement,
        "pose_ambiguity_resolved": False,
        "independent_accuracy_validated": False,
        "calibration_approved": False,
        "execution_allowed": False,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
        handle.write("\n")

    print("Candidate T_base_link_camera:")
    print(np.array2string(transform, precision=6, suppress_small=True))
    print(f"Report: {args.output.resolve()}")
    print("Diagnostic only. Operational calibration was not changed.")


if __name__ == "__main__":
    main()
