"""Offline classification and selection report. No ROS or motion commands."""

import argparse
import json
from pathlib import Path

from classify_scene import classify_scene
from load_perception_geometry import load_geometry
from select_source import select_source


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scene", type=Path)
    parser.add_argument(
        "--variant", required=True, choices=("VARa", "VARb", "VARc")
    )
    parser.add_argument(
        "--config", type=Path,
        default=Path("config/perception_geometry.yaml"),
    )
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    scene = json.loads(args.scene.read_text(encoding="utf-8"))
    config, K, D = load_geometry(args.config)
    classified = classify_scene(scene, config, K, D)
    selection = select_source(classified, args.variant)

    report = {
        "schema_version": 1,
        "purpose": "offline_source_inspection",
        "input_scene": str(args.scene.resolve()),
        "geometry_config": str(args.config.resolve()),
        "selection": selection,
        "classified_scene": classified,
        "execution_allowed": False,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write("\n")

    print(f"Requested variant: {args.variant}")
    print(f"Status: {selection['status']}")
    print(f"Reason: {selection['reason']}")
    print(f"Candidate: {selection['candidate_id']}")
    print("Execution allowed: False")
    print(f"Report: {args.output.resolve()}")


if __name__ == "__main__":
    main()
