"""Diagnostic XY validation using independent points at measured heights."""

import argparse
import json
from pathlib import Path

import numpy as np

from load_perception_geometry import load_geometry
from plane_geometry import pixels_to_plane, validate_transform


def evaluate_points(points, K, D, transform, image_size):
    transform = validate_transform(transform)
    if len(points) < 3:
        raise ValueError("Provide at least three independent reference points")

    width, height = image_size
    records = []
    identifiers = set()

    for point in points:
        identifier = point["id"]
        if not isinstance(identifier, str) or not identifier.strip():
            raise ValueError("Each point requires a nonempty string ID")
        if identifier in identifiers:
            raise ValueError(f"Duplicate point ID: {identifier}")
        identifiers.add(identifier)

        if point.get("used_for_transform_fit") is not False:
            raise ValueError(
                f"{identifier}: explicitly declare this point was not "
                "used for fitting or choosing the transform"
            )

        zone = point.get("zone")
        if zone not in ("source", "profile", "destination"):
            raise ValueError(f"{identifier}: unknown workspace zone")

        pixel = np.asarray(point["pixel_xy"], dtype=np.float64)
        measured = np.asarray(point["measured_xyz_m"], dtype=np.float64)
        if pixel.shape != (2,) or not np.isfinite(pixel).all():
            raise ValueError(f"{identifier}: invalid pixel coordinates")
        if measured.shape != (3,) or not np.isfinite(measured).all():
            raise ValueError(f"{identifier}: invalid measured coordinates")
        if not (0 <= pixel[0] < width and 0 <= pixel[1] < height):
            raise ValueError(f"{identifier}: pixel outside image")

        predicted = pixels_to_plane(
            pixel.reshape(1, 2), K, D, transform, measured[2]
        )[0]
        delta_mm = (predicted[:2] - measured[:2]) * 1000
        error_mm = float(np.linalg.norm(delta_mm))

        records.append({
            "id": identifier,
            "zone": zone,
            "pixel_xy": pixel.tolist(),
            "measured_xyz_m": measured.tolist(),
            "predicted_xyz_m": predicted.tolist(),
            "delta_xy_mm": delta_mm.tolist(),
            "xy_error_mm": error_mm,
        })

    errors = np.asarray([r["xy_error_mm"] for r in records])
    zones = sorted({r["zone"] for r in records})
    return {
        "schema_version": 1,
        "purpose": "independent_xy_diagnostic",
        "point_count": len(records),
        "zones_observed": zones,
        "zones_missing": sorted(
            {"source", "profile", "destination"} - set(zones)
        ),
        "mean_xy_error_mm": float(errors.mean()),
        "rms_xy_error_mm": float(np.sqrt(np.mean(errors ** 2))),
        "max_xy_error_mm": float(errors.max()),
        "points": records,
        "independence": "declared_by_operator_not_programmatically_verified",
        "z_accuracy_tested": False,
        "calibration_approved": False,
        "execution_allowed": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("measurements", type=Path)
    parser.add_argument(
        "--config", type=Path, default=Path("config/perception_geometry.yaml")
    )
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    data = json.loads(args.measurements.read_text(encoding="utf-8"))
    config, K, D = load_geometry(args.config)

    if data["schema_version"] != 1:
        raise ValueError("Unsupported measurement schema")
    if data["coordinate_space"] != "original_distorted_image_pixels":
        raise ValueError("Use original distorted image pixels")
    if data["workcell_frame_id"] != config["workcell"]["frame_id"]:
        raise ValueError("Workcell frame mismatch")
    if data["image_size_px"] != config["camera"]["image_size_px"]:
        raise ValueError("Image size differs from calibration")
    if not isinstance(data.get("source_image"), str) or not data["source_image"]:
        raise ValueError("Record the source image path")
    if not isinstance(data.get("measurement_method"), str) or not data["measurement_method"]:
        raise ValueError("Describe how reference coordinates were measured")

    # Explicit candidate transform: validation does not require approving it first.
    report = evaluate_points(
        data["points"], K, D, data["T_workcell_camera"],
        data["image_size_px"],
    )
    report.update({
        "measurements_file": str(args.measurements.resolve()),
        "source_image": data["source_image"],
        "measurement_method": data["measurement_method"],
        "workcell_frame_id": data["workcell_frame_id"],
        "T_workcell_camera": data["T_workcell_camera"],
        "camera_matrix": K.tolist(),
        "distortion_coefficients": D.tolist(),
        "image_size_px": data["image_size_px"],
        "geometry_config": str(args.config.resolve()),
        "intrinsics_workspace_validated": (
            config["camera"]["intrinsics_workspace_validated"]
        ),
    })

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write("\n")

    for point in report["points"]:
        dx, dy = point["delta_xy_mm"]
        print(
            f"{point['id']}: dx={dx:+.3f} mm, dy={dy:+.3f} mm, "
            f"XY error={point['xy_error_mm']:.3f} mm"
        )
    print(f"RMS XY error: {report['rms_xy_error_mm']:.3f} mm")
    print(f"Maximum XY error: {report['max_xy_error_mm']:.3f} mm")
    print(f"Zones missing: {report['zones_missing']}")
    print("Diagnostic only. Z accuracy and robot placement are not validated.")
    print(f"Report: {args.output.resolve()}")


if __name__ == "__main__":
    main()
