"""Load calibration and convert pixels to diagnostic surface points."""

from pathlib import Path

import numpy as np
import yaml

from plane_geometry import pixels_to_plane, validate_transform


ROOT = Path(__file__).resolve().parents[1]


class IncompleteCalibration(ValueError):
    pass


def load_geometry(config_path):
    config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    if config["schema_version"] != 1:
        raise ValueError("Unsupported geometry schema")
    if config["units"] != {"length": "metres", "angle": "radians"}:
        raise ValueError("Expected metres and radians")

    camera = config["camera"]
    if camera["intrinsics_path_relative_to"] != "repository_root":
        raise ValueError("Unsupported intrinsics path convention")

    calibration_path = ROOT / camera["intrinsics_file"]
    calibration = yaml.safe_load(
        calibration_path.read_text(encoding="utf-8")
    )
    if calibration["distortion_model"] != "plumb_bob":
        raise ValueError("Expected plumb_bob camera calibration")

    size = [calibration["image_width"], calibration["image_height"]]
    if size != camera["image_size_px"]:
        raise ValueError("Intrinsic and geometry image sizes differ")
    if not all(type(value) is int and value > 0 for value in size):
        raise ValueError("Image dimensions must be positive integers")

    matrix = calibration["camera_matrix"]
    distortion = calibration["distortion_coefficients"]
    if (matrix["rows"], matrix["cols"]) != (3, 3):
        raise ValueError("Camera matrix must be 3x3")
    if distortion["rows"] * distortion["cols"] != len(distortion["data"]):
        raise ValueError("Distortion coefficient dimensions are inconsistent")

    K = np.asarray(matrix["data"], dtype=np.float64).reshape(3, 3)
    D = np.asarray(distortion["data"], dtype=np.float64).reshape(-1)
    if (
        not np.isfinite(K).all()
        or K[0, 0] <= 0
        or K[1, 1] <= 0
        or not np.allclose(K[2], [0, 0, 1], atol=1e-9, rtol=0)
    ):
        raise ValueError("Invalid camera matrix")
    if D.size != 5 or not np.isfinite(D).all():
        raise ValueError("Expected five finite plumb_bob coefficients")

    # Missing transforms are valid in an unfinished configuration.
    # Present but malformed transforms are not.
    for value in (
        camera["T_workcell_camera"],
        config["workcell"]["T_base_link_workcell"],
    ):
        if value is not None:
            validate_transform(value)

    return config, K, D


def surface_points(config, K, D, pixels, image_size, surface, variant):
    """Diagnostic points on the top of one flat board.

    surface: source, profile or destination.
    Input pixels must be from the raw distorted image, at its original size.
    This does not compute a suction TCP pose or authorize robot execution.
    """
    if surface not in ("source", "profile", "destination"):
        raise ValueError("Surface must be source, profile or destination")
    if variant not in config["variants"]:
        raise ValueError(f"Unknown variant: {variant}")
    if list(image_size) != config["camera"]["image_size_px"]:
        raise ValueError("Input image resolution differs from calibration")

    camera = config["camera"]
    surfaces = config["surfaces"]
    if surface == "destination":
        support_z = surfaces["destination_support_z_m"][variant]
    else:
        support_z = surfaces[f"{surface}_support_z_m"]

    missing = []
    if camera["intrinsics_workspace_validated"] is not True:
        missing.append("workspace validation of camera intrinsics/settings")
    if camera["T_workcell_camera"] is None:
        missing.append("T_workcell_camera")
    if support_z is None:
        missing.append(f"{surface} support height")
    if missing:
        raise IncompleteCalibration("Missing: " + "; ".join(missing))

    dimensions = np.asarray(
        config["variants"][variant]["size_m"], dtype=np.float64
    )
    if (
        dimensions.shape != (3,)
        or not np.isfinite(dimensions).all()
        or np.any(dimensions <= 0)
    ):
        raise ValueError("Board dimensions must be three positive lengths")
    if isinstance(support_z, bool) or not np.isfinite(support_z):
        raise ValueError("Support height must be a finite number")

    points = np.asarray(pixels, dtype=np.float64)
    width, height = image_size
    if (
        points.ndim != 2 or points.shape[1] != 2 or len(points) == 0
        or not np.isfinite(points).all()
    ):
        raise ValueError("Pixels must be a finite nonempty Nx2 array")
    if np.any(
        (points[:, 0] < 0) | (points[:, 0] >= width)
        | (points[:, 1] < 0) | (points[:, 1] >= height)
    ):
        raise ValueError("Pixel lies outside the calibrated image")

    top_z = float(support_z) + float(dimensions[2])
    return pixels_to_plane(
        points, K, D, camera["T_workcell_camera"], top_z
    )
