"""Desired board pose from metric marker corners. No robot commands."""

import math
import numpy as np


def desired_board_pose(corners_m, tag_id, config):
    """Corners must be in ArUco order and expressed in configured base_link."""
    if config["workcell"]["frame_id"] != "base_link":
        raise ValueError("Expected base_link coordinates")

    tag_id = str(tag_id)
    markers = config["destination_markers"]
    entry = markers["placement_offsets"].get(tag_id)
    if not isinstance(entry, dict):
        raise ValueError("Destination offset is unset")

    variant = entry["variant"]
    spec = config["variants"][variant]
    if str(spec["destination_tag_id"]) != tag_id:
        raise ValueError("Variant/tag mismatch")

    points = np.asarray(corners_m, dtype=float)
    if points.shape != (4, 3) or not np.isfinite(points).all():
        raise ValueError("Expected four finite metric XYZ corners")
    if np.ptp(points[:, 2]) > 0.001:
        raise ValueError("Marker corners must lie on a horizontal plane")

    xy = points[:, :2]
    edges = np.roll(xy, -1, axis=0) - xy
    lengths = np.linalg.norm(edges, axis=1)
    side = float(markers["black_square_side_m"])
    if not math.isfinite(side) or side <= 0:
        raise ValueError("Invalid marker size")

    # Provisional diagnostic tolerance; not a physical accuracy guarantee.
    if np.any(np.abs(lengths - side) > 0.005):
        raise ValueError("Metric marker side lengths disagree with configuration")
    if np.any(lengths < 1e-6):
        raise ValueError("Degenerate marker edge")

    following = np.roll(edges, -1, axis=0)
    turns = edges[:, 0] * following[:, 1] - edges[:, 1] * following[:, 0]
    if not (np.all(turns > 1e-10) or np.all(turns < -1e-10)):
        raise ValueError("Marker corners are crossed or nonconvex")

    unit_edges = edges / lengths[:, None]
    cosines = np.sum(
        unit_edges * np.roll(unit_edges, -1, axis=0), axis=1
    )
    if np.any(np.abs(cosines) > 0.10):
        raise ValueError("Metric marker is not approximately rectangular")

    # Average the two edges pointing in the corner-0-to-1 direction.
    x_axis = unit_edges[0] - unit_edges[2]
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.array([-x_axis[1], x_axis[0]])  # Z cross X

    offset = np.asarray(entry["offset_xy_m"], dtype=float)
    if offset.shape != (2,) or not np.isfinite(offset).all():
        raise ValueError("Invalid destination XY offset")

    yaw_offset = entry["board_yaw_rad"]
    support = config["surfaces"]["destination_support_z_m"][variant]
    if yaw_offset is None or support is None:
        raise ValueError("Board orientation or destination support height unset")
    if not np.isfinite([yaw_offset, support]).all():
        raise ValueError("Invalid destination yaw or support height")

    size = np.asarray(spec["size_m"], dtype=float)
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError("Invalid board dimensions")

    centre_xy = xy.mean(axis=0) + offset[0] * x_axis + offset[1] * y_axis
    yaw = math.atan2(x_axis[1], x_axis[0]) + float(yaw_offset)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))

    return {
        "purpose": "desired_board_pose_diagnostic",
        "frame_id": "base_link",
        "tag_id": int(tag_id),
        "variant": variant,
        "board_centre_xyz_m": [
            float(centre_xy[0]),
            float(centre_xy[1]),
            float(support + size[2] / 2),
        ],
        "board_yaw_rad": yaw,
        "board_top_z_m": float(support + size[2]),
        "physical_accuracy_validated": False,
        "execution_allowed": False,
    }
