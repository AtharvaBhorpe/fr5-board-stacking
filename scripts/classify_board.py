"""Classify a single board from ordered metric corners. No robot targets."""

import numpy as np


VARIANTS = {
    "VARa": ("white", (0.210, 0.295)),
    "VARb": ("white", (0.135, 0.210)),
    "VARc": ("red", (0.135, 0.210)),
}


def classify_board(corners_m, colour, dimension_tolerance_m=0.005):
    """Corners: four perimeter-ordered XYZ points in a Z-up metric frame.

    Tolerances are provisional and require validation on real observations.
    This function does not establish calibration validity or single-sheet pickup.
    """
    result = {
        "variant": None,
        "status": "rejected",
        "reason": None,
        "measured_sides_m": None,
        "matching_variants": [],
        "execution_allowed": False,
    }

    def reject(reason):
        result["reason"] = reason
        return result

    if (
        isinstance(dimension_tolerance_m, bool)
        or not np.isscalar(dimension_tolerance_m)
        or not np.isfinite(dimension_tolerance_m)
        or dimension_tolerance_m <= 0
    ):
        raise ValueError("Dimension tolerance must be finite and positive")

    try:
        points = np.asarray(corners_m, dtype=np.float64)
    except (TypeError, ValueError):
        return reject("invalid_coordinates")

    if points.shape != (4, 3) or not np.isfinite(points).all():
        return reject("expected_four_finite_xyz_corners")

    if colour not in ("white", "red"):
        return reject("unsupported_colour")

    if np.ptp(points[:, 2]) > 0.001:
        return reject("corners_not_on_horizontal_plane")

    xy = points[:, :2]
    edges = np.roll(xy, -1, axis=0) - xy
    lengths = np.linalg.norm(edges, axis=1)
    if np.any(lengths < 1e-6):
        return reject("degenerate_edge")

    # Strictly convex perimeter ordering; either winding direction is allowed.
    following = np.roll(edges, -1, axis=0)
    turns = edges[:, 0] * following[:, 1] - edges[:, 1] * following[:, 0]
    if not (np.all(turns > 1e-10) or np.all(turns < -1e-10)):
        return reject("nonconvex_or_unordered_corners")

    unit_edges = edges / lengths[:, None]
    adjacent_cosines = np.sum(
        unit_edges * np.roll(unit_edges, -1, axis=0), axis=1
    )
    if np.any(np.abs(adjacent_cosines) > 0.10):
        return reject("corners_not_rectangular")

    if (
        abs(lengths[0] - lengths[2]) > dimension_tolerance_m
        or abs(lengths[1] - lengths[3]) > dimension_tolerance_m
    ):
        return reject("opposite_edges_disagree")

    # Pair opposite edges, then remove dependence on board yaw.
    sides = np.sort([
        (lengths[0] + lengths[2]) / 2,
        (lengths[1] + lengths[3]) / 2,
    ])
    result["measured_sides_m"] = sides.tolist()

    matches = []
    for variant, (expected_colour, expected_sides) in VARIANTS.items():
        if colour != expected_colour:
            continue
        expected_sides = np.asarray(expected_sides)
        if np.all(np.abs(sides - expected_sides) <= dimension_tolerance_m):
            matches.append(variant)

    result["matching_variants"] = matches
    if not matches:
        return reject("dimensions_do_not_match")
    if len(matches) != 1:
        return reject("ambiguous_variant")

    result.update({
        "variant": matches[0],
        "status": "matched",
        "reason": None,
    })
    return result
