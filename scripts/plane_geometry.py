"""Offline pixel-to-plane geometry. No ROS, TF publication or robot commands."""

import cv2
import numpy as np


def validate_transform(value):
    """T_workcell_camera maps camera optical coordinates into workcell."""
    if value is None:
        raise ValueError("Camera-to-workcell transform is not calibrated")

    transform = np.asarray(value, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("Transform must be a finite 4x4 matrix")
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
        raise ValueError("Invalid homogeneous-transform bottom row")

    rotation = transform[:3, :3]
    if not np.allclose(
        rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0
    ):
        raise ValueError("Rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1, atol=1e-6, rtol=0):
        raise ValueError("Rotation must have determinant +1")

    return transform


def pixels_to_plane(pixels, K, D, T_workcell_camera, plane_z_m):
    """Intersect raw distorted-image rays with workcell plane z=plane_z_m.

    Camera optical axes: x right, y down, z forward.
    Workcell axes: right-handed, z up.
    Inputs and outputs use metres. Returns an Nx3 array.

    Requires a horizontal surface in the chosen workcell frame.
    Does not establish calibration quality or authorize execution.
    """
    transform = validate_transform(T_workcell_camera)
    if plane_z_m is None or not np.isfinite(plane_z_m):
        raise ValueError("A measured finite plane height is required")

    pixels = np.asarray(pixels, dtype=np.float64)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or len(pixels) == 0:
        raise ValueError("Pixels must be a nonempty Nx2 array")
    if not np.isfinite(pixels).all():
        raise ValueError("Pixels contain non-finite values")

    K = np.asarray(K, dtype=np.float64)
    D = np.asarray(D, dtype=np.float64).reshape(-1)
    if (
        K.shape != (3, 3)
        or not np.isfinite(K).all()
        or K[0, 0] <= 0
        or K[1, 1] <= 0
        or not np.allclose(K[2], [0, 0, 1], atol=1e-9, rtol=0)
    ):
        raise ValueError("Invalid camera intrinsic matrix")
    if D.size not in (4, 5, 8, 12, 14) or not np.isfinite(D).all():
        raise ValueError("Invalid OpenCV distortion coefficients")

    normalized = cv2.undistortPoints(
        pixels.reshape(-1, 1, 2), K, D
    ).reshape(-1, 2)
    camera_rays = np.column_stack((normalized, np.ones(len(pixels))))

    rotation = transform[:3, :3]
    camera_origin = transform[:3, 3]
    workcell_rays = (rotation @ camera_rays.T).T

    # A near-horizontal ray has an unstable intersection with a horizontal plane.
    lengths = np.linalg.norm(workcell_rays, axis=1)
    if np.any(np.abs(workcell_rays[:, 2]) / lengths < 1e-6):
        raise ValueError("A viewing ray is parallel or too close to the plane")

    scale = (float(plane_z_m) - camera_origin[2]) / workcell_rays[:, 2]
    if np.any(scale <= 0):
        raise ValueError("Plane intersection is at or behind the camera")

    points = camera_origin + scale[:, None] * workcell_rays
    if not np.isfinite(points).all():
        raise ValueError("Invalid plane intersection")
    return points
