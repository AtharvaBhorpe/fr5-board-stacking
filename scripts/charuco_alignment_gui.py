"""Camera-only ChArUco alignment aid. No ROS TF or robot commands."""
import json
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from load_perception_geometry import load_geometry

config, K, D = load_geometry("config/perception_geometry.yaml")
spec = config["calibration_board"]
board = cv2.aruco.CharucoBoard(
    tuple(spec["squares"]),
    spec["square_side_m"],
    spec["marker_side_m"],
    cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, spec["dictionary"])
    ),
)
parameters = cv2.aruco.CharucoParameters()
parameters.cameraMatrix = K
parameters.distCoeffs = D
detector = cv2.aruco.CharucoDetector(board, parameters)
model = np.asarray(board.getChessboardCorners(), dtype=np.float64)
reference = model[0]
axis_length = spec["square_side_m"] * 2

device = Path(
    "/dev/v4l/by-id/"
    "usb-046d_HD_Pro_Webcam_C920_692E0F9F-video-index0"
).resolve(strict=True)

cap = cv2.VideoCapture(str(device), cv2.CAP_V4L2)
window = "ChArUco alignment - camera only"
output = Path("artifacts/workspace-calibration")

def label(image, text, pixel, colour):
    point = tuple(np.rint(pixel).astype(int))
    cv2.putText(
        image, text, point, cv2.FONT_HERSHEY_SIMPLEX,
        0.7, (0, 0, 0), 5, cv2.LINE_AA,
    )
    cv2.putText(
        image, text, point, cv2.FONT_HERSHEY_SIMPLEX,
        0.7, colour, 2, cv2.LINE_AA,
    )

try:
    if not cap.isOpened():
        raise RuntimeError("Cannot open C920; close other camera apps.")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 15)
    if not cap.set(cv2.CAP_PROP_AUTOFOCUS, 0):
        raise RuntimeError("Cannot disable autofocus.")
    if not cap.set(cv2.CAP_PROP_FOCUS, 15):
        raise RuntimeError("Cannot request focus 15.")

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 720)
    print("Show ONE full ChArUco board.")
    print("S: save capture. Q/Escape: quit. Click image before keys.")
    print("Measure printed corner C0, not the paper edge.")
    print("C0 board-model XYZ metres:", reference.tolist())

    while True:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("Camera frame acquisition failed.")
        if [frame.shape[1], frame.shape[0]] != config["camera"]["image_size_px"]:
            raise RuntimeError("Unexpected camera resolution.")
        focus = cap.get(cv2.CAP_PROP_FOCUS)
        autofocus = cap.get(cv2.CAP_PROP_AUTOFOCUS)
        if focus != 15 or autofocus != 0:
            raise RuntimeError(
                f"Unexpected settings: focus={focus}, autofocus={autofocus}"
            )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, marker_corners, marker_ids = detector.detectBoard(gray)
        overlay = frame.copy()
        count = 0 if ids is None else len(ids)
        solutions = []

        if ids is not None:
            cv2.aruco.drawDetectedCornersCharuco(overlay, corners, ids)
            indices = ids.flatten().astype(int)
            reference_indices = np.flatnonzero(indices == 0)
            if len(reference_indices):
                c0_pixel = corners.reshape(-1, 2)[reference_indices[0]]
                cv2.circle(
                    overlay, tuple(np.rint(c0_pixel).astype(int)),
                    12, (255, 0, 255), 3,
                )
                label(overlay, "C0: MEASURE HERE",
                      c0_pixel + [15, -15], (255, 0, 255))

            if count >= 8:
                obj = np.ascontiguousarray(model[indices])
                img = np.ascontiguousarray(
                    corners.reshape(-1, 2), dtype=np.float64
                )
                if np.linalg.matrix_rank(obj[:, :2] - obj[:, :2].mean(0)) == 2:
                    result = cv2.solvePnPGeneric(
                        obj, img, K, D, flags=cv2.SOLVEPNP_IPPE
                    )
                    if result[0]:
                        for rvec, tvec in zip(result[1], result[2]):
                            if not (np.isfinite(rvec).all()
                                    and np.isfinite(tvec).all()):
                                continue
                            R, _ = cv2.Rodrigues(rvec)
                            if np.any((R @ obj.T + tvec.reshape(3, 1))[2] <= 0):
                                continue
                            projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
                            residual = projected.reshape(-1, 2) - img
                            rms = float(np.sqrt(np.mean(
                                np.sum(residual ** 2, axis=1)
                            )))
                            solutions.append((rms, rvec, tvec))

        solutions.sort(key=lambda item: item[0])
        if solutions:
            _, rvec, tvec = solutions[0]
            # Axes are shifted to C0 for easy physical identification.
            axes = np.vstack([
                reference,
                reference + [axis_length, 0, 0],
                reference + [0, axis_length, 0],
                reference + [0, 0, axis_length],
            ])
            projected, _ = cv2.projectPoints(axes, rvec, tvec, K, D)
            pixels = projected.reshape(-1, 2)
            start = tuple(np.rint(pixels[0]).astype(int))
            for i, (name, colour) in enumerate((
                ("+X", (0, 0, 255)),
                ("+Y", (0, 255, 0)),
                ("+Z", (255, 0, 0)),
            ), 1):
                end = tuple(np.rint(pixels[i]).astype(int))
                cv2.arrowedLine(overlay, start, end, colour, 3, tipLength=0.12)
                label(overlay, name, pixels[i], colour)

        errors = ", ".join(f"{s[0]:.3f}" for s in solutions) or "none"
        label(overlay, f"Corners {count}/{len(model)} | RMS px: {errors}",
              [20, 35], (0, 255, 255))
        label(overlay, "BOARD AXES AT C0 - NOT BASE_LINK TF | S save | Q quit",
              [20, 70], (0, 255, 255))
        cv2.imshow(window, overlay)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            break
        if key in (ord("s"), ord("S")):
            if count != len(model) or not solutions:
                print("Not saved: show all board corners clearly.")
                continue

            folder = output / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            folder.mkdir(parents=True, exist_ok=False)
            for name, image in (("raw.png", frame), ("axes.png", overlay)):
                if not cv2.imwrite(str(folder / name), image):
                    raise RuntimeError(f"Cannot save {folder / name}")
            metadata = {
                "reference_corner_id": 0,
                "reference_corner_board_xyz_m": reference.tolist(),
                "calibration_board": spec,
                "focus_readback": focus,
                "autofocus_readback": autofocus,
                "image_size_px": [frame.shape[1], frame.shape[0]],
                "corner_ids": ids.flatten().tolist(),
                "corners_px": corners.reshape(-1, 2).tolist(),
                "candidate_rms_px": [s[0] for s in solutions],
                "camera_matrix": K.tolist(),
                "distortion_coefficients": D.tolist(),
                "pose_ambiguity_resolved": False,
                "execution_allowed": False,
            }
            (folder / "capture.json").write_text(
                json.dumps(metadata, indent=2, allow_nan=False) + "\n",
                encoding="utf-8",
            )
            print(f"\nSAVED: {folder.resolve()}", flush=True)
finally:
    cap.release()
    cv2.destroyAllWindows()
