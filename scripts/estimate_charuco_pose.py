"""Offline ChArUco pose diagnostics; never updates operational calibration."""

import argparse
import json
from pathlib import Path
import tarfile

import cv2
import numpy as np

from load_perception_geometry import load_geometry


def read_inputs(args):
    if args.image is not None:
        frame = cv2.imread(str(args.image))
        if frame is None:
            raise RuntimeError(f"Cannot read {args.image}")
        yield str(args.image), frame
        return

    with tarfile.open(args.archive, "r:*") as archive:
        members = sorted(
            (
                m for m in archive.getmembers()
                if m.isfile()
                and Path(m.name).suffix.lower() in (".png", ".jpg", ".jpeg")
            ),
            key=lambda m: m.name,
        )
        if not members:
            raise RuntimeError("No images found in archive")

        for member in members:
            # Read image bytes directly; do not extract archive paths.
            with archive.extractfile(member) as handle:
                raw = np.frombuffer(handle.read(), dtype=np.uint8)
            frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if frame is None:
                raise RuntimeError(f"Cannot decode {member.name}")
            yield f"{args.archive}::{member.name}", frame


def main():
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path)
    source.add_argument("--archive", type=Path)
    parser.add_argument(
        "--config", type=Path, default=Path("config/perception_geometry.yaml")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("artifacts/charuco-pose")
    )
    args = parser.parse_args()

    config, K, D = load_geometry(args.config)
    spec = config["calibration_board"]
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, spec["dictionary"])
    )
    board = cv2.aruco.CharucoBoard(
        tuple(spec["squares"]),
        spec["square_side_m"],
        spec["marker_side_m"],
        dictionary,
    )
    parameters = cv2.aruco.CharucoParameters()
    parameters.cameraMatrix = K
    parameters.distCoeffs = D
    detector = cv2.aruco.CharucoDetector(board, parameters)
    board_points = np.asarray(
        board.getChessboardCorners(), dtype=np.float64
    )

    args.output.mkdir(parents=True, exist_ok=True)
    reports = []

    for index, (source_name, frame) in enumerate(read_inputs(args), 1):
        if [frame.shape[1], frame.shape[0]] != config["camera"]["image_size_px"]:
            raise RuntimeError(f"Calibration resolution mismatch: {source_name}")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, marker_corners, marker_ids = detector.detectBoard(gray)
        count = 0 if ids is None else len(ids)

        record = {
            "source": source_name,
            "opencv_version": cv2.__version__,
            "calibration_board": spec,
            "corner_count": count,
            "status": "insufficient_corners",
            "solutions": [],
            "pose_ambiguity_resolved": False,
            "workspace_transform_established": False,
            "independent_accuracy_validated": False,
            "execution_allowed": False,
        }
        annotated = frame.copy()
        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(
                annotated, marker_corners, marker_ids
            )
        if ids is not None:
            cv2.aruco.drawDetectedCornersCharuco(
                annotated, corners, ids
            )

        # Eight is a diagnostic minimum, not an accuracy acceptance threshold.
        if count >= 8:
            indices = ids.reshape(-1).astype(int)
            if (
                len(set(indices.tolist())) != count
                or np.any(indices < 0)
                or np.any(indices >= len(board_points))
            ):
                raise RuntimeError("Invalid or duplicate ChArUco corner IDs")

            object_points = np.ascontiguousarray(board_points[indices])
            image_points = np.ascontiguousarray(
                corners.reshape(-1, 2), dtype=np.float64
            )
            record["corner_ids"] = indices.tolist()
            record["corners_px"] = image_points.tolist()

            spread = object_points[:, :2] - object_points[:, :2].mean(axis=0)
            if np.linalg.matrix_rank(spread) < 2:
                record["status"] = "collinear_corners"
            else:
                result = cv2.solvePnPGeneric(
                    object_points, image_points, K, D,
                    flags=cv2.SOLVEPNP_IPPE,
                )
                record["status"] = "no_valid_pose"
                if result[0]:
                    for rvec, tvec in zip(result[1], result[2]):
                        if not (
                            np.isfinite(rvec).all()
                            and np.isfinite(tvec).all()
                        ):
                            continue
                        rotation, _ = cv2.Rodrigues(rvec)
                        camera_points = (
                            rotation @ object_points.T
                            + tvec.reshape(3, 1)
                        ).T
                        if np.any(camera_points[:, 2] <= 0):
                            continue

                        projected, _ = cv2.projectPoints(
                            object_points, rvec, tvec, K, D
                        )
                        residual = (
                            projected.reshape(-1, 2) - image_points
                        )
                        errors = np.linalg.norm(residual, axis=1)

                        T_camera_board = np.eye(4)
                        T_camera_board[:3, :3] = rotation
                        T_camera_board[:3, 3] = tvec.reshape(3)

                        record["solutions"].append({
                            "rvec": rvec.reshape(3).tolist(),
                            "tvec_m": tvec.reshape(3).tolist(),
                            "corner_rms_px": float(
                                np.sqrt(np.mean(errors ** 2))
                            ),
                            "max_corner_error_px": float(errors.max()),
                            "T_camera_board": T_camera_board.tolist(),
                            "T_board_camera": np.linalg.inv(
                                T_camera_board
                            ).tolist(),
                        })

        record["solutions"].sort(key=lambda s: s["corner_rms_px"])
        if record["solutions"]:
            record["status"] = "diagnostic_pose_candidates"
            best = record["solutions"][0]
            cv2.drawFrameAxes(
                annotated, K, D,
                np.array(best["rvec"]),
                np.array(best["tvec_m"]),
                spec["square_side_m"],
                2,
            )

        caption = f"Corners {count}/24 | diagnostic only"
        cv2.putText(
            annotated, caption, (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2,
        )
        output_image = args.output / f"{index:03d}-pose.png"
        if not cv2.imwrite(str(output_image), annotated):
            raise RuntimeError(f"Cannot write {output_image}")
        record["annotated_image"] = str(output_image)
        reports.append(record)

        errors = [
            round(s["corner_rms_px"], 3) for s in record["solutions"]
        ]
        print(
            f"{index:03d}: corners={count}/24; "
            f"RMS candidates={errors}; {record['status']}",
            flush=True,
        )

    report_path = args.output / "report.json"
    report_path.write_text(
        json.dumps(reports, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    usable = sum(bool(r["solutions"]) for r in reports)
    print(f"\nPose candidates found in {usable}/{len(reports)} images.")
    print(f"Report: {report_path.resolve()}")
    print("Axes show the lowest-error candidate, not an approved transform.")
    print("Operational calibration was not modified.")


if __name__ == "__main__":
    main()
