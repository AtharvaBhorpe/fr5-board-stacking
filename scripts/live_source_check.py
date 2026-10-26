"""Live image diagnostics only. No ROS or robot commands."""
import json
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from perceive_scene import perceive, annotate

device = Path(
    "/dev/v4l/by-id/"
    "usb-046d_HD_Pro_Webcam_C920_E8F8C89F-video-index0"
).resolve(strict=True)

output = Path("artifacts/live-source")
output.mkdir(parents=True, exist_ok=True)
cap = cv2.VideoCapture(str(device), cv2.CAP_V4L2)
window = "Live source check"

try:
    if not cap.isOpened():
        raise RuntimeError("Cannot open C920; close other camera applications.")

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
    roi = None
    print("Select source region when prompted; ENTER confirms.")
    print("S saves raw image + annotated image + scene JSON.")
    print("R selects a new region. Q exits. Click image before keys.")

    while True:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("Camera acquisition failed.")
        if frame.shape[:2] != (1080, 1920):
            raise RuntimeError("Expected actual resolution 1920x1080.")

        autofocus = cap.get(cv2.CAP_PROP_AUTOFOCUS)
        focus = cap.get(cv2.CAP_PROP_FOCUS)
        if autofocus != 0 or focus != 15:
            raise RuntimeError(
                f"Camera settings changed: autofocus={autofocus}, focus={focus}"
            )

        if roi is None:
            x, y, w, h = cv2.selectROI(
                window, frame, showCrosshair=True, fromCenter=False
            )
            if w == 0 or h == 0:
                break
            roi = (int(x), int(y), int(x + w), int(y + h))

        scene = perceive(frame, roi)
        scene.update({
            "mode": "live_camera_diagnostic",
            "historical_image": False,
            "host_processed_at": datetime.now().astimezone().isoformat(),
            "camera_device": str(device),
            "focus_readback": focus,
            "autofocus_readback": autofocus,
            "execution_allowed": False,
        })
        overlay = annotate(frame, scene)

        for candidate in scene["source_candidates"]:
            centre = np.asarray(candidate["outline_px"]).mean(axis=0)
            candidate["diagnostic_centre_px"] = centre.tolist()
            u, v = np.rint(centre).astype(int)
            cv2.drawMarker(
                overlay, (u, v), (255, 0, 255),
                cv2.MARKER_CROSS, 20, 2,
            )
            cv2.putText(
                overlay, f"pixel ({u}, {v})", (u, v + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2,
            )

        cv2.imshow(window, overlay)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            break
        if key in (ord("r"), ord("R")):
            roi = None
        if key in (ord("s"), ord("S")):
            stem = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            raw_path = output / f"{stem}-raw.png"
            overlay_path = output / f"{stem}-overlay.png"
            for path, image in ((raw_path, frame), (overlay_path, overlay)):
                if not cv2.imwrite(str(path), image):
                    raise RuntimeError(f"Cannot save {path}")
            scene["image"] = str(raw_path.resolve())
            (output / f"{stem}-scene.json").write_text(
                json.dumps(scene, indent=2, allow_nan=False) + "\n",
                encoding="utf-8",
            )
            print(f"Saved {raw_path}")
            for candidate in scene["source_candidates"]:
                print(candidate["candidate_id"],
                      candidate["colour"],
                      candidate["diagnostic_centre_px"])
finally:
    cap.release()
    cv2.destroyAllWindows()
