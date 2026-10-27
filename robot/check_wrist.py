"""Is the D405 on the robot model where it is on the real robot? Draws into the wrist image what the model predicts.

    pixi run python robot/check_wrist.py      # needs robot (TF), pixi run vision (boards), pixi run wrist (D405)

Boards and templates found by the overhead camera are projected through TF (robot joints -> URDF tool chain ->
d405_camera_link -> wrist_camera_*_optical_frame) into the wrist color image: green = board, red = template, yellow =
the cup tip (TCP). Mount right: the outlines lie on the real boards. Camera on the wrong side of the tool, rolled or
mirrored: they land shifted / rotated / outside. Hover above a board first (pick_one.py --hover-only).
Output: /wrist/check (sensor_msgs/Image, RViz: Add -> By topic) and /tmp/wrist_check.png, once per second.
"""
import json
import math
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "vision1"))
from table import SIZES_MM  # noqa: E402

NS = "/wrist/wrist_camera/color"


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rect(x, y, z, yaw, size_mm):
    """Board / template footprint corners (4, 3) in base_link."""
    a, b = size_mm[0] / 2000, size_mm[1] / 2000
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[x + c * u - s * v, y + s * u + c * v, z] for u, v in ((a, b), (-a, b), (-a, -b), (a, -b))])


def project(K, R, t, pts):
    """base_link points (n, 3) -> pixels (n, 2), in-front mask. R, t: base_link -> optical frame."""
    p = pts @ R.T + t
    front = p[:, 2] > 0.01
    uv = (p @ K.T)[:, :2] / np.maximum(p[:, 2:3], 1e-6)
    return uv, front


def draw_poly(img, uv, front, col, label):
    if not front.all():
        return
    cv2.polylines(img, [np.int32(uv)], True, col, 2)
    c = uv.mean(0)
    cv2.putText(img, label, (int(c[0]) - 30, int(c[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)


def draw_point(img, uv, front, col, label):
    """Point, or an arrow at the image border toward it when it is outside the view."""
    h, w = img.shape[:2]
    u, v = uv
    if front and 0 <= u < w and 0 <= v < h:
        cv2.drawMarker(img, (int(u), int(v)), col, cv2.MARKER_CROSS, 30, 3)
        cv2.putText(img, label, (int(u) + 10, int(v) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        return
    c = np.array([w / 2, h / 2])
    d = (np.array([u, v]) - c) * (1 if front else -1)
    d = d / max(abs(d[0]) / (w / 2 - 25), abs(d[1]) / (h / 2 - 25), 1e-6)
    cv2.arrowedLine(img, tuple(np.int32(c + 0.6 * d)), tuple(np.int32(c + d)), col, 3, tipLength=0.4)
    cv2.putText(img, label + " (outside)", tuple(np.int32(c + 0.45 * d)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)


def main():
    rclpy.init()
    n = rclpy.create_node("check_wrist")
    tf = Buffer()
    TransformListener(tf, n)
    st = {"img": None, "info": None, "boards": None}
    n.create_subscription(Image, f"{NS}/image_raw", lambda m: st.update(img=m), qos_profile_sensor_data)
    n.create_subscription(CameraInfo, f"{NS}/camera_info", lambda m: st.update(info=m), qos_profile_sensor_data)
    n.create_subscription(String, "/vision/boards", lambda m: st.update(boards=json.loads(m.data)), 1)
    pub = n.create_publisher(Image, "/wrist/check", 2)

    def tick():
        m, info, vb = st["img"], st["info"], st["boards"]
        if m is None or info is None:
            n.get_logger().info("waiting for the wrist camera (pixi run wrist)", throttle_duration_sec=5.0)
            return
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()
        if m.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        try:
            T = tf.lookup_transform(m.header.frame_id, "base_link", rclpy.time.Time())
            Tt = tf.lookup_transform(m.header.frame_id, "tcp", rclpy.time.Time())
        except Exception as e:  # noqa: BLE001  TF not there yet: say why, keep trying
            n.get_logger().warn(f"TF {m.header.frame_id} <- base_link / tcp: {e}", throttle_duration_sec=5.0)
            return
        R = quat_to_R(T.transform.rotation)
        t = np.array([T.transform.translation.x, T.transform.translation.y, T.transform.translation.z])
        K = np.array(info.k).reshape(3, 3)
        for b in (vb or {}).get("boards", []):
            if "x" in b:
                uv, f = project(K, R, t, rect(b["x"], b["y"], b["z"], b["yaw"], SIZES_MM.get(b["variant"], b["size_mm"])))
                draw_poly(img, uv, f, (0, 255, 0), b["variant"])
        for d in (vb or {}).get("destinations", []):
            uv, f = project(K, R, t, rect(d["x"], d["y"], d["support_z"], d["yaw"], SIZES_MM[d["variant"]]))
            draw_poly(img, uv, f, (0, 0, 255), d["variant"] + " tpl")
        tp = Tt.transform.translation
        p = np.array([tp.x, tp.y, tp.z])
        draw_point(img, (K @ p)[:2] / max(p[2], 1e-6), p[2] > 0.01, (0, 255, 255), "TCP")
        cv2.putText(img, "model prediction: green boards, red templates, yellow TCP", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        out = Image(height=img.shape[0], width=img.shape[1], encoding="bgr8", step=img.shape[1] * 3,
                    data=img.tobytes())
        out.header = m.header
        pub.publish(out)
        cv2.imwrite("/tmp/wrist_check.png", img)

    n.create_timer(1.0, tick)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
