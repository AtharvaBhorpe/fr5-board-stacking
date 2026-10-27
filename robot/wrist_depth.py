"""Wrist D405 depth -> top of the board under the tool and its centre, in base_link (dynamic pick / place height).

    from wrist_depth import WristDepth
    wd = WristDepth(node)                 # node: any rclpy node that the caller spins (spin_once is used here)
    m = wd.measure(x, y)                  # None if no cloud / TF; else dict, see measure()

    pixi run python robot/wrist_depth.py  # one measurement under the current TCP, printed

Uses /wrist/wrist_camera/depth/color/points (pixi run wrist) and TF (robot running). Accuracy = camera pose on the
robot model (CAD + mount yaw + roll, checked 2026-10-27, no hand-eye calibration): height ~mm, x / y a few mm.
"""
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

TOPIC = "/wrist/wrist_camera/depth/color/points"
PATCH = 0.035      # m radius under the tool used for the board top height (fits inside the smallest board)
SEARCH = 0.25      # m radius searched for the board outline
PLANE_TOL = 0.002  # m: points within this of the board top belong to it (D405 noise ~0.5 mm here; boards ~5 mm)


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def xyz(msg):
    """PointCloud2 (float32 x, y, z) -> (n, 3), NaN / zero points dropped."""
    off = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(msg.data, np.uint8).reshape(-1, msg.point_step)
    p = np.stack([raw[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in "xyz"], 1)
    return p[np.isfinite(p).all(1) & (p[:, 2] > 0.01)]


class WristDepth:
    def __init__(self, node):
        self.node = node
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        self.last = None
        node.create_subscription(PointCloud2, TOPIC, lambda m: setattr(self, "last", m), qos_profile_sensor_data)

    def measure(self, x, y, timeout=3.0):
        """Board top under base_link (x, y). -> {'top_z', 'table_z', 'centre': (x, y) | None, 'size': (a, b) | None,
        'points'} or None (no cloud / TF within timeout, or nothing under the tool)."""
        t0, self.last = time.time(), None
        while time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.last is not None and self.tf.can_transform("base_link", self.last.header.frame_id,
                                                               rclpy.time.Time()):
                break
        else:
            return None
        m = self.last
        T = self.tf.lookup_transform("base_link", m.header.frame_id, rclpy.time.Time())
        R = quat_to_R(T.transform.rotation)
        t = np.array([T.transform.translation.x, T.transform.translation.y, T.transform.translation.z])
        p = xyz(m) @ R.T + t
        d = np.hypot(p[:, 0] - x, p[:, 1] - y)
        patch = p[d < PATCH]
        if len(patch) < 30:
            return None
        top = float(np.median(patch[:, 2]))
        near = p[d < SEARCH]
        table = float(np.percentile(near[:, 2], 5))  # lowest dense surface around it
        board = near[np.abs(near[:, 2] - top) < PLANE_TOL]
        centre = size = None
        if len(board) > 100:
            c = board[:, :2].mean(0)
            # ponytail: centroid + PCA extent of all points at board-top height nearby; a touching board at the same
            # height merges in (upgrade: connected component in the image)
            e, v = np.linalg.eigh(np.cov((board[:, :2] - c).T))
            proj = (board[:, :2] - c) @ v
            size = tuple(np.round(proj.max(0) - proj.min(0), 3)[::-1])
            centre = (float(c[0]), float(c[1]))
        return {"top_z": top, "table_z": table, "centre": centre, "size": size, "points": int(len(patch))}


def main():
    rclpy.init()
    n = rclpy.create_node("wrist_depth")
    wd = WristDepth(n)
    t0 = time.time()
    while not wd.tf.can_transform("base_link", "tcp", rclpy.time.Time()) and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    tp = wd.tf.lookup_transform("base_link", "tcp", rclpy.time.Time()).transform.translation
    m = wd.measure(tp.x, tp.y)
    if m is None:
        raise SystemExit("no wrist point cloud / TF (pixi run wrist, robot running?) or nothing under the tool")
    print(f"TCP (cup tip)  x={tp.x:.4f} y={tp.y:.4f} z={tp.z:.4f}")
    print(f"board top z={m['top_z']:.4f} ({m['points']} pts)   table z={m['table_z']:.4f} (known -0.060)  "
          f"board height above table {1000 * (m['top_z'] - m['table_z']):.1f} mm")
    print(f"cup tip -> board top: {1000 * (tp.z - m['top_z']):.1f} mm")
    if m["centre"]:
        dx, dy = m["centre"][0] - tp.x, m["centre"][1] - tp.y
        print(f"board centre x={m['centre'][0]:.4f} y={m['centre'][1]:.4f}, size {m['size']} m; "
              f"centre - TCP: dx={1000 * dx:.1f} dy={1000 * dy:.1f} mm ({1000 * np.hypot(dx, dy):.1f} mm)")


if __name__ == "__main__":
    main()
