"""vision_transform node: /vision/pixel_to_table (camera pixels -> table x / y in base_link), decision-log V-8.

Needs no camera and no tags: uses the stored calibration (config/calib.yaml, written by the calibration app).
The file is re-checked every reload_period and reloaded when it changes, so a new calibration is live without a
restart. Missing / placeholder / bad file -> every request answers success = false with the reason (the node never
keeps serving an old calibration that the file no longer holds).

    ros2 run hk_fr5_vision vision_transform [--ros-args --params-file <share>/config/vision.yaml]
"""
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hk_fr5_vision import calib
from hk_fr5_vision_interfaces.srv import PixelToTable


class VisionTransform(Node):
    def __init__(self):
        super().__init__("vision_transform", namespace="vision")
        self.declare_parameter("calib_file", "")      # "" -> <hk_fr5_vision share>/config/calib.yaml
        self.declare_parameter("reload_period", 1.0)  # s between file checks
        path = self.get_parameter("calib_file").value
        period = self.get_parameter("reload_period").value
        if not isinstance(period, (int, float)) or not period > 0:
            raise RuntimeError(f"'reload_period' must be > 0, got {period!r}")
        self.path = path or os.path.join(get_package_share_directory("hk_fr5_vision"), "config", "calib.yaml")
        self.calib, self.reason, self.stamp = None, "not loaded yet", None
        self._check_file(force=True)
        self.create_timer(float(period), self._check_file)
        self.create_service(PixelToTable, "pixel_to_table", self._pixel_to_table)
        self.get_logger().info(f"/vision/pixel_to_table ready, calibration file {self.path} (checked every {period} s)")

    def _check_file(self, force=False):
        """Reload when the file's mtime / size changed (or it appeared / disappeared); force = first check."""
        try:
            st = os.stat(self.path)   # follows the symlink-install chain to the source file
            stamp = (st.st_mtime_ns, st.st_size)
        except OSError:
            stamp = None
        if stamp == self.stamp and not force:
            return
        self.stamp = stamp
        try:
            c = calib.load(self.path)
        except ValueError as e:   # NotCalibrated is a ValueError too
            if str(e) != self.reason or force:   # one call site per severity: rclpy raises if a site changes it
                msg = f"no calibration: {e} -> /vision/pixel_to_table answers success = false"
                if isinstance(e, calib.NotCalibrated):
                    self.get_logger().warn(msg)
                else:
                    self.get_logger().error(msg)
            self.calib, self.reason = None, str(e)
            return
        self.calib, self.reason = c, ""
        self.get_logger().info(f"calibration loaded: {c['date']}, {len(c['tags'])} tags, rms {c['rms_mm']} mm, "
                               f"image {c['image_size'][0]}x{c['image_size'][1]}, camera '{c['camera']}'"
                               + (f", suspect tag {c['suspect']}" if c.get("suspect") is not None else ""))

    def _pixel_to_table(self, req, res):
        u, v = list(req.u), list(req.v)
        if self.calib is None:
            res.success, res.message = False, f"no calibration: {self.reason}"
        elif not u:
            res.success, res.message = False, "no points given (u / v empty)"
        elif len(u) != len(v):
            res.success, res.message = False, f"u and v lengths differ ({len(u)} vs {len(v)})"
        elif not all(math.isfinite(a) for a in u + v):
            res.success, res.message = False, "u / v must be finite numbers"
        else:
            res.x, res.y, res.inside = calib.apply(self.calib, u, v)
            out = len(res.inside) - sum(res.inside)
            res.success = True
            res.message = f"{len(u)} point(s)" + (f", {out} outside the calibrated tag area (extrapolated)" if out else "")
        return res


def main():
    rclpy.init()
    try:
        node = VisionTransform()
    except RuntimeError as e:
        print(f"[vision_transform] ERROR: {e}")
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)   # single-threaded: timer and service never run at the same time
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
