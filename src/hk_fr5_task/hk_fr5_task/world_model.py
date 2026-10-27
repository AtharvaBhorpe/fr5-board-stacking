"""world_model node (T3, control-loop.md sections 2-3): Vision 1 detections -> base_link facts -> /task/world.

    /vision1/detections  vision_msgs/Detection2DArray (Vision 1 contract, project-plan P-1..P-3)
    /task/events         std_msgs/String JSON from the skills: picked {board, x, y} | placed {board, zone, x, y, yaw,
                         variant} | dropped {board} | reset_zone {zone}
    /gripper/state       hk_fr5_interfaces/GripperState
    -> /task/world       hk_fr5_task_interfaces/World (latched; on every frame + 2 Hz)

Pixels -> table: /vision/pixel_to_table (pixel_source: service, the real API) or the same calibration maths in-process
(pixel_source: local, for fast tests).
"""
import json
import math
import os

import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from hk_fr5_interfaces.msg import GripperState
from hk_fr5_task.fake_core import Camera, bbox_corners
from hk_fr5_task import mask
from hk_fr5_task.world_core import World
from hk_fr5_task_interfaces.msg import Board, Obstacle, World as WorldMsg, Zone
from hk_fr5_vision_interfaces.srv import PixelToTable

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class WorldModel(Node):
    def __init__(self):
        super().__init__("world_model", namespace="task")
        share = get_package_share_directory("hk_fr5_task")
        self.declare_parameter("config", os.path.join(share, "config", "world.yaml"))
        self.declare_parameter("pixel_source", "service")      # service | local
        self.declare_parameter("calib_file", "")               # local: "" -> hk_fr5_vision share config/calib.yaml
        cfg_path = self.get_parameter("config").value
        with open(cfg_path) as f:
            self.core = World(yaml.safe_load(f))
        self.source = self.get_parameter("pixel_source").value
        if self.source not in ("service", "local"):
            raise RuntimeError(f"pixel_source must be 'service' or 'local', got '{self.source}'")
        self.cam, self.cli, self.pending = None, None, False
        if self.source == "local":
            from hk_fr5_vision import calib
            p = self.get_parameter("calib_file").value or os.path.join(
                get_package_share_directory("hk_fr5_vision"), "config", "calib.yaml")
            self.cam = Camera(calib.load(p))
        else:
            self.cli = self.create_client(PixelToTable, "/vision/pixel_to_table")
        self.declare_parameter("shadow_radius", 0.12)          # CL-20 B band base -> tcp half-width (0 = no mask)
        self.declare_parameter("mask_extra", 0.03)             # CL-31 extra margin on the held-board area
        self.declare_parameter("tcp_frame", "tcp")
        self.shadow_radius = float(self.get_parameter("shadow_radius").value)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.gripper_state, self.dropped_frames = "", 0
        self.pub = self.create_publisher(WorldMsg, "world", LATCHED)
        self.create_subscription(Detection2DArray, "/vision1/detections", self._detections, 5)
        self.create_subscription(String, "events", self._event, 50)
        self.create_subscription(GripperState, "/gripper/state", lambda m: setattr(self, "gripper_state", m.state), LATCHED)
        self.create_timer(0.5, self._publish)
        self.get_logger().info(f"world model: {cfg_path}, pixels -> table via {self.source}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _detections(self, msg):
        items = []
        for d in msg.detections:
            if not d.results:
                continue
            h = d.results[0].hypothesis
            c = d.bbox.center
            items.append((h.class_id, d.id, h.score, bbox_corners(c.position.x, c.position.y, c.theta, d.bbox.size_x,
                                                                  d.bbox.size_y), (c.position.x, c.position.y)))
        pts = [p for it in items for p in it[3] + [list(it[4])]]          # 4 corners + centre per detection
        if self.source == "local":
            xy = [self.cam.to_table(u, v) for u, v in pts]
            self._apply(items, xy, [True] * len(pts))
            return
        if self.pending:
            self.dropped_frames += 1                                     # never queue frames behind a slow call
            return
        if not self.cli.service_is_ready():
            self.get_logger().warn("/vision/pixel_to_table not available", throttle_duration_sec=5.0)
            return
        self.pending = True
        req = PixelToTable.Request(u=[float(p[0]) for p in pts], v=[float(p[1]) for p in pts])
        fut = self.cli.call_async(req)
        fut.add_done_callback(lambda f: self._service_done(f, items))

    def _service_done(self, fut, items):
        self.pending = False
        try:
            r = fut.result()
        except Exception as e:
            self.get_logger().error(f"pixel_to_table failed: {e}", throttle_duration_sec=5.0)
            return
        if not r.success:
            self.get_logger().warn(f"pixel_to_table: {r.message}", throttle_duration_sec=5.0)
            return
        self._apply(items, list(zip(r.x, r.y)), list(r.inside))

    def _apply(self, items, xy, inside):
        dets = []
        for k, (cls, tid, score, _, _) in enumerate(items):
            dets.append({"cls": cls, "id": tid, "score": score, "corners": [list(p) for p in xy[5 * k:5 * k + 4]],
                         "inside": bool(inside[5 * k + 4])})
        self.core.update(self.now(), dets, self._mask())
        self._publish()

    def _mask(self):
        """CL-31 mask, transitional (step 7: link capsules from TF + camera rays): the band base -> tcp as 'arm', a
        circle of the held board's half-diagonal at the tcp as 'tool' (only while holding; the gripper alone is covered
        by the band). No TF (fake world / no robot) -> None."""
        if self.shadow_radius <= 0:
            return None
        try:
            tr = self.tf.lookup_transform("base_link", self.get_parameter("tcp_frame").value, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        tcp = (tr.transform.translation.x, tr.transform.translation.y)
        hb = self.core.boards.get(self.core.held) if self.core.held else None
        r = 0.0
        if hb is not None:
            v = self.core.c["variants"].get(hb.get("variant", ""))
            l, w = (v["length"], v["width"]) if v else (hb.get("length", 0.0), hb.get("width", 0.0))
            r = math.hypot(l, w) / 2 + float(self.get_parameter("mask_extra").value)
        return mask.band(tcp, self.shadow_radius, r)

    def _event(self, msg):
        try:
            self.core.event(self.now(), json.loads(msg.data))
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().error(f"bad event {msg.data!r}: {e}")
            return
        self._publish()

    def _publish(self):
        t = self.now()
        s = self.core.snapshot(t)
        m = WorldMsg(vision_ok=s["vision_ok"], vision_age=s["vision_age"], held_board_id=s["held_board_id"],
                     gripper_state=self.gripper_state)
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.boards = [Board(**b) for b in s["boards"]]
        m.zones = [Zone(**z) for z in s["zones"]]
        m.obstacles = [Obstacle(**o) for o in s["obstacles"]]
        self.pub.publish(m)


def main():
    rclpy.init()
    try:
        node = WorldModel()
    except (RuntimeError, OSError, ValueError) as e:
        print(f"[world_model] ERROR: {e}")
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
