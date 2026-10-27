"""world_model node (T3, control-loop.md sections 2-3): Vision 1 frames -> base_link facts -> /task/world.

    /vision/boards       std_msgs/String JSON from Vision 1 (schema: notes/vision1-changes.md, parsed by vision1.py;
                         boards / zones already in base_link). V3 (TM-22): replaced /vision1/detections (pixels).
    /task/events         std_msgs/String JSON from the skills: picked {board, x, y} | placed {board, zone, x, y, yaw,
                         variant} | dropped {board} | reset_zone {zone}
    /gripper/state       hk_fr5_interfaces/GripperState
    -> /task/world       hk_fr5_task_interfaces/World (latched; on every frame + 2 Hz)

Frames captured (Vision 1 `stamp`, C9) before the last /task/events message are dropped: they show the table before
the robot changed it (e.g. a just-picked board still lying there).
"""
import json
import math
import os
import signal

import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from hk_fr5_interfaces.msg import GripperState
from hk_fr5_task import mask, vision1
from hk_fr5_task.world_core import World
from hk_fr5_task_interfaces.msg import Board, Obstacle, World as WorldMsg, Zone

VERBOSE = os.environ.get("HK_VERBOSE", "0") == "1"       # TM-27: tools/live_env.sh debug -> every step logged
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class WorldModel(Node):
    def __init__(self):
        super().__init__("world_model", namespace="task")
        share = get_package_share_directory("hk_fr5_task")
        self.declare_parameter("config", os.path.join(share, "config", "world.yaml"))
        self.declare_parameter("max_clock_skew", 5.0)          # |now - stamp| above this: Vision 1 clock wrong -> arrival time
        cfg_path = self.get_parameter("config").value
        with open(cfg_path) as f:
            self.core = World(yaml.safe_load(f))
        self.declare_parameter("shadow_radius", 0.12)          # CL-20 B band base -> tcp half-width (0 = no mask)
        self.declare_parameter("mask_extra", 0.03)             # CL-31 extra margin on the held-board area
        self.declare_parameter("tcp_frame", "tcp")
        self.shadow_radius = float(self.get_parameter("shadow_radius").value)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.gripper_state, self.dropped_frames, self.last_event_t = "", 0, 0.0
        self.notes_logged = 0                                    # TM-26: world_core notes already in the log
        self.pub = self.create_publisher(WorldMsg, "world", LATCHED)
        self.create_subscription(String, "/vision/boards", self._frame, 5)
        self.create_subscription(String, "events", self._event, 50)
        self.create_subscription(GripperState, "/gripper/state", lambda m: setattr(self, "gripper_state", m.state), LATCHED)
        self.create_timer(0.5, self._publish)
        self.get_logger().info(f"world model: {cfg_path}, Vision 1 frames from /vision/boards")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _frame(self, msg):
        now = self.now()
        try:
            stamp, dets, problems = vision1.parse(json.loads(msg.data))
        except ValueError as e:                                          # json.JSONDecodeError is a ValueError too
            self.get_logger().warn(f"Vision 1 frame ignored: {e}", throttle_duration_sec=5.0)
            return
        zone_probs = [p for p in problems if p.startswith("zone ")]
        board_probs = [p for p in problems if not p.startswith("zone ")]
        if zone_probs:                                                   # TM-22 A: no defaults, so say it loudly
            self.get_logger().error(f"Vision 1 zone(s) SKIPPED: {'; '.join(zone_probs)} - Vision 1 interface "
                                    "incomplete (vision1-changes.md C2 / C3 / C15 / C18). Such a zone does not exist "
                                    "for us: no place there, boards lying on it count as input boards",
                                    throttle_duration_sec=5.0)
        if board_probs:
            self.get_logger().warn(f"Vision 1 board / obstacle entries skipped: {'; '.join(board_probs)}", throttle_duration_sec=5.0)
        if abs(now - stamp) > self.get_parameter("max_clock_skew").value:
            self.get_logger().error(f"Vision 1 stamp {now - stamp:+.1f} s off this clock (same PC? C9) - using the "
                                    "arrival time", throttle_duration_sec=10.0)
            stamp = now
        if stamp < self.last_event_t:
            self.dropped_frames += 1                                     # captured before the robot changed the table
            if VERBOSE:
                self.get_logger().info(f"Vision 1 frame dropped: captured {self.last_event_t - stamp:.2f} s before the "
                                       f"last robot event ({self.dropped_frames} so far)")
            return
        self.core.update(now, dets, self._mask())
        self._log_notes()
        self._publish()

    def _log_notes(self):
        """TM-26: every new world_core note (zone states, new / lost boards, variant changes, events) once in /rosout."""
        n = getattr(self.core, "note_count", 0)
        new = min(n - self.notes_logged, len(self.core.log))
        for t, text in self.core.log[len(self.core.log) - new:] if new > 0 and VERBOSE else []:
            self.get_logger().info(f"world: {text}")
        self.notes_logged = n

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
        self.last_event_t = self.now()
        try:
            self.core.event(self.last_event_t, json.loads(msg.data))
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().error(f"bad event {msg.data!r}: {e}")
            return
        if VERBOSE:
            self.get_logger().info(f"event applied: {msg.data}")
        self._log_notes()
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
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # TM-27: a 2nd Ctrl+C (terminal + pixi) must not break the shutdown
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
