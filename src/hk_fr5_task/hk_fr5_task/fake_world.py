"""fake_world node (T3, decision-log TM-5 / TM-8): stands in for Vision 1 until the teammate's node lands.
Publishes /vision1/detections (vision_msgs/Detection2DArray, REAL camera pixels via the real calibration) from a
scenario yaml, reacts to /task/events (picked / placed / dropped), simulates the operator (processing) and hands.

    ros2 run hk_fr5_task fake_world --ros-args -p scenario:=<yaml>
"""
import json
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from hk_fr5_task.fake_core import Camera, FakeWorld


class FakeWorldNode(Node):
    def __init__(self):
        super().__init__("fake_world", namespace="task")
        share = get_package_share_directory("hk_fr5_task")
        self.declare_parameter("scenario", os.path.join(share, "config", "scenario_demo.yaml"))
        self.declare_parameter("config", os.path.join(share, "config", "world.yaml"))
        self.declare_parameter("calib_file", "")     # "" -> hk_fr5_vision share config/calib.yaml
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("seed", 0)
        from hk_fr5_vision import calib
        cpath = self.get_parameter("calib_file").value or os.path.join(
            get_package_share_directory("hk_fr5_vision"), "config", "calib.yaml")
        with open(self.get_parameter("scenario").value) as f:
            scenario = yaml.safe_load(f)
        with open(self.get_parameter("config").value) as f:
            variants = yaml.safe_load(f)["variants"]
        self.world = FakeWorld(scenario, Camera(calib.load(cpath)), variants, self.get_parameter("seed").value)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(Detection2DArray, "/vision1/detections", 5)
        self.create_subscription(String, "events", self._event, 50)
        self.create_timer(1.0 / self.get_parameter("rate").value, self._tick)
        self._logged = 0
        self.get_logger().info(f"fake world: {self.get_parameter('scenario').value}, calibration {cpath}")

    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9 - self.t0

    def _event(self, msg):
        try:
            self.world.event(self.t(), json.loads(msg.data))
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().error(f"bad event {msg.data!r}: {e}")

    def _tick(self):
        dets = self.world.detections(self.t())
        m = Detection2DArray()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "c920"
        for d in dets:
            x = Detection2D(id=d["id"])
            x.header = m.header
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = d["cls"], float(d["score"])
            x.results = [h]
            x.bbox.center.position.x, x.bbox.center.position.y, x.bbox.center.theta = d["u"], d["v"], d["theta"]
            x.bbox.size_x, x.bbox.size_y = d["sx"], d["sy"]
            m.detections.append(x)
        self.pub.publish(m)
        for t, text in self.world.log[self._logged:]:
            self.get_logger().info(f"[{t:6.1f}] {text}")
        self._logged = len(self.world.log)


def main():
    rclpy.init()
    try:
        node = FakeWorldNode()
    except (OSError, ValueError, KeyError) as e:
        print(f"[fake_world] ERROR: {e}")
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
