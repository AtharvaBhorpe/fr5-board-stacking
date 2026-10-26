"""fake_world node (T3, decision-log TM-5 / TM-8): stands in for Vision 1 until the teammate's node lands.
Publishes /vision1/detections (vision_msgs/Detection2DArray, REAL camera pixels via the real calibration) from a
scenario yaml, reacts to /task/events (picked / placed / dropped), simulates the operator (processing) and hands.

    ros2 run hk_fr5_task fake_world --ros-args -p scenario:=<yaml>
"""
import json
import os

import rclpy
import tf2_ros
import yaml
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from hk_fr5_interfaces.msg import GripperState
from hk_fr5_task.fake_core import Camera, FakeWorld
from rclpy.qos import DurabilityPolicy, QoSProfile


class FakeWorldNode(Node):
    def __init__(self):
        super().__init__("fake_world", namespace="task")
        share = get_package_share_directory("hk_fr5_task")
        self.declare_parameter("scenario", os.path.join(share, "config", "scenario_demo.yaml"))
        self.declare_parameter("config", os.path.join(share, "config", "world.yaml"))
        self.declare_parameter("calib_file", "")     # "" -> hk_fr5_vision share config/calib.yaml
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("seed", 0)
        self.declare_parameter("contact_sim", False)  # drive the FAKE gripper's vacuum from the tcp height (sim only)
        self.declare_parameter("table_z", -0.060)
        self.declare_parameter("force_vacuum", "")    # test hook: "off" -> no seal / board lost while held
        self.declare_parameter("hand_now", "")        # test hook: "x y duration" -> a hand appears now
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
        self._hand_req = ""
        if self.get_parameter("contact_sim").value:
            self.tf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf, self)
            self.vacuum_on, self.sealed, self.sent = False, False, None
            self.create_subscription(GripperState, "/gripper/state", self._gripper,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.param_cli = self.create_client(SetParameters, "/gripper/gripper_controller/set_parameters")
            self.create_timer(0.02, self._contact)
            self.get_logger().info("contact simulation on: fake gripper vacuum follows the tcp height")
        self.get_logger().info(f"fake world: {self.get_parameter('scenario').value}, calibration {cpath}")

    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9 - self.t0

    def _event(self, msg):
        try:
            self.world.event(self.t(), json.loads(msg.data))
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().error(f"bad event {msg.data!r}: {e}")

    def _gripper(self, m):
        self.vacuum_on = m.vacuum_on

    def _contact(self):
        """Seal = vacuum on and the cup tip at / below the surface under it; stays sealed while the vacuum is on."""
        try:
            tr = self.tf.lookup_transform("base_link", "tcp", rclpy.time.Time()).transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        force_off = self.get_parameter("force_vacuum").value == "off"
        if not self.vacuum_on or force_off:
            self.sealed = False
        elif not self.sealed:
            self.sealed = tr.z <= self.world.surface_at(tr.x, tr.y, self.get_parameter("table_z").value) + 0.0005
        if self.sealed != self.sent and self.param_cli.service_is_ready():
            p = Parameter(name="fake_vacuum", value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=self.sealed))
            self.param_cli.call_async(SetParameters.Request(parameters=[p]))
            self.sent = self.sealed

    def _tick(self):
        h = self.get_parameter("hand_now").value
        if h and h != self._hand_req:
            x, y, dur = (float(v) for v in h.split())
            self.world.hands.append({"t": self.t(), "x": x, "y": y, "duration": dur})
            self._hand_req = h
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
