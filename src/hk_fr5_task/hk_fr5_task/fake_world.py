"""fake_world node (T3, decision-log TM-5 / TM-8; V5 TM-24): stands in for Vision 1.
Publishes /vision/boards (std_msgs/String, the agreed Vision 1 JSON, base_link; stamp = this clock) from a scenario
yaml, reacts to /task/events (picked / placed / dropped), simulates the operator (processing) and hands (obstacles[]).

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

from hk_fr5_interfaces.msg import GripperState
from hk_fr5_task.fake_core import FakeWorld
from rclpy.qos import DurabilityPolicy, QoSProfile


class FakeWorldNode(Node):
    def __init__(self):
        super().__init__("fake_world", namespace="task")
        share = get_package_share_directory("hk_fr5_task")
        self.declare_parameter("scenario", os.path.join(share, "config", "scenario_demo.yaml"))
        self.declare_parameter("config", os.path.join(share, "config", "world.yaml"))
        self.declare_parameter("rate", 5.0)           # Hz: Vision 1 runs at ~5 Hz (user, 2026-10-27)
        self.declare_parameter("seed", 0)
        self.declare_parameter("contact_sim", False)  # drive the FAKE gripper's vacuum from the tcp height (sim only)
        self.declare_parameter("table_z", -0.060)
        self.declare_parameter("force_vacuum", "")    # test hook: "off" -> no seal / board lost while held
        self.declare_parameter("hand_now", "")        # test hook: "x y duration" -> a hand appears now
        with open(self.get_parameter("scenario").value) as f:
            scenario = yaml.safe_load(f)
        with open(self.get_parameter("config").value) as f:
            variants = yaml.safe_load(f)["variants"]
        self.world = FakeWorld(scenario, variants, self.get_parameter("seed").value)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(String, "/vision/boards", 5)
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
        self.get_logger().info(f"fake world: {self.get_parameter('scenario').value} -> /vision/boards")

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
        stamp = self.get_clock().now().nanoseconds * 1e-9                # capture time (C9), same clock
        self.pub.publish(String(data=json.dumps(self.world.frame(self.t(), stamp))))
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
    except RuntimeError:                         # Ctrl+C during a message take: rclpy raises a pybind RuntimeError
        if rclpy.ok():                           # (seen in main_ros_check, TM-25); only then a clean exit
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
