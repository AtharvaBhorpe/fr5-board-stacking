#!/usr/bin/env python3
"""Vacuum gripper node.

Service /gripper/set_vacuum (std_srvs/SetBool): true = vacuum on, false = off.
Topic   /gripper/vacuum_on  (std_msgs/Bool, latched): last commanded state.

Real mode switches the Schmalz vacuum through the Fairino controller's XML-RPC
interface (SetToolDO). We do not use Fairino's ros2_cmd_server: it fights the
motion driver for the controller's single-client realtime port.
Fake mode only logs, so task code runs in simulation.
The vacuum state is left unchanged on start and shutdown (no surprise drops).
"""
import xmlrpc.client

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class GripperNode(Node):
    def __init__(self):
        super().__init__("gripper")
        self.declare_parameter("use_fake_hardware", True)
        self.declare_parameter("robot_ip", "")
        self.declare_parameter("tool_do", 0)
        self.fake = bool(self.get_parameter("use_fake_hardware").value)
        self.tool_do = int(self.get_parameter("tool_do").value)
        ip = str(self.get_parameter("robot_ip").value)

        if self.fake:
            self.rpc = None
            self.get_logger().info("Gripper in FAKE mode (no hardware).")
        else:
            if not ip:
                raise RuntimeError("robot_ip parameter is empty (set it in config/hardware.yaml)")
            self.rpc = xmlrpc.client.ServerProxy(f"http://{ip}:20003/RPC2")
            self.get_logger().info(f"Gripper on tool DO{self.tool_do} via XML-RPC at {ip}.")

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(Bool, "gripper/vacuum_on", latched)
        self.create_service(SetBool, "gripper/set_vacuum", self.on_set_vacuum)

    def on_set_vacuum(self, request, response):
        state = "ON" if request.data else "OFF"
        if self.fake:
            code = 0
        else:
            try:
                code = int(self.rpc.SetToolDO(self.tool_do, 1 if request.data else 0, 0, 0))
            except Exception as exc:  # network or controller problem
                response.success = False
                response.message = f"XML-RPC error: {exc}"
                self.get_logger().error(response.message)
                return response

        response.success = code == 0
        response.message = f"vacuum {state}" if code == 0 else f"controller error code {code}"
        if response.success:
            self.state_pub.publish(Bool(data=request.data))
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return response


def main():
    rclpy.init()
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
