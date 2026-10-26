"""Minimal consumer example (what the control loop does): print one /vision/boards message.

    pixi run listen
"""
import json

import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("vision1_listen")
got = []
node.create_subscription(String, "/vision/boards", lambda m: got.append(json.loads(m.data)), 10)
while not got:
    rclpy.spin_once(node, timeout_sec=1.0)
msg = got[0]
for b in msg["boards"]:
    print(f'board {b["id"]} {b["variant"]}  x {b.get("x")} y {b.get("y")} yaw {b.get("yaw")}  size {b.get("size_mm")} '
          f'zone {b.get("zone")} inside_calib {b.get("inside_calibration")}')
for r in msg["rejected"]:
    print("rejected", r)
