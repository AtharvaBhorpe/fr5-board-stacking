"""ROS 2 node: overhead camera -> /vision/boards (std_msgs/String, JSON), latest state at `rate` Hz.

    pixi run vision                                              # camera /dev/video0
    pixi run vision --ros-args -p device:=2 -p rate:=5.0
    pixi run vision --ros-args -p device:=samples/01.jpg         # replay one image (no camera)
"""
import json
import os
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(__file__))
from detect import detect  # noqa: E402


class Vision1(Node):
    def __init__(self):
        super().__init__("vision1_overhead")
        device = str(self.declare_parameter("device", "0").value)
        rate = float(self.declare_parameter("rate", 5.0).value)
        self.still = cv2.imread(device) if os.path.isfile(device) else None
        if self.still is None:
            self.cap = cv2.VideoCapture(int(device) if device.isdigit() else device, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # latest frame, not a stale queue
            if not self.cap.isOpened():
                raise RuntimeError(f"cannot open camera {device!r}")
        self.pub = self.create_publisher(String, "/vision/boards", 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"publishing /vision/boards at {rate} Hz from {device!r}")

    def tick(self):
        if self.still is not None:
            frame = self.still
        else:
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn("camera read failed", throttle_duration_sec=2.0)
                return
        boards = detect(frame)
        for b in boards:
            b.pop("area_px")
        msg = {"stamp": time.time(), "image_size": [frame.shape[1], frame.shape[0]], "boards": boards}
        self.pub.publish(String(data=json.dumps(msg)))


def main():
    rclpy.init()
    try:
        rclpy.spin(Vision1())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
