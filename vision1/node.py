"""ROS 2 node: overhead camera -> /vision/boards (std_msgs/String, JSON), latest state at `rate` Hz.

    pixi run vision                                              # first camera whose name contains "C920"
    pixi run vision --ros-args -p device:=2 -p rate:=5.0         # /dev/video2
    ros2 service call /vision/detect std_srvs/srv/Trigger        # fresh snapshot, JSON in .message
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
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(__file__))
from detect import detect  # noqa: E402
import table  # noqa: E402


def find_camera(name):
    """/dev/video index of the first capture device whose name contains `name` (e.g. 'C920')."""
    for i in range(16):
        try:
            with open(f"/sys/class/video4linux/video{i}/name") as f:
                if name.lower() in f.read().lower():
                    return i
        except OSError:
            pass
    raise RuntimeError(f"no camera named like {name!r} (see /sys/class/video4linux/*/name)")


class Vision1(Node):
    def __init__(self):
        super().__init__("vision1_overhead")
        device = str(self.declare_parameter("device", "C920").value)
        rate = float(self.declare_parameter("rate", 5.0).value)
        self.still = cv2.imread(device) if os.path.isfile(device) else None
        if self.still is None:
            index = int(device) if device.isdigit() else find_camera(device)
            self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # latest frame, not a stale queue
            if not self.cap.isOpened():
                raise RuntimeError(f"cannot open camera {device!r}")
        try:
            self.calib = table.load()
        except (OSError, ValueError) as e:
            self.calib = None
            self.get_logger().warn(f"no calibration ({e}): publishing pixels only")
        self.last = "{}"
        self.create_service(Trigger, "/vision/detect", self.snapshot)
        self.pub = self.create_publisher(String, "/vision/boards", 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"publishing /vision/boards at {rate} Hz from {device!r}")

    def snapshot(self, _req, res):
        if self.still is None:
            self.cap.grab()  # drop the buffered frame so the answer is fresh
        res.success = self.tick()
        res.message = self.last
        return res

    def tick(self):
        if self.still is not None:
            frame = self.still
        else:
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn("camera read failed", throttle_duration_sec=2.0)
                return False
        rejected = []
        boards = detect(frame, rejected)
        if self.calib is not None:
            boards = table.pickable(self.calib, boards, rejected)
        else:
            for b in boards:
                b.pop("area_px")
        msg = {"stamp": time.time(), "image_size": [frame.shape[1], frame.shape[0]],
               "calibrated": self.calib is not None, "boards": boards, "rejected": rejected}
        self.last = json.dumps(msg)
        self.pub.publish(String(data=self.last))
        return True


def main():
    rclpy.init()
    try:
        rclpy.spin(Vision1())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
