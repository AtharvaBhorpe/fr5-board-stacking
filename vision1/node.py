"""ROS 2 node: overhead camera -> /vision/boards (std_msgs/String, JSON), latest state at `rate` Hz.

    pixi run vision                                              # first camera whose name contains "C920"
    pixi run vision --ros-args -p device:=2 -p rate:=5.0         # /dev/video2
    ros2 service call /vision/detect std_srvs/srv/Trigger        # fresh snapshot, JSON in .message
    pixi run vision --ros-args -p device:=samples/01.jpg         # replay one image (no camera)
"""
import json
import math
import os
import sys
import time

import cv2
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(__file__))
from detect import detect, draw  # noqa: E402
import table  # noqa: E402

DEST_MEMORY = 10.0  # s a destination tag stays valid after it was last seen


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
        device = str(self.declare_parameter("device", "C920", ParameterDescriptor(dynamic_typing=True)).value)
        rate = float(self.declare_parameter("rate", 5.0).value)
        focus = int(self.declare_parameter("focus", 15).value)  # fixed focus as in the team calibration; -1 = leave as is
        self.still = cv2.imread(device) if os.path.isfile(device) else None
        if self.still is None:
            index = int(device) if device.isdigit() else find_camera(device)
            self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # 1080p at 30 fps (YUYV: ~5 fps)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # latest frame, not a stale queue
            if not self.cap.isOpened():
                raise RuntimeError(f"cannot open camera {device!r}")
            if focus >= 0:
                self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                self.cap.set(cv2.CAP_PROP_FOCUS, focus)
            ok, f = self.cap.read()
            if not ok or f.shape[:2] != (1080, 1920):
                raise RuntimeError(f"camera {device!r} gives {None if not ok else f.shape[1::-1]}, need 1920x1080 "
                                   "(the calibration is for that size)")
            self.get_logger().info(f"camera /dev/video{index}: 1920x1080, autofocus "
                                   f"{self.cap.get(cv2.CAP_PROP_AUTOFOCUS):.0f}, focus {self.cap.get(cv2.CAP_PROP_FOCUS):.0f}")
        try:
            self.calib = table.load()
        except (OSError, ValueError) as e:
            self.calib = None
            self.get_logger().warn(f"no calibration ({e}): publishing pixels only")
        self.last = "{}"
        self.dest = {}
        self.create_service(Trigger, "/vision/detect", self.snapshot)
        time.sleep(1.5)  # DDS discovery
        if self.count_publishers("/vision/boards"):
            raise RuntimeError("another node already publishes /vision/boards: stop it first (two sources would mix)")
        self.device = device
        self.pub = self.create_publisher(String, "/vision/boards", 10)
        self.markers = self.create_publisher(MarkerArray, "/vision/markers", 10)  # RViz: Add -> By topic
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
        rejected, tags = [], []
        boards = detect(frame, rejected, tags)
        if self.calib is not None:
            boards = table.pickable(self.calib, boards, rejected)
        else:
            for b in boards:
                b.pop("area_px")
        now = time.time()
        if self.calib is not None:
            for t in tags:  # templates don't move: remember each tag, small tags are missed in ~10 % of frames
                d = table.destination(self.calib, t)
                if d:
                    self.dest[d["variant"]] = dict(d, seen=now)
        dests = [{**{k: v for k, v in d.items() if k != "seen"}, "age": round(now - d["seen"], 1)}
                 for d in self.dest.values() if now - d["seen"] < DEST_MEMORY]
        msg = {"stamp": now, "destinations": dests, "image_size": [frame.shape[1], frame.shape[0]],
               "source": self.device, "calibrated": self.calib is not None, "boards": boards, "rejected": rejected}
        self.last = json.dumps(msg)
        if time.time() - getattr(self, "saved", 0) > 2.0:  # debug overlay for the lab, every 2 s
            self.saved = time.time()
            img = draw(frame, boards)
            for r in rejected:
                cv2.putText(img, r["reason"], (int(r["u"]) - 90, int(r["v"])), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 3)
            cv2.imwrite("/tmp/vision1_last.jpg", img)
        self.pub.publish(String(data=self.last))
        self.publish_markers(boards, dests)
        return True

    def publish_markers(self, boards, dests):
        """Display only (not in the MoveIt scene): green = pickable boards, red = destination board poses."""
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        items = [(b, 0.0, 0.8, 0.0, b.get("z", 0.0) - 0.0025) for b in boards if "x" in b]
        items += [(d, 0.9, 0.0, 0.0, d["support_z"] + 0.0025) for d in dests]
        for i, (o, r, g, bl, z) in enumerate(items):
            m = Marker(type=Marker.CUBE, action=Marker.ADD, id=i, ns=o["variant"])
            m.header.frame_id = "base_link"
            m.pose.position.x, m.pose.position.y, m.pose.position.z = o["x"], o["y"], z
            m.pose.orientation.z, m.pose.orientation.w = math.sin(o["yaw"] / 2), math.cos(o["yaw"] / 2)
            m.scale.x, m.scale.y, m.scale.z = [v / 1000 for v in table.SIZES_MM[o["variant"]]] + [0.005]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, bl, 0.6
            arr.markers.append(m)
        self.markers.publish(arr)


def main():
    rclpy.init()
    try:
        rclpy.spin(Vision1())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
