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
from collections import deque

# one thread per core in numpy (OpenBLAS) / OpenCV spun ~9 cores at 5 Hz on small matrices: cap before importing
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "2")
import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

cv2.setNumThreads(2)
sys.path.insert(0, os.path.dirname(__file__))
from detect import detect, draw  # noqa: E402
import table  # noqa: E402

DEST_MEMORY = 60.0  # s a destination tag stays valid after it was last seen (templates are fixed; the arm covers them
                    # >10 s while placing, real robot 2026-10-27)
DEST_FRAMES = 15    # median over the last N detections of a tag: single frames jitter ~2.5 mm / 1 deg (p95)


def median_dest(hist):
    """[(seen, dest)] -> one dest: median x / y, circular mean yaw (yaw wraps at +-pi)."""
    ds = [d for _, d in hist]
    yaw = math.atan2(sum(math.sin(d["yaw"]) for d in ds), sum(math.cos(d["yaw"]) for d in ds))
    return dict(ds[-1], x=round(float(np.median([d["x"] for d in ds])), 4),
                y=round(float(np.median([d["y"] for d in ds])), 4), yaw=round(yaw, 4), frames=len(ds))


def remember(store, key, now, item):
    store.setdefault(key, deque(maxlen=DEST_FRAMES)).append((now, item))


def recall(store, now):
    """Median pose + age of every key seen within DEST_MEMORY."""
    return [dict(median_dest([h for h in hist if now - h[0] < DEST_MEMORY]), age=round(now - hist[-1][0], 1))
            for hist in store.values() if now - hist[-1][0] < DEST_MEMORY]


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
        self.use_model = bool(self.declare_parameter("model", False).value)  # learned segmenter (stacks, UNKWN)
        focus = int(self.declare_parameter("focus", 15).value)  # fixed focus as in the team calibration; -1 = leave as is
        self.still = cv2.imread(device) if os.path.isfile(device) else None
        self.focus, self.fails = focus, 0
        if self.still is None:
            self.open_camera(device)
        try:
            self.calib = table.load()
        except (OSError, ValueError) as e:
            self.calib = None
            self.get_logger().warn(f"no calibration ({e}): publishing pixels only")
        self.last = "{}"
        self.dest, self.zones = {}, {}  # tag memory: by variant (pick_one.py), by zone name (main controller)
        self.create_service(Trigger, "/vision/detect", self.snapshot)
        time.sleep(1.5)  # DDS discovery
        if self.count_publishers("/vision/boards"):
            raise RuntimeError("another node already publishes /vision/boards: stop it first (two sources would mix)")
        self.device = device
        self.pub = self.create_publisher(String, "/vision/boards", 10)
        self.markers = self.create_publisher(MarkerArray, "/vision/markers", 10)  # RViz: Add -> By topic
        self.image = self.create_publisher(Image, "/vision/image", 2)  # annotated camera view, half size (RViz Image)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"publishing /vision/boards at {rate} Hz from {device!r}")

    def open_camera(self, device):
        """(Re)open the camera by name: after a USB hub reset it comes back under a new /dev/video index."""
        focus = self.focus
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

    def snapshot(self, _req, res):
        if self.still is None:
            self.cap.grab()  # drop the buffered frame so the answer is fresh
        res.success = self.tick()
        res.message = self.last
        return res

    def tick(self):
        if self.still is not None:  # replay: re-read so a test can swap the file (scene changed)
            frame = cv2.imread(self.device)
            frame = self.still if frame is None else frame
            stamp = time.time()
        else:
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn("camera read failed", throttle_duration_sec=2.0)
                self.fails += 1
                if self.fails % 10 == 0:  # ~2 s of failures: USB re-enumerated? reopen by name
                    try:
                        self.cap.release()
                        self.open_camera(self.device)
                        self.get_logger().info("camera reopened")
                    except RuntimeError as e:
                        self.get_logger().warn(f"camera reopen failed: {e}")
                return False
            stamp = time.time()  # capture time (C9): detection below takes up to ~1 s on the CPU
            self.fails = 0
        rejected, tags, occluded = [], [], []
        boards = detect(frame, rejected, tags, use_model=self.use_model)
        if self.calib is not None:
            boards = table.pickable(self.calib, boards, rejected, occluded)
        else:
            for b in boards:
                b.pop("area_px")
        now = stamp
        if self.calib is not None:
            for t in tags:  # templates don't move: remember each tag, small tags are missed in ~10 % of frames
                d, z = table.destination(self.calib, t), table.zone(self.calib, t)
                if d:
                    remember(self.dest, d["variant"], now, d)
                if z:
                    remember(self.zones, z["name"], now, z)
        dests = recall(self.dest, now)
        # zone yaw = long side, modulo pi (median above on the unambiguous tag yaw, then folded)
        zones = sorted((dict(z, yaw=round((z["yaw"] + math.pi / 2) % math.pi - math.pi / 2, 4))
                        for z in recall(self.zones, now)), key=lambda z: z["name"])
        msg = {"stamp": now, "destinations": dests, "zones": zones, "image_size": [frame.shape[1], frame.shape[0]],
               "source": self.device, "live": self.still is None, "calibrated": self.calib is not None, "boards": boards, "occluded": occluded, "rejected": rejected}
        self.last = json.dumps(msg)
        img = self.annotate(frame, boards, occluded, rejected, tags)
        if time.time() - getattr(self, "saved", 0) > 2.0:  # debug overlay for the lab, every 2 s
            self.saved = time.time()
            cv2.imwrite("/tmp/vision1_last.jpg", img)
        self.pub.publish(String(data=self.last))
        if self.image.get_subscription_count():
            small = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2), interpolation=cv2.INTER_AREA)
            m = Image(height=small.shape[0], width=small.shape[1], encoding="bgr8", step=small.shape[1] * 3,
                      data=small.tobytes())
            m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "overhead_camera"
            self.image.publish(m)
        self.publish_markers(boards, dests)
        return True

    def annotate(self, frame, boards, occluded, rejected, tags):
        """Camera frame + pick area, boards (green = pickable, orange = under another), rejects, template tags."""
        img = draw(frame, boards + occluded, font=1.8)  # shown at ~1/3 size in the RViz panel
        for r in rejected:
            cv2.putText(img, r["reason"], (int(r["u"]) - 90, int(r["v"])), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 255), 4)
        for t in tags:
            c = np.int32(t["corners"])
            cv2.polylines(img, [c], True, (0, 0, 255), 3)
            name = (table.ZONES[t["id"]][0] if t["id"] in table.ZONES else
                    table.DEST[t["id"]][0] if t["id"] in table.DEST else f"tag {t['id']}")
            cv2.putText(img, name, (int(c[:, 0].min()) - 40, int(c[:, 1].max()) + 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5,
                        (0, 0, 255), 4)
        n = {}
        for b in boards:
            n[b["variant"]] = n.get(b["variant"], 0) + 1
        text = "pickable: " + (", ".join(f"{k} x{v}" for k, v in sorted(n.items())) or "none")
        if occluded:
            text += f" | under others: {len(occluded)}"
        cv2.rectangle(img, (0, 0), (img.shape[1], 90), (0, 0, 0), -1)
        cv2.putText(img, text, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 2.2, (255, 255, 255), 5)
        return img

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
            m.scale.x, m.scale.y, m.scale.z = [v / 1000 for v in table.SIZES_MM.get(o["variant"], o.get("size_mm", (100, 100)))] + [0.005]
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
