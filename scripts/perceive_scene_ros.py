from collections import Counter
import json
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from perceive_scene import perceive


ROI = (1120, 330, 1840, 980)


class ScenePerception(Node):
    def __init__(self):
        super().__init__("fr5_replay_scene_perception")
        self.bridge = CvBridge()
        self.sequence = 0
        self.publisher = self.create_publisher(
            String, "/fr5/replay/scene_json", 10
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Image, "/fr5/replay/image_raw", self.on_image, image_qos
        )
        self.get_logger().info(
            "Listening on /fr5/replay/image_raw. "
            "Historical-photo ROI; no robot targets."
        )

    def on_image(self, message):
        started = time.perf_counter()
        received = self.get_clock().now()
        self.sequence += 1

        source_stamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        provenance = message.header.frame_id
        filename = (
            provenance[len("replay:"):]
            if provenance.startswith("replay:") else None
        )

        try:
            if filename is None:
                raise ValueError("Expected replay: filename provenance")

            frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            )
            if frame.shape[:2] != (1080, 1920):
                raise ValueError("Historical ROI requires 1920x1080 images")

            scene = perceive(frame, ROI)
            scene["status"] = "processed"
            scene["error"] = None

        except Exception as error:
            # Publish an explicit error instead of retaining a prior good scene.
            scene = {
                "schema_version": 1,
                "status": "error",
                "error": str(error),
                "destination_tags": [],
                "source_candidates": [],
                "rejected_source_regions": [],
                "diagnostics": {},
            }

        finished = self.get_clock().now()
        elapsed_ms = (time.perf_counter() - started) * 1000

        scene.update({
            "mode": "ros_image_replay",
            "sequence": self.sequence,
            "source_image": filename,
            "input_topic": "/fr5/replay/image_raw",
            "image_publication_stamp_ns": source_stamp_ns,
            "received_stamp_ns": received.nanoseconds,
            "processed_stamp_ns": finished.nanoseconds,
            "processing_ms": round(elapsed_ms, 2),
            "transport_age_at_receive_ms": round(
                (received.nanoseconds - source_stamp_ns) / 1_000_000, 2
            ),
            "capture_timestamp_known": False,
            "historical_image": True,
            "metric_calibrated": False,
            "execution_allowed": False,
            "robot_targets": [],
        })

        output = String()
        output.data = json.dumps(scene, allow_nan=False)
        self.publisher.publish(output)

        if scene["status"] == "error":
            self.get_logger().error(scene["error"])
            return

        ids = sorted(t["tag_id"] for t in scene["destination_tags"])
        colours = Counter(
            candidate["colour"] for candidate in scene["source_candidates"]
        )
        missing = scene["diagnostics"]["missing_destination_ids"]
        self.get_logger().info(
            f"#{self.sequence} {filename}: tags={ids}, missing={missing}, "
            f"white={colours['white']}, red={colours['red']}, "
            f"processing={elapsed_ms:.0f}ms"
        )


def main():
    rclpy.init()
    node = None
    try:
        node = ScenePerception()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
