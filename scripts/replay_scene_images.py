import argparse
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ReplayImages(Node):
    def __init__(self, paths, period):
        super().__init__("fr5_scene_image_replay")
        self.bridge = CvBridge()
        self.frames = []
        for path in paths:
            frame = cv2.imread(str(path))
            if frame is None:
                raise RuntimeError(f"Cannot read {path}")
            if frame.shape[:2] != (1080, 1920):
                raise RuntimeError(f"Expected 1920x1080: {path}")
            self.frames.append((path.name, frame))

        self.publisher = self.create_publisher(
            Image, "/fr5/replay/image_raw", qos_profile_sensor_data
        )
        self.index = 0
        self.timer = self.create_timer(period, self.publish_next)
        self.get_logger().info(
            f"Loaded {len(self.frames)} photos. REPLAY ONLY. "
            "Waiting for an image subscriber."
        )

    def publish_next(self):
        if self.publisher.get_subscription_count() == 0:
            return

        filename, frame = self.frames[self.index]
        message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        message.header.stamp = self.get_clock().now().to_msg()
        # Replay-only provenance, not a physical TF frame.
        message.header.frame_id = f"replay:{filename}"
        self.publisher.publish(message)
        self.get_logger().info(f"Published {filename}")
        self.index = (self.index + 1) % len(self.frames)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("images", nargs="+", type=Path)
    args = parser.parse_args()

    rclpy.init()
    node = None
    try:
        # Slow replay initially: measure detector cost before increasing rate.
        node = ReplayImages(args.images, period=3.0)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
